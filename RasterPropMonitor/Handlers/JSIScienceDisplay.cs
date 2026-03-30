using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using UnityEngine;

namespace JSI
{
    public class ExperimentDetailsMenu : TextMenu
    {
        public ExperimentDetailsMenu(ModuleScienceExperiment experimentModule)
        {
            m_experimentModule = experimentModule;
			m_results = string.Empty;

            menuTitle = experimentModule.experiment.experimentTitle;

            for (int actionIndex = 0; actionIndex < experimentModule.Actions.Count; ++actionIndex)
            {
                var action = experimentModule.Actions[actionIndex];

                Add(new TextMenu.Item(action.guiName, (id, item) => {
                    if (action.name == "DeployAction")
                    {
                        JSIScienceDisplay.RunScience(experimentModule);
                    }
                    else
                    {
                        action.Invoke(JSIScienceDisplay.activateParam);
                    }
                }));
            }
        }

        public override void ShowMenu(StringBuilder menuString, int width, int height)
        {
            base.ShowMenu(menuString, width, height);

            menuString.AppendLine();

            var scienceData = m_experimentModule.GetData().FirstOrDefault();

            if (scienceData != null)
            {
                Util.WordWrap(scienceData.title, menuString, width);

				// Some experiment results have multiple flavor texts, and GetResults returns a random one
				// So cache the subjectID and results string and only update results if the subject changes.
				if (m_subjectID != scienceData.subjectID)
				{
					m_results = ResearchAndDevelopment.GetResults(scienceData.subjectID);
					m_subjectID = scienceData.subjectID;
				}

                Util.WordWrap(m_results, menuString, width);
            }
        }

        ModuleScienceExperiment m_experimentModule;
		string m_subjectID;
		string m_results;
    }

    static class Util
    {
        public static int WordWrap(string message, StringBuilder builder, int width)
        {
            int currentMessageCharacter = 0;
            int linesUsed = 0;

            while (currentMessageCharacter < message.Length)
            {
                AddMessageChunk(message, builder, ref currentMessageCharacter, width);
                ++linesUsed;
            }

            return linesUsed;
        }

        static void AddMessageChunk(string message, StringBuilder stringBuilder, ref int currentMessageCharacter, int spaceRemainingOnLine)
        {
            int charactersToCopy = message.Length - currentMessageCharacter;

            // if the rest of this message can't fit on this line..
            if (charactersToCopy > spaceRemainingOnLine)
            {
                int lastSpaceIndex = message.LastIndexOf(' ', currentMessageCharacter + spaceRemainingOnLine, spaceRemainingOnLine);

                if (lastSpaceIndex >= 0)
                {
                    charactersToCopy = lastSpaceIndex - currentMessageCharacter;
                }
                else
                {
                    charactersToCopy = spaceRemainingOnLine;
                }
            }

            stringBuilder.Append(message, currentMessageCharacter, charactersToCopy);
            stringBuilder.AppendLine();
            currentMessageCharacter += charactersToCopy;
        }
    }

    public class JSIScienceDisplay : InternalModule
    {
        [KSPField]
        public string pageTitle;

        [KSPField]
        public int buttonUp = 0;
        [KSPField]
        public int buttonDown = 1;
        [KSPField]
        public int buttonEnter = 2;
        [KSPField]
        public int buttonEsc = 3;
        [KSPField]
        public int buttonHome = 4;
        [KSPField]
        public int buttonRight = 5;
        [KSPField]
        public int buttonLeft = 6;
        [KSPField]
        public int buttonNext = 7;
        [KSPField]
        public int buttonPrev = 8;

        private List<ModuleScienceExperiment> experimentModules;
        private List<ModuleScienceContainer> containerModules;
        private StringBuilder stringBuilder = new StringBuilder();
        private string response;

        public static readonly KSPActionParam activateParam = new KSPActionParam(KSPActionGroup.None, KSPActionType.Activate);

        private TextMenu topMenu;
        private TextMenu experimentsMenu;
        private TextMenu containersMenu;

        private List<TextMenu> menuStack = new List<TextMenu>(4);

        private TextMenu CurrentMenu {  get { return menuStack[menuStack.Count - 1]; } }

        public void Start()
        {
            if (!string.IsNullOrEmpty(pageTitle))
                pageTitle = pageTitle.UnMangleConfigText();

            topMenu = new TextMenu();
            topMenu.menuTitle = "== Science ==";

			experimentsMenu = new TextMenu();
			experimentsMenu.menuTitle = "== Experiments ==";
			experimentsMenu.rightColumnWidth = 6;
			experimentsMenu.rightTextColor = "[#00ff00]";

			containersMenu = new TextMenu();
			containersMenu.menuTitle = "== Containers ==";

            topMenu.Add(new TextMenu.Item("Run all science", RunAllScience));
            topMenu.Add(new TextMenu.Item("Experiments", (id, item) => OpenSubMenu(experimentsMenu)));
            topMenu.Add(new TextMenu.Item("Containers", (id, item) => OpenSubMenu(containersMenu)));

            OpenSubMenu(topMenu);
        }

        public string Display(int screenWidth, int screenHeight)
        {
            RefreshModules();

            stringBuilder.Length = 0;

            if (!string.IsNullOrEmpty(pageTitle))
            {
                stringBuilder.AppendLine(pageTitle);
                --screenHeight;
            }

            CurrentMenu.ShowMenu(stringBuilder, screenWidth, screenHeight);

            response = stringBuilder.ToString();

            return response;
        }

        public void ButtonProcessor(int buttonID)
        {
            if (buttonID == buttonUp)
            {
                CurrentMenu.PreviousItem();
            }
            if (buttonID == buttonDown)
            {
                CurrentMenu.NextItem();
            }
            if (buttonID == buttonEnter)
            {
                CurrentMenu.SelectItem();
            }
            if (buttonID == buttonEsc)
            {
                CloseSubMenu();
            }
            if (buttonID == buttonHome)
            {
                menuStack.RemoveRange(1, menuStack.Count - 1);
            }

            if (buttonID == buttonRight)
            {
                JUtil.LogInfo(null, "dumping experiment");
                DumpObject(experimentModules[experimentsMenu.GetCurrentIndex()]);
            }

            if (buttonID == buttonLeft)
            {
                DumpObject(containerModules[containersMenu.GetCurrentIndex()]);
            }

            if (buttonID == buttonNext)
            {
                var experiment = experimentModules[experimentsMenu.GetCurrentIndex()];

                var scienceData = experiment.GetData();

                DumpObject(experiment);

                JUtil.LogInfo(null, "{0} science data", scienceData.Length);

                foreach (var data in scienceData)
                {
                    JUtil.LogInfo(null, "science data");
                    DumpObject(data);

                    var subject = ResearchAndDevelopment.GetSubjectByID(data.subjectID);
                    JUtil.LogInfo(null, "subject");
                    DumpObject(subject);
                }

                JUtil.LogInfo(null, "experiment");

                DumpObject(experiment.experiment);

                JUtil.LogInfo(null, "actions");

                foreach (var action in experiment.Actions)
                {
                    DumpObject(action);
                }
            }
        }

        void DumpObject(object obj)
        {
            var type = obj.GetType();
            var fields = type.GetFields(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance);
            var properties = type.GetProperties(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance);

            Debug.Log(string.Format("{0} fields", fields.Length));

            foreach (var field in fields)
            {
                Debug.Log(string.Format("{0} : {1}", field.Name, field.GetValue(obj)));
            }

            Debug.Log(string.Format("{0} properties", properties.Length));

            foreach (var prop in properties)
            {
                Debug.Log(string.Format("{0} : {1}", prop.Name, prop.GetValue(obj, null)));
            }
        }

        private void RunAllScience(int id, TextMenu.Item item)
        {
            experimentModules.ForEach(exp =>
            {
                RunScience(exp);
            });
        }

        public static void RunScience(ModuleScienceExperiment experimentModule)
        {
            bool wasStaged = experimentModule.useStaging;
            experimentModule.useStaging = true;
            experimentModule.OnActive();
            experimentModule.useStaging = wasStaged;
        }

        private void OpenSubMenu(TextMenu subMenu)
        {
            menuStack.Add(subMenu);
        }

        private void CloseSubMenu()
        {
            if (menuStack.Count > 1)
            {
                menuStack.RemoveAt(menuStack.Count - 1);
            }
        }

        private void RefreshModules()
        {
            experimentModules = vessel.FindPartModulesImplementing<ModuleScienceExperiment>();
            containerModules = vessel.FindPartModulesImplementing<ModuleScienceContainer>();

			experimentsMenu.Clear();

			int experimentModuleCount = experimentModules == null ? 0 : experimentModules.Count;
			for (int experimentIndex = 0; experimentIndex < experimentModuleCount; ++experimentIndex)
            {
                var experimentModule = experimentModules[experimentIndex];
				var experimentItem = new TextMenu.Item(experimentModule.experiment.experimentTitle, OpenExperimentDetails, experimentIndex);
				var data = experimentModule.GetData().FirstOrDefault();

                if (data != null)
                {
                    var subject = ResearchAndDevelopment.GetSubjectByID(data.subjectID);

					if (subject != null)
					{
						var scienceValue = data.dataAmount / subject.dataScale * subject.subjectValue * subject.scientificValue;
						experimentItem.rightText = scienceValue.ToString("F1");
					}
					else
					{
						DumpObject(experimentModule);
					}
                }

				experimentsMenu.Add(experimentItem);
            }

            containersMenu.Clear();
			int containerCount = containerModules == null ? 0 : containerModules.Count;
            for (int containerIndex = 0; containerIndex < containerCount; ++containerIndex)
            {
				containersMenu.Add(new TextMenu.Item(containerModules[containerIndex].part.partInfo.title, OpenContainerDetails, containerIndex));
            }
		}

        private void OpenExperimentDetails(int experimentIndex, TextMenu.Item experimentItem)
        {
            var experimentModule = experimentModules[experimentIndex];
            var detailsMenu = new ExperimentDetailsMenu(experimentModule);
            OpenSubMenu(detailsMenu);
        }

        private void OpenContainerDetails(int containerIndex, TextMenu.Item containerItem)
        {
            var containerModule = containerModules[containerIndex];
            var detailsMenu = new TextMenu();
            detailsMenu.menuTitle = containerModule.GUIName;

            containerModule.updateModuleUI();

            foreach (var evt in containerModule.events)
            {
                if (evt.guiActive && evt.active)
                {
                    detailsMenu.Add(new TextMenu.Item(evt.GUIName, (id, item) => evt.Invoke()));
                }
            }

            OpenSubMenu(detailsMenu);
        }
    }
}
