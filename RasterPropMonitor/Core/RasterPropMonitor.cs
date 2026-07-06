/*****************************************************************************
 * RasterPropMonitor
 * =================
 * Plugin for Kerbal Space Program
 *
 *  by Mihara (Eugene Medvedev), MOARdV, and other contributors
 * 
 * RasterPropMonitor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, revision
 * date 29 June 2007, or (at your option) any later version.
 * 
 * RasterPropMonitor is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RasterPropMonitor.  If not, see <http://www.gnu.org/licenses/>.
 ****************************************************************************/
using System;
using UnityEngine;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine.Profiling;

namespace JSI
{
    public class RasterPropMonitor : InternalModule
    {
        [SerializeReference] ConfigNodeHolder moduleConfig;

        [KSPField]
        public string screenTransform = "screenTransform";
        [KSPField]
        public string fontTransform = "fontTransform";
        [KSPField]
        public string textureLayerID = "_MainTex";
        [KSPField]
        public string emptyColor = string.Empty;
        public Color emptyColorValue = Color.clear;
        [KSPField]
        public int screenWidth = 32;
        [KSPField]
        public int screenHeight = 8;
        [KSPField]
        public int screenPixelWidth = 512;
        [KSPField]
        public int screenPixelHeight = 256;
        [KSPField]
        public int fontLetterWidth = 16;
        [KSPField]
        public int fontLetterHeight = 32;
        [KSPField]
        public float cameraAspect = 2f;
        [KSPField]
        public int refreshDrawRate = 2;
        [KSPField]
        public int refreshTextRate = 5;
        [KSPField]
        public int refreshDataRate = 10;
        [KSPField]
        public string globalButtons;
        [KSPField]
        public string buttonClickSound;
        [KSPField]
        public float buttonClickVolume = 0.5f;
        [KSPField]
        public bool needsElectricCharge = true;
        [KSPField]
        public string resourceName = "SYSR_ELECTRICCHARGE";
        private bool resourceDepleted = false; // Managed by rpmComp callback
        [KSPField]
        public bool needsCommConnection = false;
        private bool noCommConnection = false; // Managed by rpmComp callback
        [KSPField]
        public string defaultFontTint = string.Empty;
        public Color defaultFontTintValue = Color.white;
        [KSPField]
        public string noSignalTextureURL = string.Empty;
        [KSPField]
        public string fontDefinition = string.Empty;
        [KSPField]
        public bool doScreenshots = true;
        [KSPField]
        public bool oneshot = false;
        // Internal stuff.
        private TextRenderer textRenderer;
        private RenderTexture screenTexture;
        private Texture2D frozenScreen;
        // Local variables
        private int refreshDrawCountdown;
        private int refreshTextCountdown;
        private int vesselNumParts;
        private bool firstRenderComplete;
        private bool textRefreshRequired;
        private readonly List<MonitorPage> pages = new List<MonitorPage>();
        private MonitorPage activePage;
        private string persistentVarName;
        private FXGroup audioOutput;
        public Texture2D noSignalTexture;
        private GameObject screenObject;
        private Material screenMat;
        private bool startupComplete;
        private string fontDefinitionString = @" !""#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~Δ☊¡¢£¤¥¦§¨©ª«¬☋®¯°±²³´µ¶·¸¹º»¼½¾¿";
        private RasterPropMonitorComputer rpmComp;

        private static Texture2D LoadFont(object caller, InternalProp thisProp, string location)
        {
            Texture2D font = null;
            if (!string.IsNullOrEmpty(location))
            {
                try
                {
                    if (GameDatabase.Instance.ExistsTexture(location.EnforceSlashes()))
                    {
                        font = GameDatabase.Instance.GetTexture(location.EnforceSlashes(), false);
                        JUtil.LogMessage(caller, "Loading font texture from URL \"{0}\"", location);
                    }
                    else
                    {
                        font = (Texture2D)thisProp.FindModelTransform(location).GetComponent<Renderer>().material.mainTexture;
                        JUtil.LogMessage(caller, "Loading font texture from a transform named \"{0}\"", location);
                    }

                    font.filterMode = FilterMode.Point;
                    font.requestedMipmapLevel = 0;
                }
                catch (Exception)
                {
                    JUtil.LogErrorMessage(caller, "Failed loading font texture \"{0}\" - missing texture?", location);
                }
            }
            return font;
        }

        public override void OnLoad(ConfigNode node)
        {
            moduleConfig = ScriptableObject.CreateInstance<ConfigNodeHolder>();
            moduleConfig.Node = node;
        }

        public void Start()
        {

            // If we're not in the correct location, there's no point doing anything.
            if (!InstallationPathWarning.Warn())
            {
                return;
            }

            if (HighLogic.LoadedSceneIsEditor)
            {
                return;
            }

            try
            {
                rpmComp = RasterPropMonitorComputer.FindFromProp(internalProp);
                JUtil.LogMessage(this, "Attaching monitor {2}-{1} to {0}", rpmComp.RPMCid, internalProp.propID, internalProp.internalModel.internalName);

                // Install the calculator module.
                rpmComp.UpdateDataRefreshRate(refreshDataRate);

                // Loading the font...
                List<Texture2D> fontTexture = new List<Texture2D>();
                fontTexture.Add(LoadFont(this, internalProp, fontTransform));

                // Damn KSP's config parser!!!
                if (!string.IsNullOrEmpty(emptyColor))
                {
                    emptyColorValue = ConfigNode.ParseColor32(emptyColor);
                }
                if (!string.IsNullOrEmpty(defaultFontTint))
                {
                    defaultFontTintValue = ConfigNode.ParseColor32(defaultFontTint);
                }

                if (!string.IsNullOrEmpty(fontDefinition))
                {
                    JUtil.LogMessage(this, "Loading font definition from {0}", fontDefinition);
                    fontDefinitionString = File.ReadAllLines(KSPUtil.ApplicationRootPath + "GameData/" + fontDefinition.EnforceSlashes(), Encoding.UTF8)[0];
                }

                // Now that is done, proceed to setting up the screen.

                screenTexture = new RenderTexture(screenPixelWidth, screenPixelHeight, 24, RenderTextureFormat.ARGB32, 0);
                screenObject = internalProp.FindModelTransform(screenTransform).gameObject;
                var renderer = screenObject.AddComponent<VisibilityEnabler>();
                renderer.Initialize(this);
                screenMat = screenObject.GetComponent<Renderer>().material;

                bool manuallyInvertY = false;
                //if (SystemInfo.graphicsDeviceVersion.StartsWith("Direct3D 9") || SystemInfo.graphicsDeviceVersion.StartsWith("Direct3D 11") || SystemInfo.graphicsDeviceVersion.StartsWith("Direct3D 12"))
                //{
                //    manuallyInvertY = (UnityEngine.QualitySettings.antiAliasing > 0);
                //}

                foreach (string layerID in textureLayerID.Split())
                {
                    screenMat.SetTexture(layerID.Trim(), screenTexture);
                    // This code was written for a much older flavor of Unity, and the Unity 2017.1 update broke
                    // some assumptions about who managed the y-inversion issue between OpenGL and DX9.
                    if (manuallyInvertY)
                    {
                        screenMat.SetTextureScale(layerID.Trim(),  new Vector2(1.0f, -1.0f));
                        screenMat.SetTextureOffset(layerID.Trim(),  new Vector2(0.0f, 1.0f));
                    }
                }

                if (GameDatabase.Instance.ExistsTexture(noSignalTextureURL.EnforceSlashes()))
                {
                    noSignalTexture = GameDatabase.Instance.GetTexture(noSignalTextureURL.EnforceSlashes(), false);
                }

                ConfigNode[] pageNodes = moduleConfig.Node.GetNodes("PAGE");

                // parse page definitions
                for (int i = 0; i < pageNodes.Length; i++)
                {
                    // Mwahahaha.
                    try
                    {
                        var newPage = new MonitorPage(i, pageNodes[i], this);
                        activePage = activePage ?? newPage;
                        if (newPage.isDefault)
                            activePage = newPage;
                        pages.Add(newPage);
                    }
                    catch (ArgumentException e)
                    {
                        JUtil.LogMessage(this, "Warning - {0}", e);
                    }

                }

                // Now that all pages are loaded, we can use the moment in the loop to suck in all the extra fonts.
                foreach (string value in moduleConfig.Node.GetValues("extraFont"))
                {
                    fontTexture.Add(LoadFont(this, internalProp, value));
                }

                JUtil.LogMessage(this, "Done setting up pages, {0} pages ready.", pages.Count);

                textRenderer = new TextRenderer(fontTexture, new Vector2((float)fontLetterWidth, (float)fontLetterHeight), fontDefinitionString, 17, screenPixelWidth, screenPixelHeight);

                // Load our state from storage...
                persistentVarName = "activePage" + internalProp.propID;
                int activePageID = rpmComp.GetPersistentVariable(persistentVarName, pages.Count, false).MassageToInt();
                if (activePageID < pages.Count)
                {
                    activePage = pages[activePageID];
                }
                activePage.Active(true);

                // If we have global buttons, set them up.
                if (!string.IsNullOrEmpty(globalButtons))
                {
                    string[] tokens = globalButtons.Split(',');
                    for (int i = 0; i < tokens.Length; i++)
                    {
                        string buttonName = tokens[i].Trim();
                        // Notice that holes in the global button list ARE legal.
                        if (!string.IsNullOrEmpty(buttonName))
                            SmarterButton.CreateButton(internalProp, buttonName, i, GlobalButtonClick, GlobalButtonRelease);
                    }
                }

                audioOutput = JUtil.SetupIVASound(internalProp, buttonClickSound, buttonClickVolume, false);

                if (needsElectricCharge)
                {
                    rpmComp.RegisterResourceCallback(resourceName, ResourceDepletedCallback);
                }

                if (needsCommConnection)
                {
                    rpmComp.RegisterVariableCallback("COMMNETVESSELCONTROLSTATE", CommConnectionCallback);
                }

                // And if the try block never completed, startupComplete will never be true.
                startupComplete = true;
            }
            catch
            {
                JUtil.AnnoyUser(this);
                // We can also disable ourselves, that should help.
                enabled = false;
                // And now that we notified the user that config is borked, we rethrow the exception so that
                // it gets logged and we can debug.
                throw;
            }

        }

        public void OnDestroy()
        {
            // Makes sure we don't leak our render texture
            if (screenTexture != null)
            {
                screenTexture.Release();
                screenTexture = null;
            }
            if (frozenScreen != null)
            {
                Destroy(frozenScreen);
            }
            if (screenMat != null)
            {
                Destroy(screenMat);
            }
            if (rpmComp != null)
            {
                rpmComp.UnregisterResourceCallback(resourceName, ResourceDepletedCallback);
                rpmComp.UnregisterVariableCallback("COMMNETVESSELCONTROLSTATE", CommConnectionCallback);
            }
        }

        private static void PlayClickSound(FXGroup audioOutput)
        {
            if (audioOutput != null)
            {
                audioOutput.audio.Play();
            }
        }

        public void GlobalButtonClick(int buttonID)
        {
            if (resourceDepleted || noCommConnection)
            {
                return;
            }
            if (activePage.GlobalButtonClick(buttonID))
            {
                PlayClickSound(audioOutput);
            }
        }

        public void GlobalButtonRelease(int buttonID)
        {
            // Or do we allow a button release to have effects?
            /* Mihara: Yes, I think we should. Otherwise if the charge
             * manages to run out in the middle of a pressed button, it will never stop.
            if (needsElectricCharge && electricChargeReserve < 0.01f)
                return;
            */
            activePage.GlobalButtonRelease(buttonID);
        }

        private MonitorPage FindPageByName(string pageName)
        {
            if (!string.IsNullOrEmpty(pageName))
            {
                foreach (MonitorPage page in pages)
                {
                    if (page.name == pageName)
                        return page;
                }
            }
            return null;
        }

        public void PageButtonClick(MonitorPage triggeredPage)
        {
            if (resourceDepleted || noCommConnection)
            {
                return;
            }

            // Apply page redirect like this:
            triggeredPage = FindPageByName(activePage.ContextRedirect(triggeredPage.name)) ?? triggeredPage;
            if (triggeredPage != activePage && (activePage.SwitchingPermitted(triggeredPage.name) || triggeredPage.unlocker))
            {
                activePage.Active(false);
                activePage = triggeredPage;
                activePage.Active(true);
                rpmComp.SetPersistentVariable(persistentVarName, activePage.pageNumber, false);
                refreshDrawCountdown = refreshTextCountdown = 0;
                firstRenderComplete = false;
                PlayClickSound(audioOutput);
            }
        }

        internal void SelectNextPatch()
        {
            rpmComp.SelectNextPatch();
        }

        internal void SelectPreviousPatch()
        {
            rpmComp.SelectPreviousPatch();
        }

        // Update according to the given refresh rate.
        private bool UpdateCheck()
        {
            refreshDrawCountdown--;
            refreshTextCountdown--;
            if (vesselNumParts != vessel.Parts.Count)
            {
                refreshDrawCountdown = 0;
                refreshTextCountdown = 0;
                vesselNumParts = vessel.Parts.Count;
            }
            if (refreshTextCountdown <= 0)
            {
                textRefreshRequired = true;
                refreshTextCountdown = refreshTextRate;
            }

            if (refreshDrawCountdown <= 0)
            {
                refreshDrawCountdown = refreshDrawRate;
                return true;
            }

            return false;
        }

        private void RenderScreen()
        {
			Profiler.BeginSample("RPM.RenderScreen [" + activePage.name + "]");

			RenderTexture backupRenderTexture = RenderTexture.active;

            if (!screenTexture.IsCreated())
            {
                screenTexture.Create();
            }
            
			if (resourceDepleted || noCommConnection)
			{
                screenTexture.DiscardContents();
                RenderTexture.active = screenTexture;
                // If we're out of electric charge, we're drawing a blank screen.
                GL.Clear(true, true, emptyColorValue);
			}
			else if (textRenderer.UpdateText(activePage) || activePage.background == MonitorPage.BackgroundType.Handler)
			{
                screenTexture.DiscardContents();
                RenderTexture.active = screenTexture;

                // This is the important witchcraft. Without that, DrawTexture does not print where we expect it to.
                // Cameras don't care because they have their own matrices, but DrawTexture does.
                GL.PushMatrix();
				GL.LoadPixelMatrix(0, screenPixelWidth, screenPixelHeight, 0);

				// Actual rendering of the background is delegated to the page object.
				activePage.RenderBackground(screenTexture);

				if (!string.IsNullOrEmpty(activePage.ProcessedText))
				{
					textRenderer.Render(screenTexture);
				}

				activePage.RenderOverlay(screenTexture);
				GL.PopMatrix();
			}

			RenderTexture.active = backupRenderTexture;
			Profiler.EndSample();
		}

        private void FillScreenBuffer()
        {
			Profiler.BeginSample("RasterPropMonitor.FillScreenBuffer");
			activePage.UpdateText(rpmComp);
			Profiler.EndSample();
        }

        public void LateUpdate()
        {
            if (HighLogic.LoadedSceneIsEditor)
            {
                return;
            }

            // If we didn't complete startup, we can't do anything anyway.
            // The only trouble is that situations where update happens before startup is complete do happen sometimes,
            // particularly when docking, so we can't use it to detect being broken by a third party plugin.
            if (!startupComplete)
            {
                return;
            }
			
            if (!JUtil.RasterPropMonitorShouldUpdate(part))
            {
                return;
            }

            // Screenshots need to happen in at this moment, because otherwise they may miss.
            if (doScreenshots && GameSettings.TAKE_SCREENSHOT.GetKeyDown() && part.ActiveKerbalIsLocal())
            {
                // Let's try to save a screenshot.
                JUtil.LogMessage(this, "SCREENSHOT!");

                string screenshotName = string.Format("{0}{1}{2:yyyy-MM-dd_HH-mm-ss}_{4}_{3}.png",
                                            KSPUtil.ApplicationRootPath, "Screenshots/monitor", DateTime.Now, internalProp.propID, part.GetInstanceID());
                var screenshot = new Texture2D(screenTexture.width, screenTexture.height);
                RenderTexture backupRenderTexture = RenderTexture.active;
                RenderTexture.active = screenTexture;
                screenshot.ReadPixels(new Rect(0, 0, screenTexture.width, screenTexture.height), 0, 0);
                RenderTexture.active = backupRenderTexture;
                var bytes = ImageConversion.EncodeToPNG(screenshot);
                Destroy(screenshot);
                File.WriteAllBytes(screenshotName, bytes);
            }

            if (!UpdateCheck())
            {
                return;
            }

			Profiler.BeginSample("RasterPropMonitor.OnLateUpdate");

            if (resourceDepleted || noCommConnection)
            {
                // this is a bit wasteful since we could just store the blanked texture, but at least it's not going to do any string processing
                RenderScreen();
                firstRenderComplete = false;
                textRefreshRequired = true;
            }
            else if (!activePage.isMutable)
            {
                // In case the page is empty and has no camera, the screen is treated as turned off and blanked once.
                if (!firstRenderComplete)
                {
                    FillScreenBuffer();
                    RenderScreen();
                    firstRenderComplete = true;
                    textRefreshRequired = false;
                }
            }
            else
            {
                if (textRefreshRequired)
                {
                    FillScreenBuffer();
                    textRefreshRequired = false;
                }
                RenderScreen();

                firstRenderComplete = true;
            }

			// Oneshot screens: We create a permanent texture from our RenderTexture if the first pass of the render is complete,
			// set it in place of the rendertexture -- and then we selfdestruct.
			// MOARdV: Except we don't want to self-destruct, because we will leak the frozenScreen texture.
			if (oneshot && firstRenderComplete)
			{
				frozenScreen = new Texture2D(screenTexture.width, screenTexture.height);
				RenderTexture backupRenderTexture = RenderTexture.active;
				RenderTexture.active = screenTexture;
				frozenScreen.ReadPixels(new Rect(0, 0, screenTexture.width, screenTexture.height), 0, 0);
				RenderTexture.active = backupRenderTexture;
				foreach (string layerID in textureLayerID.Split())
				{
					screenMat.SetTexture(layerID.Trim(), frozenScreen);
				}
			}

			Profiler.EndSample();
        }

        public void OnApplicationPause(bool pause)
        {
            firstRenderComplete &= pause;
        }

        //public void LateUpdate()
        //{

        //    if (HighLogic.LoadedSceneIsEditor)
        //        return;

        //    // If we reached a set number of update loops and startup still didn't happen, we're getting killed by a third party module.
        //    // We might STILL be getting killed by a third party module even during update, but I hope this will catch at least some cases.
        //    if (!startupFailed && loopsWithoutInitCounter > 600)
        //    {
        //        ScreenMessages.PostScreenMessage("RasterPropMonitor cannot complete initialization.", 120, ScreenMessageStyle.UPPER_CENTER);
        //        ScreenMessages.PostScreenMessage("The cause is usually some OTHER broken mod.", 120, ScreenMessageStyle.UPPER_CENTER);
        //        loopsWithoutInitCounter = 0;
        //    }
        //}

        /// <summary>
        /// This little callback allows RasterPropMonitorComputer to notify
        /// this module when its required resource has gone above or below the
        /// arbitrary and hard-coded threshold of 0.01, so that each monitor is
        /// not forced to query every update "How much power is there?".
        /// </summary>
        /// <param name="newValue"></param>
        void ResourceDepletedCallback(bool newValue)
        {
            resourceDepleted = newValue;
        }

        /// <summary>
        /// Similar to ResourceDepletedCallback, allows computer to inform monitor
        /// of commnet connection status.
        /// </summary>
        /// <param name="newValue"></param>
        void CommConnectionCallback(float newValue)
        {
            //None, ProbeNone, Partial, ProbePartial
            if ((newValue == 0.0f) || (newValue == 2.0f) || (newValue == 8.0f) || (newValue == 10.0f))
            {
                noCommConnection = true;
            }
            else
            {
                noCommConnection = false;
            }
        }
    }
}

