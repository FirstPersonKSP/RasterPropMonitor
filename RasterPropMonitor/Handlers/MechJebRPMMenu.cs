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
using System.Collections.Generic;
using UnityEngine;

namespace JSI
{
    /// <summary>
    /// MechJebRPMMenu provides a comprehensive text menu interface to MechJeb 2.15+
    /// matching full feature parity with MechJeb's IMGUI interface.
    /// </summary>
    public class MechJebRPMMenu : InternalModule
    {
        #region Configuration Fields
        [KSPField]
        public string pageTitle = "%.%.%.%.%.%.%.%.%.%.%.%.%.%.%.%.% %.%.%.%.%.%.%.%.%.%.%.%.%.%.%.%.%%.%.%.%.%.%.%.%.%.%.MechJeb%.%.%.%.%.%.%.%.%.%.%.%.%.%.%.%.% %.%.%.%.%.%.%.%.%.%.%.%.%.%.%.%.%";

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
        #endregion

        #region Instance State
        private TextMenu topMenu;
        private TextMenu currentMenu;
        private bool pageActiveState = false;
        private object mjCore = null;
        private Vessel activeVessel = null;

        private TextMenu smartassOrbitalMenu;
        private TextMenu smartassSurfaceMenu;
        private TextMenu smartassTargetMenu;

        // Maneuver planner state
        private double circularizeAltitudeKm = 100.0;
        private double changeApoapsisKm = 100.0;
        private double changePeriapsisKm = 70.0;
        private double changeSmaKm = 700.0;
        private double changeInclinationDeg = 0.0;
        private double changeLanDeg = 0.0;
        private double changeEccentricity = 0.5;
        private double resonanceNumerator = 2.0;
        private double resonanceDenominator = 1.0;
        private double interceptIntervalSeconds = 3600.0;
        private double moonReturnAltitudeKm = 100.0;
        private double fineTuneDistanceKm = 100.0;
        private double surfaceLongitudeDeg = 0.0;
        private double courseCorrectionPeKm = 50.0;

        // LEGACY: Hohmann state - no longer used, wrapper uses MechJeb's OperationGeneric directly
        // Keeping for reference only - these fields are not used after wrapper conversion
        // private object genericTransferOperation;
        // private bool genericCapture = true;
        // private bool genericPlanCapture = true;
        // private bool genericRendezvous = true;
        // private bool genericCoplanar = false;
        // private double genericLagTime = 0.0;

        // LEGACY: advancedTransferOperation not used - wrapper uses MechJeb's static array
        // Display cache variables still needed for UI refresh
        private bool advancedTransferSelectLowestDV = true;  // UI state for radio button display
        private double advancedTransferDeltaV = 0.0;         // Cached for display
        private double advancedTransferDepartureUT = 0.0;    // Cached for display
        private double advancedTransferDuration = 0.0;       // Cached for display

        // Stage stats update timing
        private double lastStageStatsUpdateUT = 0.0;
        
        // Menu stacks for navigation
        private Stack<TextMenu> menuStack = new Stack<TextMenu>();
        
        // Tracked menu items that need dynamic state updates
        private List<TrackedMenuItem> trackedItems = new List<TrackedMenuItem>();
        
        // Currently focused editable field
        private TextMenu.Item editingItem = null;
        private string editBuffer = "";
        private bool isEditing = false;
        #endregion

        #region Tracked Item Classes
        private class TrackedMenuItem
        {
            public TextMenu.Item item;
            public string id;
            public Func<object, bool> isEnabled;
            public Func<object, bool> isSelected;
            public Func<object, string> getLabel;
            public Func<object, string> getValue;
            public Action<object> action;
            public bool isValueItem;
            public Func<object, double> getNumber;
            public Action<object, double> setNumber;
            public double step;
            public bool hasMin;
            public double min;
            public bool hasMax;
            public double max;
        }
        #endregion

        #region Initialization
        public void Start()
        {
            MechJebProxy.Initialize();
            
            if (!MechJebProxy.IsAvailable)
            {
                JUtil.LogMessage(this, "MechJeb not available: {0}", MechJebProxy.InitializationError ?? "Unknown");
                return;
            }
            
            BuildMenus();
        }

        private void BuildMenus()
        {
            topMenu = new TextMenu();
            topMenu.labelColor = JUtil.ColorToColorTag(Color.white);
            topMenu.selectedColor = JUtil.ColorToColorTag(Color.green);
            topMenu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            // Add all main menu entries
            AddMenuItem(topMenu, "Attitude Control (SmartASS)", () => PushMenu(BuildSmartASSMenu()));
            AddMenuItem(topMenu, "Ascent Guidance", () => PushMenu(BuildAscentMenu()), 
                core => vessel != null && IsAscentAvailable(vessel));
            AddMenuItem(topMenu, "Landing Guidance", () => PushMenu(BuildLandingMenu()),
                core => vessel != null && !vessel.LandedOrSplashed);
            AddMenuItem(topMenu, "Maneuver Planner", () => PushMenu(BuildManeuverPlannerMenu()));
            AddMenuItem(topMenu, "Node Editor", () => PushMenu(BuildNodeEditorMenu()),
                core => vessel != null && vessel.patchedConicSolver != null && 
                        vessel.patchedConicSolver.maneuverNodes.Count > 0);
            AddMenuItem(topMenu, "Execute Node", () => ExecuteNode(),
                core => vessel != null && vessel.patchedConicSolver != null && 
                        vessel.patchedConicSolver.maneuverNodes.Count > 0);
            AddMenuItem(topMenu, "Rendezvous", () => PushMenu(BuildRendezvousMenu()),
                core => FlightGlobals.fetch.VesselTarget != null);
            AddMenuItem(topMenu, "Docking Guidance", () => PushMenu(BuildDockingMenu()),
                core => FlightGlobals.fetch.VesselTarget != null);
            AddMenuItem(topMenu, "Translatron", () => PushMenu(BuildTranslatronMenu()));
            AddMenuItem(topMenu, "Rover Autopilot", () => PushMenu(BuildRoverMenu()),
                core => vessel != null && vessel.Landed);
            AddMenuItem(topMenu, "Aircraft Autopilot", () => PushMenu(BuildAircraftMenu()),
                core => vessel != null && vessel.atmDensity > 0);
            AddMenuItem(topMenu, "Spaceplane Guidance", () => PushMenu(BuildSpaceplaneMenu()),
                core => vessel != null && vessel.atmDensity > 0);
            AddMenuItem(topMenu, "Utilities", () => PushMenu(BuildUtilitiesMenu()));
            AddMenuItem(topMenu, "Info Display", () => PushMenu(BuildInfoMenu()));
            AddMenuItem(topMenu, "Settings", () => PushMenu(BuildSettingsMenu()));

            currentMenu = topMenu;
        }

        private void AddMenuItem(TextMenu menu, string label, Action action, 
            Func<object, bool> enabledCheck = null)
        {
            Action<int, TextMenu.Item> menuAction = null;
            if (action != null)
            {
                menuAction = (idx, menuItem) => action();
            }
            
            var newItem = new TextMenu.Item(label, menuAction);
            menu.Add(newItem);
            
            if (enabledCheck != null)
            {
                trackedItems.Add(new TrackedMenuItem
                {
                    item = newItem,
                    id = label,
                    isEnabled = enabledCheck
                });
            }
        }

        // Overload for dynamic labels that update on refresh
        private void AddMenuItem(TextMenu menu, Func<string> labelFunc, Action action, 
            Func<object, bool> enabledCheck = null)
        {
            string initialLabel = labelFunc();
            Action<int, TextMenu.Item> menuAction = null;
            if (action != null)
            {
                menuAction = (idx, menuItem) => action();
            }
            
            var newItem = new TextMenu.Item(initialLabel, menuAction);
            menu.Add(newItem);
            
            trackedItems.Add(new TrackedMenuItem
            {
                item = newItem,
                id = "DynamicLabel_" + initialLabel,
                isEnabled = enabledCheck ?? (core => true),
                getLabel = core => labelFunc()
            });
        }

        private void AddToggleItem(TextMenu menu, string label, 
            Func<object, bool> getValue, Action<object, bool> setValue,
            Func<object, bool> enabledCheck = null)
        {
            Action<int, TextMenu.Item> toggleAction = (idx, menuItem) =>
            {
                if (mjCore == null) return;
                bool current = getValue(mjCore);
                setValue(mjCore, !current);
                UpdateTrackedItems();
            };

            // Use color highlighting for toggles - green when enabled, normal when disabled
            // No checkbox prefix needed since RPM interprets [text] as color tags
            var newItem = new TextMenu.Item(label, toggleAction);
            menu.Add(newItem);

            trackedItems.Add(new TrackedMenuItem
            {
                item = newItem,
                id = label,
                isEnabled = enabledCheck ?? (core => true),
                isSelected = getValue  // This makes the item green when checked
            });
        }

        private void AddValueItem(TextMenu menu, string label,
            Func<object, string> getValue, Action<object, double> setValue,
            Func<object, bool> enabledCheck = null)
        {
            Action<int, TextMenu.Item> editAction = (idx, menuItem) =>
            {
                // Start editing mode
                // For now, cycle through preset values or implement number input
            };

            var newItem = new TextMenu.Item(label, editAction);
            menu.Add(newItem);

            trackedItems.Add(new TrackedMenuItem
            {
                item = newItem,
                id = label,
                isEnabled = enabledCheck ?? (core => true),
                getValue = getValue,
                getLabel = core => label + ": " + getValue(core)
            });
        }

        private void AddNumericItem(TextMenu menu, string label,
            Func<object, double> getValue, Action<object, double> setValue,
            double step, Func<double, string> format,
            Func<object, bool> enabledCheck = null,
            bool hasMin = false, double min = 0,
            bool hasMax = false, double max = 0)
        {
            Action<int, TextMenu.Item> editAction = (idx, menuItem) =>
            {
                // Enter key toggles edit mode; actual changes use left/right
            };

            var newItem = new TextMenu.Item(label, editAction);
            menu.Add(newItem);

            trackedItems.Add(new TrackedMenuItem
            {
                item = newItem,
                id = label,
                isEnabled = enabledCheck ?? (core => true),
                isValueItem = true,
                getNumber = getValue,
                setNumber = setValue,
                step = step,
                hasMin = hasMin,
                min = min,
                hasMax = hasMax,
                max = max,
                getLabel = core => label + ": " + format(getValue(core))
            });
        }
        #endregion

        #region SmartASS Menu
        private TextMenu BuildSmartASSMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddMenuItem(menu, "[MODE: ORBITAL]", () => PushMenu(BuildSmartASSOrbitalMenu()));
            AddMenuItem(menu, "[MODE: SURFACE]", () => PushMenu(BuildSmartASSSurfaceMenu()));
            AddMenuItem(menu, "[MODE: TARGET]", () => PushMenu(BuildSmartASSTargetMenu()),
                core => FlightGlobals.fetch.VesselTarget != null);
            AddMenuItem(menu, "[MODE: ADVANCED]", () => PushMenu(BuildSmartASSAdvancedMenu()));
            AddMenuItem(menu, "[MODE: AUTO]", () => SetSmartASSAuto());
            AddMenuItem(menu, "------", null);
            AddMenuItem(menu, "OFF", () => SetSmartASSTarget(MechJebProxy.Target.OFF));
            AddMenuItem(menu, "KILL ROTATION", () => SetSmartASSTarget(MechJebProxy.Target.KILLROT));
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildSmartASSOrbitalMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            smartassOrbitalMenu = menu;

            menu.Add(new TextMenu.Item("PROGRADE", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.PROGRADE), (int)MechJebProxy.Target.PROGRADE));
            menu.Add(new TextMenu.Item("RETROGRADE", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.RETROGRADE), (int)MechJebProxy.Target.RETROGRADE));
            menu.Add(new TextMenu.Item("NORMAL+", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.NORMAL_PLUS), (int)MechJebProxy.Target.NORMAL_PLUS));
            menu.Add(new TextMenu.Item("NORMAL-", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.NORMAL_MINUS), (int)MechJebProxy.Target.NORMAL_MINUS));
            menu.Add(new TextMenu.Item("RADIAL+", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.RADIAL_PLUS), (int)MechJebProxy.Target.RADIAL_PLUS));
            menu.Add(new TextMenu.Item("RADIAL-", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.RADIAL_MINUS), (int)MechJebProxy.Target.RADIAL_MINUS));
            AddMenuItem(menu, "NODE", () => SetSmartASSTarget(MechJebProxy.Target.NODE),
                core => vessel != null && vessel.patchedConicSolver != null && 
                        vessel.patchedConicSolver.maneuverNodes.Count > 0);
            AddMenuItem(menu, "------", null);
            AddToggleItem(menu, "Force Roll", 
                core => MechJebProxy.GetSmartASSForceRoll(MechJebProxy.GetSmartASS(core)),
                (core, val) => MechJebProxy.SetSmartASSForceRoll(MechJebProxy.GetSmartASS(core), val));
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildSmartASSSurfaceMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            smartassSurfaceMenu = menu;

            menu.Add(new TextMenu.Item("SURFACE PROGRADE", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.SURFACE_PROGRADE), (int)MechJebProxy.Target.SURFACE_PROGRADE));
            menu.Add(new TextMenu.Item("SURFACE RETROGRADE", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.SURFACE_RETROGRADE), (int)MechJebProxy.Target.SURFACE_RETROGRADE));
            menu.Add(new TextMenu.Item("HORIZONTAL+", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.HORIZONTAL_PLUS), (int)MechJebProxy.Target.HORIZONTAL_PLUS));
            menu.Add(new TextMenu.Item("HORIZONTAL-", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.HORIZONTAL_MINUS), (int)MechJebProxy.Target.HORIZONTAL_MINUS));
            menu.Add(new TextMenu.Item("VERTICAL+", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.VERTICAL_PLUS), (int)MechJebProxy.Target.VERTICAL_PLUS));
            menu.Add(new TextMenu.Item("SURFACE", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.SURFACE), (int)MechJebProxy.Target.SURFACE));
            AddMenuItem(menu, "------", null);
            AddNumericItem(menu, "Heading",
                core => MechJebProxy.GetSmartASSSurfaceHeading(MechJebProxy.GetSmartASS(core)),
                (core, val) => MechJebProxy.SetSmartASSSurfaceHeading(MechJebProxy.GetSmartASS(core), val),
                1.0, v => v.ToString("F1") + "°", null, true, 0, true, 360);
            AddNumericItem(menu, "Pitch",
                core => MechJebProxy.GetSmartASSSurfacePitch(MechJebProxy.GetSmartASS(core)),
                (core, val) => MechJebProxy.SetSmartASSSurfacePitch(MechJebProxy.GetSmartASS(core), val),
                1.0, v => v.ToString("F1") + "°", null, true, -90, true, 90);
            AddNumericItem(menu, "Roll",
                core => MechJebProxy.GetSmartASSSurfaceRoll(MechJebProxy.GetSmartASS(core)),
                (core, val) => MechJebProxy.SetSmartASSSurfaceRoll(MechJebProxy.GetSmartASS(core), val),
                1.0, v => v.ToString("F1") + "°", null, true, 0, true, 360);
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildSmartASSTargetMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            smartassTargetMenu = menu;

            menu.Add(new TextMenu.Item("TARGET+", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.TARGET_PLUS), (int)MechJebProxy.Target.TARGET_PLUS));
            menu.Add(new TextMenu.Item("TARGET-", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.TARGET_MINUS), (int)MechJebProxy.Target.TARGET_MINUS));
            menu.Add(new TextMenu.Item("RELATIVE VEL+", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.RELATIVE_PLUS), (int)MechJebProxy.Target.RELATIVE_PLUS));
            menu.Add(new TextMenu.Item("RELATIVE VEL-", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.RELATIVE_MINUS), (int)MechJebProxy.Target.RELATIVE_MINUS));
            menu.Add(new TextMenu.Item("PARALLEL+", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.PARALLEL_PLUS), (int)MechJebProxy.Target.PARALLEL_PLUS));
            menu.Add(new TextMenu.Item("PARALLEL-", (idx, item) => SetSmartASSTarget(MechJebProxy.Target.PARALLEL_MINUS), (int)MechJebProxy.Target.PARALLEL_MINUS));
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildSmartASSAdvancedMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddMenuItem(menu, "Set ADVANCED Mode", () => SetSmartASSTarget(MechJebProxy.Target.ADVANCED));
            var refItem = new TextMenu.Item("Reference: [ORBIT]", (idx, item) => CycleSmartASSAdvancedReference(1));
            menu.Add(refItem);
            trackedItems.Add(new TrackedMenuItem
            {
                item = refItem,
                id = "SmartASSAdvReference",
                isEnabled = core => true,
                getLabel = core => "Reference: [" + MechJebProxy.GetSmartASSAdvancedReferenceName(MechJebProxy.GetSmartASS(core)) + "]"
            });

            var dirItem = new TextMenu.Item("Direction: [FORWARD]", (idx, item) => CycleSmartASSAdvancedDirection(1));
            menu.Add(dirItem);
            trackedItems.Add(new TrackedMenuItem
            {
                item = dirItem,
                id = "SmartASSAdvDirection",
                isEnabled = core => true,
                getLabel = core => "Direction: [" + MechJebProxy.GetSmartASSAdvancedDirectionName(MechJebProxy.GetSmartASS(core)) + "]"
            });
            AddToggleItem(menu, "Force Roll",
                core => MechJebProxy.GetSmartASSForceRoll(MechJebProxy.GetSmartASS(core)),
                (core, val) => MechJebProxy.SetSmartASSForceRoll(MechJebProxy.GetSmartASS(core), val));
            AddNumericItem(menu, "Roll Angle",
                core => MechJebProxy.GetSmartASSRoll(MechJebProxy.GetSmartASS(core)),
                (core, val) => MechJebProxy.SetSmartASSRoll(MechJebProxy.GetSmartASS(core), val),
                1.0, v => v.ToString("F1") + "°", null, true, 0, true, 360);
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void SetSmartASSAuto()
        {
            SetSmartASSTarget(MechJebProxy.Target.AUTO);
        }

        private void SetSmartASSTarget(MechJebProxy.Target target)
        {
            if (mjCore == null) return;
            var smartass = MechJebProxy.GetSmartASS(mjCore);
            MechJebProxy.SetSmartASSTarget(smartass, target);
        }

        private void CycleSmartASSAdvancedReference(int direction)
        {
            if (mjCore == null) return;
            var smartass = MechJebProxy.GetSmartASS(mjCore);
            MechJebProxy.CycleSmartASSAdvancedReference(smartass, direction);
        }

        private void CycleSmartASSAdvancedDirection(int direction)
        {
            if (mjCore == null) return;
            var smartass = MechJebProxy.GetSmartASS(mjCore);
            MechJebProxy.CycleSmartASSAdvancedDirection(smartass, direction);
        }
        #endregion

        #region Ascent Menu
        private TextMenu BuildAscentMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddToggleItem(menu, "ENGAGE Ascent Autopilot",
                core => MechJebProxy.IsAscentAutopilotEngaged(core),
                (core, val) => MechJebProxy.SetAscentAutopilotEngaged(core, val, this));

            AddMenuItem(menu, "------", null);

            // Orbit parameters
            AddNumericItem(menu, "Target Altitude",
                core => MechJebProxy.GetAscentAltitude(core) / 1000.0,
                (core, val) => MechJebProxy.SetAscentAltitude(core, val * 1000.0),
                1.0, v => v.ToString("F1") + " km", null, true, 1.0, false, 0);
            AddNumericItem(menu, "Target Inclination",
                core => MechJebProxy.GetAscentInclination(core),
                (core, val) => MechJebProxy.SetAscentInclination(core, val),
                0.5, v => v.ToString("F2") + "°", null, true, 0, true, 180);
            AddMenuItem(menu, "Set to Current Inclination", () =>
            {
                if (mjCore == null || vessel == null) return;
                MechJebProxy.SetAscentInclination(mjCore, vessel.orbit.inclination);
            });

            AddMenuItem(menu, "------", null);

            // Sub-menus
            AddMenuItem(menu, "Path Editor", () => PushMenu(BuildAscentPathMenu()));
            AddMenuItem(menu, "Staging & Thrust", () => PushMenu(BuildAscentStagingMenu()));
            AddMenuItem(menu, "Launch Parameters", () => PushMenu(BuildAscentLaunchMenu()));
            AddMenuItem(menu, "Guidance & Safety", () => PushMenu(BuildAscentGuidanceMenu()));

            AddMenuItem(menu, "------", null);

            AddToggleItem(menu, "Auto-Warp",
                core => MechJebProxy.GetAscentAutowarp(core),
                (core, val) => MechJebProxy.SetAscentAutowarp(core, val));

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildAscentPathMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddToggleItem(menu, "Automatic Altitude Turn",
                core => MechJebProxy.GetAscentAutoPath(core),
                (core, val) => MechJebProxy.SetAscentAutoPath(core, val));

            AddNumericItem(menu, "Turn Start Alt",
                core => MechJebProxy.GetAscentTurnStartAltitude(core) / 1000.0,
                (core, val) => MechJebProxy.SetAscentTurnStartAltitude(core, val * 1000.0),
                1.0, v => v.ToString("F1") + " km",
                core => !MechJebProxy.GetAscentAutoPath(core));

            AddNumericItem(menu, "Turn Start Vel",
                core => MechJebProxy.GetAscentTurnStartVelocity(core),
                (core, val) => MechJebProxy.SetAscentTurnStartVelocity(core, val),
                10.0, v => v.ToString("F0") + " m/s",
                core => !MechJebProxy.GetAscentAutoPath(core));

            AddNumericItem(menu, "Turn End Alt",
                core => MechJebProxy.GetAscentTurnEndAltitude(core) / 1000.0,
                (core, val) => MechJebProxy.SetAscentTurnEndAltitude(core, val * 1000.0),
                1.0, v => v.ToString("F1") + " km",
                core => !MechJebProxy.GetAscentAutoPath(core));

            AddNumericItem(menu, "Final Flight Path Angle",
                core => MechJebProxy.GetAscentTurnEndAngle(core),
                (core, val) => MechJebProxy.SetAscentTurnEndAngle(core, val),
                0.5, v => v.ToString("F1") + "°");

            AddNumericItem(menu, "Turn Shape",
                core => MechJebProxy.GetAscentTurnShapeExponent(core),
                (core, val) => MechJebProxy.SetAscentTurnShapeExponent(core, val),
                0.01, v => (v * 100.0).ToString("F0") + "%");

            AddNumericItem(menu, "Auto Turn %",
                core => MechJebProxy.GetAscentAutoTurnPerc(core) * 100.0,
                (core, val) => MechJebProxy.SetAscentAutoTurnPerc(core, val / 100.0),
                0.5, v => v.ToString("F1") + "%",
                core => MechJebProxy.GetAscentAutoPath(core), true, 0.5, true, 105.0);

            AddNumericItem(menu, "Auto Turn Spd",
                core => MechJebProxy.GetAscentAutoTurnSpdFactor(core),
                (core, val) => MechJebProxy.SetAscentAutoTurnSpdFactor(core, val),
                0.5, v => v.ToString("F1"),
                core => MechJebProxy.GetAscentAutoPath(core), true, 4.0, true, 80.0);

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildAscentStagingMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddToggleItem(menu, "Autostage",
                core => MechJebProxy.GetAscentAutostage(core),
                (core, val) => MechJebProxy.SetAscentAutostage(core, val));
            AddNumericItem(menu, "Stop at Stage",
                core => MechJebProxy.GetAutostageLimit(core),
                (core, val) => MechJebProxy.SetAutostageLimit(core, (int)val),
                1.0, v => v.ToString("F0"), null, true, 0, false, 0);

            AddMenuItem(menu, "------", null);

            AddToggleItem(menu, "Limit to Prevent Overheats",
                core => MechJebProxy.GetLimitToPreventOverheats(core),
                (core, val) => MechJebProxy.SetLimitToPreventOverheats(core, val));
            AddToggleItem(menu, "Limit by Max Q",
                core => MechJebProxy.GetLimitToMaxDynamicPressure(core),
                (core, val) => MechJebProxy.SetLimitToMaxDynamicPressure(core, val));
            AddNumericItem(menu, "Max Q",
                core => MechJebProxy.GetMaxDynamicPressure(core),
                (core, val) => MechJebProxy.SetMaxDynamicPressure(core, val),
                1000.0, v => v.ToString("F0") + " Pa", null, true, 0, false, 0);
            AddToggleItem(menu, "Limit Acceleration",
                core => MechJebProxy.GetLimitAcceleration(core),
                (core, val) => MechJebProxy.SetLimitAcceleration(core, val));
            AddNumericItem(menu, "Max Acceleration",
                core => MechJebProxy.GetMaxAcceleration(core),
                (core, val) => MechJebProxy.SetMaxAcceleration(core, val),
                0.1, v => v.ToString("F1") + " m/s²", null, true, 0, false, 0);
            AddToggleItem(menu, "Limit Throttle",
                core => MechJebProxy.GetLimitThrottle(core),
                (core, val) => MechJebProxy.SetLimitThrottle(core, val));
            AddNumericItem(menu, "Max Throttle",
                core => MechJebProxy.GetMaxThrottle(core),
                (core, val) => MechJebProxy.SetMaxThrottle(core, val),
                1.0, v => v.ToString("F0") + "%", null, true, 0, true, 100);

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildAscentLaunchMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddNumericItem(menu, "Desired LAN",
                core => MechJebProxy.GetAscentLAN(core),
                (core, val) => MechJebProxy.SetAscentLAN(core, val),
                0.5, v => v.ToString("F2") + "°", null, true, 0, true, 360);
            AddNumericItem(menu, "Launch Phase Angle",
                core => MechJebProxy.GetAscentLaunchPhaseAngle(core),
                (core, val) => MechJebProxy.SetAscentLaunchPhaseAngle(core, val),
                0.5, v => v.ToString("F2") + "°", null, true, -360, true, 360);
            AddNumericItem(menu, "Launch LAN Difference",
                core => MechJebProxy.GetAscentLaunchLANDifference(core),
                (core, val) => MechJebProxy.SetAscentLaunchLANDifference(core, val),
                0.5, v => v.ToString("F2") + "°", null, true, -360, true, 360);

            AddMenuItem(menu, "------", null);

            AddNumericItem(menu, "Warp Countdown",
                core => MechJebProxy.GetAscentWarpCountdown(core),
                (core, val) => MechJebProxy.SetAscentWarpCountdown(core, (int)val),
                1.0, v => v.ToString("F0") + " s", null, true, 0, false, 0);
            AddToggleItem(menu, "Skip Circularization",
                core => MechJebProxy.GetAscentSkipCircularization(core),
                (core, val) => MechJebProxy.SetAscentSkipCircularization(core, val));

            AddMenuItem(menu, "[BACK]", () => PopMenu());
            return menu;
        }

        private TextMenu BuildAscentGuidanceMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddToggleItem(menu, "Force Roll",
                core => MechJebProxy.GetAscentForceRoll(core),
                (core, val) => MechJebProxy.SetAscentForceRoll(core, val));
            AddNumericItem(menu, "Vertical Roll",
                core => MechJebProxy.GetAscentVerticalRoll(core),
                (core, val) => MechJebProxy.SetAscentVerticalRoll(core, val),
                1.0, v => v.ToString("F1") + "°", null, true, -180, true, 180);
            AddNumericItem(menu, "Turn Roll",
                core => MechJebProxy.GetAscentTurnRoll(core),
                (core, val) => MechJebProxy.SetAscentTurnRoll(core, val),
                1.0, v => v.ToString("F1") + "°", null, true, -180, true, 180);
            AddNumericItem(menu, "Roll Altitude",
                core => MechJebProxy.GetAscentRollAltitude(core) / 1000.0,
                (core, val) => MechJebProxy.SetAscentRollAltitude(core, val * 1000.0),
                1.0, v => v.ToString("F1") + " km", null, true, 0, false, 0);

            AddMenuItem(menu, "------", null);

            AddToggleItem(menu, "Limit AoA",
                core => MechJebProxy.GetAscentLimitAoA(core),
                (core, val) => MechJebProxy.SetAscentLimitAoA(core, val));
            AddNumericItem(menu, "Max AoA",
                core => MechJebProxy.GetAscentMaxAoA(core),
                (core, val) => MechJebProxy.SetAscentMaxAoA(core, val),
                0.5, v => v.ToString("F1") + "°", null, true, 0, true, 45);
            AddNumericItem(menu, "AoA Fadeout Pressure",
                core => MechJebProxy.GetAscentAoAFadeoutPressure(core),
                (core, val) => MechJebProxy.SetAscentAoAFadeoutPressure(core, val),
                100.0, v => v.ToString("F0") + " Pa", null, true, 0, false, 0);

            AddMenuItem(menu, "------", null);

            AddToggleItem(menu, "Corrective Steering",
                core => MechJebProxy.GetAscentCorrectiveSteering(core),
                (core, val) => MechJebProxy.SetAscentCorrectiveSteering(core, val));
            AddNumericItem(menu, "Corrective Gain",
                core => MechJebProxy.GetAscentCorrectiveSteeringGain(core),
                (core, val) => MechJebProxy.SetAscentCorrectiveSteeringGain(core, val),
                0.1, v => v.ToString("F2"));

            AddMenuItem(menu, "[BACK]", () => PopMenu());
            return menu;
        }
        #endregion

        #region Landing Menu
        private TextMenu BuildLandingMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            // Actions
            AddMenuItem(menu, "Land at Target", () => LandAtTarget(),
                core => MechJebProxy.PositionTargetExists(core));
            AddMenuItem(menu, "Land Somewhere", () => LandSomewhere());
            AddMenuItem(menu, "STOP", () => StopLanding(),
                core => MechJebProxy.IsLandingAutopilotEngaged(core));

            AddMenuItem(menu, "------", null);

            // Targeting
            AddMenuItem(menu, "Pick Target on Map", () => PickTargetOnMap());
            AddNumericItem(menu, "Target Latitude",
                core => MechJebProxy.GetTargetLatitude(core),
                (core, val) => MechJebProxy.SetTargetLatitude(core, vessel != null ? vessel.mainBody : null, val),
                0.1, v => v.ToString("F3") + "°", null, true, -90, true, 90);
            AddNumericItem(menu, "Target Longitude",
                core => MechJebProxy.GetTargetLongitude(core),
                (core, val) => MechJebProxy.SetTargetLongitude(core, vessel != null ? vessel.mainBody : null, val),
                0.1, v => v.ToString("F3") + "°", null, true, -180, true, 180);

            AddMenuItem(menu, "------", null);

            // Settings
            AddNumericItem(menu, "Touchdown Speed",
                core => MechJebProxy.GetLandingTouchdownSpeed(core),
                (core, val) => MechJebProxy.SetLandingTouchdownSpeed(core, val),
                0.5, v => v.ToString("F1") + " m/s", null, true, 0, false, 0);
            AddToggleItem(menu, "Deploy Gear",
                core => MechJebProxy.GetLandingDeployGears(core),
                (core, val) => MechJebProxy.SetLandingDeployGears(core, val));
            AddToggleItem(menu, "Deploy Chutes",
                core => MechJebProxy.GetLandingDeployChutes(core),
                (core, val) => MechJebProxy.SetLandingDeployChutes(core, val));
            AddNumericItem(menu, "Limit Gear Stage",
                core => MechJebProxy.GetLandingLimitGearsStage(core),
                (core, val) => MechJebProxy.SetLandingLimitGearsStage(core, (int)val),
                1.0, v => v.ToString("F0"), null, true, 0, false, 0);
            AddNumericItem(menu, "Limit Chute Stage",
                core => MechJebProxy.GetLandingLimitChutesStage(core),
                (core, val) => MechJebProxy.SetLandingLimitChutesStage(core, (int)val),
                1.0, v => v.ToString("F0"), null, true, 0, false, 0);
            AddToggleItem(menu, "Use RCS",
                core => MechJebProxy.GetLandingUseRCS(core),
                (core, val) => MechJebProxy.SetLandingUseRCS(core, val));

            AddMenuItem(menu, "------", null);

            // Predictions sub-menu
            AddMenuItem(menu, "Predictions Info", () => PushMenu(BuildLandingPredictionsMenu()));

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildLandingPredictionsMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddToggleItem(menu, "Show Trajectory",
                core => MechJebProxy.GetLandingShowTrajectory(core),
                (core, val) => MechJebProxy.SetLandingShowTrajectory(core, val));

            AddMenuItem(menu, "------", null);

            // These are info items, will be updated dynamically
            AddMenuItem(menu, "Predicted Landing:", null);
            var latItem = new TextMenu.Item("  Lat: ---", null);
            var lonItem = new TextMenu.Item("  Lon: ---", null);
            var timeItem = new TextMenu.Item("  Time: ---", null);
            var geesItem = new TextMenu.Item("  Max Gees: ---", null);
            menu.Add(latItem);
            menu.Add(lonItem);
            menu.Add(timeItem);
            menu.Add(geesItem);

            trackedItems.Add(new TrackedMenuItem
            {
                item = latItem,
                id = "LandingPredLat",
                isEnabled = core => true,
                getLabel = core => "  Lat: " + GetLandingPredLatitude(core)
            });
            trackedItems.Add(new TrackedMenuItem
            {
                item = lonItem,
                id = "LandingPredLon",
                isEnabled = core => true,
                getLabel = core => "  Lon: " + GetLandingPredLongitude(core)
            });
            trackedItems.Add(new TrackedMenuItem
            {
                item = timeItem,
                id = "LandingPredTime",
                isEnabled = core => true,
                getLabel = core => "  Time: " + GetLandingPredTime(core)
            });
            trackedItems.Add(new TrackedMenuItem
            {
                item = geesItem,
                id = "LandingPredGees",
                isEnabled = core => true,
                getLabel = core => "  Max Gees: " + GetLandingPredGees(core)
            });

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void LandAtTarget()
        {
            if (mjCore == null) return;
            MechJebProxy.LandAtPositionTarget(mjCore);
        }

        private void LandSomewhere()
        {
            if (mjCore == null) return;
            MechJebProxy.LandUntargeted(mjCore);
        }

        private void StopLanding()
        {
            if (mjCore == null) return;
            MechJebProxy.StopLanding(mjCore);
        }

        private void PickTargetOnMap()
        {
            if (mjCore == null) return;
            MechJebProxy.PickPositionTargetOnMap(mjCore);
        }
        #endregion

        #region Maneuver Planner Menu
        // Menu matching IMGUI Maneuver Planner exactly
        private TextMenu BuildManeuverPlannerMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            // Match exact IMGUI dropdown order (alphabetical)
            AddMenuItem(menu, "advanced transfer to another planet", () => PushMenu(BuildAdvancedTransferMenu()),
                core => FlightGlobals.fetch.VesselTarget is CelestialBody);
            AddMenuItem(menu, "change apoapsis", () => PushMenu(BuildChangeApoapsisMenu()));
            AddMenuItem(menu, "change both Pe and Ap", () => PushMenu(BuildChangeBothPeApMenu()));
            AddMenuItem(menu, "change eccentricity", () => PushMenu(BuildChangeEccentricityMenu()));
            AddMenuItem(menu, "change inclination", () => PushMenu(BuildChangeInclinationMenu()));
            AddMenuItem(menu, "change longitude of ascending node", () => PushMenu(BuildChangeLANMenu()));
            AddMenuItem(menu, "change periapsis", () => PushMenu(BuildChangePeriapsisMenu()));
            AddMenuItem(menu, "change semi-major axis", () => PushMenu(BuildChangeSMAMenu()));
            AddMenuItem(menu, "change surface longitude of apsis", () => PushMenu(BuildChangeSurfaceLongitudeMenu()));
            AddMenuItem(menu, "circularize", () => PushMenu(BuildCircularizeMenu()));
            AddMenuItem(menu, "fine tune closest approach to target", () => PushMenu(BuildFineTuneClosestApproachMenu()),
                core => FlightGlobals.fetch.VesselTarget != null);
            AddMenuItem(menu, "intercept target at chosen time", () => PushMenu(BuildInterceptAtTimeMenu()),
                core => FlightGlobals.fetch.VesselTarget != null);
            AddMenuItem(menu, "match planes with target", () => PushMenu(BuildMatchPlanesMenu()),
                core => FlightGlobals.fetch.VesselTarget != null);
            AddMenuItem(menu, "match velocities with target", () => PushMenu(BuildMatchVelocitiesMenu()),
                core => FlightGlobals.fetch.VesselTarget != null);
            AddMenuItem(menu, "resonant orbit", () => PushMenu(BuildResonantOrbitMenu()));
            AddMenuItem(menu, "return from a moon", () => PushMenu(BuildMoonReturnMenu()),
                core => vessel != null && vessel.mainBody != null && vessel.mainBody.referenceBody != null && 
                        vessel.mainBody.referenceBody != Planetarium.fetch.Sun);
            AddMenuItem(menu, "transfer to another planet", () => PushMenu(BuildInterplanetaryTransferMenu()),
                core => FlightGlobals.fetch.VesselTarget is CelestialBody);
            AddMenuItem(menu, "two impulse (Hohmann) transfer to target", () => PushMenu(BuildHohmannMenu()),
                core => FlightGlobals.fetch.VesselTarget != null);

            AddMenuItem(menu, "------", null);
            AddMenuItem(menu, "Remove ALL nodes", () => RemoveAllNodes());
            AddMenuItem(menu, "------", null);
            
            // Node execution controls (matching IMGUI bottom controls)
            AddToggleItem(menu, "Auto-warp",
                core => MechJebProxy.GetNodeAutowarp(core),
                (core, val) => MechJebProxy.SetNodeAutowarp(core, val));
            AddNumericItem(menu, "Lead time",
                core => MechJebProxy.GetNodeLeadTime(core),
                (core, val) => MechJebProxy.SetNodeLeadTime(core, val),
                1.0, v => v.ToString("F0") + " s", null, true, 0, false, 0);

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildCircularizeMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            // Get MechJeb's circularize operation and its TimeSelector
            object circOp = MechJebProxy.GetOperationByName("circularize");
            object timeSelector = MechJebProxy.GetOperationTimeSelector(circOp);

            AddMenuItem(menu, "Circularize at:", null);
            
            // TimeReference indices for circularize: 0=APOAPSIS, 1=PERIAPSIS, 2=X_FROM_NOW, 3=ALTITUDE
            AddMenuItem(menu, "  At Next Apoapsis", () => ExecuteCircularize(0));
            AddMenuItem(menu, "  At Next Periapsis", () => ExecuteCircularize(1));
            
            // Altitude option with editable value - reads/writes directly to MechJeb's TimeSelector
            AddNumericItem(menu, "  At Altitude",
                core => MechJebProxy.GetTimeSelectorCircularizeAltitude(
                    MechJebProxy.GetOperationTimeSelector(MechJebProxy.GetOperationByName("circularize"))) / 1000.0,
                (core, val) => MechJebProxy.SetTimeSelectorCircularizeAltitude(
                    MechJebProxy.GetOperationTimeSelector(MechJebProxy.GetOperationByName("circularize")), val * 1000.0),
                1.0, v => v.ToString("F1") + " km", null, true, 1.0, false, 0);
            AddMenuItem(menu, "  [Execute at Altitude]", () => ExecuteCircularize(3));

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        /// <summary>
        /// Executes the circularize operation using MechJeb's actual Operation instance.
        /// This ensures perfect sync with IMGUI.
        /// </summary>
        /// <param name="timeRefIndex">TimeSelector index: 0=APOAPSIS, 1=PERIAPSIS, 2=X_FROM_NOW, 3=ALTITUDE</param>
        private void ExecuteCircularize(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            // Get MechJeb's circularize operation
            object circOp = MechJebProxy.GetOperationByName("circularize");
            if (circOp == null) return;
            
            // Get its TimeSelector and set the time reference
            object timeSelector = MechJebProxy.GetOperationTimeSelector(circOp);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            // Execute the operation using MechJeb's MakeNodes
            MechJebProxy.ExecuteOperation(circOp, mjCore, vessel);
        }

        private TextMenu BuildChangeApoapsisMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            // OperationApoapsis: NewApA parameter, TimeRefs: PERIAPSIS(0), APOAPSIS(1), X_FROM_NOW(2), ALTITUDE(3), EQ_DESCENDING(4), EQ_ASCENDING(5)
            AddNumericItem(menu, "New Apoapsis",
                core => MechJebProxy.GetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change apoapsis"), "NewApA") / 1000.0,
                (core, val) => MechJebProxy.SetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change apoapsis"), "NewApA", val * 1000.0),
                1.0, v => v.ToString("F1") + " km", null, true, 1.0, false, 0);
            
            AddMenuItem(menu, "Schedule the burn:", null);
            AddMenuItem(menu, "  at next periapsis", () => ExecuteChangeApoapsis(0));
            AddMenuItem(menu, "  at next apoapsis", () => ExecuteChangeApoapsis(1));
            AddMenuItem(menu, "  at an altitude", () => PushMenu(BuildTimeSelectorAltitudeMenu("change apoapsis")));
            AddMenuItem(menu, "  after a fixed time", () => PushMenu(BuildTimeSelectorLeadTimeMenu("change apoapsis")));
            AddMenuItem(menu, "  at the equatorial DN", () => ExecuteChangeApoapsis(4));
            AddMenuItem(menu, "  at the equatorial AN", () => ExecuteChangeApoapsis(5));

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        /// <summary>
        /// Executes change apoapsis operation
        /// TimeRef indices: PERIAPSIS(0), APOAPSIS(1), X_FROM_NOW(2), ALTITUDE(3), EQ_DESCENDING(4), EQ_ASCENDING(5)
        /// </summary>
        private void ExecuteChangeApoapsis(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("change apoapsis");
            if (op == null) return;
            
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        /// <summary>
        /// Builds a submenu for setting altitude timing and executing an operation
        /// </summary>
        private TextMenu BuildTimeSelectorAltitudeMenu(string operationName)
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddNumericItem(menu, "At Altitude",
                core => MechJebProxy.GetTimeSelectorCircularizeAltitude(
                    MechJebProxy.GetOperationTimeSelector(MechJebProxy.GetOperationByName(operationName))) / 1000.0,
                (core, val) => MechJebProxy.SetTimeSelectorCircularizeAltitude(
                    MechJebProxy.GetOperationTimeSelector(MechJebProxy.GetOperationByName(operationName)), val * 1000.0),
                1.0, v => v.ToString("F1") + " km", null, true, 1.0, false, 0);
            
            // Find ALTITUDE index for this operation's TimeSelector
            AddMenuItem(menu, "[Execute]", () => {
                object op = MechJebProxy.GetOperationByName(operationName);
                object ts = MechJebProxy.GetOperationTimeSelector(op);
                // Set to ALTITUDE time reference (index 3 for most operations that support it)
                MechJebProxy.SetTimeSelectorCurrentTimeRef(ts, 3);
                MechJebProxy.ExecuteOperation(op, mjCore, vessel);
                PopMenu();
            });
            
            AddMenuItem(menu, "[BACK]", () => PopMenu());
            return menu;
        }

        /// <summary>
        /// Builds a submenu for setting lead time and executing an operation
        /// </summary>
        private TextMenu BuildTimeSelectorLeadTimeMenu(string operationName)
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddNumericItem(menu, "Seconds from now",
                core => MechJebProxy.GetTimeSelectorLeadTime(
                    MechJebProxy.GetOperationTimeSelector(MechJebProxy.GetOperationByName(operationName))),
                (core, val) => MechJebProxy.SetTimeSelectorLeadTime(
                    MechJebProxy.GetOperationTimeSelector(MechJebProxy.GetOperationByName(operationName)), val),
                10.0, v => v.ToString("F0") + " s", null, true, 0, false, 0);
            
            // Find X_FROM_NOW index for this operation's TimeSelector
            AddMenuItem(menu, "[Execute]", () => {
                object op = MechJebProxy.GetOperationByName(operationName);
                object ts = MechJebProxy.GetOperationTimeSelector(op);
                // Set to X_FROM_NOW time reference (index 2 for most operations that support it)
                MechJebProxy.SetTimeSelectorCurrentTimeRef(ts, 2);
                MechJebProxy.ExecuteOperation(op, mjCore, vessel);
                PopMenu();
            });
            
            AddMenuItem(menu, "[BACK]", () => PopMenu());
            return menu;
        }

        private TextMenu BuildChangePeriapsisMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            // OperationPeriapsis: NewPeA parameter, TimeRefs: PERIAPSIS(0), APOAPSIS(1), X_FROM_NOW(2), ALTITUDE(3)
            AddNumericItem(menu, "New Periapsis",
                core => MechJebProxy.GetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change periapsis"), "NewPeA") / 1000.0,
                (core, val) => MechJebProxy.SetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change periapsis"), "NewPeA", val * 1000.0),
                1.0, v => v.ToString("F1") + " km", null, true, 1.0, false, 0);
            
            AddMenuItem(menu, "Schedule the burn:", null);
            AddMenuItem(menu, "  at next periapsis", () => ExecuteChangePeriapsis(0));
            AddMenuItem(menu, "  at next apoapsis", () => ExecuteChangePeriapsis(1));
            AddMenuItem(menu, "  at an altitude", () => PushMenu(BuildTimeSelectorAltitudeMenu("change periapsis")));
            AddMenuItem(menu, "  after a fixed time", () => PushMenu(BuildTimeSelectorLeadTimeMenu("change periapsis")));

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteChangePeriapsis(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("change periapsis");
            if (op == null) return;
            
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildChangeSMAMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationSemiMajor: NewSma parameter, TimeRefs: PERIAPSIS(0), APOAPSIS(1), X_FROM_NOW(2), ALTITUDE(3)
            AddNumericItem(menu, "New Semi-Major Axis",
                core => MechJebProxy.GetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change semi-major axis"), "NewSma") / 1000.0,
                (core, val) => MechJebProxy.SetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change semi-major axis"), "NewSma", val * 1000.0),
                1.0, v => v.ToString("F1") + " km", null, true, 1.0, false, 0);
            
            AddMenuItem(menu, "Schedule the burn:", null);
            AddMenuItem(menu, "  at next periapsis", () => ExecuteChangeSMA(0));
            AddMenuItem(menu, "  at next apoapsis", () => ExecuteChangeSMA(1));
            AddMenuItem(menu, "  at an altitude", () => PushMenu(BuildTimeSelectorAltitudeMenu("change semi-major axis")));
            AddMenuItem(menu, "  after a fixed time", () => PushMenu(BuildTimeSelectorLeadTimeMenu("change semi-major axis")));
            
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteChangeSMA(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("change semi-major axis");
            if (op == null) return;
            
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildChangeInclinationMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationInclination: NewInc parameter, TimeRefs: EQ_NEAREST_AD(0), EQ_HIGHEST_AD(1), X_FROM_NOW(2), APOAPSIS(3), PERIAPSIS(4)
            AddNumericItem(menu, "New Inclination",
                core => MechJebProxy.GetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change inclination"), "NewInc"),
                (core, val) => MechJebProxy.SetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change inclination"), "NewInc", val),
                0.5, v => v.ToString("F1") + "°", null, true, -180, true, 180);
            
            AddMenuItem(menu, "Schedule the burn:", null);
            AddMenuItem(menu, "  at the nearest AN/DN", () => ExecuteChangeInclination(0));
            AddMenuItem(menu, "  at the highest AN/DN", () => ExecuteChangeInclination(1));
            AddMenuItem(menu, "  after a fixed time", () => PushMenu(BuildTimeSelectorLeadTimeMenu("change inclination")));
            AddMenuItem(menu, "  at next apoapsis", () => ExecuteChangeInclination(3));
            AddMenuItem(menu, "  at next periapsis", () => ExecuteChangeInclination(4));
            
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteChangeInclination(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("change inclination");
            if (op == null) return;
            
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildChangeLANMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationLan uses EditableAngle for targetLongitude - we need to handle this differently
            // TimeRefs: X_FROM_NOW(0), APOAPSIS(1), PERIAPSIS(2)
            AddNumericItem(menu, "New LAN",
                core => MechJebProxy.GetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change longitude of ascending node"), "targetLongitude"),
                (core, val) => MechJebProxy.SetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change longitude of ascending node"), "targetLongitude", val),
                0.5, v => v.ToString("F1") + "°", null, true, 0, true, 360);
            
            AddMenuItem(menu, "Schedule the burn:", null);
            AddMenuItem(menu, "  after a fixed time", () => PushMenu(BuildTimeSelectorLeadTimeMenu("change longitude of ascending node")));
            AddMenuItem(menu, "  at next apoapsis", () => ExecuteChangeLAN(1));
            AddMenuItem(menu, "  at next periapsis", () => ExecuteChangeLAN(2));
            
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteChangeLAN(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("change longitude of ascending node");
            if (op == null) return;
            
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildHohmannMenu()
        {
            // Wrapper for IMGUI "two impulse (Hohmann) transfer to target" - wraps MechJeb's OperationGeneric
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            const string opName = "two impulse (Hohmann) transfer to target";

            // "no insertion burn (impact/flyby)" checkbox - inverted Capture bool
            AddToggleItem(menu, "no insertion burn (impact/flyby)",
                core => !MechJebProxy.GetOperationBool(MechJebProxy.GetOperationByName(opName), "Capture"),
                (core, val) => MechJebProxy.SetOperationBool(MechJebProxy.GetOperationByName(opName), "Capture", !val));

            // "Plan insertion burn" checkbox
            AddToggleItem(menu, "Plan insertion burn",
                core => MechJebProxy.GetOperationBool(MechJebProxy.GetOperationByName(opName), "PlanCapture"),
                (core, val) => MechJebProxy.SetOperationBool(MechJebProxy.GetOperationByName(opName), "PlanCapture", val));

            // "coplanar maneuver" checkbox
            AddToggleItem(menu, "coplanar maneuver",
                core => MechJebProxy.GetOperationBool(MechJebProxy.GetOperationByName(opName), "Coplanar"),
                (core, val) => MechJebProxy.SetOperationBool(MechJebProxy.GetOperationByName(opName), "Coplanar", val));

            // Rendezvous vs Transfer radio buttons - use isSelected for green highlighting
            var rendezvousItem = new TextMenu.Item("Rendezvous", (idx, item) => MechJebProxy.SetOperationBool(MechJebProxy.GetOperationByName(opName), "Rendezvous", true));
            menu.Add(rendezvousItem);
            trackedItems.Add(new TrackedMenuItem { item = rendezvousItem, id = "HohmannRendezvous", isSelected = core => MechJebProxy.GetOperationBool(MechJebProxy.GetOperationByName(opName), "Rendezvous") });
            
            var transferItem = new TextMenu.Item("Transfer", (idx, item) => MechJebProxy.SetOperationBool(MechJebProxy.GetOperationByName(opName), "Rendezvous", false));
            menu.Add(transferItem);
            trackedItems.Add(new TrackedMenuItem { item = transferItem, id = "HohmannTransfer", isSelected = core => !MechJebProxy.GetOperationBool(MechJebProxy.GetOperationByName(opName), "Rendezvous") });

            // Rendezvous time offset (LagTime in seconds)
            AddNumericItem(menu, "rendezvous time offset",
                core => MechJebProxy.GetOperationEditableDouble(MechJebProxy.GetOperationByName(opName), "LagTime"),
                (core, val) => MechJebProxy.SetOperationEditableDouble(MechJebProxy.GetOperationByName(opName), "LagTime", val),
                1.0, v => v.ToString("F0") + " sec", null, false, 0, false, 0);

            // Schedule the burn - TimeSelector options
            // OperationGeneric TimeRefs: COMPUTED(0), PERIAPSIS(1), APOAPSIS(2), X_FROM_NOW(3), ALTITUDE(4),
            //   EQ_DESCENDING(5), EQ_ASCENDING(6), REL_NEAREST_AD(7), REL_ASCENDING(8), REL_DESCENDING(9), CLOSEST_APPROACH(10)
            AddMenuItem(menu, "Schedule the burn:", null);
            AddMenuItem(menu, "  at optimum time", () => ExecuteHohmann(0));  // COMPUTED
            AddMenuItem(menu, "  at next periapsis", () => ExecuteHohmann(1));  // PERIAPSIS
            AddMenuItem(menu, "  at next apoapsis", () => ExecuteHohmann(2));  // APOAPSIS
            AddMenuItem(menu, "  at rel. AN with target", () => ExecuteHohmann(8));  // REL_ASCENDING
            AddMenuItem(menu, "  at rel. DN with target", () => ExecuteHohmann(9));  // REL_DESCENDING
            AddMenuItem(menu, "  at nearest rel. AN/DN", () => ExecuteHohmann(7));  // REL_NEAREST_AD
            AddMenuItem(menu, "  at closest approach", () => ExecuteHohmann(10)); // CLOSEST_APPROACH
            AddMenuItem(menu, "  after a fixed time", () => PushMenu(BuildTimeSelectorLeadTimeMenu(opName)));

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteHohmann(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("two impulse (Hohmann) transfer to target");
            if (op == null) return;
            
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        // LEGACY: SyncGenericTransferOperation - no longer needed with wrapper pattern
        private void SyncGenericTransferOperation()
        {
            // Legacy - kept for compatibility but no longer used
        }

        // Wrapper implementations for remaining operations
        private TextMenu BuildChangeBothPeApMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationEllipticize: NewPeA, NewApA parameters, TimeRefs: PERIAPSIS(0), APOAPSIS(1), X_FROM_NOW(2)
            AddNumericItem(menu, "New periapsis",
                core => MechJebProxy.GetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change both Pe and Ap"), "NewPeA") / 1000.0,
                (core, val) => MechJebProxy.SetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change both Pe and Ap"), "NewPeA", val * 1000.0),
                1.0, v => v.ToString("F1") + " km", null, true, 1.0, false, 0);
            AddNumericItem(menu, "New apoapsis",
                core => MechJebProxy.GetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change both Pe and Ap"), "NewApA") / 1000.0,
                (core, val) => MechJebProxy.SetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change both Pe and Ap"), "NewApA", val * 1000.0),
                1.0, v => v.ToString("F1") + " km", null, true, 1.0, false, 0);
            
            AddMenuItem(menu, "Schedule the burn:", null);
            AddMenuItem(menu, "  at next periapsis", () => ExecuteChangeBothPeAp(0));
            AddMenuItem(menu, "  at next apoapsis", () => ExecuteChangeBothPeAp(1));
            AddMenuItem(menu, "  after a fixed time", () => PushMenu(BuildTimeSelectorLeadTimeMenu("change both Pe and Ap")));
            
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteChangeBothPeAp(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("change both Pe and Ap");
            if (op == null) return;
            
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildChangeEccentricityMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationEccentricity: NewEcc parameter, TimeRefs: PERIAPSIS(0), APOAPSIS(1), X_FROM_NOW(2), ALTITUDE(3)
            AddNumericItem(menu, "New eccentricity",
                core => MechJebProxy.GetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change eccentricity"), "NewEcc"),
                (core, val) => MechJebProxy.SetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change eccentricity"), "NewEcc", val),
                0.01, v => v.ToString("F3"), null, true, 0, true, 0.99);
            
            AddMenuItem(menu, "Schedule the burn:", null);
            AddMenuItem(menu, "  at next periapsis", () => ExecuteChangeEccentricity(0));
            AddMenuItem(menu, "  at next apoapsis", () => ExecuteChangeEccentricity(1));
            AddMenuItem(menu, "  at an altitude", () => PushMenu(BuildTimeSelectorAltitudeMenu("change eccentricity")));
            AddMenuItem(menu, "  after a fixed time", () => PushMenu(BuildTimeSelectorLeadTimeMenu("change eccentricity")));
            
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteChangeEccentricity(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("change eccentricity");
            if (op == null) return;
            
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildChangeSurfaceLongitudeMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationLongitude: targetLongitude (EditableAngle), TimeRefs: PERIAPSIS(0), APOAPSIS(1)
            AddNumericItem(menu, "Target longitude",
                core => MechJebProxy.GetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change surface longitude of apsis"), "targetLongitude"),
                (core, val) => MechJebProxy.SetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("change surface longitude of apsis"), "targetLongitude", val),
                1.0, v => v.ToString("F1") + "°", null, true, -180, true, 180);
            
            AddMenuItem(menu, "Schedule the burn:", null);
            AddMenuItem(menu, "  at next periapsis", () => ExecuteChangeSurfaceLongitude(0));
            AddMenuItem(menu, "  at next apoapsis", () => ExecuteChangeSurfaceLongitude(1));
            
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteChangeSurfaceLongitude(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("change surface longitude of apsis");
            if (op == null) return;
            
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildFineTuneClosestApproachMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationCourseCorrection: CourseCorrectFinalPeA or InterceptDistance parameters (no time selector)
            // Check if target is celestial body or vessel
            ITargetable target = FlightGlobals.fetch.VesselTarget;
            bool isCelestialTarget = target is CelestialBody;
            
            if (isCelestialTarget)
            {
                AddNumericItem(menu, "Target periapsis",
                    core => MechJebProxy.GetOperationEditableDouble(
                        MechJebProxy.GetOperationByName("fine tune closest approach to target"), "CourseCorrectFinalPeA") / 1000.0,
                    (core, val) => MechJebProxy.SetOperationEditableDouble(
                        MechJebProxy.GetOperationByName("fine tune closest approach to target"), "CourseCorrectFinalPeA", val * 1000.0),
                    1.0, v => v.ToString("F1") + " km", null, true, 0, false, 0);
            }
            else
            {
                AddNumericItem(menu, "Distance at closest approach",
                    core => MechJebProxy.GetOperationEditableDouble(
                        MechJebProxy.GetOperationByName("fine tune closest approach to target"), "InterceptDistance"),
                    (core, val) => MechJebProxy.SetOperationEditableDouble(
                        MechJebProxy.GetOperationByName("fine tune closest approach to target"), "InterceptDistance", val),
                    10.0, v => v.ToString("F0") + " m", null, true, 0, false, 0);
            }
            
            AddMenuItem(menu, "[Create Node]", () => ExecuteCourseCorrection());
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteCourseCorrection()
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("fine tune closest approach to target");
            if (op == null) return;
            
            // CourseCorrection has no time selector - it computes automatically
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildInterceptAtTimeMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationLambert: InterceptInterval parameter, TimeRef: X_FROM_NOW only
            AddNumericItem(menu, "Intercept after",
                core => MechJebProxy.GetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("intercept target at chosen time"), "InterceptInterval"),
                (core, val) => MechJebProxy.SetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("intercept target at chosen time"), "InterceptInterval", val),
                60.0, v => FormatTime(v), null, true, 60, false, 0);
            
            AddMenuItem(menu, "[Create Node]", () => ExecuteInterceptAtTime());
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteInterceptAtTime()
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("intercept target at chosen time");
            if (op == null) return;
            
            // Lambert has X_FROM_NOW as its only time reference (index 0)
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, 0);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildMatchPlanesMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationPlane: no parameters, TimeRefs: REL_ASCENDING(0), REL_DESCENDING(1), REL_NEAREST_AD(2), REL_HIGHEST_AD(3)
            AddMenuItem(menu, "[Match at nearest AN/DN]", () => ExecuteMatchPlanes(2));
            AddMenuItem(menu, "[Match at highest AN/DN]", () => ExecuteMatchPlanes(3));
            AddMenuItem(menu, "[Match at Ascending Node]", () => ExecuteMatchPlanes(0));
            AddMenuItem(menu, "[Match at Descending Node]", () => ExecuteMatchPlanes(1));
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteMatchPlanes(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("match planes with target");
            if (op == null) return;
            
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildResonantOrbitMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationResonantOrbit: ResonanceNumerator, ResonanceDenominator, TimeRefs: PERIAPSIS(0), APOAPSIS(1), X_FROM_NOW(2), ALTITUDE(3)
            AddNumericItem(menu, "Resonance numerator",
                core => MechJebProxy.GetOperationEditableInt(
                    MechJebProxy.GetOperationByName("resonant orbit"), "ResonanceNumerator"),
                (core, val) => MechJebProxy.SetOperationEditableInt(
                    MechJebProxy.GetOperationByName("resonant orbit"), "ResonanceNumerator", (int)val),
                1.0, v => ((int)v).ToString(), null, true, 1, false, 0);
            AddNumericItem(menu, "Resonance denominator",
                core => MechJebProxy.GetOperationEditableInt(
                    MechJebProxy.GetOperationByName("resonant orbit"), "ResonanceDenominator"),
                (core, val) => MechJebProxy.SetOperationEditableInt(
                    MechJebProxy.GetOperationByName("resonant orbit"), "ResonanceDenominator", (int)val),
                1.0, v => ((int)v).ToString(), null, true, 1, false, 0);
            
            AddMenuItem(menu, "Schedule the burn:", null);
            AddMenuItem(menu, "  at next periapsis", () => ExecuteResonantOrbit(0));
            AddMenuItem(menu, "  at next apoapsis", () => ExecuteResonantOrbit(1));
            AddMenuItem(menu, "  at an altitude", () => PushMenu(BuildTimeSelectorAltitudeMenu("resonant orbit")));
            AddMenuItem(menu, "  after a fixed time", () => PushMenu(BuildTimeSelectorLeadTimeMenu("resonant orbit")));
            
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteResonantOrbit(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("resonant orbit");
            if (op == null) return;
            
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildMoonReturnMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationMoonReturn: MoonReturnAltitude parameter (no time selector - auto computed)
            AddNumericItem(menu, "Return altitude",
                core => MechJebProxy.GetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("return from a moon"), "MoonReturnAltitude") / 1000.0,
                (core, val) => MechJebProxy.SetOperationEditableDouble(
                    MechJebProxy.GetOperationByName("return from a moon"), "MoonReturnAltitude", val * 1000.0),
                10.0, v => v.ToString("F0") + " km", null, true, 10, false, 0);
            AddMenuItem(menu, "[Create Node]", () => ExecuteMoonReturn());
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteMoonReturn()
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("return from a moon");
            if (op == null) return;
            
            // MoonReturn has no time selector - it computes the optimal ejection time automatically
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildInterplanetaryTransferMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationInterplanetaryTransfer: WaitForPhaseAngle (bool) parameter
            AddToggleItem(menu, "Wait for optimal phase angle",
                core => MechJebProxy.GetOperationBool(
                    MechJebProxy.GetOperationByName("transfer to another planet"), "WaitForPhaseAngle"),
                (core, val) => MechJebProxy.SetOperationBool(
                    MechJebProxy.GetOperationByName("transfer to another planet"), "WaitForPhaseAngle", val));
            
            AddMenuItem(menu, "[Create Node]", () => ExecuteInterplanetaryTransfer());
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteInterplanetaryTransfer()
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("transfer to another planet");
            if (op == null) return;
            
            // InterplanetaryTransfer has no time selector - computes optimal time automatically
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildAdvancedTransferMenu()
        {
            // Wrapper for IMGUI "advanced transfer to another planet" - wraps MechJeb's OperationAdvancedTransfer
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            const string opName = "advanced transfer to another planet";

            // Mode selection header
            AddMenuItem(menu, "--- Porkchop selection ---", null);

            // Status display - shows computation progress/ready status
            var statusItem = new TextMenu.Item("Status: ---", null);
            menu.Add(statusItem);
            trackedItems.Add(new TrackedMenuItem
            {
                item = statusItem,
                id = "AdvancedTransferStatus",
                isEnabled = core => true,
                getLabel = core => GetAdvancedTransferStatusText()
            });

            // ΔV display
            var dvItem = new TextMenu.Item("ΔV: ---", null);
            menu.Add(dvItem);
            trackedItems.Add(new TrackedMenuItem
            {
                item = dvItem,
                id = "AdvancedTransferDV",
                isEnabled = core => true,
                getLabel = core => {
                    object op = MechJebProxy.GetOperationByName(opName);
                    if (op == null) return "ΔV: ---";
                    double dv, dep, dur;
                    if (MechJebProxy.GetAdvancedTransferSelection(op, out dep, out dur, out dv) && dv > 0)
                        return "ΔV: " + dv.ToString("F1") + " m/s";
                    return "ΔV: ---";
                }
            });

            // Include capture burn checkbox - wraps operation field
            AddToggleItem(menu, "Include capture burn",
                core => MechJebProxy.GetAdvancedTransferIncludeCapture(MechJebProxy.GetOperationByName(opName)),
                (core, val) => MechJebProxy.SetAdvancedTransferIncludeCapture(MechJebProxy.GetOperationByName(opName), val));

            // Periapsis input - wraps periapsisHeight field (in km)
            AddNumericItem(menu, "Periapsis",
                core => MechJebProxy.GetAdvancedTransferPeriapsisKm(MechJebProxy.GetOperationByName(opName)),
                (core, val) => MechJebProxy.SetAdvancedTransferPeriapsisKm(MechJebProxy.GetOperationByName(opName), val),
                10.0, v => v.ToString("F0") + " km", null, true, 10.0, false, 0);

            // Selection mode - Lowest ΔV vs ASAP - use isSelected for green highlighting
            var lowestDVItem = new TextMenu.Item("Lowest ΔV", (idx, item) => { advancedTransferSelectLowestDV = true; SelectAdvancedTransferLowestDV(); });
            menu.Add(lowestDVItem);
            trackedItems.Add(new TrackedMenuItem { item = lowestDVItem, id = "AdvTransferLowestDV", isSelected = core => advancedTransferSelectLowestDV });
            
            var asapItem = new TextMenu.Item("ASAP", (idx, item) => { advancedTransferSelectLowestDV = false; SelectAdvancedTransferASAP(); });
            menu.Add(asapItem);
            trackedItems.Add(new TrackedMenuItem { item = asapItem, id = "AdvTransferASAP", isSelected = core => !advancedTransferSelectLowestDV });

            // Departure info
            var departureItem = new TextMenu.Item("Departure: ---", null);
            menu.Add(departureItem);
            trackedItems.Add(new TrackedMenuItem
            {
                item = departureItem,
                id = "AdvancedTransferDeparture",
                isEnabled = core => true,
                getLabel = core => "Departure in " + GetAdvancedTransferDepartureText()
            });

            // Transit duration
            var transitItem = new TextMenu.Item("Transit: ---", null);
            menu.Add(transitItem);
            trackedItems.Add(new TrackedMenuItem
            {
                item = transitItem,
                id = "AdvancedTransferTransit",
                isEnabled = core => true,
                getLabel = core => "Transit duration " + GetAdvancedTransferTransitText()
            });

            AddMenuItem(menu, "------", null);
            AddMenuItem(menu, "[Start/Refresh Compute]", () => StartAdvancedTransferCompute());
            AddMenuItem(menu, "[Create node]", () => CreateAdvancedTransferNode());
            AddMenuItem(menu, "[Create and execute]", () => CreateAndExecuteAdvancedTransfer());
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private string GetAdvancedTransferStatusText()
        {
            object op = MechJebProxy.GetOperationByName("advanced transfer to another planet");
            if (op == null) return "Not available";
            int progress;
            bool finished = MechJebProxy.IsAdvancedTransferFinished(op, out progress);
            if (finished)
            {
                // Update cached selection info for display
                MechJebProxy.GetAdvancedTransferSelection(op, 
                    out advancedTransferDepartureUT, out advancedTransferDuration, out advancedTransferDeltaV);
                return "Ready";
            }
            return "Computing: " + progress + "%";
        }

        private string GetAdvancedTransferDepartureText()
        {
            if (advancedTransferDepartureUT <= 0) return "---";
            double dt = advancedTransferDepartureUT - Planetarium.GetUniversalTime();
            if (dt < 0) return "any time now";
            return FormatTime(dt);
        }

        private string GetAdvancedTransferTransitText()
        {
            if (advancedTransferDuration <= 0) return "---";
            return FormatTime(advancedTransferDuration);
        }

        private void CreateAndExecuteAdvancedTransfer()
        {
            CreateAdvancedTransferNode();
            if (mjCore != null)
            {
                MechJebProxy.ExecuteOneNode(mjCore, null);
            }
        }

        private TextMenu BuildCourseCorrectionMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            AddNumericItem(menu, "Target PE",
                core => courseCorrectionPeKm,
                (core, val) => courseCorrectionPeKm = val,
                1.0, v => v.ToString("F1") + " km", null, true, 1.0, false, 0);
            AddMenuItem(menu, "[Create Correction]", () => CreateCourseCorrection());
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildMatchVelocitiesMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // OperationKillRelVel: no parameters, TimeRefs: CLOSEST_APPROACH(0), X_FROM_NOW(1)
            AddMenuItem(menu, "[Match at Closest Approach]", () => ExecuteMatchVelocities(0));
            AddMenuItem(menu, "[Match after fixed time]", () => PushMenu(BuildTimeSelectorLeadTimeMenu("match velocities with target")));
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void ExecuteMatchVelocities(int timeRefIndex)
        {
            if (vessel == null || mjCore == null) return;
            
            object op = MechJebProxy.GetOperationByName("match velocities with target");
            if (op == null) return;
            
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, timeRefIndex);
            }
            
            MechJebProxy.ExecuteOperation(op, mjCore, vessel);
        }

        private TextMenu BuildMatchPlanesANMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            AddMenuItem(menu, "[Match at Ascending Node]", () => ExecuteMatchPlanes(0));
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildMatchPlanesDNMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            AddMenuItem(menu, "[Match at Descending Node]", () => ExecuteMatchPlanes(1));
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        // ============================================================================
        // LEGACY IMPLEMENTATIONS BELOW - kept for reference but no longer used
        // These have been replaced by the wrapper implementations above
        // ============================================================================
        
        // Maneuver planner implementations (LEGACY - not using wrapper)
        private void CircularizeAtApoapsis()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.NextApoapsisTime(Planetarium.GetUniversalTime());
            Vector3d dV = MechJebProxy.CalcDeltaVToCircularize(vessel.orbit, ut);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void CircularizeAtPeriapsis()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.NextPeriapsisTime(Planetarium.GetUniversalTime());
            Vector3d dV = MechJebProxy.CalcDeltaVToCircularize(vessel.orbit, ut);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void CircularizeAtAltitude()
        {
            if (vessel == null) return;
            double ut = Planetarium.GetUniversalTime();
            try
            {
                double radius = vessel.mainBody.Radius + (circularizeAltitudeKm * 1000.0);
                ut = vessel.orbit.NextTimeOfRadius(ut, radius);
            }
            catch { }

            Vector3d dV = MechJebProxy.CalcDeltaVToCircularize(vessel.orbit, ut);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeApoapsisAtPeriapsis()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.NextPeriapsisTime(Planetarium.GetUniversalTime());
            double newApR = vessel.mainBody.Radius + (changeApoapsisKm * 1000.0);
            Vector3d dV = MechJebProxy.CalcDeltaVToChangeApoapsis(vessel.orbit, ut, newApR);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeApoapsisNow()
        {
            if (vessel == null) return;
            double ut = Planetarium.GetUniversalTime();
            double newApR = vessel.mainBody.Radius + (changeApoapsisKm * 1000.0);
            Vector3d dV = MechJebProxy.CalcDeltaVToChangeApoapsis(vessel.orbit, ut, newApR);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangePeriapsisAtApoapsis()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.NextApoapsisTime(Planetarium.GetUniversalTime());
            double newPeR = vessel.mainBody.Radius + (changePeriapsisKm * 1000.0);
            Vector3d dV = MechJebProxy.CalcDeltaVToChangePeriapsis(vessel.orbit, ut, newPeR);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangePeriapsisNow()
        {
            if (vessel == null) return;
            double ut = Planetarium.GetUniversalTime();
            double newPeR = vessel.mainBody.Radius + (changePeriapsisKm * 1000.0);
            Vector3d dV = MechJebProxy.CalcDeltaVToChangePeriapsis(vessel.orbit, ut, newPeR);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeSmaNow()
        {
            if (vessel == null) return;
            double ut = Planetarium.GetUniversalTime();
            double newSma = vessel.mainBody.Radius + (changeSmaKm * 1000.0);
            Vector3d dV = MechJebProxy.CalcDeltaVForSemiMajorAxis(vessel.orbit, ut, newSma);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeInclinationAtAN()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.TimeOfAscendingNodeEquatorial(Planetarium.GetUniversalTime());
            Vector3d dV = MechJebProxy.CalcDeltaVToChangeInclination(vessel.orbit, ut, changeInclinationDeg);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeInclinationAtDN()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.TimeOfDescendingNodeEquatorial(Planetarium.GetUniversalTime());
            Vector3d dV = MechJebProxy.CalcDeltaVToChangeInclination(vessel.orbit, ut, changeInclinationDeg);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeLANAtApoapsis()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.NextApoapsisTime(Planetarium.GetUniversalTime());
            Vector3d dV = MechJebProxy.CalcDeltaVToShiftLAN(vessel.orbit, ut, changeLanDeg);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeLANAtPeriapsis()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.NextPeriapsisTime(Planetarium.GetUniversalTime());
            Vector3d dV = MechJebProxy.CalcDeltaVToShiftLAN(vessel.orbit, ut, changeLanDeg);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeLANNow()
        {
            if (vessel == null) return;
            double ut = Planetarium.GetUniversalTime();
            Vector3d dV = MechJebProxy.CalcDeltaVToShiftLAN(vessel.orbit, ut, changeLanDeg);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void CreateHohmannTransfer()
        {
            if (vessel == null) return;
            Orbit target = FlightGlobals.fetch.VesselTarget != null ? FlightGlobals.fetch.VesselTarget.GetOrbit() : null;
            if (target == null) return;

            Vector3d dv1, dv2;
            double ut1, ut2;
            if (MechJebProxy.TryCalcHohmannTransfer(vessel.orbit, target, Planetarium.GetUniversalTime(), out dv1, out ut1, out dv2, out ut2))
            {
                MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dv1, ut1);
                if (dv2.sqrMagnitude > 0.0)
                {
                    MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dv2, ut2);
                }
            }
        }

        private void MatchVelocitiesAtClosestApproach()
        {
            if (vessel == null || FlightGlobals.fetch.VesselTarget == null) return;
            Orbit target = FlightGlobals.fetch.VesselTarget.GetOrbit();
            if (target == null) return;

            double ut = vessel.orbit.NextClosestApproachTime(target, Planetarium.GetUniversalTime());
            Vector3d dV = MechJebProxy.CalcDeltaVToMatchVelocities(vessel.orbit, ut, target);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void MatchPlanesAtAscendingNode()
        {
            if (vessel == null || FlightGlobals.fetch.VesselTarget == null) return;
            Orbit target = FlightGlobals.fetch.VesselTarget.GetOrbit();
            if (target == null) return;

            Vector3d dV;
            double ut;
            if (MechJebProxy.TryCalcMatchPlanesAscending(vessel.orbit, target, Planetarium.GetUniversalTime(), out dV, out ut))
            {
                MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
            }
        }

        private void MatchPlanesAtDescendingNode()
        {
            if (vessel == null || FlightGlobals.fetch.VesselTarget == null) return;
            Orbit target = FlightGlobals.fetch.VesselTarget.GetOrbit();
            if (target == null) return;

            Vector3d dV;
            double ut;
            if (MechJebProxy.TryCalcMatchPlanesDescending(vessel.orbit, target, Planetarium.GetUniversalTime(), out dV, out ut))
            {
                MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
            }
        }

        private void StartAdvancedTransferCompute()
        {
            if (mjCore == null || vessel == null) return;
            object targetController = MechJebProxy.GetTargetController(mjCore);
            if (targetController == null || FlightGlobals.fetch.VesselTarget == null) return;
            if (!(FlightGlobals.fetch.VesselTarget is CelestialBody)) return;

            object op = MechJebProxy.GetOperationByName("advanced transfer to another planet");
            if (op == null) return;

            // Get current settings from the operation itself for consistency
            bool includeCapture = MechJebProxy.GetAdvancedTransferIncludeCapture(op);
            double periapsisKm = MechJebProxy.GetAdvancedTransferPeriapsisKm(op);

            MechJebProxy.StartAdvancedTransferCompute(
                op,
                vessel.orbit,
                Planetarium.GetUniversalTime(),
                targetController,
                includeCapture,
                periapsisKm);
        }

        private void SelectAdvancedTransferLowestDV()
        {
            object op = MechJebProxy.GetOperationByName("advanced transfer to another planet");
            if (op == null) return;
            MechJebProxy.SelectAdvancedTransferLowestDV(op);
        }

        private void SelectAdvancedTransferASAP()
        {
            object op = MechJebProxy.GetOperationByName("advanced transfer to another planet");
            if (op == null) return;
            MechJebProxy.SelectAdvancedTransferASAP(op);
        }

        private void CreateAdvancedTransferNode()
        {
            if (vessel == null || mjCore == null) return;
            object targetController = MechJebProxy.GetTargetController(mjCore);
            if (targetController == null) return;

            object op = MechJebProxy.GetOperationByName("advanced transfer to another planet");
            if (op == null) return;

            // Check if computation is finished
            int progress;
            if (!MechJebProxy.IsAdvancedTransferFinished(op, out progress))
            {
                // Not ready yet - need to compute first
                return;
            }

            MechJebProxy.CreateNodesFromOperation(op, vessel.orbit, Planetarium.GetUniversalTime(), targetController, vessel);
        }

        // Wrapper for Hohmann transfer using MechJeb's OperationGeneric
        private void CreateHohmannTransferNode()
        {
            if (vessel == null || mjCore == null) return;
            object targetController = MechJebProxy.GetTargetController(mjCore);
            if (targetController == null || FlightGlobals.fetch.VesselTarget == null) return;

            object op = MechJebProxy.GetOperationByName("two impulse (Hohmann) transfer to target");
            if (op == null) return;

            // Use COMPUTED time reference for optimal timing
            object timeSelector = MechJebProxy.GetOperationTimeSelector(op);
            if (timeSelector != null)
            {
                MechJebProxy.SetTimeSelectorCurrentTimeRef(timeSelector, 0); // COMPUTED
            }

            MechJebProxy.CreateNodesFromOperation(op, vessel.orbit, Planetarium.GetUniversalTime(), targetController, vessel);
        }

        private void CreateAndExecuteHohmannTransfer()
        {
            CreateHohmannTransferNode();
            if (mjCore != null)
            {
                MechJebProxy.ExecuteOneNode(mjCore, null);
            }
        }

        // New operations implementations
        private void ChangeBothAtPeriapsis()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.NextPeriapsisTime(Planetarium.GetUniversalTime());
            double newPeR = vessel.mainBody.Radius + (changePeriapsisKm * 1000.0);
            double newApR = vessel.mainBody.Radius + (changeApoapsisKm * 1000.0);
            Vector3d dV = MechJebProxy.CalcDeltaVToEllipticize(vessel.orbit, ut, newPeR, newApR);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeBothAtApoapsis()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.NextApoapsisTime(Planetarium.GetUniversalTime());
            double newPeR = vessel.mainBody.Radius + (changePeriapsisKm * 1000.0);
            double newApR = vessel.mainBody.Radius + (changeApoapsisKm * 1000.0);
            Vector3d dV = MechJebProxy.CalcDeltaVToEllipticize(vessel.orbit, ut, newPeR, newApR);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeBothNow()
        {
            if (vessel == null) return;
            double ut = Planetarium.GetUniversalTime();
            double newPeR = vessel.mainBody.Radius + (changePeriapsisKm * 1000.0);
            double newApR = vessel.mainBody.Radius + (changeApoapsisKm * 1000.0);
            Vector3d dV = MechJebProxy.CalcDeltaVToEllipticize(vessel.orbit, ut, newPeR, newApR);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeEccentricityAtPeriapsis()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.NextPeriapsisTime(Planetarium.GetUniversalTime());
            Vector3d dV = MechJebProxy.CalcDeltaVToChangeEccentricity(vessel.orbit, ut, changeEccentricity);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeEccentricityAtApoapsis()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.NextApoapsisTime(Planetarium.GetUniversalTime());
            Vector3d dV = MechJebProxy.CalcDeltaVToChangeEccentricity(vessel.orbit, ut, changeEccentricity);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void ChangeSurfaceLongitude()
        {
            if (vessel == null) return;
            double ut = Planetarium.GetUniversalTime();
            Vector3d dV = MechJebProxy.CalcDeltaVToChangeSurfaceLongitude(vessel.orbit, ut, surfaceLongitudeDeg);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }

        private void FineTuneClosestApproach()
        {
            if (vessel == null || mjCore == null || FlightGlobals.fetch.VesselTarget == null) return;
            object targetController = MechJebProxy.GetTargetController(mjCore);
            if (targetController == null) return;

            object op = MechJebProxy.CreateFineTuneClosestApproachOperation();
            if (op == null) return;

            MechJebProxy.SetFineTuneDistance(op, fineTuneDistanceKm);
            MechJebProxy.CreateNodesFromOperation(op, vessel.orbit, Planetarium.GetUniversalTime(), targetController, vessel);
        }

        private void InterceptAtTime()
        {
            if (vessel == null || mjCore == null || FlightGlobals.fetch.VesselTarget == null) return;
            object targetController = MechJebProxy.GetTargetController(mjCore);
            if (targetController == null) return;

            object op = MechJebProxy.CreateInterceptAtTimeOperation();
            if (op == null) return;

            MechJebProxy.SetInterceptInterval(op, interceptIntervalSeconds);
            MechJebProxy.CreateNodesFromOperation(op, vessel.orbit, Planetarium.GetUniversalTime(), targetController, vessel);
        }

        private void CreateResonantOrbit()
        {
            if (vessel == null || mjCore == null) return;
            object targetController = MechJebProxy.GetTargetController(mjCore);

            object op = MechJebProxy.CreateResonantOrbitOperation();
            if (op == null) return;

            MechJebProxy.SetResonance(op, resonanceNumerator, resonanceDenominator);
            MechJebProxy.CreateNodesFromOperation(op, vessel.orbit, Planetarium.GetUniversalTime(), targetController, vessel);
        }

        private void CreateMoonReturn()
        {
            if (vessel == null || mjCore == null) return;
            object targetController = MechJebProxy.GetTargetController(mjCore);

            object op = MechJebProxy.CreateMoonReturnOperation();
            if (op == null) return;

            MechJebProxy.SetMoonReturnAltitude(op, moonReturnAltitudeKm);
            MechJebProxy.CreateNodesFromOperation(op, vessel.orbit, Planetarium.GetUniversalTime(), targetController, vessel);
        }

        private void CreateInterplanetaryTransfer()
        {
            if (vessel == null || mjCore == null) return;
            object targetController = MechJebProxy.GetTargetController(mjCore);
            if (targetController == null || !(FlightGlobals.fetch.VesselTarget is CelestialBody)) return;

            object op = MechJebProxy.CreateInterplanetaryTransferOperation();
            if (op == null) return;

            MechJebProxy.CreateNodesFromOperation(op, vessel.orbit, Planetarium.GetUniversalTime(), targetController, vessel);
        }

        private void RemoveAllNodes()
        {
            if (vessel == null || vessel.patchedConicSolver == null) return;
            while (vessel.patchedConicSolver.maneuverNodes.Count > 0)
            {
                vessel.patchedConicSolver.maneuverNodes[0].RemoveSelf();
            }
        }
        #endregion

        #region Node Editor Menu
        private TextMenu BuildNodeEditorMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddMenuItem(menu, "-- ADJUST NODE --", null);
            AddMenuItem(menu, "Prograde +1 m/s", () => AdjustNode(Vector3d.forward, 1));
            AddMenuItem(menu, "Prograde -1 m/s", () => AdjustNode(Vector3d.forward, -1));
            AddMenuItem(menu, "Prograde +10 m/s", () => AdjustNode(Vector3d.forward, 10));
            AddMenuItem(menu, "Prograde -10 m/s", () => AdjustNode(Vector3d.forward, -10));
            AddMenuItem(menu, "------", null);
            AddMenuItem(menu, "Normal +1 m/s", () => AdjustNode(Vector3d.up, 1));
            AddMenuItem(menu, "Normal -1 m/s", () => AdjustNode(Vector3d.up, -1));
            AddMenuItem(menu, "Normal +10 m/s", () => AdjustNode(Vector3d.up, 10));
            AddMenuItem(menu, "Normal -10 m/s", () => AdjustNode(Vector3d.up, -10));
            AddMenuItem(menu, "------", null);
            AddMenuItem(menu, "Radial +1 m/s", () => AdjustNode(Vector3d.right, 1));
            AddMenuItem(menu, "Radial -1 m/s", () => AdjustNode(Vector3d.right, -1));
            AddMenuItem(menu, "Radial +10 m/s", () => AdjustNode(Vector3d.right, 10));
            AddMenuItem(menu, "Radial -10 m/s", () => AdjustNode(Vector3d.right, -10));
            AddMenuItem(menu, "------", null);
            AddMenuItem(menu, "Delete Node", () => DeleteCurrentNode());
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void AdjustNode(Vector3d direction, double amount)
        {
            if (vessel == null || vessel.patchedConicSolver == null) return;
            if (vessel.patchedConicSolver.maneuverNodes.Count == 0) return;

            ManeuverNode node = vessel.patchedConicSolver.maneuverNodes[0];
            Vector3d dv = node.DeltaV;
            dv += direction * amount;
            node.DeltaV = dv;
            node.solver.UpdateFlightPlan();
        }

        private void DeleteCurrentNode()
        {
            if (vessel == null || vessel.patchedConicSolver == null) return;
            if (vessel.patchedConicSolver.maneuverNodes.Count == 0) return;

            vessel.patchedConicSolver.maneuverNodes[0].RemoveSelf();
        }
        #endregion

        #region Execute Node
        private void ExecuteNode()
        {
            if (mjCore == null) return;
            if (MechJebProxy.IsNodeExecutorRunning(mjCore))
            {
                MechJebProxy.AbortNode(mjCore);
            }
            else
            {
                MechJebProxy.ExecuteOneNode(mjCore, null);
            }
        }
        #endregion

        #region Rendezvous Menu
        private TextMenu BuildRendezvousMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddToggleItem(menu, "ENGAGE Rendezvous Autopilot",
                core => MechJebProxy.IsRendezvousAutopilotEngaged(core),
                (core, val) => MechJebProxy.SetRendezvousAutopilotEngaged(core, val));

            AddMenuItem(menu, "------", null);

            AddNumericItem(menu, "Desired Distance",
                core => MechJebProxy.GetRendezvousDesiredDistance(core),
                (core, val) => MechJebProxy.SetRendezvousDesiredDistance(core, val),
                10.0, v => v.ToString("F0") + " m", null, true, 0, false, 0);

            AddNumericItem(menu, "Max Phasing Orbits",
                core => MechJebProxy.GetRendezvousMaxPhasingOrbits(core),
                (core, val) => MechJebProxy.SetRendezvousMaxPhasingOrbits(core, (int)val),
                1.0, v => v.ToString("F0"), null, true, 0, false, 0);
            AddNumericItem(menu, "Max Closing Speed",
                core => MechJebProxy.GetRendezvousMaxClosingSpeed(core),
                (core, val) => MechJebProxy.SetRendezvousMaxClosingSpeed(core, val),
                1.0, v => v.ToString("F1") + " m/s", null, true, 0, false, 0);

            AddMenuItem(menu, "------", null);

            // Info display
            AddMenuItem(menu, "-- RENDEZVOUS INFO --", null);
            var rendezvousStatus = new TextMenu.Item("Status: ---", null);
            menu.Add(rendezvousStatus);
            trackedItems.Add(new TrackedMenuItem
            {
                item = rendezvousStatus,
                id = "RendezvousStatus",
                isEnabled = core => true,
                getLabel = core => "Status: " + MechJebProxy.GetRendezvousStatus(core)
            });

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }
        #endregion

        #region Docking Menu
        private TextMenu BuildDockingMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddToggleItem(menu, "ENGAGE Docking Autopilot",
                core => MechJebProxy.IsDockingAutopilotEngaged(core),
                (core, val) => MechJebProxy.SetDockingAutopilotEngaged(core, val));

            AddMenuItem(menu, "------", null);

            AddNumericItem(menu, "Speed Limit",
                core => MechJebProxy.GetDockingSpeedLimit(core),
                (core, val) => MechJebProxy.SetDockingSpeedLimit(core, val),
                0.1, v => v.ToString("F1") + " m/s", null, true, 0, false, 0);

            AddToggleItem(menu, "Force Roll",
                core => MechJebProxy.GetDockingForceRoll(core),
                (core, val) => MechJebProxy.SetDockingForceRoll(core, val));
            AddNumericItem(menu, "Roll",
                core => MechJebProxy.GetDockingRoll(core),
                (core, val) => MechJebProxy.SetDockingRoll(core, val),
                1.0, v => v.ToString("F1") + "°", null, true, -180, true, 180);

            AddToggleItem(menu, "Override Safe Distance",
                core => MechJebProxy.GetDockingOverrideSafeDistance(core),
                (core, val) => MechJebProxy.SetDockingOverrideSafeDistance(core, val));
            AddNumericItem(menu, "Safe Distance",
                core => MechJebProxy.GetDockingOverridenSafeDistance(core),
                (core, val) => MechJebProxy.SetDockingOverridenSafeDistance(core, val),
                0.1, v => v.ToString("F1") + " m", core => MechJebProxy.GetDockingOverrideSafeDistance(core), true, 0, false, 0);

            AddToggleItem(menu, "Override Target Size",
                core => MechJebProxy.GetDockingOverrideTargetSize(core),
                (core, val) => MechJebProxy.SetDockingOverrideTargetSize(core, val));
            AddNumericItem(menu, "Target Size",
                core => MechJebProxy.GetDockingOverridenTargetSize(core),
                (core, val) => MechJebProxy.SetDockingOverridenTargetSize(core, val),
                0.1, v => v.ToString("F1") + " m", core => MechJebProxy.GetDockingOverrideTargetSize(core), true, 0, false, 0);

            AddToggleItem(menu, "Draw Bounding Box",
                core => MechJebProxy.GetDockingDrawBoundingBox(core),
                (core, val) => MechJebProxy.SetDockingDrawBoundingBox(core, val));

            AddMenuItem(menu, "------", null);

            // Status
            AddMenuItem(menu, "Status:", null);
            var dockingStatus = new TextMenu.Item("  ---", null);
            menu.Add(dockingStatus);
            trackedItems.Add(new TrackedMenuItem
            {
                item = dockingStatus,
                id = "DockingStatus",
                isEnabled = core => true,
                getLabel = core => "  " + MechJebProxy.GetDockingStatus(core)
            });

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }
        #endregion

        #region Translatron Menu
        private TextMenu BuildTranslatronMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddMenuItem(menu, "-- MODE --", null);
            AddMenuItem(menu, "OFF", () => MechJebProxy.SetTranslatronMode(mjCore, MechJebProxy.TranslatronMode.OFF));
            AddMenuItem(menu, "Keep Orbital Vel", () => MechJebProxy.SetTranslatronMode(mjCore, MechJebProxy.TranslatronMode.KEEP_OBT));
            AddMenuItem(menu, "Keep Surface Vel", () => MechJebProxy.SetTranslatronMode(mjCore, MechJebProxy.TranslatronMode.KEEP_SURF));
            AddMenuItem(menu, "Keep Vertical Vel", () => MechJebProxy.SetTranslatronMode(mjCore, MechJebProxy.TranslatronMode.KEEP_VERT));
            AddMenuItem(menu, "Keep Relative Vel", () => MechJebProxy.SetTranslatronMode(mjCore, MechJebProxy.TranslatronMode.KEEP_REL),
                core => FlightGlobals.fetch.VesselTarget != null);
            AddMenuItem(menu, "Direct", () => MechJebProxy.SetTranslatronMode(mjCore, MechJebProxy.TranslatronMode.DIRECT));

            AddMenuItem(menu, "------", null);

            AddNumericItem(menu, "Target Speed",
                core => MechJebProxy.GetTranslatronSpeed(core),
                (core, val) => MechJebProxy.SetTranslatronSpeed(core, val),
                0.1, v => v.ToString("F1") + " m/s", null, true, 0, false, 0);

            AddToggleItem(menu, "Kill Horizontal",
                core => MechJebProxy.GetTranslatronKillH(core),
                (core, val) => MechJebProxy.SetTranslatronKillH(core, val));

            AddMenuItem(menu, "------", null);

            AddMenuItem(menu, "!! PANIC !!", () => MechJebProxy.PanicSwitch(mjCore));

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }
        #endregion

        #region Rover Menu
        private TextMenu BuildRoverMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddMenuItem(menu, "Drive to Target", () => DriveToTarget(),
                core => MechJebProxy.PositionTargetExists(core));
            AddMenuItem(menu, "STOP", () => StopRover());

            AddMenuItem(menu, "------", null);

            AddToggleItem(menu, "Control Heading",
                core => MechJebProxy.GetRoverControlHeading(core),
                (core, val) => MechJebProxy.SetRoverControlHeading(core, val));
            AddNumericItem(menu, "Heading",
                core => MechJebProxy.GetRoverHeading(core),
                (core, val) => MechJebProxy.SetRoverHeading(core, val),
                1.0, v => v.ToString("F1") + "°", null, true, 0, true, 360);

            AddToggleItem(menu, "Control Speed",
                core => MechJebProxy.GetRoverControlSpeed(core),
                (core, val) => MechJebProxy.SetRoverControlSpeed(core, val));
            AddNumericItem(menu, "Speed",
                core => MechJebProxy.GetRoverSpeed(core),
                (core, val) => MechJebProxy.SetRoverSpeed(core, val),
                0.5, v => v.ToString("F1") + " m/s", null, true, 0, false, 0);

            AddMenuItem(menu, "------", null);

            AddToggleItem(menu, "Stability Control",
                core => MechJebProxy.GetRoverStabilityControl(core),
                (core, val) => MechJebProxy.SetRoverStabilityControl(core, val));
            AddToggleItem(menu, "Brake on Eject",
                core => MechJebProxy.GetRoverBrakeOnEject(core),
                (core, val) => MechJebProxy.SetRoverBrakeOnEject(core, val));
            AddToggleItem(menu, "Brake on Energy Depletion",
                core => MechJebProxy.GetRoverBrakeOnEnergyDepletion(core),
                (core, val) => MechJebProxy.SetRoverBrakeOnEnergyDepletion(core, val));
            AddToggleItem(menu, "Warp to Daylight",
                core => MechJebProxy.GetRoverWarpToDaylight(core),
                (core, val) => MechJebProxy.SetRoverWarpToDaylight(core, val));

            AddMenuItem(menu, "------", null);

            AddMenuItem(menu, "Waypoints", () => PushMenu(BuildRoverWaypointsMenu()));

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildRoverWaypointsMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            AddMenuItem(menu, "Add Waypoint", () => AddRoverWaypoint());
            AddMenuItem(menu, "Clear All Waypoints", () => ClearRoverWaypoints());
            // Waypoint list would go here

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void DriveToTarget()
        {
            if (mjCore == null) return;
            MechJebProxy.DriveToTarget(mjCore);
        }

        private void StopRover()
        {
            if (mjCore == null) return;
            MechJebProxy.StopRover(mjCore);
        }

        private void AddRoverWaypoint()
        {
            if (mjCore == null || vessel == null) return;
            MechJebProxy.AddRoverWaypointAtCurrentPosition(mjCore, vessel);
        }

        private void ClearRoverWaypoints()
        {
            if (mjCore == null) return;
            MechJebProxy.ClearRoverWaypoints(mjCore);
        }
        #endregion

        #region Aircraft Menu
        private TextMenu BuildAircraftMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddMenuItem(menu, "-- ALTITUDE --", null);
            AddToggleItem(menu, "Altitude Hold",
                core => MechJebProxy.GetAirplaneAltitudeHold(core),
                (core, val) => MechJebProxy.SetAirplaneAltitudeHold(core, val));
            AddNumericItem(menu, "Target Altitude",
                core => MechJebProxy.GetAirplaneAltitudeTarget(core),
                (core, val) => MechJebProxy.SetAirplaneAltitudeTarget(core, val),
                50.0, v => v.ToString("F0") + " m", null, true, 0, false, 0);
            AddToggleItem(menu, "Vertical Speed Hold",
                core => MechJebProxy.GetAirplaneVertSpeedHold(core),
                (core, val) => MechJebProxy.SetAirplaneVertSpeedHold(core, val));
            AddNumericItem(menu, "Target Vert Speed",
                core => MechJebProxy.GetAirplaneVertSpeedTarget(core),
                (core, val) => MechJebProxy.SetAirplaneVertSpeedTarget(core, val),
                1.0, v => v.ToString("F1") + " m/s", null, false, 0, false, 0);

            AddMenuItem(menu, "------", null);

            AddMenuItem(menu, "-- HEADING --", null);
            AddToggleItem(menu, "Heading Hold",
                core => MechJebProxy.GetAirplaneHeadingHold(core),
                (core, val) => MechJebProxy.SetAirplaneHeadingHold(core, val));
            AddNumericItem(menu, "Target Heading",
                core => MechJebProxy.GetAirplaneHeadingTarget(core),
                (core, val) => MechJebProxy.SetAirplaneHeadingTarget(core, val),
                1.0, v => v.ToString("F1") + "°", null, true, 0, true, 360);
            AddToggleItem(menu, "Roll Hold",
                core => MechJebProxy.GetAirplaneRollHold(core),
                (core, val) => MechJebProxy.SetAirplaneRollHold(core, val));
            AddNumericItem(menu, "Target Roll",
                core => MechJebProxy.GetAirplaneRollTarget(core),
                (core, val) => MechJebProxy.SetAirplaneRollTarget(core, val),
                1.0, v => v.ToString("F1") + "°", null, true, -180, true, 180);

            AddMenuItem(menu, "------", null);

            AddMenuItem(menu, "-- SPEED --", null);
            AddToggleItem(menu, "Speed Hold",
                core => MechJebProxy.GetAirplaneSpeedHold(core),
                (core, val) => MechJebProxy.SetAirplaneSpeedHold(core, val));
            AddNumericItem(menu, "Target Speed",
                core => MechJebProxy.GetAirplaneSpeedTarget(core),
                (core, val) => MechJebProxy.SetAirplaneSpeedTarget(core, val),
                1.0, v => v.ToString("F1") + " m/s", null, true, 0, false, 0);

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }
        #endregion

        #region Spaceplane Menu
        private TextMenu BuildSpaceplaneMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddMenuItem(menu, "Autoland", () => SpaceplaneAutoland());
            AddMenuItem(menu, "Hold Heading & Altitude", () => SpaceplaneHoldHeadingAlt());
            AddMenuItem(menu, "Autopilot OFF", () => SpaceplaneAutopilotOff());

            AddMenuItem(menu, "------", null);

            AddNumericItem(menu, "Glideslope",
                core => MechJebProxy.GetSpaceplaneGlideslope(core),
                (core, val) => MechJebProxy.SetSpaceplaneGlideslope(core, val),
                0.1, v => v.ToString("F1") + "°", null, true, 0, true, 30);
            AddNumericItem(menu, "Approach Speed",
                core => MechJebProxy.GetSpaceplaneApproachSpeed(core),
                (core, val) => MechJebProxy.SetSpaceplaneApproachSpeed(core, val),
                1.0, v => v.ToString("F1") + " m/s", null, true, 0, false, 0);
            AddNumericItem(menu, "Touchdown Speed",
                core => MechJebProxy.GetSpaceplaneTouchdownSpeed(core),
                (core, val) => MechJebProxy.SetSpaceplaneTouchdownSpeed(core, val),
                1.0, v => v.ToString("F1") + " m/s", null, true, 0, false, 0);

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void SpaceplaneAutoland()
        {
            if (mjCore == null) return;
            MechJebProxy.SpaceplaneAutoland(mjCore);
        }

        private void SpaceplaneHoldHeadingAlt()
        {
            if (mjCore == null) return;
            MechJebProxy.SpaceplaneHoldHeadingAndAltitude(mjCore);
        }

        private void SpaceplaneAutopilotOff()
        {
            if (mjCore == null) return;
            MechJebProxy.SpaceplaneAutopilotOff(mjCore);
        }
        #endregion

        #region Utilities Menu
        private TextMenu BuildUtilitiesMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);
            menu.disabledColor = JUtil.ColorToColorTag(Color.gray);

            AddToggleItem(menu, "Autostage",
                core => MechJebProxy.GetAutostage(core),
                (core, val) => MechJebProxy.SetAutostage(core, val));
            AddNumericItem(menu, "Stop at Stage",
                core => MechJebProxy.GetAutostageLimit(core),
                (core, val) => MechJebProxy.SetAutostageLimit(core, (int)val),
                1.0, v => v.ToString("F0"), null, true, 0, false, 0);
            AddMenuItem(menu, "Stage Once", () => StageOnce());
            AddMenuItem(menu, "Autostage Options", () => PushMenu(BuildAutostageOptionsMenu()));

            AddMenuItem(menu, "------", null);

            // Delta-V info
            AddMenuItem(menu, "-- DELTA-V INFO --", null);
            var stageVacItem = new TextMenu.Item("Stage dV (Vac):", null);
            var totalVacItem = new TextMenu.Item("Total dV (Vac):", null);
            var stageAtmItem = new TextMenu.Item("Stage dV (Atm):", null);
            var totalAtmItem = new TextMenu.Item("Total dV (Atm):", null);
            menu.Add(stageVacItem);
            menu.Add(totalVacItem);
            menu.Add(stageAtmItem);
            menu.Add(totalAtmItem);

            trackedItems.Add(new TrackedMenuItem
            {
                item = stageVacItem,
                id = "StageDVVac",
                isEnabled = core => true,
                getLabel = core => "Stage dV (Vac): " + GetStageDeltaVText(core, true)
            });
            trackedItems.Add(new TrackedMenuItem
            {
                item = totalVacItem,
                id = "TotalDVVac",
                isEnabled = core => true,
                getLabel = core => "Total dV (Vac): " + FormatDeltaV(MechJebProxy.GetTotalVacuumDeltaV(core))
            });
            trackedItems.Add(new TrackedMenuItem
            {
                item = stageAtmItem,
                id = "StageDVAtm",
                isEnabled = core => true,
                getLabel = core => "Stage dV (Atm): " + GetStageDeltaVText(core, false)
            });
            trackedItems.Add(new TrackedMenuItem
            {
                item = totalAtmItem,
                id = "TotalDVAtm",
                isEnabled = core => true,
                getLabel = core => "Total dV (Atm): " + FormatDeltaV(MechJebProxy.GetTotalAtmoDeltaV(core))
            });

            AddMenuItem(menu, "------", null);

            AddMenuItem(menu, "Warp Helper", () => PushMenu(BuildWarpHelperMenu()));

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildAutostageOptionsMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            AddNumericItem(menu, "Pre-Delay",
                core => MechJebProxy.GetAutostagePreDelay(core),
                (core, val) => MechJebProxy.SetAutostagePreDelay(core, val),
                0.1, v => v.ToString("F1") + " s", null, true, 0, false, 0);
            AddNumericItem(menu, "Post-Delay",
                core => MechJebProxy.GetAutostagePostDelay(core),
                (core, val) => MechJebProxy.SetAutostagePostDelay(core, val),
                0.1, v => v.ToString("F1") + " s", null, true, 0, false, 0);
            AddNumericItem(menu, "Clamp Thrust %",
                core => MechJebProxy.GetClampAutoStageThrustPct(core),
                (core, val) => MechJebProxy.SetClampAutoStageThrustPct(core, val),
                1.0, v => v.ToString("F0") + "%", null, true, 0, true, 100);

            AddMenuItem(menu, "------", null);

            AddNumericItem(menu, "Fairing Max Flux",
                core => MechJebProxy.GetFairingMaxAerothermalFlux(core),
                (core, val) => MechJebProxy.SetFairingMaxAerothermalFlux(core, val),
                1000.0, v => v.ToString("F0"), null, true, 0, false, 0);
            AddNumericItem(menu, "Fairing Max Q",
                core => MechJebProxy.GetFairingMaxDynamicPressure(core),
                (core, val) => MechJebProxy.SetFairingMaxDynamicPressure(core, val),
                1000.0, v => v.ToString("F0") + " Pa", null, true, 0, false, 0);
            AddNumericItem(menu, "Fairing Min Alt",
                core => MechJebProxy.GetFairingMinAltitude(core) / 1000.0,
                (core, val) => MechJebProxy.SetFairingMinAltitude(core, val * 1000.0),
                1.0, v => v.ToString("F1") + " km", null, true, 0, false, 0);

            AddMenuItem(menu, "------", null);

            AddNumericItem(menu, "Hot Staging Lead",
                core => MechJebProxy.GetHotStagingLeadTime(core),
                (core, val) => MechJebProxy.SetHotStagingLeadTime(core, val),
                0.1, v => v.ToString("F1") + " s", null, true, 0, false, 0);
            AddToggleItem(menu, "Drop Solids",
                core => MechJebProxy.GetDropSolids(core),
                (core, val) => MechJebProxy.SetDropSolids(core, val));
            AddNumericItem(menu, "Drop Solids Lead",
                core => MechJebProxy.GetDropSolidsLeadTime(core),
                (core, val) => MechJebProxy.SetDropSolidsLeadTime(core, val),
                0.1, v => v.ToString("F1") + " s", core => MechJebProxy.GetDropSolids(core), true, 0, false, 0);

            AddMenuItem(menu, "[BACK]", () => PopMenu());
            return menu;
        }

        private TextMenu BuildWarpHelperMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            AddMenuItem(menu, "Warp to Apoapsis", () => WarpToApoapsis());
            AddMenuItem(menu, "Warp to Periapsis", () => WarpToPeriapsis());
            AddMenuItem(menu, "Warp to Node", () => WarpToNode(),
                core => vessel != null && vessel.patchedConicSolver != null &&
                        vessel.patchedConicSolver.maneuverNodes.Count > 0);
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private void StageOnce()
        {
            if (mjCore == null) return;
            MechJebProxy.AutostageOnce(mjCore);
        }
        #endregion

        #region Info Display Menu
        private TextMenu BuildInfoMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            AddMenuItem(menu, "Orbit Info", () => PushMenu(BuildOrbitInfoMenu()));
            AddMenuItem(menu, "Surface Info", () => PushMenu(BuildSurfaceInfoMenu()));
            AddMenuItem(menu, "Target Info", () => PushMenu(BuildTargetInfoMenu()),
                core => FlightGlobals.fetch.VesselTarget != null);
            AddMenuItem(menu, "Vessel Info", () => PushMenu(BuildVesselInfoMenu()));
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildOrbitInfoMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            // These items will have their labels updated dynamically
            var apItem = new TextMenu.Item("Apoapsis:", null);
            var peItem = new TextMenu.Item("Periapsis:", null);
            var eccItem = new TextMenu.Item("Eccentricity:", null);
            var incItem = new TextMenu.Item("Inclination:", null);
            var lanItem = new TextMenu.Item("LAN:", null);
            var argPeItem = new TextMenu.Item("Arg. of PE:", null);
            var periodItem = new TextMenu.Item("Period:", null);
            var tApItem = new TextMenu.Item("Time to AP:", null);
            var tPeItem = new TextMenu.Item("Time to PE:", null);
            menu.Add(apItem);
            menu.Add(peItem);
            menu.Add(eccItem);
            menu.Add(incItem);
            menu.Add(lanItem);
            menu.Add(argPeItem);
            menu.Add(periodItem);
            menu.Add(tApItem);
            menu.Add(tPeItem);

            trackedItems.Add(new TrackedMenuItem { item = apItem, id = "OrbitAp", isEnabled = core => true, getLabel = core => "Apoapsis: " + FormatDistance(vessel != null ? vessel.orbit.ApA : 0) });
            trackedItems.Add(new TrackedMenuItem { item = peItem, id = "OrbitPe", isEnabled = core => true, getLabel = core => "Periapsis: " + FormatDistance(vessel != null ? vessel.orbit.PeA : 0) });
            trackedItems.Add(new TrackedMenuItem { item = eccItem, id = "OrbitEcc", isEnabled = core => true, getLabel = core => "Eccentricity: " + (vessel != null ? vessel.orbit.eccentricity.ToString("F4") : "---") });
            trackedItems.Add(new TrackedMenuItem { item = incItem, id = "OrbitInc", isEnabled = core => true, getLabel = core => "Inclination: " + FormatAngle(vessel != null ? vessel.orbit.inclination : 0) });
            trackedItems.Add(new TrackedMenuItem { item = lanItem, id = "OrbitLAN", isEnabled = core => true, getLabel = core => "LAN: " + FormatAngle(vessel != null ? vessel.orbit.LAN : 0) });
            trackedItems.Add(new TrackedMenuItem { item = argPeItem, id = "OrbitArgPe", isEnabled = core => true, getLabel = core => "Arg. of PE: " + FormatAngle(vessel != null ? vessel.orbit.argumentOfPeriapsis : 0) });
            trackedItems.Add(new TrackedMenuItem { item = periodItem, id = "OrbitPeriod", isEnabled = core => true, getLabel = core => "Period: " + FormatTime(vessel != null ? vessel.orbit.period : 0) });
            trackedItems.Add(new TrackedMenuItem { item = tApItem, id = "OrbitTAP", isEnabled = core => true, getLabel = core => "Time to AP: " + FormatTime(GetTimeToApoapsis()) });
            trackedItems.Add(new TrackedMenuItem { item = tPeItem, id = "OrbitTPE", isEnabled = core => true, getLabel = core => "Time to PE: " + FormatTime(GetTimeToPeriapsis()) });
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildSurfaceInfoMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            var altAslItem = new TextMenu.Item("Altitude (ASL):", null);
            var altAglItem = new TextMenu.Item("Altitude (AGL):", null);
            var latItem = new TextMenu.Item("Latitude:", null);
            var lonItem = new TextMenu.Item("Longitude:", null);
            var srfSpdItem = new TextMenu.Item("Surface Speed:", null);
            var vertSpdItem = new TextMenu.Item("Vertical Speed:", null);
            var horizSpdItem = new TextMenu.Item("Horizontal Speed:", null);
            var headingItem = new TextMenu.Item("Heading:", null);
            menu.Add(altAslItem);
            menu.Add(altAglItem);
            menu.Add(latItem);
            menu.Add(lonItem);
            menu.Add(srfSpdItem);
            menu.Add(vertSpdItem);
            menu.Add(horizSpdItem);
            menu.Add(headingItem);

            trackedItems.Add(new TrackedMenuItem { item = altAslItem, id = "SurfAltASL", isEnabled = core => true, getLabel = core => "Altitude (ASL): " + FormatDistance(vessel != null ? vessel.altitude : 0) });
            trackedItems.Add(new TrackedMenuItem { item = altAglItem, id = "SurfAltAGL", isEnabled = core => true, getLabel = core => "Altitude (AGL): " + FormatDistance(vessel != null ? vessel.radarAltitude : 0) });
            trackedItems.Add(new TrackedMenuItem { item = latItem, id = "SurfLat", isEnabled = core => true, getLabel = core => "Latitude: " + FormatAngle(vessel != null ? vessel.latitude : 0) });
            trackedItems.Add(new TrackedMenuItem { item = lonItem, id = "SurfLon", isEnabled = core => true, getLabel = core => "Longitude: " + FormatAngle(vessel != null ? vessel.longitude : 0) });
            trackedItems.Add(new TrackedMenuItem { item = srfSpdItem, id = "SurfSpd", isEnabled = core => true, getLabel = core => "Surface Speed: " + FormatSpeed(vessel != null ? vessel.srfSpeed : 0) });
            trackedItems.Add(new TrackedMenuItem { item = vertSpdItem, id = "VertSpd", isEnabled = core => true, getLabel = core => "Vertical Speed: " + FormatSpeed(vessel != null ? vessel.verticalSpeed : 0) });
            trackedItems.Add(new TrackedMenuItem { item = horizSpdItem, id = "HorizSpd", isEnabled = core => true, getLabel = core => "Horizontal Speed: " + FormatSpeed(vessel != null ? vessel.horizontalSrfSpeed : 0) });
            trackedItems.Add(new TrackedMenuItem { item = headingItem, id = "Heading", isEnabled = core => true, getLabel = core => "Heading: " + FormatAngle(GetSurfaceHeading()) });
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildTargetInfoMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            var distItem = new TextMenu.Item("Distance:", null);
            var relVelItem = new TextMenu.Item("Relative Velocity:", null);
            var caItem = new TextMenu.Item("Closest Approach:", null);
            var tcaItem = new TextMenu.Item("Time to Closest:", null);
            var relIncItem = new TextMenu.Item("Rel Inclination:", null);
            menu.Add(distItem);
            menu.Add(relVelItem);
            menu.Add(caItem);
            menu.Add(tcaItem);
            menu.Add(relIncItem);

            trackedItems.Add(new TrackedMenuItem { item = distItem, id = "TgtDist", isEnabled = core => true, getLabel = core => "Distance: " + GetTargetDistanceText() });
            trackedItems.Add(new TrackedMenuItem { item = relVelItem, id = "TgtRelVel", isEnabled = core => true, getLabel = core => "Relative Velocity: " + GetTargetRelVelText() });
            trackedItems.Add(new TrackedMenuItem { item = caItem, id = "TgtCA", isEnabled = core => true, getLabel = core => "Closest Approach: " + GetTargetClosestApproachText() });
            trackedItems.Add(new TrackedMenuItem { item = tcaItem, id = "TgtTCA", isEnabled = core => true, getLabel = core => "Time to Closest: " + GetTargetTimeToClosestText() });
            trackedItems.Add(new TrackedMenuItem { item = relIncItem, id = "TgtRelInc", isEnabled = core => true, getLabel = core => "Rel Inclination: " + GetTargetRelInclinationText() });
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }

        private TextMenu BuildVesselInfoMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            var massItem = new TextMenu.Item("Mass:", null);
            var twrItem = new TextMenu.Item("TWR:", null);
            var maxThrustItem = new TextMenu.Item("Max Thrust:", null);
            var curThrustItem = new TextMenu.Item("Current Thrust:", null);
            var dvVacItem = new TextMenu.Item("Total dV (Vac):", null);
            var dvAtmItem = new TextMenu.Item("Total dV (Atm):", null);
            menu.Add(massItem);
            menu.Add(twrItem);
            menu.Add(maxThrustItem);
            menu.Add(curThrustItem);
            menu.Add(dvVacItem);
            menu.Add(dvAtmItem);

            trackedItems.Add(new TrackedMenuItem { item = massItem, id = "VesselMass", isEnabled = core => true, getLabel = core => "Mass: " + GetVesselMassText() });
            trackedItems.Add(new TrackedMenuItem { item = twrItem, id = "VesselTWR", isEnabled = core => true, getLabel = core => "TWR: " + GetVesselTwrText() });
            trackedItems.Add(new TrackedMenuItem { item = maxThrustItem, id = "VesselMaxThrust", isEnabled = core => true, getLabel = core => "Max Thrust: " + GetVesselMaxThrustText() });
            trackedItems.Add(new TrackedMenuItem { item = curThrustItem, id = "VesselCurThrust", isEnabled = core => true, getLabel = core => "Current Thrust: " + GetVesselCurrentThrustText() });
            trackedItems.Add(new TrackedMenuItem { item = dvVacItem, id = "VesselDVVac", isEnabled = core => true, getLabel = core => "Total dV (Vac): " + FormatDeltaV(MechJebProxy.GetTotalVacuumDeltaV(core)) });
            trackedItems.Add(new TrackedMenuItem { item = dvAtmItem, id = "VesselDVAtm", isEnabled = core => true, getLabel = core => "Total dV (Atm): " + FormatDeltaV(MechJebProxy.GetTotalAtmoDeltaV(core)) });
            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }
        #endregion

        #region Settings Menu
        private TextMenu BuildSettingsMenu()
        {
            var menu = new TextMenu();
            menu.labelColor = JUtil.ColorToColorTag(Color.white);
            menu.selectedColor = JUtil.ColorToColorTag(Color.green);

            AddMenuItem(menu, "-- THRUST LIMITS --", null);
            AddToggleItem(menu, "Prevent Overheats",
                core => MechJebProxy.GetLimitToPreventOverheats(core),
                (core, val) => MechJebProxy.SetLimitToPreventOverheats(core, val));
            AddToggleItem(menu, "Limit by Max Q",
                core => MechJebProxy.GetLimitToMaxDynamicPressure(core),
                (core, val) => MechJebProxy.SetLimitToMaxDynamicPressure(core, val));
            AddToggleItem(menu, "Limit to Terminal Velocity",
                core => MechJebProxy.GetLimitToTerminalVelocity(core),
                (core, val) => MechJebProxy.SetLimitToTerminalVelocity(core, val));
            AddToggleItem(menu, "Limit Acceleration",
                core => MechJebProxy.GetLimitAcceleration(core),
                (core, val) => MechJebProxy.SetLimitAcceleration(core, val));
            AddToggleItem(menu, "Limit Throttle",
                core => MechJebProxy.GetLimitThrottle(core),
                (core, val) => MechJebProxy.SetLimitThrottle(core, val));

            AddMenuItem(menu, "------", null);

            AddToggleItem(menu, "Prevent Flameout",
                core => MechJebProxy.GetLimitToPreventFlameout(core),
                (core, val) => MechJebProxy.SetLimitToPreventFlameout(core, val));
            AddNumericItem(menu, "Flameout Safety",
                core => MechJebProxy.GetFlameoutSafetyPct(core),
                (core, val) => MechJebProxy.SetFlameoutSafetyPct(core, val),
                1.0, v => v.ToString("F0") + "%", null, true, 0, true, 100);
            AddToggleItem(menu, "Smooth Throttle",
                core => MechJebProxy.GetSmoothThrottle(core),
                (core, val) => MechJebProxy.SetSmoothThrottle(core, val));
            AddToggleItem(menu, "Manage Intakes",
                core => MechJebProxy.GetManageIntakes(core),
                (core, val) => MechJebProxy.SetManageIntakes(core, val));
            AddToggleItem(menu, "Differential Throttle",
                core => MechJebProxy.GetDifferentialThrottle(core),
                (core, val) => MechJebProxy.SetDifferentialThrottle(core, val));

            AddMenuItem(menu, "------", null);

            AddMenuItem(menu, "-- NODE EXECUTION --", null);
            AddToggleItem(menu, "Auto-Warp",
                core => MechJebProxy.GetNodeAutowarp(core),
                (core, val) => MechJebProxy.SetNodeAutowarp(core, val));

            AddMenuItem(menu, "[BACK]", () => PopMenu());

            return menu;
        }
        #endregion

        #region Menu Navigation
        private void PushMenu(TextMenu newMenu)
        {
            if (newMenu != null)
            {
                menuStack.Push(currentMenu);
                currentMenu = newMenu;
            }
        }

        private void PopMenu()
        {
            if (menuStack.Count > 0)
            {
                currentMenu = menuStack.Pop();
            }
        }

        private void GoHome()
        {
            menuStack.Clear();
            currentMenu = topMenu;
        }

        private bool IsAscentAvailable(Vessel v)
        {
            if (v == null) return false;
            if (v.LandedOrSplashed) return true;

            if (v.situation == Vessel.Situations.ORBITING)
            {
                double atmosphere = v.mainBody != null ? v.mainBody.atmosphereDepth : 0;
                if (atmosphere <= 0) atmosphere = 0;
                return !(v.orbit.PeA > atmosphere && v.orbit.ApA > atmosphere);
            }

            return true;
        }
        #endregion

        #region Update Loop
        public void Update()
        {
            if (vessel == null || !MechJebProxy.IsAvailable) return;

            // Update MechJeb core reference if vessel changed
            if (vessel != activeVessel || mjCore == null)
            {
                activeVessel = vessel;
                mjCore = MechJebProxy.GetMasterMechJeb(vessel);
            }

            // Update tracked items
            UpdateTrackedItems();
            UpdateAdvancedTransferStatus();

            double ut = Planetarium.GetUniversalTime();
            if (ut - lastStageStatsUpdateUT > 1.0)
            {
                MechJebProxy.RequestStageStatsUpdate(mjCore, this);
                lastStageStatsUpdateUT = ut;
            }
        }

        // LEGACY: UpdateAdvancedTransferStatus - no longer needed
        // Status is now computed dynamically in GetAdvancedTransferStatusText() using wrapper
        private void UpdateAdvancedTransferStatus()
        {
            // Status is now computed on-demand in GetAdvancedTransferStatusText()
            // using MechJeb's actual operation instance from GetOperationByName
        }

        private void UpdateTrackedItems()
        {
            if (mjCore == null) return;

            foreach (var tracked in trackedItems)
            {
                try
                {
                    // Update enabled state
                    if (tracked.isEnabled != null)
                    {
                        tracked.item.isDisabled = !tracked.isEnabled(mjCore);
                    }

                    // Update label
                    if (tracked.getLabel != null)
                    {
                        string newLabel = tracked.getLabel(mjCore);
                        if (!string.IsNullOrEmpty(newLabel))
                        {
                            tracked.item.labelText = newLabel;
                        }
                    }

                    // Update selected state (for toggles)
                    if (tracked.isSelected != null)
                    {
                        tracked.item.isSelected = tracked.isSelected(mjCore);
                    }
                }
                catch (Exception)
                {
                    // Silently ignore - keep existing label
                }
            }

            UpdateSmartASSSelections();
        }

        private void UpdateSmartASSSelections()
        {
            object smartass = MechJebProxy.GetSmartASS(mjCore);
            if (smartass == null) return;

            int currentTarget = MechJebProxy.GetSmartASSTarget(smartass);

            UpdateMenuSelectionById(smartassOrbitalMenu, currentTarget);
            UpdateMenuSelectionById(smartassSurfaceMenu, currentTarget);
            UpdateMenuSelectionById(smartassTargetMenu, currentTarget);
        }

        private void UpdateMenuSelectionById(TextMenu menu, int targetId)
        {
            if (menu == null) return;

            for (int i = 0; i < menu.Count; i++)
            {
                bool match = (menu[i].id == targetId);
                menu[i].isSelected = match;
            }
        }
        #endregion

        #region Maneuver Planner Extras
        private void CreateCourseCorrection()
        {
            if (vessel == null || mjCore == null) return;
            object targetController = MechJebProxy.GetTargetController(mjCore);
            if (targetController == null) return;

            object op = MechJebProxy.CreateCourseCorrectionOperation();
            if (op == null) return;

            MechJebProxy.SetCourseCorrectionTargetPe(op, courseCorrectionPeKm);
            MechJebProxy.CreateNodesFromOperation(op, vessel.orbit, Planetarium.GetUniversalTime(), targetController, vessel);
        }

        private void MatchVelocitiesNow()
        {
            if (vessel == null || FlightGlobals.fetch.VesselTarget == null) return;
            Orbit target = FlightGlobals.fetch.VesselTarget.GetOrbit();
            if (target == null) return;

            double ut = Planetarium.GetUniversalTime();
            Vector3d dV = MechJebProxy.CalcDeltaVToMatchVelocities(vessel.orbit, ut, target);
            MechJebProxy.PlaceManeuverNode(vessel, vessel.orbit, dV, ut);
        }
        #endregion

        #region Warp Helpers
        private void WarpToApoapsis()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.NextApoapsisTime(Planetarium.GetUniversalTime());
            MechJebProxy.WarpToUT(mjCore, ut);
        }

        private void WarpToPeriapsis()
        {
            if (vessel == null) return;
            double ut = vessel.orbit.NextPeriapsisTime(Planetarium.GetUniversalTime());
            MechJebProxy.WarpToUT(mjCore, ut);
        }

        private void WarpToNode()
        {
            if (vessel == null || vessel.patchedConicSolver == null) return;
            if (vessel.patchedConicSolver.maneuverNodes.Count == 0) return;
            double ut = vessel.patchedConicSolver.maneuverNodes[0].UT;
            MechJebProxy.WarpToUT(mjCore, ut);
        }

        private void WarpToSOI()
        {
            if (vessel == null) return;
            if (vessel.orbit.patchEndTransition == Orbit.PatchTransitionType.FINAL) return;
            double ut = vessel.orbit.EndUT;
            MechJebProxy.WarpToUT(mjCore, ut);
        }
        #endregion

        #region Landing Prediction Helpers
        private string GetLandingPredLatitude(object core)
        {
            object result = MechJebProxy.GetLandingPredictionResult(core);
            if (result == null) return "---";
            double lat, lon;
            MechJebProxy.GetLandingEndPosition(result, out lat, out lon);
            return lat.ToString("F3") + "°";
        }

        private string GetLandingPredLongitude(object core)
        {
            object result = MechJebProxy.GetLandingPredictionResult(core);
            if (result == null) return "---";
            double lat, lon;
            MechJebProxy.GetLandingEndPosition(result, out lat, out lon);
            return lon.ToString("F3") + "°";
        }

        private string GetLandingPredTime(object core)
        {
            object result = MechJebProxy.GetLandingPredictionResult(core);
            if (result == null) return "---";
            double ut = MechJebProxy.GetLandingEndUT(result);
            double dt = ut - Planetarium.GetUniversalTime();
            return FormatTime(dt);
        }

        private string GetLandingPredGees(object core)
        {
            object result = MechJebProxy.GetLandingPredictionResult(core);
            if (result == null) return "---";
            double gees = MechJebProxy.GetLandingMaxDragGees(result);
            return gees.ToString("F2");
        }
        #endregion

        #region Formatting Helpers
        private static string FormatDistance(double meters)
        {
            if (double.IsNaN(meters)) return "---";
            if (Math.Abs(meters) >= 1000.0) return (meters / 1000.0).ToString("F1") + " km";
            return meters.ToString("F1") + " m";
        }

        private static string FormatSpeed(double mps)
        {
            if (double.IsNaN(mps)) return "---";
            return mps.ToString("F1") + " m/s";
        }

        private static string FormatDeltaV(double mps)
        {
            if (double.IsNaN(mps)) return "---";
            return mps.ToString("F0") + " m/s";
        }

        private static string FormatAngle(double deg)
        {
            if (double.IsNaN(deg)) return "---";
            return deg.ToString("F2") + "°";
        }

        private static string FormatTime(double seconds)
        {
            if (double.IsNaN(seconds)) return "---";
            if (seconds < 0) seconds = 0;
            return KSPUtil.PrintTimeCompact(seconds, false);
        }

        private double GetTimeToApoapsis()
        {
            if (vessel == null) return 0;
            double ut = vessel.orbit.NextApoapsisTime(Planetarium.GetUniversalTime());
            return ut - Planetarium.GetUniversalTime();
        }

        private double GetTimeToPeriapsis()
        {
            if (vessel == null) return 0;
            double ut = vessel.orbit.NextPeriapsisTime(Planetarium.GetUniversalTime());
            return ut - Planetarium.GetUniversalTime();
        }

        private string GetStageDeltaVText(object core, bool vacuum)
        {
            var stats = vacuum ? MechJebProxy.GetVacuumStageStats(core) : MechJebProxy.GetAtmoStageStats(core);
            if (stats == null || stats.Count == 0) return "---";
            double dv = MechJebProxy.GetStageDeltaV(stats[0]);
            return FormatDeltaV(dv);
        }

        private string GetTargetDistanceText()
        {
            if (vessel == null || FlightGlobals.fetch.VesselTarget == null) return "---";
            ITargetable target = FlightGlobals.fetch.VesselTarget;
            Vector3d tgtPos = target.GetTransform().position;
            return FormatDistance(Vector3d.Distance(vessel.GetWorldPos3D(), tgtPos));
        }

        private string GetTargetRelVelText()
        {
            if (vessel == null || FlightGlobals.fetch.VesselTarget == null) return "---";
            Orbit targetOrbit = FlightGlobals.fetch.VesselTarget.GetOrbit();
            if (targetOrbit == null) return "---";
            double ut = Planetarium.GetUniversalTime();
            Vector3d v1 = vessel.orbit.SwappedOrbitalVelocityAtUT(ut);
            Vector3d v2 = targetOrbit.SwappedOrbitalVelocityAtUT(ut);
            return FormatSpeed((v1 - v2).magnitude);
        }

        private string GetTargetClosestApproachText()
        {
            if (vessel == null || FlightGlobals.fetch.VesselTarget == null) return "---";
            Orbit targetOrbit = FlightGlobals.fetch.VesselTarget.GetOrbit();
            if (targetOrbit == null) return "---";
            double dist = vessel.orbit.NextClosestApproachDistance(targetOrbit, Planetarium.GetUniversalTime());
            return FormatDistance(dist);
        }

        private string GetTargetTimeToClosestText()
        {
            if (vessel == null || FlightGlobals.fetch.VesselTarget == null) return "---";
            Orbit targetOrbit = FlightGlobals.fetch.VesselTarget.GetOrbit();
            if (targetOrbit == null) return "---";
            double ut = vessel.orbit.NextClosestApproachTime(targetOrbit, Planetarium.GetUniversalTime());
            return FormatTime(ut - Planetarium.GetUniversalTime());
        }

        private string GetTargetRelInclinationText()
        {
            if (vessel == null || FlightGlobals.fetch.VesselTarget == null) return "---";
            Orbit targetOrbit = FlightGlobals.fetch.VesselTarget.GetOrbit();
            if (targetOrbit == null) return "---";
            double rel = Vector3d.Angle(vessel.orbit.GetOrbitNormal(), targetOrbit.GetOrbitNormal());
            return rel.ToString("F2") + "°";
        }

        private string GetVesselMassText()
        {
            if (vessel == null) return "---";
            return vessel.GetTotalMass().ToString("F2") + " t";
        }

        private string GetVesselTwrText()
        {
            if (vessel == null) return "---";
            double g = vessel.mainBody != null ? vessel.mainBody.GeeASL * 9.80665 : 9.80665;
            double thrust = GetMaxThrust();
            double mass = vessel.GetTotalMass();
            if (mass <= 0) return "---";
            return (thrust / (mass * g)).ToString("F2");
        }

        private string GetVesselMaxThrustText()
        {
            if (vessel == null) return "---";
            return GetMaxThrust().ToString("F0") + " kN";
        }

        private string GetVesselCurrentThrustText()
        {
            if (vessel == null) return "---";
            return GetCurrentThrust().ToString("F0") + " kN";
        }

        private double GetSurfaceHeading()
        {
            if (vessel == null || vessel.ReferenceTransform == null) return 0;
            return vessel.ReferenceTransform.rotation.eulerAngles.y;
        }

        private double GetMaxThrust()
        {
            if (vessel == null) return 0;
            double max = 0;
            var engines = vessel.FindPartModulesImplementing<ModuleEngines>();
            for (int i = 0; i < engines.Count; i++)
            {
                ModuleEngines engine = engines[i];
                if (engine == null) continue;
                double limiter = engine.thrustPercentage / 100.0;
                max += engine.maxThrust * limiter;
            }
            return max;
        }

        private double GetCurrentThrust()
        {
            if (vessel == null) return 0;
            double current = 0;
            var engines = vessel.FindPartModulesImplementing<ModuleEngines>();
            for (int i = 0; i < engines.Count; i++)
            {
                ModuleEngines engine = engines[i];
                if (engine == null) continue;
                current += engine.finalThrust;
            }
            return current;
        }
        #endregion

        #region Button Handlers
        public void PageActive(bool active, int pageNumber)
        {
            pageActiveState = active;
        }

        // Alias for compatibility with configs that use ClickProcessor
        public void ClickProcessor(int buttonID)
        {
            ButtonProcessor(buttonID);
        }

        public void ButtonProcessor(int buttonID)
        {
            if (!pageActiveState || currentMenu == null) return;

            if (buttonID == buttonUp)
            {
                currentMenu.PreviousItem();
            }
            else if (buttonID == buttonDown)
            {
                currentMenu.NextItem();
            }
            else if (buttonID == buttonEnter)
            {
                currentMenu.SelectItem();
                UpdateTrackedItems();
            }
            else if (buttonID == buttonEsc)
            {
                PopMenu();
            }
            else if (buttonID == buttonHome)
            {
                GoHome();
            }
            else if (buttonID == buttonRight)
            {
                // For value items, increase
                IncrementCurrentValue(1);
            }
            else if (buttonID == buttonLeft)
            {
                // For value items, decrease
                IncrementCurrentValue(-1);
            }
        }

        private void IncrementCurrentValue(int direction)
        {
            if (mjCore == null || currentMenu == null) return;

            TextMenu.Item currentItem = currentMenu.GetCurrentItem();
            if (currentItem == null) return;

            for (int i = 0; i < trackedItems.Count; i++)
            {
                TrackedMenuItem tracked = trackedItems[i];
                if (tracked.item == currentItem && tracked.isValueItem && tracked.getNumber != null && tracked.setNumber != null)
                {
                    double current = tracked.getNumber(mjCore);
                    double next = current + (tracked.step * direction);

                    if (tracked.hasMin && next < tracked.min) next = tracked.min;
                    if (tracked.hasMax && next > tracked.max) next = tracked.max;

                    tracked.setNumber(mjCore, next);
                    UpdateTrackedItems();
                    break;
                }
            }
        }
        #endregion

        #region Render
        public string ShowMenu(int screenWidth, int screenHeight)
        {
            if (!MechJebProxy.IsAvailable)
            {
                return "MechJeb not available\n\n" + (MechJebProxy.InitializationError ?? "Unknown error");
            }

            if (mjCore == null)
            {
                return "No MechJeb core found on this vessel";
            }

            UpdateTrackedItems();

            return pageTitle + Environment.NewLine + currentMenu.ShowMenu(screenWidth, screenHeight - 1);
        }
        #endregion
    }
}
