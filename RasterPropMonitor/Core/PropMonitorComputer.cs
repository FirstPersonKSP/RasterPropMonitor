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
    public class RasterPropMonitorComputer : PartModule
    {
        // The only public configuration variable.
        [KSPField]
        public bool debugLogging = false;

        // The OTHER public configuration variable.
        [KSPField]
        public string storedStrings = string.Empty;

        [KSPField]
        public string triggeredEvents = string.Empty;

        // Yes, it's a really braindead way of doing it, but I ran out of elegant ones,
        // because nothing appears to work as documented -- IF it's documented.
        // This one is sure to work and isn't THAT much of a performance drain, really.
        // Pull requests welcome
        // Vessel description storage and related code.
        [KSPField(isPersistant = true)]
        public string vesselDescription = string.Empty;
        private string vesselDescriptionForDisplay = string.Empty;
        private readonly string editorNewline = ((char)0x0a).ToString();
        private string lastVesselDescription = string.Empty;

        // Public functions:
        // Request the instance, create it if one doesn't exist:
        public static RasterPropMonitorComputer Instantiate(MonoBehaviour referenceLocation)
        {
            var thatProp = referenceLocation as InternalProp;
            var thatPart = referenceLocation as Part;
            if (thatPart == null)
            {
                if (thatProp == null)
                {
                    throw new ArgumentException("Cannot instantiate RPMC in this location.");
                }
                thatPart = thatProp.part;
            }
            for (int i = 0; i < thatPart.Modules.Count; i++)
            {
                if (thatPart.Modules[i].ClassName == typeof(RasterPropMonitorComputer).Name)
                {
                    return thatPart.Modules[i] as RasterPropMonitorComputer;
                }
            }
            return thatPart.AddModule(typeof(RasterPropMonitorComputer).Name) as RasterPropMonitorComputer;
        }

        // Page handler interface for vessel description page.
        // Analysis disable UnusedParameter
        public string VesselDescriptionRaw(int screenWidth, int screenHeight)
        {
            // Analysis restore UnusedParameter
            return vesselDescriptionForDisplay.UnMangleConfigText();
        }

        // Analysis disable UnusedParameter
        public string VesselDescriptionWordwrapped(int screenWidth, int screenHeight)
        {
            // Analysis restore UnusedParameter
            return JUtil.WordWrap(vesselDescriptionForDisplay.UnMangleConfigText(), screenWidth);
        }


        public void Start()
        {
            JUtil.LogMessage(this, "Setting RasterPropMonitor debugging to {0}", debugLogging);
            if (debugLogging)
            {
                // Allow opt-in; do not allow it to be shut off if someone wanted it on.
                JUtil.debugLoggingEnabled = debugLogging;
            }

            if (!HighLogic.LoadedSceneIsEditor)
            {
                RPMVesselComputer comp = RPMVesselComputer.Instance(vessel);
                comp.SetVesselDescription(vesselDescription);

                // Make sure we have the description strings parsed.
                string[] descriptionStrings = vesselDescription.UnMangleConfigText().Split(JUtil.LineSeparator, StringSplitOptions.None);
                for (int i = 0; i < descriptionStrings.Length; i++)
                {
                    if (descriptionStrings[i].StartsWith("AG", StringComparison.Ordinal) && descriptionStrings[i][3] == '=')
                    {
                        uint groupID;
                        if (uint.TryParse(descriptionStrings[i][2].ToString(), out groupID))
                        {
                            descriptionStrings[i] = string.Empty;
                        }
                    }
                }
                vesselDescriptionForDisplay = string.Join(Environment.NewLine, descriptionStrings).MangleConfigText();
                if (string.IsNullOrEmpty(vesselDescriptionForDisplay)) {
                    vesselDescriptionForDisplay = " "; // Workaround for issue #466.
                }

                // Now let's parse our stored strings...
                if (!string.IsNullOrEmpty(storedStrings))
                {
                    var storedStringsArray = storedStrings.Split('|');
                    comp.SetStoredStrings(storedStringsArray);
                }

                // TODO: If there are triggered events, register for an undock
                // callback so we can void and rebuild the callbacks after undocking.
                // Although it didn't work when I tried it...
                if (!string.IsNullOrEmpty(triggeredEvents))
                {
                    string[] varstring = triggeredEvents.Split('|');
                    for (int i = 0; i < varstring.Length; ++i)
                    {
                        comp.AddTriggeredEvent(varstring[i].Trim());
                    }
                }
            }
        }

        public void Update()
        {
            if (HighLogic.LoadedSceneIsEditor)
            {
                // well, it looks sometimes it might become null..
                string s = EditorLogic.fetch.shipDescriptionField != null ? EditorLogic.fetch.shipDescriptionField.text : string.Empty;
                if (s != lastVesselDescription)
                {
                    lastVesselDescription = s;
                    // For some unclear reason, the newline in this case is always 0A, rather than Environment.NewLine.
                    vesselDescription = s.Replace(editorNewline, "$$$");
                }
            }
        }
    }
}
