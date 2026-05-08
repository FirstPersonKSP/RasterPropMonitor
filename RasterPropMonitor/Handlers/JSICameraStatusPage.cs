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
using System.Text;
using UnityEngine;

namespace JSI
{
    /// <summary>
    /// Page handler that reports camera status from the current page's background
    /// JSISteerableCamera instance. This avoids global PLUGIN_*/PERSISTENT_*
    /// resolution ambiguity when multiple MFDs exist in the same cockpit.
    /// </summary>
    public class JSICameraStatusPage : InternalModule
    {
        private const string CameraStatusToken = "__CAMERA_STATUS__";
        private static readonly string DefaultOverlayTemplate = @"[font0][@y+2][hw]         __CAMERA_STATUS__";

        [KSPField]
        public string prefix = "";
        [KSPField]
        public string idLabel = "ID";
        [KSPField]
        public string nameLabel = "CAMERA";
        [KSPField]
        public string unknownText = "N/A";
        [KSPField]
        public bool showId = true;
        [KSPField]
        public bool showName = true;
        [KSPField]
        public string statusColor = "[#FFFF0066]";
        [KSPField]
        public string statusSuffix = "[#FFFFFF55]";
        [KSPField]
        public string overlayTemplate = string.Empty;
        [KSPField]
        public string overlayTemplatePath = string.Empty;
        [KSPField]
        public string cameraStatusToken = CameraStatusToken;

        private JSISteerableCamera steerableCamera;
        private string resolvedTemplate;

        public void GetHandlerReferences(MonoBehaviour pageHandler, MonoBehaviour backgroundHandler)
        {
            steerableCamera = backgroundHandler as JSISteerableCamera;
        }

        // Analysis disable UnusedParameter
        public string ShowStatus(int width, int height)
        {
            if (steerableCamera == null)
            {
                // Fallback: if handler references were not wired for some reason,
                // try to resolve a camera from the current prop only.
                foreach (InternalModule module in internalProp.internalModules)
                {
                    steerableCamera = module as JSISteerableCamera;
                    if (steerableCamera != null)
                    {
                        break;
                    }
                }
            }

            string statusLine = BuildStatusLine();
            string template = GetTemplate();
            return InjectStatusLine(template, statusLine);
        }
        // Analysis restore UnusedParameter

        private string GetTemplate()
        {
            if (!string.IsNullOrEmpty(resolvedTemplate))
            {
                return resolvedTemplate;
            }

            if (!string.IsNullOrEmpty(overlayTemplate))
            {
                resolvedTemplate = overlayTemplate;
                return resolvedTemplate;
            }

            if (!string.IsNullOrEmpty(overlayTemplatePath))
            {
                resolvedTemplate = JUtil.LoadPageDefinition(overlayTemplatePath);
                if (!string.IsNullOrEmpty(resolvedTemplate))
                {
                    return resolvedTemplate;
                }
            }

            resolvedTemplate = DefaultOverlayTemplate;
            return resolvedTemplate;
        }

        private string InjectStatusLine(string template, string statusLine)
        {
            string token = string.IsNullOrEmpty(cameraStatusToken) ? CameraStatusToken : cameraStatusToken;

            if (template.Contains(token))
            {
                return template.Replace(token, statusLine);
            }

            // Final fallback if no marker is present.
            return statusLine + "\n" + template;
        }

        private string BuildStatusLine()
        {
            if (steerableCamera == null)
            {
                return string.Format("{0}{1} {2}{3}", statusColor, prefix, unknownText, statusSuffix);
            }

            double activeId = steerableCamera.GetActiveCameraID();
            string activeName = steerableCamera.GetActiveCameraTransform();
            bool validId = activeId >= 1.0;
            if (string.IsNullOrEmpty(activeName))
            {
                activeName = unknownText;
            }

            StringBuilder sb = new StringBuilder();
            sb.Append(statusColor);
            sb.Append(prefix);

            if (showId)
            {
                sb.Append(' ');
                sb.Append(idLabel);
                sb.Append(':');
                sb.Append(validId ? ((int)activeId).ToString() : unknownText);
            }

            if (showName)
            {
                if (showId)
                {
                    sb.Append(' ');
                }
                sb.Append(nameLabel);
                sb.Append(':');
                sb.Append(validId ? activeName : unknownText);
            }

            sb.Append(statusSuffix);
            return sb.ToString();
        }
    }
}
