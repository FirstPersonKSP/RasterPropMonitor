﻿/*****************************************************************************
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
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using Expansions.Missions;
using KSP.UI.Screens.Flight;
using UnityEngine;
using UnityEngine.Profiling;

namespace JSI
{
    public static class MapIcons
    {
        public enum OtherIcon
        {
            None,
            PE,
            AP,
            AN,
            DN,
            NODE,
            SHIPATINTERCEPT,
            TGTATINTERCEPT,
            ENTERSOI,
            EXITSOI,
            PLANET,
        }

        public static Rect VesselTypeIcon(VesselType type, OtherIcon icon)
        {
            int x = 0;
            int y = 0;
            const float symbolSpan = 0.2f;
            if (icon != OtherIcon.None)
            {
                switch (icon)
                {
                    case OtherIcon.AP:
                        x = 1;
                        y = 4;
                        break;
                    case OtherIcon.PE:
                        x = 0;
                        y = 4;
                        break;
                    case OtherIcon.AN:
                        x = 2;
                        y = 4;
                        break;
                    case OtherIcon.DN:
                        x = 3;
                        y = 4;
                        break;
                    case OtherIcon.NODE:
                        x = 2;
                        y = 1;
                        break;
                    case OtherIcon.SHIPATINTERCEPT:
                        x = 0;
                        y = 1;
                        break;
                    case OtherIcon.TGTATINTERCEPT:
                        x = 1;
                        y = 1;
                        break;
                    case OtherIcon.ENTERSOI:
                        x = 0;
                        y = 2;
                        break;
                    case OtherIcon.EXITSOI:
                        x = 1;
                        y = 2;
                        break;
                    case OtherIcon.PLANET:
                        // Not sure if it is (2,3) or (3,2) - both are round
                        x = 2;
                        y = 3;
                        break;
                }
            }
            else
            {
                switch (type)
                {
                    case VesselType.Debris:
                        x = 1;
                        y = 3;
                        break;
                    case VesselType.SpaceObject:
                        x = 4;
                        y = 1;
                        break;
                    case VesselType.Unknown:
                        x = 3;
                        y = 3;
                        break;
                    case VesselType.Probe:
                        x = 1;
                        y = 0;
                        break;
					case VesselType.Relay:
						x = 4;
						y = 3;
						break;
                    case VesselType.Rover:
                        x = 0;
                        y = 0;
                        break;
                    case VesselType.Lander:
                        x = 3;
                        y = 0;
                        break;
                    case VesselType.Ship:
                        x = 0;
                        y = 3;
						break;
					case VesselType.Plane:
						x = 4;
						y = 4;
						break;
                    case VesselType.Station:
                        x = 3;
                        y = 1;
                        break;
					case VesselType.Base:
                        x = 2;
                        y = 0;
                        break;
                    case VesselType.EVA:
                        x = 2;
                        y = 2;
                        break;
                    case VesselType.Flag:
                        x = 4;
                        y = 0;
                        break;
                    default:
                        x = 4;
                        y = 2;
                        break;
                }
            }
            var result = new Rect();
            result.x = symbolSpan * x;
            result.y = symbolSpan * y;
            result.height = result.width = symbolSpan;
            return result;
        }
    }

    public static class GizmoIcons
    {
        public enum IconType
        {
            PROGRADE,
            RETROGRADE,
            MANEUVERPLUS,
            MANEUVERMINUS,
            TARGETPLUS,
            TARGETMINUS,
            NORMALPLUS,
            NORMALMINUS,
            RADIALPLUS,
            RADIALMINUS,
        };

        public static Rect GetIconLocation(IconType type)
        {
            Rect loc = new Rect(0.0f, 0.0f, 1.0f / 3.0f, 1.0f / 3.0f);
            switch (type)
            {
                case IconType.PROGRADE:
                    loc.x = 0.0f / 3.0f;
                    loc.y = 2.0f / 3.0f;
                    break;
                case IconType.RETROGRADE:
                    loc.x = 1.0f / 3.0f;
                    loc.y = 2.0f / 3.0f;
                    break;
                case IconType.MANEUVERPLUS:
                    loc.x = 2.0f / 3.0f;
                    loc.y = 0.0f / 3.0f;
                    break;
                case IconType.MANEUVERMINUS:
                    loc.x = 1.0f / 3.0f;
                    loc.y = 2.0f / 3.0f;
                    break;
                case IconType.TARGETPLUS:
                    loc.x = 2.0f / 3.0f;
                    loc.y = 2.0f / 3.0f;
                    break;
                case IconType.TARGETMINUS:
                    loc.x = 2.0f / 3.0f;
                    loc.y = 1.0f / 3.0f;
                    break;
                case IconType.NORMALPLUS:
                    loc.x = 0.0f / 3.0f;
                    loc.y = 0.0f / 3.0f;
                    break;
                case IconType.NORMALMINUS:
                    loc.x = 1.0f / 3.0f;
                    loc.y = 0.0f / 3.0f;
                    break;
                case IconType.RADIALPLUS:
                    loc.x = 1.0f / 3.0f;
                    loc.y = 1.0f / 3.0f;
                    break;
                case IconType.RADIALMINUS:
                    loc.x = 0.0f / 3.0f;
                    loc.y = 1.0f / 3.0f;
                    break;
            }

            return loc;
        }
    }

    public static class JUtil
    {
        public static readonly string[] VariableListSeparator = { "$&$" };
        public static readonly string[] VariableSeparator = { };
        public static readonly string[] LineSeparator = { Environment.NewLine };
        private static readonly int ClosestApproachRefinementInterval = 16;
        internal static Dictionary<string, Shader> parsedShaders = new Dictionary<string, Shader>();
        internal static Dictionary<string, Font> loadedFonts = new Dictionary<string, Font>();
        internal static Dictionary<string, Color32> globalColors = new Dictionary<string, Color32>();

        internal static GameObject CreateSimplePlane(string name, float vectorSize, int drawingLayer)
        {
            return CreateSimplePlane(name, new Vector2(vectorSize, vectorSize), new Rect(0.0f, 0.0f, 1.0f, 1.0f), drawingLayer);
        }

        internal static GameObject CreateSimplePlane(string name, Vector2 vectorSize, Rect textureCoords, int drawingLayer)
        {
            var mesh = new Mesh();

            var obj = new GameObject(name);
            MeshFilter meshFilter = obj.AddComponent<MeshFilter>();
            obj.AddComponent<MeshRenderer>();

            mesh.vertices = new[] 
            {
                new Vector3(-vectorSize.x, -vectorSize.y, 0.0f), 
                new Vector3(vectorSize.x, -vectorSize.y, 0.0f), 
                new Vector3(-vectorSize.x, vectorSize.y, 0.0f),
                new Vector3(vectorSize.x, vectorSize.y, 0.0f)
            };

            mesh.uv = new[] 
            {
                new Vector2(textureCoords.xMin, textureCoords.yMin), 
                new Vector2(textureCoords.xMax, textureCoords.yMin), 
                new Vector2(textureCoords.xMin, textureCoords.yMax),
                new Vector2(textureCoords.xMax, textureCoords.yMax)
            };

            mesh.triangles = new[] 
            {
                1, 0, 2,
                3, 1, 2
            };

            mesh.RecalculateBounds();

            meshFilter.mesh = mesh;

            obj.layer = drawingLayer;

            UnityEngine.Object.Destroy(obj.GetComponent<Collider>());

            return obj;
        }

        internal static Shader LoadInternalShader(string shaderName)
        {
            if (!parsedShaders.ContainsKey(shaderName))
            {
                JUtil.LogErrorMessage(null, "Failed to find shader {0}", shaderName);
                return null;
            }
            else
            {
                return parsedShaders[shaderName];
            }
        }

        internal static void Swap<T>(ref T a, ref T b)
        {
            T temp = a;
            a = b;
            b = temp;
        }

        // https://forum.unity.com/threads/camera-render-seems-to-trigger-canvas-sendwillrendercanvases.462099/
        static FieldInfo canvasHackField = typeof(Canvas).GetField("willRenderCanvases", BindingFlags.NonPublic | BindingFlags.Static);
        internal static void RenderTextureCamera(Camera camera)
        {
            var canvasHackObject = canvasHackField.GetValue(null);
            canvasHackField.SetValue(null, null);
            camera.Render();
            canvasHackField.SetValue(null, canvasHackObject);
        }

        /// <summary>
        /// Parse a config file color string into a Color32.  The colorString
        /// parameter is a sequnce of R, G, B, A (ranging [0,255]), or it is a
        /// string prefixed with "COLOR_".  In the latter case, we'll look up
        /// the color from config files specified in the parent part's
        /// RasterPropMonitorComputer module, or from a globally-defined color
        /// table.
        /// </summary>
        /// <param name="colorString">The color string to parse.</param>
        /// <param name="rpmComp">The rpmComp for the specified part; if null,
        /// ParseColor32 looks up the RPMC module.</param>
        /// 
        /// <returns>Color32; white if colorString is empty, obnoxious magenta
        /// if an unknown COLOR_ string is provided.</returns>
        internal static Color32 ParseColor32(string colorString, RasterPropMonitorComputer rpmComp)
        {
            if (string.IsNullOrEmpty(colorString))
            {
                return Color.white;
            }

            colorString = colorString.Trim();
            if (colorString.StartsWith("COLOR_"))
            {
                if (rpmComp != null && rpmComp.overrideColors.ContainsKey(colorString))
                {
                    return rpmComp.overrideColors[colorString];
                }

                if (globalColors.ContainsKey(colorString))
                {
                    return globalColors[colorString];
                }
                else
                {
                    JUtil.LogErrorMessage(null, "Unrecognized color '{0}' in ParseColor32", colorString);
                    return new Color32(255, 0, 255, 255);
                }
            }
            else
            {
                return ConfigNode.ParseColor32(colorString);
            }
        }

        internal static void ShowHide(bool status, params GameObject[] objects)
        {
            for (int i = 0; i < objects.Length; ++i)
            {
                if (objects[i] != null)
                {
                    objects[i].SetActive(status);
                    Renderer renderer = null;
                    objects[i].GetComponentCached<Renderer>(ref renderer);
                    if (renderer != null)
                    {
                        renderer.enabled = status;
                    }
                }
            }
        }

        public static bool ValueChanged(double oldValue, double newValue)
        {
            if (double.IsNaN(oldValue) != double.IsNaN(newValue) || Math.Abs(newValue - oldValue) > 1e-4)
            {
                return true;
            }
            return false;
        }

        public static Vector3d SwizzleXZY(this Vector3d vector)
        {
            return new Vector3d(vector.x, vector.z, vector.y);
        }

        public static void MakeReferencePart(this Part thatPart)
        {
            if (thatPart != null)
            {
                foreach (PartModule thatModule in thatPart.Modules)
                {
                    var thatNode = thatModule as ModuleDockingNode;
                    var thatPod = thatModule as ModuleCommand;
                    var thatClaw = thatModule as ModuleGrappleNode;
                    if (thatNode != null)
                    {
                        thatNode.MakeReferenceTransform();
                        break;
                    }
                    if (thatPod != null)
                    {
                        thatPod.MakeReference();
                        break;
                    }
                    if (thatClaw != null)
                    {
                        thatClaw.MakeReferenceTransform();
                    }
                }
            }
        }
        /* I wonder why this isn't working. 
         * It's like the moment I unseat a kerbal, no matter what else I do,
         * the entire internal goes poof. Although I'm pretty sure it doesn't quite,
         * because the modules keep working and generating errors.
         * What's really going on here, and why the same thing works for Crew Manifest?
        public static void ReseatKerbalInPart(this Kerbal thatKerbal) {
            if (thatKerbal.InPart == null || !JUtil.VesselIsInIVA(thatKerbal.InPart.vessel))
                return;

            InternalModel thatModel = thatKerbal.InPart.internalModel;
            Part thatPart = thatKerbal.InPart;
            int spareSeat = thatModel.GetNextAvailableSeatIndex();
            if (spareSeat >= 0) {
                ProtoCrewMember crew = thatKerbal.protoCrewMember;
                CameraManager.Instance.SetCameraFlight();
                thatPart.internalModel.UnseatKerbal(crew);
                thatPart.internalModel.SitKerbalAt(crew,thatPart.internalModel.seats[spareSeat]);
                thatPart.internalModel.part.vessel.SpawnCrew();
                CameraManager.Instance.SetCameraIVA(thatPart.internalModel.seats[spareSeat].kerbalRef,true);
            }
        }
        */

        /// <summary>
        /// Returns true if the active Kerbal is in the specified part.
        /// </summary>
        /// <param name="thisPart"></param>
        /// <returns></returns>
        public static bool ActiveKerbalIsLocal(this Part thisPart)
        {
            Kerbal thatKerbal = CameraManager.Instance.IVACameraActiveKerbal;
            if (thatKerbal != null)
            {
                return CameraManager.Instance.activeInternalPart == thisPart;
            }
            else
            {
                return false;
            }
        }

        /// <summary>
        /// Returns the index of the active seat in the current part, or -1 if
        /// there is none.
        /// </summary>
        /// <param name="thisPart"></param>
        /// <returns></returns>
        public static int CurrentActiveSeat(this Part thisPart)
        {
            Kerbal activeKerbal = CameraManager.Instance.IVACameraActiveKerbal;
            if (activeKerbal != null)
            {
                return (CameraManager.Instance.activeInternalPart == thisPart) ? activeKerbal.protoCrewMember.seatIdx : -1;
            }
            else
            {
                return -1;
            }
        }

        public static void HideShowProp(InternalProp thatProp, bool visibility)
        {
            foreach (Renderer thatRenderer in thatProp.FindModelComponents<MeshRenderer>())
            {
                thatRenderer.enabled = visibility;
            }
            foreach (Renderer thatRenderer in thatProp.FindModelComponents<SkinnedMeshRenderer>())
            {
                thatRenderer.enabled = visibility;
            }
        }

        public static Camera GetCameraByName(string name)
        {
            Camera[] allCameras = Camera.allCameras;
            for (int i = 0; i < allCameras.Length; ++i)
            {
                if (allCameras[i].name == name)
                {
                    return allCameras[i];
                }
            }
            return null;
        }

        public static void RemoveAllNodes(PatchedConicSolver solver)
        {
            // patchedConicSolver can be null in early career mode.
            if (solver != null)
            {
                while (solver.maneuverNodes.Count > 0)
                {
                    solver.maneuverNodes.Last().RemoveSelf();
                }
            }
        }

        internal static void DisposeOfGameObjects(GameObject[] objs)
        {
            for (int i = 0; i < objs.Length; ++i)
            {
                if (objs[i] != null)
                {
                    MeshFilter meshFilter = objs[i].GetComponent<MeshFilter>();
                    if (meshFilter != null)
                    {
                        UnityEngine.Object.Destroy(meshFilter.mesh);
                        UnityEngine.Object.Destroy(meshFilter);
                    }
                    UnityEngine.Object.Destroy(objs[i].GetComponent<Renderer>().material);
                    UnityEngine.Object.Destroy(objs[i]);
                }
            }
        }

        internal static bool DoesCameraExist(string name)
        {
            Camera[] allCameras = Camera.allCameras;
            for (int i = 0; i < allCameras.Length; ++i)
            {
                if (allCameras[i].name == name)
                {
                    return true;
                }
            }

            return false;
        }

        public static Material DrawLineMaterial()
        {
            var lineMaterial = new Material(LoadInternalShader("RPM/FontShader"));
            lineMaterial.hideFlags = HideFlags.HideAndDontSave;
            lineMaterial.shader.hideFlags = HideFlags.HideAndDontSave;
            return lineMaterial;
        }

        public static Texture2D GetGizmoTexture()
        {
            if (HighLogic.LoadedSceneIsFlight)
            {
                // This clever method at getting at the stock texture asset originates in Enhanced Navball.
                ManeuverGizmo maneuverGizmo = MapView.ManeuverNodePrefab.GetComponent<ManeuverGizmo>();
                ManeuverGizmoHandle maneuverGizmoHandle = maneuverGizmo.handleNormal;
                Transform gizmoTransform = maneuverGizmoHandle.flag;
                Renderer gizmoRenderer = gizmoTransform.GetComponent<Renderer>();
                return (Texture2D)gizmoRenderer.sharedMaterial.mainTexture;
            }

            return null;
        }

        public static void AnnoyUser(object caller)
        {
            LogErrorMessage(caller, "INITIALIZATION ERROR, CHECK CONFIGURATION.");
            ScreenMessages.PostScreenMessage(string.Format("{0}: INITIALIZATION ERROR, CHECK CONFIGURATION.", caller.GetType().Name), 120, ScreenMessageStyle.UPPER_CENTER);
        }

        /// <summary>
        /// Utility function to find a JSIFlashModule configured for a particular
        /// refresh rate on a given part, and to create one if it doesn't already
        /// exist.
        /// </summary>
        /// <param name="part">The part where the module will be installed</param>
        /// <param name="flashRate">The flash rate for the module.</param>
        /// <returns></returns>
        public static JSIFlashModule InstallFlashModule(Part part, float flashRate)
        {
            JSIFlashModule[] loadedModules = part.GetComponents<JSIFlashModule>();
            for (int i = 0; i < loadedModules.Length; ++i)
            {
                if (loadedModules[i].flashRate == flashRate)
                {
                    return loadedModules[i];
                }
            }

            JSIFlashModule newModule = part.AddModule("JSIFlashModule") as JSIFlashModule;
            newModule.flashRate = flashRate;

            return newModule;
        }

        /// <summary>
        /// Try to figure out which part on the craft is the current part.
        /// </summary>
        /// <returns></returns>
        public static Part DeduceCurrentPart(Vessel vessel)
        {
            Part currentPart = null;

            if (JUtil.VesselIsInIVA(vessel))
            {
                Kerbal thatKerbal = CameraManager.Instance.IVACameraActiveKerbal;
                if (thatKerbal != null)
                {
                    // This should be a drastically faster way to determine
                    // where we are.  I hope.
                    currentPart = thatKerbal.InPart;
                }

                if (currentPart == null && CameraManager.Instance.currentCameraMode == CameraManager.CameraMode.Internal)
                {
                    Transform internalCameraTransform = InternalCamera.Instance.transform;
                    foreach (Part thisPart in InternalModelParts(vessel))
                    {
                        for (int seatIdx = 0; seatIdx < thisPart.internalModel.seats.Count; ++seatIdx)
                        {
                            if (thisPart.internalModel.seats[seatIdx].kerbalRef != null)
                            {
                                if (thisPart.internalModel.seats[seatIdx].kerbalRef.eyeTransform == internalCameraTransform.parent)
                                {
                                    currentPart = thisPart;
                                    break;
                                }
                            }
                        }

                        if (CameraManager.Instance.currentCameraMode == CameraManager.CameraMode.Internal)
                        {
                            Transform[] modelTransforms = thisPart.internalModel.GetComponentsInChildren<Transform>();
                            for (int xformIdx = 0; xformIdx < modelTransforms.Length; ++xformIdx)
                            {
                                if (modelTransforms[xformIdx] == InternalCamera.Instance.transform.parent)
                                {
                                    currentPart = thisPart;
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            return currentPart;
        }

        /// <summary>
        /// Iterate over the parts of a vessel and return only those that
        /// contain an internal model that has been instantiated.
        /// </summary>
        /// <param name="vessel"></param>
        /// <returns></returns>
        private static IEnumerable<Part> InternalModelParts(Vessel vessel)
        {
            for (int i = 0; i < vessel.parts.Count; ++i)
            {
                if (vessel.parts[i].internalModel != null)
                {
                    yield return vessel.parts[i];
                }
            }
        }

        public static bool RasterPropMonitorShouldUpdate(Part part)
        {
            if (HighLogic.LoadedSceneIsFlight)
            {
                if (IsActiveVessel(part.vessel))
                {
                    return UserIsInPod(part) || StockOverlayCamIsOn();
                }
                else
                {
                    // TODO: Under what circumstances would I set this to true?
                    // Since the computer module is a VesselModule, it's a per-
                    // craft module, so I think it's safe to update other pods
                    // while StockOverlayCamIsOn is true.
                    return false;
                }
            }
            else
            {
                return false;
            }
        }

        /// <summary>
        /// Returns true if the vessel is the current vessel, we're in flight,
        /// and we're in IVA.
        /// </summary>
        /// <param name="thatVessel"></param>
        /// <returns></returns>
        public static bool VesselIsInIVA(Vessel thatVessel)
        {
            // Inactive IVAs are renderer.enabled = false, this can and should be used...
            // ... but now it can't because we're doing transparent pods, so we need a more complicated way to find which pod the player is in.
            return HighLogic.LoadedSceneIsFlight && IsActiveVessel(thatVessel) && IsInIVA();
        }
        
        public static bool StockOverlayCamIsOn()
        {
            return KerbalPortraitGallery.isIVAOverlayVisible;
        }

        public static bool UserIsInPod(Part thisPart)
        {

            // Just in case, check for whether we're not in flight.
            if (!HighLogic.LoadedSceneIsFlight)
            {
                return false;
            }

            // If we're not in IVA, or the part does not have an instantiated IVA, the user can't be in it.
            if (thisPart.internalModel == null || !VesselIsInIVA(thisPart.vessel))
            {
                return false;
            }

            // Now that we got that out of the way, we know that the user is in SOME pod on our ship. We just don't know which.
            // Let's see if he's controlling a kerbal in our pod.
            if (ActiveKerbalIsLocal(thisPart))
            {
                return true;
            }

            // There still remains an option of InternalCamera which we will now sort out.
            if (CameraManager.Instance.currentCameraMode == CameraManager.CameraMode.Internal)
            {
                var internalModel = InternalCamera.Instance.GetComponentInParent<InternalModel>();

                if (internalModel != null)
                {
                    return thisPart == internalModel.part;
                }
            }

            return false;
        }

        public static bool IsActiveVessel(Vessel thatVessel)
        {
            return (HighLogic.LoadedSceneIsFlight && thatVessel != null && thatVessel.isActiveVessel);
        }

        public static bool IsInIVA()
        {
            return (CameraManager.Instance != null && (CameraManager.Instance.currentCameraMode == CameraManager.CameraMode.IVA || CameraManager.Instance.currentCameraMode == CameraManager.CameraMode.Internal));
        }

        // LogMessage, but unconditional (logs regardless of debugLoggingEnabled state).
        public static void LogInfo(object caller, string line, params object[] list)
        {
            if (caller != null)
            {
                Debug.Log(String.Format("[" + caller.GetType().Name + "]: " + line, list));
            }
            else
            {
                Debug.Log(String.Format("[RasterPropMonitor]: " + line, list));
            }
        }

        public static void LogMessage(object caller, string line, params object[] list)
        {
            if (RPMGlobals.debugLoggingEnabled)
            {
                string callerName = (caller != null) ? caller.GetType().Name : "RasterPropMonitor";

                if (RPMGlobals.debugShowOnly.Count == 0 || RPMGlobals.debugShowOnly.Contains(callerName))
                {
                    Debug.Log(String.Format("[" + callerName + "]: " + line, list));
                }
            }
        }

        public static void LogErrorMessage(object caller, string line, params object[] list)
        {
            if (caller != null)
            {
                Debug.LogError(String.Format("[" + caller.GetType().Name + "]: " + line, list));
            }
            else
            {
                Debug.LogError(String.Format("[RasterPropMonitor]: " + line, list));
            }
        }

        // Working in a generic to make that a generic function for all numbers is too much work
        public static float DualLerp(Vector2 destRange, Vector2 sourceRange, float value)
        {
            return DualLerp(destRange.x, destRange.y, sourceRange.x, sourceRange.y, value);
        }

        public static float DualLerp(float destMin, float destMax, float sourceMin, float sourceMax, float value)
        {
            if (sourceMin < sourceMax)
            {
                if (value < sourceMin)
                    value = sourceMin;
                else if (value > sourceMax)
                    value = sourceMax;
            }
            else
            {
                if (value < sourceMax)
                    value = sourceMax;
                else if (value > sourceMin)
                    value = sourceMin;
            }
            return (destMax - destMin) * ((value - sourceMin) / (sourceMax - sourceMin)) + destMin;
        }

        public static double DualLerp(double destMin, double destMax, double sourceMin, double sourceMax, double value)
        {
            if (sourceMin < sourceMax)
            {
                if (value < sourceMin)
                    value = sourceMin;
                else if (value > sourceMax)
                    value = sourceMax;
            }
            else
            {
                if (value < sourceMax)
                    value = sourceMax;
                else if (value > sourceMin)
                    value = sourceMin;
            }
            return (destMax - destMin) * ((value - sourceMin) / (sourceMax - sourceMin)) + destMin;
        }

        /// <summary>
        /// Convert a variable to a log10-like value (log10 for values > 1,
        /// pass-through for values [-1, 1], and -log10(abs(value)) for values
        /// values less than -1.  Useful for logarithmic VSI and altitude strips.
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        public static double PseudoLog10(double value)
        {
            if (Math.Abs(value) <= 1.0)
            {
                return value;
            }
            return (1.0 + Math.Log10(Math.Abs(value))) * Math.Sign(value);
        }

        /// <summary>
        /// ibid, just using a float
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        public static float PseudoLog10(float value)
        {
            if (Mathf.Abs(value) <= 1.0f)
            {
                return value;
            }
            return (1.0f + Mathf.Log10(Mathf.Abs(value))) * Mathf.Sign(value);
        }

        public static string LoadPageDefinition(string pageDefinition)
        {
            try
            {
                return string.Join(Environment.NewLine, File.ReadAllLines(KSPUtil.ApplicationRootPath + "GameData/" + pageDefinition.EnforceSlashes(), Encoding.UTF8));
            }
            catch
            {
                return pageDefinition.UnMangleConfigText();
            }
        }

        /// <summary>
        /// Translate a Color32 to a color tag [#rrggbbaa].
        /// </summary>
        /// <param name="color"></param>
        /// <returns></returns>
        public static string ColorToColorTag(Color32 color)
        {
            var result = new StringBuilder();
            result.Append("[");
            result.Append(XKCDColors.ColorTranslator.ToHexA(color));
            result.Append("]");
            return result.ToString();
        }

        public static bool OrbitMakesSense(Vessel thatVessel)
        {
            if (thatVessel == null)
                return false;
            if (thatVessel.situation == Vessel.Situations.FLYING ||
                thatVessel.situation == Vessel.Situations.SUB_ORBITAL ||
                thatVessel.situation == Vessel.Situations.ORBITING ||
                thatVessel.situation == Vessel.Situations.ESCAPING ||
                thatVessel.situation == Vessel.Situations.DOCKED) // Not sure about this last one.
                return true;
            return false;
        }

        public static FXGroup SetupIVASound(InternalProp thatProp, string buttonClickSound, float buttonClickVolume, bool loopState)
        {
            FXGroup audioOutput = null;
            if (!string.IsNullOrEmpty(buttonClickSound))
            {
                buttonClickSound = buttonClickSound.EnforceSlashes();
                audioOutput = new FXGroup("RPM" + thatProp.propID);
                audioOutput.audio = thatProp.gameObject.AddComponent<AudioSource>();
                audioOutput.audio.clip = GameDatabase.Instance.GetAudioClip(buttonClickSound);
                audioOutput.audio.Stop();
                audioOutput.audio.volume = GameSettings.SHIP_VOLUME * buttonClickVolume;
                audioOutput.audio.rolloffMode = AudioRolloffMode.Logarithmic;
                audioOutput.audio.maxDistance = 10f;
                audioOutput.audio.minDistance = 2f;
                audioOutput.audio.dopplerLevel = 0f;
                audioOutput.audio.panStereo = 0f;
                audioOutput.audio.playOnAwake = false;
                audioOutput.audio.loop = loopState;
                audioOutput.audio.pitch = 1f;
            }
            return audioOutput;
        }

        public static string WordWrap(string text, int maxLineLength)
        {
            var sb = new StringBuilder();
            char[] prc = { ' ', ',', '.', '?', '!', ':', ';', '-' };
            char[] ws = { ' ' };

            foreach (string line in text.Split(new[] { Environment.NewLine }, StringSplitOptions.RemoveEmptyEntries))
            {
                int currentIndex;
                int lastWrap = 0;
                do
                {
                    currentIndex = lastWrap + maxLineLength > line.Length ? line.Length : (line.LastIndexOfAny(prc, Math.Min(line.Length - 1, lastWrap + maxLineLength)) + 1);
                    if (currentIndex <= lastWrap)
                        currentIndex = Math.Min(lastWrap + maxLineLength, line.Length);
                    sb.AppendLine(line.Substring(lastWrap, currentIndex - lastWrap).Trim(ws));
                    lastWrap = currentIndex;
                } while (currentIndex < line.Length);
            }
            return sb.ToString();
        }

        public static Vector3d ProjectPositionOntoSurface(this Vessel vessel)
        {
            Vector3d coM = vessel.CoMD;

            double latitude = vessel.mainBody.GetLatitude(coM);
            double longitude = vessel.mainBody.GetLongitude(coM);

            double groundHeight;
            if (vessel.mainBody.pqsController != null)
            {
                groundHeight = vessel.mainBody.pqsController.GetSurfaceHeight(QuaternionD.AngleAxis(longitude, Vector3d.down) *
                QuaternionD.AngleAxis(latitude, Vector3d.forward) *
                Vector3d.right) - vessel.mainBody.pqsController.radius;
            }
            else
            {
                // No pqsController
                groundHeight = 0.0;
            }

            return vessel.mainBody.GetWorldSurfacePosition(latitude, longitude, groundHeight);
        }

        public static Vector3d ProjectPositionOntoSurface(this ITargetable target, CelestialBody vesselBody)
        {
            Vector3d position = target.GetTransform().position;
            CelestialBody body = null;
            if (target.GetVessel() != null)
            {
                body = target.GetVessel().mainBody;

                if (body != vesselBody)
                {
                    return Vector3d.zero;
                }
            }
            else if (vesselBody != null)
            {
                body = vesselBody;
            }

            if (body == null)
            {
                return Vector3d.zero;
            }

            double latitude = body.GetLatitude(position);
            double longitude = body.GetLongitude(position);
            // handy!
            double groundHeight = FinePrint.Utilities.CelestialUtilities.TerrainAltitude(body, latitude, longitude);

            return body.GetWorldSurfacePosition(latitude, longitude, groundHeight);
        }

        // Some snippets from MechJeb...
        public static double ClampDegrees360(double angle)
        {
            angle = angle % 360.0;
            if (angle < 0)
                return angle + 360.0;
            return angle;
        }
        //keeps angles in the range -180 to 180
        public static double ClampDegrees180(double angle)
        {
            angle = ClampDegrees360(angle);
            if (angle > 180)
                angle -= 360;
            return angle;
        }

        public static double ClampRadiansTwoPi(double angle)
        {
            angle = angle % (2 * Math.PI);
            if (angle < 0)
                return angle + 2 * Math.PI;
            return angle;
        }

        public static double ClampRadiansPi(double angle)
        {
            angle = ClampRadiansTwoPi(angle);
            if (angle > Math.PI) angle -= 2.0f * Math.PI;
            return angle;
        }
        //acosh(x) = log(x + sqrt(x^2 - 1))
        public static double Acosh(double x)
        {
            return Math.Log(x + Math.Sqrt(x * x - 1));
        }

        public static double NormalAngle(Vector3 a, Vector3 b, Vector3 up)
        {
            return SignedAngle(Vector3.Cross(up, a), Vector3.Cross(up, b), up);
        }

        public static double SignedAngle(Vector3 v1, Vector3 v2, Vector3 up)
        {
            return Vector3.Dot(Vector3.Cross(v1, v2), up) < 0 ? -Vector3.Angle(v1, v2) : Vector3.Angle(v1, v2);
        }

        public static Orbit OrbitFromStateVectors(Vector3d pos, Vector3d vel, CelestialBody body, double UT)
        {
            Orbit ret = new Orbit();
            ret.UpdateFromStateVectors((pos - body.position).xzy, vel.xzy, body, UT);
            return ret;
        }

        // Another MechJeb import.
        public static string CurrentBiome(this Vessel thatVessel)
        {
            if (!string.IsNullOrEmpty(thatVessel.landedAt))
            {
                return thatVessel.landedAt;
            }
            try
            {
                string biome = thatVessel.mainBody.BiomeMap.GetAtt(thatVessel.latitude * Math.PI / 180d, thatVessel.longitude * Math.PI / 180d).name;
                switch (thatVessel.situation)
                {
                    //ExperimentSituations.SrfLanded
                    case Vessel.Situations.LANDED:
                    case Vessel.Situations.PRELAUNCH:
                        return thatVessel.mainBody.bodyName + "'s " + (biome == "" ? "surface" : biome);
                    //ExperimentSituations.SrfSplashed
                    case Vessel.Situations.SPLASHED:
                        return thatVessel.mainBody.bodyName + "'s " + (biome == "" ? "oceans" : biome);
                    case Vessel.Situations.FLYING:
                        if (thatVessel.altitude < thatVessel.mainBody.scienceValues.flyingAltitudeThreshold)
                        {
                            //ExperimentSituations.FlyingLow
                            return "Flying over " + thatVessel.mainBody.bodyName + (biome == "" ? "" : "'s " + biome);
                        }
                        //ExperimentSituations.FlyingHigh
                        return "Upper atmosphere of " + thatVessel.mainBody.bodyName + (biome == "" ? "" : "'s " + biome);
                    default:
                        if (thatVessel.altitude < thatVessel.mainBody.scienceValues.spaceAltitudeThreshold)
                        {
                            //ExperimentSituations.InSpaceLow
                            return "Space just above " + thatVessel.mainBody.bodyName;
                        }
                        // ExperimentSituations.InSpaceHigh
                        return "Space high over " + thatVessel.mainBody.bodyName;
                }
            }
            catch { }

            return "Space over " + thatVessel.mainBody.bodyName;
        }

        public static Vector3d ClosestApproachSrfOrbit(Orbit vesselOrbit, Vessel target, out double UT, out double distance)
        {
            CelestialBody body = target.mainBody;

            // longitude and latitude calculations are offset by a different amount every
            // time we load the scene. We can use a zero latitude/longitude to find out what
            // that offset is.
            Vector3d zeroPos = body.GetRelSurfacePosition(0, 0, 0);
            body.GetLatLonAltOrbital(zeroPos, out var zeroLat, out var zeroLon, out var _);

            Vector3d pos = body.GetRelSurfacePosition(target.latitude - zeroLat, target.longitude - zeroLon, target.altitude);
            distance = GetClosestApproach(vesselOrbit, body, pos, out UT);
            return pos;
        }

        public static double GetClosestApproach(Orbit vesselOrbit, ITargetable target, out double timeAtClosestApproach)
        {
            if (target == null)
            {
                timeAtClosestApproach = -1.0;
                return -1.0;
            }

            if (target is CelestialBody)
            {
                return GetClosestApproach(vesselOrbit, target as CelestialBody, out timeAtClosestApproach);
            }
            else if (target is ModuleDockingNode)
            {
                return GetClosestApproach(vesselOrbit, (target as ModuleDockingNode).GetVessel().GetOrbit(), out timeAtClosestApproach);
            }
            else if (target is Vessel)
            {
                Vessel targetVessel = target as Vessel;
                if (targetVessel.LandedOrSplashed)
                {
                    double closestApproach;
                    ClosestApproachSrfOrbit(vesselOrbit, targetVessel, out timeAtClosestApproach, out closestApproach);
                    return closestApproach;
                }
                else
                {
                    return JUtil.GetClosestApproach(vesselOrbit, targetVessel.GetOrbit(), out timeAtClosestApproach);
                }
            }
            else
            {
                LogErrorMessage(target, "Unknown / unsupported target type in GetClosestApproach");
                timeAtClosestApproach = -1.0;
                return -1.0;
            }
        }

        // Closest Approach algorithms based on Protractor mod
        private static double GetClosestApproach(Orbit vesselOrbit, CelestialBody targetCelestial, out double timeAtClosestApproach)
        {
            Orbit closestorbit = GetClosestOrbit(vesselOrbit, targetCelestial);
            if (closestorbit.referenceBody == targetCelestial)
            {
                timeAtClosestApproach = closestorbit.StartUT + closestorbit.timeToPe;
                return closestorbit.PeA;
            }
            if (closestorbit.referenceBody == targetCelestial.referenceBody)
            {
                return MinTargetDistance(closestorbit, targetCelestial.orbit, closestorbit.StartUT, closestorbit.EndUT, out timeAtClosestApproach) - targetCelestial.Radius;
            }
            return MinTargetDistance(closestorbit, targetCelestial.orbit, Planetarium.GetUniversalTime(), Planetarium.GetUniversalTime() + closestorbit.period, out timeAtClosestApproach) - targetCelestial.Radius;
        }

        private static double GetClosestApproach(Orbit vesselOrbit, CelestialBody targetCelestial, Vector3d srfTarget, out double timeAtClosestApproach)
        {
            Orbit closestorbit = GetClosestOrbit(vesselOrbit, targetCelestial);
            if (closestorbit.referenceBody == targetCelestial)
            {
                double t0 = Planetarium.GetUniversalTime();
                Func<double, Vector3d> fn = delegate(double t)
                {
                    double angle = targetCelestial.rotates ? (t - t0) * 360.0 / targetCelestial.rotationPeriod : 0;
                    return targetCelestial.position + QuaternionD.AngleAxis(angle, Vector3d.down) * srfTarget;
                };
                double d = MinTargetDistance(closestorbit, fn, closestorbit.StartUT, closestorbit.EndUT, out timeAtClosestApproach);
                // When just passed over the target, some look ahead may be needed
                if ((timeAtClosestApproach <= closestorbit.StartUT || timeAtClosestApproach >= closestorbit.EndUT) &&
                    closestorbit.eccentricity < 1 && closestorbit.patchEndTransition == Orbit.PatchTransitionType.FINAL)
                {
                    d = MinTargetDistance(closestorbit, fn, closestorbit.EndUT, closestorbit.EndUT + closestorbit.period / 2, out timeAtClosestApproach);
                }
                return d;
            }
            return GetClosestApproach(vesselOrbit, targetCelestial, out timeAtClosestApproach);
        }

        public static double GetClosestApproach(Orbit vesselOrbit, Orbit targetOrbit, out double timeAtClosestApproach)
        {
            Orbit closestorbit = GetClosestOrbit(vesselOrbit, targetOrbit);

            double startTime = Planetarium.GetUniversalTime();
            double endTime;
            if (closestorbit.patchEndTransition != Orbit.PatchTransitionType.FINAL)
            {
                endTime = closestorbit.EndUT;
            }
            else
            {
                endTime = startTime + Math.Max(closestorbit.period, targetOrbit.period);
            }

            return MinTargetDistance(closestorbit, targetOrbit, startTime, endTime, out timeAtClosestApproach);
        }

        // Closest Approach support methods
        private static Orbit GetClosestOrbit(Orbit vesselOrbit, CelestialBody targetCelestial)
        {
            Orbit checkorbit = vesselOrbit;
            int orbitcount = 0;

            while (checkorbit.nextPatch != null && checkorbit.patchEndTransition != Orbit.PatchTransitionType.FINAL && orbitcount < 3)
            {
                checkorbit = checkorbit.nextPatch;
                orbitcount += 1;
                if (checkorbit.referenceBody == targetCelestial)
                {
                    return checkorbit;
                }

            }
            checkorbit = vesselOrbit;
            orbitcount = 0;

            while (checkorbit.nextPatch != null && checkorbit.patchEndTransition != Orbit.PatchTransitionType.FINAL && orbitcount < 3)
            {
                checkorbit = checkorbit.nextPatch;
                orbitcount += 1;
                if (checkorbit.referenceBody == targetCelestial.orbit.referenceBody)
                {
                    return checkorbit;
                }
            }

            return vesselOrbit;
        }

        private static Orbit GetClosestOrbit(Orbit vesselOrbit, Orbit targetOrbit)
        {
            Orbit checkorbit = vesselOrbit;
            int orbitcount = 0;

            while (checkorbit.nextPatch != null && checkorbit.patchEndTransition != Orbit.PatchTransitionType.FINAL && orbitcount < 3)
            {
                checkorbit = checkorbit.nextPatch;
                orbitcount += 1;
                if (checkorbit.referenceBody == targetOrbit.referenceBody)
                {
                    return checkorbit;
                }

            }

            return vesselOrbit;
        }

        private static double MinTargetDistance(Orbit vesselOrbit, Orbit targetOrbit, double startTime, double endTime, out double timeAtClosestApproach)
        {
            return MinTargetDistance(vesselOrbit, t => targetOrbit.getPositionAtUT(t), startTime, endTime, out timeAtClosestApproach);
        }

        private static double MinTargetDistance(Orbit vesselOrbit, Func<double, Vector3d> targetOrbit, double startTime, double endTime, out double timeAtClosestApproach)
        {
            var dist_at_int = new double[ClosestApproachRefinementInterval + 1];
            double step = startTime;
            double dt = (endTime - startTime) / (double)ClosestApproachRefinementInterval;
            for (int i = 0; i <= ClosestApproachRefinementInterval; i++)
            {
                dist_at_int[i] = (targetOrbit(step) - vesselOrbit.getPositionAtUT(step)).magnitude;
                step += dt;
            }
            double mindist = dist_at_int.Min();
            double maxdist = dist_at_int.Max();
            int minindex = Array.IndexOf(dist_at_int, mindist);
            if ((maxdist - mindist) / maxdist >= 0.00001)
            {
                // Don't allow negative times.  Clamp the startTime to the current startTime.
                mindist = MinTargetDistance(vesselOrbit, targetOrbit, startTime + (Math.Max(minindex - 1, 0) * dt), startTime + ((minindex + 1) * dt), out timeAtClosestApproach);
            }
            else
            {
                timeAtClosestApproach = startTime + minindex * dt;
            }

            return mindist;
        }

        // Piling all the extension methods into the same utility class to reduce the number of classes.
        // Because DLL size. Not really important and probably a bad practice, but one function static classes are silly.
        public static float? GetFloat(this string source)
        {
            float result;
            return float.TryParse(source, out result) ? result : (float?)null;
        }

        public static float? GetFloat(this ConfigNode node, string valueName)
        {
            return node.HasValue(valueName) ? node.GetValue(valueName).GetFloat() : (float?)null;
        }

        public static int? GetInt(this string source)
        {
            int result;
            return int.TryParse(source, out result) ? result : (int?)null;
        }

        public static int? GetInt(this ConfigNode node, string valueName)
        {
            return node.HasValue(valueName) ? node.GetValue(valueName).GetInt() : (int?)null;
        }

        public static string EnforceSlashes(this string input)
        {
            return input.Replace('\\', '/');
        }

        public static string UnMangleConfigText(this string input)
        {
            return input
                .Replace("<=", "{")
                .Replace("=>", "}")
                .Replace("$$$", Environment.NewLine);
        }

        public static string MangleConfigText(this string input)
        {
            return input
                .Replace("{", "<=")
                .Replace("}", "=>")
                .Replace("\n", "$$$")
                .Replace("\r", string.Empty);
        }

        public static T Clamp<T>(this T val, T min, T max) where T : IComparable<T>
        {
            if (val.CompareTo(min) < 0)
                return min;
            return val.CompareTo(max) > 0 ? max : val;
        }

        /// <summary>
        /// Method to instantiate one of the IComplexVariable objects on an rpmComp.
        /// </summary>
        /// <param name="node">The config node fetched from RPMGlobals</param>
        /// <param name="rpmComp">The RasterPropMonitorComputer that is hosting the variable</param>
        /// <returns>The complex variable, or null</returns>
        internal static IComplexVariable InstantiateComplexVariable(ConfigNode node, RasterPropMonitorComputer rpmComp)
        {
            if (node == null)
            {
                throw new ArgumentNullException("node was null. how did that happen?");
            }

            switch (node.name)
            {
                case "RPM_CUSTOM_VARIABLE":
                    return new CustomVariable(node, rpmComp);
                case "RPM_MAPPED_VARIABLE":
                    return new MappedVariable(node, rpmComp);
                case "RPM_MATH_VARIABLE":
                    return new MathVariable(node, rpmComp);
                case "RPM_SELECT_VARIABLE":
                    return new SelectVariable(node, rpmComp);
            }

            throw new ArgumentException("Unrecognized complex variable " + node.name);
        }

        /// <summary>
        /// Convert a numeric object to a float where.
        /// </summary>
        /// <param name="thatValue"></param>
        /// <returns></returns>
        public static float MassageToFloat(this object thatValue)
        {
            // RPMC only produces doubles, floats, ints, bools, and strings.
            if (thatValue is float floatVal)
                return floatVal;
            if (thatValue is double doubleVal)
                return (float)doubleVal;
            if (thatValue is int intVal)
                return (float)intVal;
            if (thatValue is bool boolVal)
                return boolVal ? 1.0f : 0.0f;
            return float.NaN;
        }

        /// <summary>
        /// Convert a numeric object to an integer.
        /// </summary>
        /// <param name="thatValue"></param>
        /// <returns></returns>
        public static int MassageToInt(this object thatValue)
        {
            // RPMC only produces doubles, floats, ints, bools, and strings.
            if (thatValue is int intVal)
                return intVal;
            if (thatValue is double doubleVal)
                return (int)doubleVal;
            if (thatValue is float floatVal)
                return (int)floatVal;
            if (thatValue is bool boolVal)
                return boolVal ? 1 : 0;
            return 0;
        }

        /// <summary>
        /// Convert a numeric object to a double.
        /// </summary>
        /// <param name="thatValue"></param>
        /// <returns></returns>
        public static double MassageToDouble(this object thatValue)
        {
            // RPMC only produces doubles, floats, ints, bools, and strings.
            if (thatValue is double doubleVal)
                return doubleVal;
            if (thatValue is float floatVal)
                return (double)floatVal;
            if (thatValue is int intVal)
                return (double)intVal;
            if (thatValue is bool boolVal)
                return boolVal ? 1.0 : 0.0;
            return double.NaN;
        }

        internal static InternalModule FindInternalModuleByName(InternalProp prop, string className)
        {
            foreach (InternalModule potentialModule in prop.internalModules)
            {
                if (potentialModule.ClassName == className)
                {
                    return potentialModule;
                }
            }

            return null;
        }

        internal static Delegate GetMethod(string packedMethod, InternalProp internalProp, Type delegateType)
        {
            string moduleName, stateMethod;
            string[] tokens = packedMethod.Split(':');
            if (tokens.Length != 2)
            {
                JUtil.LogErrorMessage(internalProp, "Bad format on {0}", packedMethod);
                throw new ArgumentException("stateMethod incorrectly formatted");
            }
            moduleName = tokens[0];
            stateMethod = tokens[1];

            // First look on the prop
            InternalModule thatModule = FindInternalModuleByName(internalProp, moduleName);
            
            // then on the internal as a whole
            if (thatModule == null)
            {
                foreach (var prop in internalProp.internalModel.props)
                {
                    // this really means "was this a MODULE placed directly in the INTERNAL"
                    if (!prop.hasModel)
                    {
                        thatModule = FindInternalModuleByName(prop, moduleName);
                        if (thatModule != null)
                        {
                            break;
                        }
                    }
                }
            }

            if (thatModule == null)
            {
                JUtil.LogErrorMessage(internalProp, "Failed finding module {0} for method {1}", moduleName, stateMethod);
                return null;
            }

            Type returnType = delegateType.GetMethod("Invoke").ReturnType;
            Delegate stateCall = null;
            foreach (MethodInfo m in thatModule.GetType().GetMethods())
            {
                if (!string.IsNullOrEmpty(stateMethod) && m.Name == stateMethod && m.ReturnParameter.ParameterType == returnType)
                {
                    stateCall = Delegate.CreateDelegate(delegateType, thatModule, m);
                }
            }

            return stateCall;
        }

        private static List<string> knownFonts = null;
        internal static Font LoadFont(string fontName, int size)
        {
            if (loadedFonts.ContainsKey(fontName))
            {
                return loadedFonts[fontName];
            }
            else if (loadedFonts.ContainsKey(fontName + size.ToString()))
            {
                return loadedFonts[fontName + size.ToString()];
            }

            if (knownFonts == null)
            {
                string[] fn = Font.GetOSInstalledFontNames();
                if (fn != null)
                {
                    knownFonts = fn.ToList<string>();
                }
            }

            if (knownFonts.Contains(fontName))
            {
                Font fontFn = Font.CreateDynamicFontFromOSFont(fontName, size);
                loadedFonts.Add(fontName + size.ToString(), fontFn);
                return fontFn;
            }
            else
            {
                if (!loadedFonts.ContainsKey("LiberationSans-Regular"))
                {
                    throw new ArgumentException("Failed finding fallback font LiberationSans-Regular");
                }

                return loadedFonts["LiberationSans-Regular"];
            }
        }

        internal static Transform FindPropTransform(InternalProp prop, string nameOrPath)
        {
            if (nameOrPath == null)
            {
                return null;
            }
            else if (nameOrPath.IndexOf('/') == -1)
            {
                if (prop.hasModel)
                {
                    return InternalProp.FindHeirarchyTransform(prop.transform.Find("model"), nameOrPath);
                }
                else
                {
                    return prop.internalModel.FindModelTransform(nameOrPath);
                }
            }
            else
            {
                return prop.transform.Find(nameOrPath);
            }
        }

        internal static Transform FindPropTransformOrThrow(InternalProp prop, string nameOrPath)
        {
            Transform result = FindPropTransform(prop, nameOrPath);
            if (result == null)
            {
                throw new ArgumentException($"could not find transform '{nameOrPath}' in prop {prop.propName}");
            }
            return result;
        }

        internal static Transform FindInternalTransform(InternalModel model, string nameOrPath)
        {
            if (nameOrPath.IndexOf('/') == -1)
            {
                return model.FindModelTransform(nameOrPath);
            }
            else
            {
                return model.transform.Find(nameOrPath);
            }
        }
    }

    // This, instead, is a static class on it's own because it needs its private static variables.
    public static class InstallationPathWarning
    {
        private static readonly List<string> warnedList = new List<string>();
        private const string gameData = "GameData";
        private static readonly string[] pathSep = { gameData };

        public static bool Warn(string path = "JSI/RasterPropMonitor/Plugins")
        {
            string assemblyPath = Assembly.GetCallingAssembly().Location;
            string fileName = Path.GetFileName(assemblyPath);
            bool wrongpath = false;
            if (!warnedList.Contains(fileName))
            {
                string installedLocation = Path.GetDirectoryName(assemblyPath).Split(pathSep, StringSplitOptions.None)[1].TrimStart('/').TrimStart('\\').EnforceSlashes();
                if (installedLocation != path)
                {
                    ScreenMessages.PostScreenMessage(string.Format("ERROR: {0} must be in GameData/{1} but it's in GameData/{2}", fileName, path, installedLocation),
                        120, ScreenMessageStyle.UPPER_CENTER);
                    Debug.LogError("RasterPropMonitor components are incorrectly installed. I should stop working and make you fix it, but KSP won't let me.");
                    wrongpath = true;
                }
                warnedList.Add(fileName);
            }
            return !wrongpath;
        }
    }

    public static class TEnum
    {
        public static EnumType[] GetValues<EnumType>() where EnumType : Enum
        {
            return (EnumType[])Enum.GetValues(typeof(EnumType));
        }
    }
}
