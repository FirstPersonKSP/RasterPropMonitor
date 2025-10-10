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
using UnityEngine;
using System;
using FinePrint;

namespace JSI
{
    public class JSIPrimaryFlightDisplay : InternalModule
    {
        [KSPField]
        public int drawingLayer = 17;
        [KSPField]
        public string horizonTexture = "JSI/RasterPropMonitor/Library/Components/NavBall/NavBall000";
        [KSPField]
        public string navBallModel = "JSI/RasterPropMonitor/Library/Components/NavBall/NavBall";
        [KSPField]
        public string staticOverlay = string.Empty;
        [KSPField]
        public string headingBar = string.Empty;
        [KSPField]
        public string backgroundColor = string.Empty;
        private Color backgroundColorValue = Color.black;
        [KSPField]
        public float ballOpacity = 0.8f;
        [KSPField]
        public Vector2 navBallCenter = Vector2.zero;
        [KSPField]
        public float navBallDiameter = 1.0f;
        [KSPField]
        public float markerSize = 32.0f;
        [KSPField] // x,y, width, height
        public Vector4 headingBarPosition = new Vector4(0, 0.8f, 0.8f, 0.1f);
        [KSPField]
        public float headingSpan = 0.25f;
        [KSPField]
        public bool headingAboveOverlay;
        [KSPField]
        public string progradeColor = string.Empty;
        private Color progradeColorValue = new Color(0.84f, 0.98f, 0);
        [KSPField]
        public string maneuverColor = string.Empty;
        private Color maneuverColorValue = new Color(0, 0.1137f, 1);
        [KSPField]
        public string targetColor = string.Empty;
        private Color targetColorValue = Color.magenta;
        [KSPField]
        public string waypointColor = string.Empty;
        private Color waypointColorValue = Color.magenta;
        [KSPField]
        public string normalColor = string.Empty;
        private Color normalColorValue = new Color(0.930f, 0, 1);
        [KSPField]
        public string radialColor = string.Empty;
        private Color radialColorValue = new Color(0, 1, 0.958f);
        [KSPField]
        public string dockingColor = string.Empty;
        private Color dockingColorValue = Color.red;
        [KSPField]
        public int speedModeButton = 4;

        private readonly Quaternion rotateNavBall = Quaternion.Euler(0.0f, 180.0f, 0.0f);

        private GameObject navBall;
        private GameObject overlay;
        private GameObject heading;

        private Vector3 navBallOrigin;
        private float markerDepth;
        private float navballRadius;

        struct Marker
        {
            public GameObject gameObject; // this is intended to mostly be temporary - all we really need here is a matrix
            public Mesh mesh;
            public Material material;
            public bool visible;
        }

        // Markers...
        private Marker markerPrograde;
        private Marker markerRetrograde;
        private Marker markerManeuver;
        private Marker markerManeuverMinus;
        private Marker markerTarget;
        private Marker markerTargetMinus;
        private Marker markerNormal;
        private Marker markerNormalMinus;
        private Marker markerRadial;
        private Marker markerRadialMinus;
        private Marker markerDockingAlignment;
        private Marker markerNavWaypoint;

        // Misc...
        private bool startupComplete;
        private bool firstRenderComplete;

        float cameraSpan;
        float cameraSpanWidth;

        private void ConfigureElements(float screenWidth, float screenHeight)
        {
            // How big is the nav ball, anyway?
            navballRadius = 0.0f;
            MeshFilter meshFilter = navBall.GetComponent<MeshFilter>();
            if (meshFilter != null)
            {
                // NOTE: I assume this really is a nav*ball*, not something
                // weird, and that it's centered on the origin.
                navballRadius = meshFilter.mesh.bounds.size.x * 0.5f;
                if (!(navballRadius > 0.0f))
                {
                    throw new Exception("JSIPrimaryFlightDisplay navball had an invalid size");
                }
            }
            else
            {
                throw new Exception("JSIPrimaryFlightDisplay could not get the navball mesh");
            }

            // Figure out how we have to manipulate the camera to get the
            // navball in the right place, and in the right size.
            cameraSpan = navballRadius * screenHeight / navBallDiameter;
            cameraSpanWidth = navballRadius * screenWidth / navBallDiameter;
            float pixelSize = cameraSpan / (screenHeight * 0.5f);

            float newXPos = navBallCenter.x - screenWidth * 0.5f;
            float newYPos = screenHeight * 0.5f - navBallCenter.y;
            navBallOrigin = navBall.transform.position;
            navBallOrigin.x += newXPos * pixelSize;
            navBallOrigin.y += newYPos * pixelSize;
            navBall.transform.position = navBallOrigin;
            // Because we use this value to offset the markers, we don't
            // want/need depth info.
            navBallOrigin.z = 0.0f;

            float overlayDepth = navBall.transform.position.z - navballRadius - 0.1f;

            Shader displayShader = JUtil.LoadInternalShader("RPM/DisplayShader");

            if (!string.IsNullOrEmpty(staticOverlay))
            {
                Material overlayMaterial = new Material(displayShader);
                overlayMaterial.mainTexture = GameDatabase.Instance.GetTexture(staticOverlay.EnforceSlashes(), false);

                overlay = JUtil.CreateSimplePlane("RPMPFDOverlay" + internalProp.propID, cameraSpan, drawingLayer);
                overlay.layer = drawingLayer;
                overlay.transform.position = new Vector3(0, 0, overlayDepth);
                overlay.GetComponent<Renderer>().material = overlayMaterial;

                overlay.SetActive(false);
            }

            if (!string.IsNullOrEmpty(headingBar))
            {
                Material headingMaterial = new Material(displayShader);
                headingMaterial.mainTexture = GameDatabase.Instance.GetTexture(headingBar.EnforceSlashes(), false);

                float hbXPos = headingBarPosition.x - screenWidth * 0.5f;
                float hbYPos = screenHeight * 0.5f - headingBarPosition.y;

                heading = JUtil.CreateSimplePlane("RPMPFDHeading" + internalProp.propID, new Vector2(headingBarPosition.z * pixelSize, headingBarPosition.w * pixelSize), new Rect(0.0f, 0.0f, 1.0f, 1.0f), drawingLayer);
                heading.transform.position = new Vector3(hbXPos * pixelSize, hbYPos * pixelSize, headingAboveOverlay ? (overlayDepth - 0.1f) : (overlayDepth + 0.1f));
                Renderer hdgMatl = null;
                heading.GetComponentCached<Renderer>(ref hdgMatl).material = headingMaterial;
                hdgMatl.material.SetTextureScale("_MainTex", new Vector2(headingSpan, 1f));

                heading.SetActive(false);
            }

            Texture2D gizmoTexture = JUtil.GetGizmoTexture();
            markerDepth = navBall.transform.position.z - navballRadius - 0.05f;
            float scaledMarkerSize = markerSize * 0.5f * pixelSize;
            markerPrograde = BuildMarker(0, 2, scaledMarkerSize, gizmoTexture, progradeColorValue, drawingLayer, internalProp.propID, displayShader);
            markerRetrograde = BuildMarker(1, 2, scaledMarkerSize, gizmoTexture, progradeColorValue, drawingLayer, internalProp.propID, displayShader);
            markerManeuver = BuildMarker(2, 0, scaledMarkerSize, gizmoTexture, maneuverColorValue, drawingLayer, internalProp.propID, displayShader);
            markerManeuverMinus = BuildMarker(1, 2, scaledMarkerSize, gizmoTexture, maneuverColorValue, drawingLayer, internalProp.propID, displayShader);
            markerTarget = BuildMarker(2, 1, scaledMarkerSize, gizmoTexture, targetColorValue, drawingLayer, internalProp.propID, displayShader);
            markerTargetMinus = BuildMarker(2, 2, scaledMarkerSize, gizmoTexture, targetColorValue, drawingLayer, internalProp.propID, displayShader);
            markerNormal = BuildMarker(0, 0, scaledMarkerSize, gizmoTexture, normalColorValue, drawingLayer, internalProp.propID, displayShader);
            markerNormalMinus = BuildMarker(1, 0, scaledMarkerSize, gizmoTexture, normalColorValue, drawingLayer, internalProp.propID, displayShader);
            markerRadial = BuildMarker(1, 1, scaledMarkerSize, gizmoTexture, radialColorValue, drawingLayer, internalProp.propID, displayShader);
            markerRadialMinus = BuildMarker(0, 1, scaledMarkerSize, gizmoTexture, radialColorValue, drawingLayer, internalProp.propID, displayShader);

            markerDockingAlignment = BuildMarker(0, 2, scaledMarkerSize, gizmoTexture, dockingColorValue, drawingLayer, internalProp.propID, displayShader);
            markerNavWaypoint = BuildMarker(0, 2, scaledMarkerSize, gizmoTexture, waypointColorValue, drawingLayer, internalProp.propID, displayShader);
        }

        public bool RenderPFD(RenderTexture screen, float aspect)
        {
            if (screen == null || !startupComplete || HighLogic.LoadedSceneIsEditor)
            {
                return false;
            }

            // Analysis disable once CompareOfFloatsByEqualityOperator
            if (firstRenderComplete == false)
            {
                firstRenderComplete = true;
                ConfigureElements((float)screen.width, (float)screen.height);
            }

            GL.Clear(true, true, backgroundColorValue);

			RPMVesselComputer comp = RPMVesselComputer.Instance(vessel);

			// Navball is rotated around the Y axis 180 degrees since the
			// original implementation had the camera positioned differently.
			// We still need MirrorX since KSP does something odd with the
			// gimbal
			navBall.transform.rotation = (rotateNavBall * MirrorX(comp.RelativeGymbal));
            
            if (heading != null)
            {
                heading.GetComponent<Renderer>().material.SetTextureOffset("_MainTex",
                    new Vector2(JUtil.DualLerp(0f, 1f, 0f, 360f, comp.RotationVesselSurface.eulerAngles.y) - headingSpan / 2f, 0));
            }

            Quaternion gymbal = comp.AttitudeGymbal;

            double speedInCurrentMode = 0;

            if (FlightGlobals.speedDisplayMode == FlightGlobals.SpeedDisplayModes.Orbit)
            {
                Vector3 velocityVesselOrbitUnit = comp.Prograde;
                speedInCurrentMode = FlightGlobals.ship_obtSpeed;
                Vector3 radialPlus = comp.RadialOut;
                Vector3 normalPlus = comp.NormalPlus;
                // stockNavBall.relativeGymbal * (stockNavBall.progradeVector.localRotation * stockNavBall.progradeVector.right == gymbal * velocityVesselOrbitUnit
                // But...  Same is true for stockNavBall.normalVector and stockNavBall.radialOutVector

                MoveMarker(markerPrograde, velocityVesselOrbitUnit, gymbal);
                MoveMarker(markerRetrograde, -velocityVesselOrbitUnit, gymbal);

                MoveMarker(markerNormal, normalPlus, gymbal);
                MoveMarker(markerNormalMinus, -normalPlus, gymbal);

                MoveMarker(markerRadial, radialPlus, gymbal);
                MoveMarker(markerRadialMinus, -radialPlus, gymbal);

                markerNormal.visible = true;
                markerNormalMinus.visible = true;
                markerRadial.visible = true;
                markerRadialMinus.visible = true;
            }
            else if (FlightGlobals.speedDisplayMode == FlightGlobals.SpeedDisplayModes.Surface)
            {
                Vector3 velocityVesselSurfaceUnit = vessel.srf_velocity.normalized;
                speedInCurrentMode = FlightGlobals.ship_srfSpeed;
                MoveMarker(markerPrograde, velocityVesselSurfaceUnit, gymbal);
                MoveMarker(markerRetrograde, -velocityVesselSurfaceUnit, gymbal);
            }
            else // FlightGlobals.speedDisplayMode == FlightGlobals.SpeedDisplayModes.Target
            {
                Vector3 targetDirection = FlightGlobals.ship_tgtVelocity.normalized;
                speedInCurrentMode = FlightGlobals.ship_tgtSpeed;

                MoveMarker(markerPrograde, targetDirection, gymbal);
                MoveMarker(markerRetrograde, -targetDirection, gymbal);
            }

            bool progradeVisible = speedInCurrentMode >= 0.05;
            markerPrograde.visible = progradeVisible;
            markerRetrograde.visible = progradeVisible;

            if (vessel.patchedConicSolver != null && vessel.patchedConicSolver.maneuverNodes.Count > 0)
            {
                Vector3 burnVector = vessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(vessel.orbit).normalized;
                MoveMarker(markerManeuver, burnVector, gymbal);
                MoveMarker(markerManeuverMinus, -burnVector, gymbal);

                markerManeuver.visible = true;
                markerManeuverMinus.visible = true;
            }

            DrawWaypoint(gymbal);

            ITargetable target = FlightGlobals.fetch.VesselTarget;
            if (target != null)
            {
                Vector3 targetSeparation = comp.TargetSeparation.normalized;
                MoveMarker(markerTarget, targetSeparation, gymbal);
                MoveMarker(markerTargetMinus, -targetSeparation, gymbal);
                var targetPort = target as ModuleDockingNode;
                if (targetPort != null)
                {
                    // Thanks to Michael Enßlin 
                    Transform targetTransform = targetPort.transform;
                    Transform selfTransform = vessel.ReferenceTransform;
                    Vector3 targetOrientationVector = -targetTransform.up.normalized;

                    Vector3 v1 = Vector3.Cross(selfTransform.up, targetTransform.forward);
                    Vector3 v2 = Vector3.Cross(selfTransform.up, selfTransform.forward);
                    float angle = Vector3.Angle(v1, v2);
                    if (Vector3.Dot(selfTransform.up, Vector3.Cross(v1, v2)) < 0)
                    {
                        angle = -angle;
                    }
                    MoveMarker(markerDockingAlignment, targetOrientationVector, gymbal);
                    markerDockingAlignment.gameObject.transform.Rotate(Vector3.up, -angle);
                    markerDockingAlignment.visible = true;
                }
                markerTarget.visible = true;
                markerTargetMinus.visible = true;
            }

            // I wonder if doing this as a command buffer might be more efficient?

            GL.PushMatrix();
            GL.LoadProjectionMatrix(Matrix4x4.Ortho(-cameraSpanWidth, cameraSpanWidth, -cameraSpan, cameraSpan, 0, 1.5f + navballRadius * 3));

            var navballMeshFilter = navBall.GetComponent<MeshFilter>();
            var navballRenderer = navBall.GetComponent<MeshRenderer>();
            navballRenderer.material.SetPass(0);
            Graphics.DrawMeshNow(navballMeshFilter.mesh, navBall.transform.localToWorldMatrix);

            DrawMarker(markerPrograde);
            DrawMarker(markerRetrograde);
            DrawMarker(markerManeuver);
            DrawMarker(markerManeuverMinus);
            DrawMarker(markerTarget);
            DrawMarker(markerTargetMinus);
            DrawMarker(markerNormal);
            DrawMarker(markerNormalMinus);
            DrawMarker(markerRadial);
            DrawMarker(markerRadialMinus);
            DrawMarker(markerDockingAlignment);
            DrawMarker(markerNavWaypoint);

            markerPrograde.visible = false;
            markerRetrograde.visible = false;
            markerManeuver.visible = false;
            markerManeuverMinus.visible = false;
            markerTarget.visible = false;
            markerTargetMinus.visible = false;
            markerNormal.visible = false;
            markerNormalMinus.visible = false;
            markerRadial.visible = false;
            markerRadialMinus.visible = false;
            markerDockingAlignment.visible = false;
            markerNavWaypoint.visible = false;

            if (overlay != null)
            {
                var overlayMesh = overlay.GetComponent<MeshFilter>().mesh;
                var overlayMaterial = overlay.GetComponent<MeshRenderer>().material;
                overlayMaterial.SetPass(0);
                Graphics.DrawMeshNow(overlayMesh, overlay.transform.localToWorldMatrix);
            }

            if (heading != null)
            {
                var headingMesh = heading.GetComponent<MeshFilter>().mesh;
                var headingMaterial = heading.GetComponent<MeshRenderer>().material;
                headingMaterial.SetPass(0);
                Graphics.DrawMeshNow(headingMesh, heading.transform.localToWorldMatrix);
            }

            GL.PopMatrix();

            return true;
        }

        public void ButtonProcessor(int buttonID)
        {
            if (buttonID == speedModeButton)
            {
                FlightGlobals.CycleSpeedModes();
            }
        }

        private readonly int opacityIndex = Shader.PropertyToID("_Opacity");
        
        private void MoveMarker(Marker marker, Vector3 position, Quaternion voodooGymbal)
        {
            Vector3 newPosition = ((voodooGymbal * position) * navballRadius) + navBallOrigin;
            marker.material.SetFloat(opacityIndex, Mathf.Clamp01(newPosition.z + 0.5f));
            marker.gameObject.transform.position = new Vector3(newPosition.x, newPosition.y, markerDepth);
        }

        private static Quaternion MirrorX(Quaternion input)
        {
            // Witchcraft: It's called mirroring the X axis of the quaternion's conjugate.
            // We have to do this because the KSP navball gimbal is oddly mapped.
            return new Quaternion(input.x, -input.y, -input.z, input.w);
        }

        private static Marker BuildMarker(int iconX, int iconY, float markerSize, Texture gizmoTexture, Color nativeColor, int drawingLayer, int propID, Shader shader)
        {
            Marker marker = new Marker();

            marker.gameObject = JUtil.CreateSimplePlane("RPMPFDMarker" + iconX + iconY + propID, markerSize, drawingLayer);

            marker.mesh = marker.gameObject.GetComponent<MeshFilter>().sharedMesh;

            marker.material = new Material(shader);
            marker.material.mainTexture = gizmoTexture;
            marker.material.mainTextureScale = Vector2.one / 3f;
            marker.material.mainTextureOffset = new Vector2(iconX * (1f / 3f), iconY * (1f / 3f));
            marker.material.color = Color.white;
            marker.material.SetVector("_Color", nativeColor);

            marker.gameObject.transform.position = Vector3.zero;

            marker.gameObject.SetActive(false);
            marker.visible = false;

            return marker;
        }

        private static void DestroyMarker(Marker marker)
        {
            UnityEngine.Object.Destroy(marker.mesh);
        }

        private static void DrawMarker(Marker marker)
        {
            if (marker.visible)
            {
                marker.material.SetPass(0);
                Graphics.DrawMeshNow(marker.mesh, marker.gameObject.transform.localToWorldMatrix);
            }
        }

        private void DrawWaypoint(Quaternion gymbal)
        {
            NavWaypoint waypoint = NavWaypoint.fetch;
            if (waypoint == null || !waypoint.IsActive)
            {
                return;
            }
            
            if (waypoint.IsBlinking && Planetarium.GetUniversalTime() % 1.0 > 0.5)
            {
                return;
            }

            try
            {
                Material material = waypoint.Visual.GetComponent<Renderer>().sharedMaterial;
                markerNavWaypoint.gameObject.transform.localScale = new Vector3(0.6f, 0.6f, 0.6f);
                markerNavWaypoint.material.mainTexture = material.GetTexture("_MainTexture");
                markerNavWaypoint.material.mainTextureScale = Vector2.one;
                markerNavWaypoint.material.mainTextureOffset = Vector2.zero;
                if (string.IsNullOrEmpty(waypointColor))
                {
                    markerNavWaypoint.material.SetVector("_Color", material.GetVector("_TintColor"));
                }
            }
            catch (Exception e)
            {
                Debug.Log($"Waypoint material failure: {e}");
            }

            Vector3d waypointPosition = waypoint.Body.GetWorldSurfacePosition(waypoint.Latitude, waypoint.Longitude, waypoint.Altitude);
            Vector3 waypointDirection = (waypointPosition - vessel.CoM).normalized;
            MoveMarker(markerNavWaypoint, waypointDirection, gymbal);
            markerNavWaypoint.visible = true;
        }

        public void Start()
        {
            if (HighLogic.LoadedSceneIsEditor)
            {
                return;
            }

            try
            {
                // Parse bloody KSPField colors.
                if (!string.IsNullOrEmpty(backgroundColor))
                {
                    backgroundColorValue = ConfigNode.ParseColor32(backgroundColor);
                }
                if (!string.IsNullOrEmpty(progradeColor))
                {
                    progradeColorValue = ConfigNode.ParseColor32(progradeColor);
                }
                if (!string.IsNullOrEmpty(maneuverColor))
                {
                    maneuverColorValue = ConfigNode.ParseColor32(maneuverColor);
                }
                if (!string.IsNullOrEmpty(targetColor))
                {
                    targetColorValue = ConfigNode.ParseColor32(targetColor);
                }
                if (!string.IsNullOrEmpty(normalColor))
                {
                    normalColorValue = ConfigNode.ParseColor32(normalColor);
                }
                if (!string.IsNullOrEmpty(radialColor))
                {
                    radialColorValue = ConfigNode.ParseColor32(radialColor);
                }
                if (!string.IsNullOrEmpty(dockingColor))
                {
                    dockingColorValue = ConfigNode.ParseColor32(dockingColor);
                }
                if (!string.IsNullOrEmpty(waypointColor))
                {
                    waypointColorValue = ConfigNode.ParseColor32(waypointColor);
                }

                Shader displayShader = JUtil.LoadInternalShader("RPM/DisplayShader");

                // Non-moving parts...

                Vector3 navBallPosition = new Vector3(0.0f, 0.0f, 1.5f);

                navBall = GameDatabase.Instance.GetModel(navBallModel.EnforceSlashes());
                if (navBall == null)
                {
                    JUtil.LogErrorMessage(this, "Failed to load navball model {0}", navBallModel);
                    // Early return here - if we don't even have a navball, this module is pointless.
                    return;
                }

                Destroy(navBall.GetComponent<Collider>());
                navBall.name = "RPMNB" + navBall.GetInstanceID();
                navBall.layer = drawingLayer;
                navBall.transform.position = navBallPosition;
                navBall.GetComponent<Renderer>().material.shader = displayShader;
                Texture2D horizonTex = GameDatabase.Instance.GetTexture(horizonTexture.EnforceSlashes(), false);
                if (horizonTex != null)
                {
                    navBall.GetComponent<Renderer>().material.mainTexture = horizonTex;
                }
                else
                {
                    JUtil.LogErrorMessage(this, "Failed to load horizon texture {0}", horizonTexture);
                }

                navBall.GetComponent<Renderer>().material.SetFloat("_Opacity", ballOpacity);

                navBall.SetActive(false);

                startupComplete = true;
            }
            catch
            {
                JUtil.AnnoyUser(this);
                throw;
            }
        }

        public void OnDestroy()
        {
            if (HighLogic.LoadedSceneIsEditor)
            {
                // Nothing configured, nothing to destroy.
                return;
            }

            DestroyMarker(markerPrograde);
            DestroyMarker(markerRetrograde);
            DestroyMarker(markerManeuver);
            DestroyMarker(markerManeuverMinus);
            DestroyMarker(markerTarget);
            DestroyMarker(markerTargetMinus);
            DestroyMarker(markerNormal);
            DestroyMarker(markerNormalMinus);
            DestroyMarker(markerRadial);
            DestroyMarker(markerRadialMinus);
            DestroyMarker(markerDockingAlignment);
            DestroyMarker(markerNavWaypoint);

            JUtil.DisposeOfGameObjects(new GameObject[] { navBall, overlay, heading });
        }
    }
}
