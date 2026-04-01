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

// JSIOrbitDisplay: Display a schematic (line-art) drawing of the vessel's
// orbit, marking highlights (Pe, Ap, AN, DN), along with the mainbody's
// surface and atmosphere (if applicable).
using System;
using UnityEngine;

namespace JSI
{
    public class JSIOrbitDisplay : InternalModule
    {
        [KSPField]
        public string backgroundColor = string.Empty;
        private Color backgroundColorValue = Color.black;
        [KSPField]
        public string iconColorSelf = string.Empty;
        private Color iconColorSelfValue = new Color(1f, 1f, 1f, 0.6f);
        [KSPField]
        public string orbitColorSelf = string.Empty;
        private Color orbitColorSelfValue = new Color(1f, 1f, 1f, 0.6f);
        [KSPField]
        public string iconColorTarget = string.Empty;
        private Color iconColorTargetValue = new Color32(255, 235, 4, 153);
        [KSPField]
        public string iconColorShadow = string.Empty;
        private Color iconColorShadowValue = new Color(0f, 0f, 0f, 0.5f);
        [KSPField]
        public string iconColorAP = string.Empty;
        private Color iconColorAPValue = new Color(1f, 1f, 1f, 0.6f);
        [KSPField]
        public string iconColorPE = string.Empty;
        private Color iconColorPEValue = new Color(1f, 1f, 1f, 0.6f);
        [KSPField]
        public string iconColorClosestApproach = string.Empty;
        private Color iconColorClosestApproachValue = new Color(0.7f, 0.0f, 0.7f, 0.6f);
        [KSPField]
        public string orbitColorNextNode = string.Empty;
        private Color orbitColorNextNodeValue = new Color(1f, 1f, 1f, 0.6f);
        [KSPField]
        public Vector4 orbitDisplayPosition = new Vector4(0f, 0f, 512f, 512f);
        [KSPField]
        public float iconPixelSize = 8f;
        [KSPField]
        public Vector2 iconShadowShift = new Vector2(1, 1);

        private bool startupComplete;
		private Material lineMaterial;

        static readonly int CIRCLE_POINTS = 60;
        static readonly int ORBIT_POINTS = 60;

		public override void OnAwake()
		{
			base.OnAwake();

			if (lineMaterial == null)
			{
				lineMaterial = JUtil.DrawLineMaterial();
			}
		}

        // TODO: this could all be improved by implementint adaptive screen-space tesselation:
        // http://blog.johannesmp.com/2022/06/30/KSP2-Dev-Diary_Orbit-Tessellation/

        // All units in pixels.  Assumes GL.Begin(LINES) and GL.Color() have
        // already been called for this circle.
        private static void DrawCircle(float centerX, float centerY, float radius)
        {
            int maxOrbitPoints = CIRCLE_POINTS;

            // Figure out the tessellation level to use, based on circle size
            // and user limits.
            float circumferenceInPixels = 2.0f * Mathf.PI * radius;
            // Our ideal is a tessellation that gives us 2 pixels per segment,
            // which should look like a smooth circle.
            int idealOrbitPoints = Math.Max(1, (int)(circumferenceInPixels / 2.0f));
            int numSegments = Math.Min(maxOrbitPoints, idealOrbitPoints);
            float dTheta = (float)(2.0 * Math.PI / (double)(numSegments));
            float theta = 0.0f;

            var lastVertex = new Vector3(centerX + radius, centerY, 0.0f);
            for (int i = 0; i < numSegments; ++i)
            {
                GL.Vertex(lastVertex);
                theta += dTheta;

                float cosTheta = Mathf.Cos(theta);
                float sinTheta = Mathf.Sin(theta);
                var newVertex = new Vector3(centerX + cosTheta * radius, centerY + sinTheta * radius, 0.0f);
                GL.Vertex(newVertex);
                // Pity LINE_STRIP isn't supported.  We have to double the
                // number of vertices we shove at the GPU.
                lastVertex = newVertex;
            }
        }

        static Vector3 ScreenPositionFromOrbitAtTA(Orbit o, CelestialBody referenceBody, Matrix4x4 screenTransform, double ta, double now)
        {
            Vector3 relativePosition = o.getRelativePositionFromTrueAnomaly(ta).xzy;

            if (o.referenceBody != referenceBody)
            {
                double timeAtTA = o.GetUTforTrueAnomaly(ta, now);
                relativePosition += o.referenceBody.getTruePositionAtUT(timeAtTA) - referenceBody.getTruePositionAtUT(timeAtTA);
            }

            return screenTransform.MultiplyPoint3x4(relativePosition);
        }

        private static void DrawOrbit(Orbit o, CelestialBody referenceBody, Matrix4x4 screenTransform)
        {
            const double MIN_POINTS = 4;
            if (!o.activePatch)
            {
                return;
            }

            double startTA;
            double endTA;
            double now = Planetarium.GetUniversalTime();
            if (o.eccentricity < 1 && o.patchEndTransition == Orbit.PatchTransitionType.FINAL)
            {
                startTA = 0;
                endTA = 2 * Math.PI;
            }
            else if (o.patchEndTransition != Orbit.PatchTransitionType.FINAL)
            {
                startTA = o.TrueAnomalyAtUT(o.StartUT);
                endTA = o.TrueAnomalyAtUT(o.EndUT);
                if (endTA < startTA)
                {
                    endTA += 2.0 * Math.PI;
                }
            }
            else
            {
                startTA = o.GetUTforTrueAnomaly(0.0, now);
                endTA = startTA + 2.0 * Math.PI;
            }

            double dTheta = (endTA - startTA) / MIN_POINTS;
            double theta = startTA;
            Vector3 lastVertex = ScreenPositionFromOrbitAtTA(o, referenceBody, screenTransform, theta, now);
            for (int i = 0; i < MIN_POINTS; ++i)
            {
                double nextTheta = theta + dTheta;
                Vector3 nextVertex = ScreenPositionFromOrbitAtTA(o, referenceBody, screenTransform, nextTheta, now);
                DrawOrbitSegment(o, referenceBody, screenTransform, now, theta, lastVertex, nextTheta, nextVertex);
                lastVertex = nextVertex;
                theta = nextTheta;
            }
        }

        private static void DrawOrbitSegment(
            Orbit o,
            CelestialBody referenceBody,
            Matrix4x4 screenTransform,
            double now,
            double startTA,
            Vector3 startVertex,
            double endTA,
            Vector3 endVertex)
        {
            double midTA = (startTA + endTA) / 2.0;
            Vector3 midVertex = ScreenPositionFromOrbitAtTA(o, referenceBody, screenTransform, midTA, now);
            Vector3 midStraight = (startVertex + endVertex) * 0.5f;

            if (Math.Abs(startTA - endTA) <  0.01 || (midStraight - midVertex).sqrMagnitude < 16.0)
            {
                GL.Vertex3(startVertex.x, startVertex.y, 0.0f);
                GL.Vertex3(midVertex.x, midVertex.y, 0.0f);
                GL.Vertex3(midVertex.x, midVertex.y, 0.0f);
                GL.Vertex3(endVertex.x, endVertex.y, 0.0f);
                return;
            }

            DrawOrbitSegment(o, referenceBody, screenTransform, now, startTA, startVertex, midTA, midVertex);
            DrawOrbitSegment(o, referenceBody, screenTransform, now, midTA, midVertex, endTA, endVertex);
        }

        // Fallback method: The orbit should be valid, but it's not showing as
        // active.  I've encountered this when targeting a vessel or planet.
        private static void ReallyDrawOrbit(Orbit o, CelestialBody referenceBody, Matrix4x4 screenTransform)
        {
            int numSegments = ORBIT_POINTS;

            if (o.eccentricity >= 1.0)
            {
                Debug.Log("JSIOrbitDisplay.ReallyDrawOrbit(): I can't draw an orbit with e >= 1.0");
                return;
            }

            // https://www.wolframalpha.com/input?i=y+%3D+x-+0.5+*sin%28x*2%29+from+x+%3D+0+to+2+*+pi

            double theta = 0.0;
            double dTheta = 2 * Math.PI / (double)numSegments;
            double now = Planetarium.GetUniversalTime();
            double bias = Mathf.Lerp(0, -0.5f, (float)o.eccentricity);
            Vector3 lastVertex = ScreenPositionFromOrbitAtTA(o, referenceBody, screenTransform, theta, now); ;
            for (int i = 0; i < numSegments; ++i)
            {
                GL.Vertex3(lastVertex.x, lastVertex.y, 0.0f);

                theta += dTheta;
                double ta = theta + bias * Math.Sin(theta * 2);

                Vector3 newVertex = ScreenPositionFromOrbitAtTA(o, referenceBody, screenTransform, ta, now);
                GL.Vertex3(newVertex.x, newVertex.y, 0.0f);

                lastVertex = newVertex;
            }
        }

        private void DrawNextAp(Orbit o, CelestialBody referenceBody, double referenceTime, Color iconColor, Matrix4x4 screenTransform)
        {
            if (o.eccentricity >= 1.0)
            {
                // Early return: There is no apoapsis on a hyperbolic orbit
                return;
            }
            double nextApTime = o.GetNextApoapsisTime(referenceTime);

            if (nextApTime < o.EndUT || (o.patchEndTransition == Orbit.PatchTransitionType.FINAL))
            {
                Vector3d relativePosition = o.SwappedRelativePositionAtUT(nextApTime) + o.referenceBody.getTruePositionAtUT(nextApTime) - referenceBody.getTruePositionAtUT(nextApTime);
                var transformedPosition = screenTransform.MultiplyPoint3x4(relativePosition);
                DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, iconColor, MapIcons.OtherIcon.AP);
            }
        }

        private void DrawNextPe(Orbit o, CelestialBody referenceBody, double referenceTime, Color iconColor, Matrix4x4 screenTransform)
        {
            /*
            switch (o.patchEndTransition)
            {
                case Orbit.PatchTransitionType.ENCOUNTER:
                    Debug.Log("ENCOUNTER patch end type");
                    break;
                case Orbit.PatchTransitionType.ESCAPE:
                    Debug.Log("ESCAPE patch end type");
                    break;
                // FINAL is applied to the active vessel in a stable elliptical
                // orbit.
                case Orbit.PatchTransitionType.FINAL:
                    Debug.Log("FINAL patch end type");
                    break;
                // INITIAL patchEndTransition appears to be applied to inactive
                // vessels (targeted vessels).
                case Orbit.PatchTransitionType.INITIAL:
                    Debug.Log("INITIAL patch end type");
                    break;
                case Orbit.PatchTransitionType.MANEUVER:
                    Debug.Log("MANEUVER patch end type");
                    break;
            }
             */

            double nextPeTime = o.GetNextPeriapsisTime(referenceTime);
            if (nextPeTime < o.EndUT || (o.patchEndTransition == Orbit.PatchTransitionType.FINAL))
            {
                Vector3d relativePosition = o.SwappedRelativePositionAtUT(nextPeTime) + o.referenceBody.getTruePositionAtUT(nextPeTime) - referenceBody.getTruePositionAtUT(nextPeTime);
                var transformedPosition = screenTransform.MultiplyPoint3x4(relativePosition);
                DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, iconColor, MapIcons.OtherIcon.PE);
            }
        }

        private static Orbit GetPatchAtUT(Orbit startOrbit, double UT)
        {
            Orbit o = startOrbit;
            while (o.patchEndTransition != Orbit.PatchTransitionType.FINAL)
            {
                if (o.EndUT >= UT)
                {
                    // EndUT for this patch is later than what we're looking for.  Exit.
                    break;
                }
                if (o.nextPatch == null || !o.nextPatch.activePatch)
                {
                    // There is no valid next patch.  Exit.
                    break;
                }

                // Check the next one.
                o = o.nextPatch;
            }
            return o;
        }

        // Analysis disable once UnusedParameter
        public bool RenderOrbit(RenderTexture screen, float cameraAspect)
        {
            if (!startupComplete || HighLogic.LoadedSceneIsEditor)
                return false;
            // Make sure the parameters fit on the screen.
            Vector4 displayPosition = orbitDisplayPosition;
            displayPosition.z = Mathf.Min(screen.width - displayPosition.x, displayPosition.z);
            displayPosition.w = Mathf.Min(screen.height - displayPosition.y, displayPosition.w);

            // Here is our pixel budget in each direction:
            double horizPixelSize = displayPosition.z - iconPixelSize;
            double vertPixelSize = displayPosition.w - iconPixelSize;

            RasterPropMonitorComputer rpmComp = RasterPropMonitorComputer.FindFromProp(internalProp);
            (int patchIndex, Orbit selectedPatch) = rpmComp.GetSelectedPatch();

            // Find a basis for transforming values into the framework of
            // vessel.orbit.  The rendering framework assumes the periapsis
            // is drawn directly to the right of the mainBody center of mass.
            // It assumes the orbit's prograde direction is "up" (screen
            // relative) at the periapsis, providing a counter-clockwise
            // motion for vessel.
            // Once we have the basic transform, we will add in scalars
            // that will ultimately transform an arbitrary point (relative to
            // the planet's center) into screen space.
            Matrix4x4 screenTransform = Matrix4x4.identity;
            double now = Planetarium.GetUniversalTime();
            double timeAtPe = selectedPatch.GetNextPeriapsisTime(now);

            // Get the 3 direction vectors, based on Pe being on the right of the screen
            // OrbitExtensions provides handy utilities to get these.
            Vector3d right = selectedPatch.Up(timeAtPe);
            Vector3d forward = selectedPatch.SwappedOrbitNormal();
            // MOARdV: OrbitExtensions.Horizontal is unstable.  I've seen it
            // become (0, 0, 0) intermittently in flight.  Instead, use the
            // cross product of the other two.
            // We flip the sign of this vector because we are using an inverted
            // y coordinate system to keep the icons right-side up.
            Vector3d up = -Vector3d.Cross(forward, right);
            //Vector3d up = -vessel.orbit.Horizontal(timeAtPe);

            screenTransform.SetRow(0, new Vector4d(right.x, right.y, right.z, 0.0));
            screenTransform.SetRow(1, new Vector4d(up.x, up.y, up.z, 0.0));
            screenTransform.SetRow(2, new Vector4d(forward.x, forward.y, forward.z, 0.0));

            // Figure out our bounds.  First, make sure the entire planet
            // fits on the screen.  We define the center of the vessel.mainBody
            // as the origin of our coodinate system.
            Bounds bounds = new Bounds(selectedPatch.referenceBody.Radius);

            if (selectedPatch.referenceBody.atmosphere)
            {
                double radius = selectedPatch.referenceBody.Radius + selectedPatch.referenceBody.atmosphereDepth;
                bounds.Add(radius, radius);
                bounds.Add(-radius, -radius);
            }

            // Now make sure the entire orbit fits on the screen.
            Vector3 vesselPos;
            // The PeR, ApR, and semiMinorAxis are all one dimensional, so we
            // can just apply them directly to these values.
            bounds.Add(selectedPatch.PeR, 0);

            if (selectedPatch.eccentricity < 1.0)
            {
                bounds.Add(-selectedPatch.ApR, 0);
                bounds.Add(0, selectedPatch.semiMinorAxis);
                bounds.Add(0, -selectedPatch.semiMinorAxis);
            }

            if (selectedPatch.EndUT > 0.0)
            {
                // If we're hyperbolic, let's get the SoI transition
                vesselPos = screenTransform.MultiplyPoint3x4(selectedPatch.SwappedRelativePositionAtUT(selectedPatch.EndUT));
                bounds.Add(vesselPos.x, vesselPos.y);
            }

            if (patchIndex > 0)
            {
                // Include the start SOI transition
                vesselPos = screenTransform.MultiplyPoint3x4(selectedPatch.SwappedRelativePositionAtUT(selectedPatch.StartUT));
                bounds.Add(vesselPos.x, vesselPos.y);
            }

            // Make sure the vessel shows up on-screen.  Since a hyperbolic
            // orbit doesn't have a meaningful ApR, we use this as a proxy for
            // how far we need to extend the bounds to show the vessel.
            if (selectedPatch.referenceBody == vessel.orbit.referenceBody)
            {
                vesselPos = screenTransform.MultiplyPoint3x4(vessel.orbit.SwappedRelativePositionAtUT(now));
                bounds.Add(vesselPos.x, vesselPos.y);
            }

            // Account for a target vessel
            var targetBody = FlightGlobals.fetch.VesselTarget as CelestialBody;
            var targetVessel = FlightGlobals.fetch.VesselTarget as Vessel;
            if (targetVessel != null && targetVessel.mainBody != selectedPatch.referenceBody)
            {
                // We only care about tgtVessel if it is in the same SoI.
                targetVessel = null;
            }

            if (targetVessel != null && !targetVessel.LandedOrSplashed)
            {
                double tgtPe = targetVessel.orbit.GetNextPeriapsisTime(now);

                vesselPos = screenTransform.MultiplyPoint3x4(targetVessel.orbit.SwappedRelativePositionAtUT(tgtPe));
                bounds.Add(vesselPos.x, vesselPos.y);

                if (targetVessel.orbit.eccentricity < 1.0)
                {
                    vesselPos = screenTransform.MultiplyPoint3x4(targetVessel.orbit.SwappedRelativePositionAtUT(targetVessel.orbit.GetNextApoapsisTime(now)));
                    bounds.Add(vesselPos.x, vesselPos.y);
                }

                vesselPos = screenTransform.MultiplyPoint3x4(targetVessel.orbit.SwappedRelativePositionAtUT(now));
                bounds.Add(vesselPos.x, vesselPos.y);
            }

            if (targetBody != null)
            {
                // Validate some values up front, so we don't need to test them later.
                if (targetBody.GetOrbit() == null)
                {
                    targetBody = null;
                }
                else if (targetBody.orbit.referenceBody == selectedPatch.referenceBody)
                {
                    // If the target body orbits our current world, let's at
                    // least make sure the body's location is visible.
                    vesselPos = screenTransform.MultiplyPoint3x4(targetBody.GetOrbit().SwappedRelativePositionAtUT(now));
                    bounds.Add(vesselPos.x, vesselPos.y);
                }
            }

            ManeuverNode node = null;
            if (vessel.patchedConicSolver != null)
            {
                node = (vessel.patchedConicSolver.maneuverNodes.Count > 0) ? vessel.patchedConicSolver.maneuverNodes[0] : null;
            }

            if (node != null)
            {
                double nodePe = node.nextPatch.GetNextPeriapsisTime(now);
                vesselPos = screenTransform.MultiplyPoint3x4(node.nextPatch.SwappedRelativePositionAtUT(nodePe));
                bounds.Add(vesselPos.x, vesselPos.y);

                if (node.nextPatch.eccentricity < 1.0)
                {
                    double nodeAp = node.nextPatch.GetNextApoapsisTime(now);
                    vesselPos = screenTransform.MultiplyPoint3x4(node.nextPatch.SwappedRelativePositionAtUT(nodeAp));
                    bounds.Add(vesselPos.x, vesselPos.y);
                }
                else if (node.nextPatch.EndUT > 0.0)
                {
                    // If the next patch is hyperbolic, include the endpoint.
                    vesselPos = screenTransform.MultiplyPoint3x4(selectedPatch.SwappedRelativePositionAtUT(node.nextPatch.EndUT));
                    bounds.Add(vesselPos.x, vesselPos.y);
                }
            }

            // Add translation.  This will ensure that all of the features
            // under consideration above will be displayed.
            screenTransform[0, 3] = -0.5f * (float)(bounds.maxX + bounds.minX);
            screenTransform[1, 3] = -0.5f * (float)(bounds.maxY + bounds.minY);

            double neededWidth = bounds.maxX - bounds.minX;
            double neededHeight = bounds.maxY - bounds.minY;

            // Pick a scalar that will fit the bounding box we just created.
            float pixelScalar = (float)Math.Min(horizPixelSize / neededWidth, vertPixelSize / neededHeight);
            screenTransform = Matrix4x4.Scale(new Vector3(pixelScalar, pixelScalar, pixelScalar)) * screenTransform;

            GL.Clear(true, true, backgroundColorValue);
            GL.PushMatrix();
            GL.LoadPixelMatrix(-displayPosition.z * 0.5f, displayPosition.z * 0.5f, displayPosition.w * 0.5f, -displayPosition.w * 0.5f);
            GL.Viewport(new Rect(displayPosition.x, screen.height - displayPosition.y - displayPosition.w, displayPosition.z, displayPosition.w));

            lineMaterial.SetPass(0);
            GL.Begin(GL.LINES);

            // Draw the planet:
            Vector3 focusCenter = screenTransform.MultiplyPoint3x4(new Vector3(0.0f, 0.0f, 0.0f));

            // orbitDriver is null on the sun, so we'll just use white instead.
            GL.Color((selectedPatch.referenceBody.orbitDriver == null) ? new Color(1.0f, 1.0f, 1.0f) : selectedPatch.referenceBody.orbitDriver.orbitColor);
            DrawCircle(focusCenter.x, focusCenter.y, (float)(selectedPatch.referenceBody.Radius * pixelScalar));
            if (selectedPatch.referenceBody.atmosphere)
            {
                // Use the atmospheric ambient to color the atmosphere circle.
                GL.Color(selectedPatch.referenceBody.atmosphericAmbientColor);

                DrawCircle(focusCenter.x, focusCenter.y, (float)((selectedPatch.referenceBody.Radius + selectedPatch.referenceBody.atmosphereDepth) * pixelScalar));
            }

            if (targetVessel != null && !targetVessel.LandedOrSplashed)
            {
                GL.Color(iconColorTargetValue);
                if (!targetVessel.orbit.activePatch && targetVessel.orbit.eccentricity < 1.0 && targetVessel.orbit.referenceBody == selectedPatch.referenceBody)
                {
                    // For some reason, activePatch is false for targetVessel.
                    // If we have a stable orbit for the target, use a fallback
                    // rendering method:
                    ReallyDrawOrbit(targetVessel.orbit, selectedPatch.referenceBody, screenTransform);
                }
                else
                {
                    DrawOrbit(targetVessel.orbit, selectedPatch.referenceBody, screenTransform);
                }
            }

            foreach (CelestialBody moon in selectedPatch.referenceBody.orbitingBodies)
            {
                if (moon != targetBody)
                {
                    GL.Color(moon.orbitDriver.orbitColor);
                    ReallyDrawOrbit(moon.GetOrbit(), selectedPatch.referenceBody, screenTransform);
                }
            }

            if (targetBody != null)
            {
                GL.Color(iconColorTargetValue);
                ReallyDrawOrbit(targetBody.GetOrbit(), selectedPatch.referenceBody, screenTransform);
            }

            if (node != null)
            {
                GL.Color(orbitColorNextNodeValue);
                DrawOrbit(node.nextPatch, selectedPatch.referenceBody, screenTransform);
            }

            if (selectedPatch.nextPatch != null && selectedPatch.nextPatch.activePatch)
            {
                GL.Color(orbitColorNextNodeValue);
                DrawOrbit(selectedPatch.nextPatch, selectedPatch.referenceBody, screenTransform);
            }

            // Draw the vessel orbit
            GL.Color(orbitColorSelfValue);
            DrawOrbit(selectedPatch, selectedPatch.referenceBody, screenTransform);

            // Done drawing lines.  Reset color to white, so we don't mess up anyone else.
            GL.Color(Color.white);
            GL.End();

            // Draw target vessel icons.
            Vector3 transformedPosition;
            foreach (CelestialBody moon in selectedPatch.referenceBody.orbitingBodies)
            {
                if (moon != targetBody)
                {
                    transformedPosition = screenTransform.MultiplyPoint3x4(moon.getTruePositionAtUT(now) - selectedPatch.referenceBody.getTruePositionAtUT(now));
                    DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, moon.orbitDriver.orbitColor, MapIcons.OtherIcon.PLANET);
                }
            }

            if (targetVessel != null || targetBody != null)
            {
                var orbit = (targetVessel != null) ? targetVessel.GetOrbit() : targetBody.GetOrbit();

                double tClosestApproach;

                if (targetVessel != null && targetVessel.LandedOrSplashed)
                {
                    Vector3d position = JUtil.ClosestApproachSrfOrbit(selectedPatch, targetVessel, out tClosestApproach, out double _);
                    transformedPosition = screenTransform.MultiplyPoint3x4(position);
                    DrawIcon(transformedPosition.x, transformedPosition.y, targetVessel.vesselType, iconColorTargetValue);
                }
                else
                {
                    JUtil.GetClosestApproach(selectedPatch, orbit, out tClosestApproach);

                    DrawNextPe(orbit, selectedPatch.referenceBody, now, iconColorTargetValue, screenTransform);
                    DrawNextAp(orbit, selectedPatch.referenceBody, now, iconColorTargetValue, screenTransform);

                    if (targetBody != null)
                    {
                        transformedPosition = screenTransform.MultiplyPoint3x4(targetBody.getTruePositionAtUT(now) - selectedPatch.referenceBody.getTruePositionAtUT(now));
                        DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, iconColorTargetValue, MapIcons.OtherIcon.PLANET);
                    }
                    else
                    {
                        transformedPosition = screenTransform.MultiplyPoint3x4(orbit.SwappedRelativePositionAtUT(now));
                        DrawIcon(transformedPosition.x, transformedPosition.y, targetVessel.vesselType, iconColorTargetValue);
                    }
                }

                if (selectedPatch.AscendingNodeExists(orbit))
                {
                    double anTime = selectedPatch.TimeOfAscendingNode(orbit, now);
                    if (anTime < selectedPatch.EndUT || (selectedPatch.patchEndTransition != Orbit.PatchTransitionType.ESCAPE && selectedPatch.patchEndTransition != Orbit.PatchTransitionType.ENCOUNTER))
                    {
                        transformedPosition = screenTransform.MultiplyPoint3x4(selectedPatch.SwappedRelativePositionAtUT(anTime));
                        DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, orbitColorSelfValue, MapIcons.OtherIcon.AN);
                    }
                }
                if (selectedPatch.DescendingNodeExists(orbit))
                {
                    double dnTime = selectedPatch.TimeOfDescendingNode(orbit, now);
                    if (dnTime < selectedPatch.EndUT || (selectedPatch.patchEndTransition != Orbit.PatchTransitionType.ESCAPE && selectedPatch.patchEndTransition != Orbit.PatchTransitionType.ENCOUNTER))
                    {
                        transformedPosition = screenTransform.MultiplyPoint3x4(selectedPatch.SwappedRelativePositionAtUT(dnTime));
                        DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, orbitColorSelfValue, MapIcons.OtherIcon.DN);
                    }
                }

                Orbit o = GetPatchAtUT(selectedPatch, tClosestApproach);
                if (o != null)
                {
                    Vector3d encounterPosition = o.SwappedRelativePositionAtUT(tClosestApproach) + o.referenceBody.getTruePositionAtUT(tClosestApproach) - selectedPatch.referenceBody.getTruePositionAtUT(tClosestApproach);
                    transformedPosition = screenTransform.MultiplyPoint3x4(encounterPosition);
                    DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, iconColorClosestApproachValue, MapIcons.OtherIcon.SHIPATINTERCEPT);
                }

                if (targetVessel == null || !targetVessel.LandedOrSplashed)
                {
                    // Unconditionally try to draw the closest approach point on
                    // the target orbit.
                    transformedPosition = screenTransform.MultiplyPoint3x4(orbit.SwappedRelativePositionAtUT(tClosestApproach));
                    DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, iconColorClosestApproachValue, MapIcons.OtherIcon.TGTATINTERCEPT);
                }
            }
            else
            {
                if (selectedPatch.AscendingNodeEquatorialExists())
                {
                    double anTime = selectedPatch.TimeOfAscendingNodeEquatorial(now);
                    if (anTime < selectedPatch.EndUT || (selectedPatch.patchEndTransition != Orbit.PatchTransitionType.ESCAPE && selectedPatch.patchEndTransition != Orbit.PatchTransitionType.ENCOUNTER))
                    {
                        transformedPosition = screenTransform.MultiplyPoint3x4(selectedPatch.SwappedRelativePositionAtUT(anTime));
                        DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, orbitColorSelfValue, MapIcons.OtherIcon.AN);
                    }
                }
                if (selectedPatch.DescendingNodeEquatorialExists())
                {
                    double dnTime = selectedPatch.TimeOfDescendingNodeEquatorial(now);
                    if (dnTime < selectedPatch.EndUT || (selectedPatch.patchEndTransition != Orbit.PatchTransitionType.ESCAPE && selectedPatch.patchEndTransition != Orbit.PatchTransitionType.ENCOUNTER))
                    {
                        transformedPosition = screenTransform.MultiplyPoint3x4(selectedPatch.SwappedRelativePositionAtUT(dnTime));
                        DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, orbitColorSelfValue, MapIcons.OtherIcon.DN);
                    }
                }
            }

            // Draw orbital features
            DrawNextPe(selectedPatch, selectedPatch.referenceBody, now, iconColorPEValue, screenTransform);

            DrawNextAp(selectedPatch, selectedPatch.referenceBody, now, iconColorAPValue, screenTransform);

            if (selectedPatch.nextPatch != null && selectedPatch.nextPatch.activePatch)
            {
                transformedPosition = screenTransform.MultiplyPoint3x4(selectedPatch.SwappedRelativePositionAtUT(selectedPatch.EndUT));
                DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, orbitColorSelfValue, MapIcons.OtherIcon.EXITSOI);

                Orbit nextPatch = selectedPatch.nextPatch.nextPatch;
                if (nextPatch != null && nextPatch.activePatch)
                {
                    transformedPosition = screenTransform.MultiplyPoint3x4(nextPatch.SwappedRelativePositionAtUT(nextPatch.EndUT) + nextPatch.referenceBody.getTruePositionAtUT(nextPatch.EndUT) - selectedPatch.referenceBody.getTruePositionAtUT(nextPatch.EndUT));
                    DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, orbitColorNextNodeValue, MapIcons.OtherIcon.EXITSOI);
                }
            }

            if (patchIndex > 0)
            {
                transformedPosition = screenTransform.MultiplyPoint3x4(selectedPatch.SwappedRelativePositionAtUT(selectedPatch.EndUT));
                DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, orbitColorSelfValue, MapIcons.OtherIcon.ENTERSOI);
            }

            if (node != null && node.nextPatch.activePatch)
                {
                    DrawNextPe(node.nextPatch, selectedPatch.referenceBody, now, orbitColorNextNodeValue, screenTransform);

                    DrawNextAp(node.nextPatch, selectedPatch.referenceBody, now, orbitColorNextNodeValue, screenTransform);

                    transformedPosition = screenTransform.MultiplyPoint3x4(selectedPatch.SwappedRelativePositionAtUT(node.UT));
                    DrawIcon(transformedPosition.x, transformedPosition.y, VesselType.Unknown, orbitColorNextNodeValue, MapIcons.OtherIcon.NODE);
                }

            // Draw ownship icon
            if (selectedPatch.referenceBody == vessel.orbit.referenceBody)
            {
                transformedPosition = screenTransform.MultiplyPoint3x4(vessel.orbit.SwappedRelativePositionAtUT(now));
                DrawIcon(transformedPosition.x, transformedPosition.y, vessel.vesselType, iconColorSelfValue);
            }

            GL.PopMatrix();
            GL.Viewport(new Rect(0, 0, screen.width, screen.height));

            return true;
        }

        private void DrawIcon(float xPos, float yPos, VesselType vt, Color iconColor, MapIcons.OtherIcon icon = MapIcons.OtherIcon.None)
        {
            var position = new Rect(xPos - iconPixelSize * 0.5f, yPos - iconPixelSize * 0.5f,
                               iconPixelSize, iconPixelSize);

            //Rect shadow = position;
            //shadow.x += iconShadowShift.x;
            //shadow.y += iconShadowShift.y;

            //MapView.OrbitIconsMaterial.color = iconColorShadowValue;
            //Graphics.DrawTexture(shadow, MapView.OrbitIconsMap, MapIcons.VesselTypeIcon(vt, icon), 0, 0, 0, 0, MapView.OrbitIconsMaterial);

			// the old icon material wasn't working, so just use this one
			// but I don't fully understand the color/blend system
			// a = 1.0 is far too faint; 4.0 looks pretty good
            MapView.OrbitIconsMaterial.color = new Color(iconColor.r, iconColor.g, iconColor.b, 4.0f);
            Graphics.DrawTexture(position, MapView.OrbitIconsMap, MapIcons.VesselTypeIcon(vt, icon), 0, 0, 0, 0, MapView.OrbitIconsMaterial);
			
			// if the icon texture ever changes, you can use this code to dump it out for inspection
			#if false
			var filepath = "orbiticonsmap.png";
			if (!System.IO.File.Exists(filepath))
			{
				var textureCopy = duplicateTexture(MapView.OrbitIconsMap);
				var textureBytes = textureCopy.EncodeToPNG();
				System.IO.File.WriteAllBytes(filepath, textureBytes);
			}
			#endif
        }

		Texture2D duplicateTexture(Texture2D source)
		{
			RenderTexture renderTex = RenderTexture.GetTemporary(
						source.width,
						source.height,
						0,
						RenderTextureFormat.Default,
						RenderTextureReadWrite.Linear);

			Graphics.Blit(source, renderTex);
			RenderTexture previous = RenderTexture.active;
			RenderTexture.active = renderTex;
			Texture2D readableText = new Texture2D(source.width, source.height);
			readableText.ReadPixels(new Rect(0, 0, renderTex.width, renderTex.height), 0, 0);
			readableText.Apply();
			RenderTexture.active = previous;
			RenderTexture.ReleaseTemporary(renderTex);
			return readableText;
		}

        public void Start()
        {
            // Skip the entire sequence when in editor -- this means we're part of a transparent pod and won't get used anyway.
            if (HighLogic.LoadedSceneIsEditor)
            {
                startupComplete = true;
                return;
            }

            try
            {
                if (!string.IsNullOrEmpty(backgroundColor))
                {
                    backgroundColorValue = ConfigNode.ParseColor32(backgroundColor);
                }
                if (!string.IsNullOrEmpty(iconColorSelf))
                {
                    iconColorSelfValue = ConfigNode.ParseColor32(iconColorSelf);
                }
                if (!string.IsNullOrEmpty(orbitColorSelf))
                {
                    orbitColorSelfValue = ConfigNode.ParseColor32(orbitColorSelf);
                }
                else
                {
                    orbitColorSelfValue = MapView.PatchColors[0];
                }
                if (!string.IsNullOrEmpty(iconColorTarget))
                {
                    iconColorTargetValue = ConfigNode.ParseColor32(iconColorTarget);
                }
                if (!string.IsNullOrEmpty(iconColorShadow))
                {
                    iconColorShadowValue = ConfigNode.ParseColor32(iconColorShadow);
                }
                if (!string.IsNullOrEmpty(iconColorAP))
                {
                    iconColorAPValue = ConfigNode.ParseColor32(iconColorAP);
                }
                else
                {
                    iconColorAPValue = MapView.PatchColors[0];
                }
                if (!string.IsNullOrEmpty(iconColorPE))
                {
                    iconColorPEValue = ConfigNode.ParseColor32(iconColorPE);
                }
                else
                {
                    iconColorPEValue = MapView.PatchColors[0];
                }
                if (!string.IsNullOrEmpty(orbitColorNextNode))
                {
                    orbitColorNextNodeValue = ConfigNode.ParseColor32(orbitColorNextNode);
                }
                else
                {
                    orbitColorNextNodeValue = MapView.PatchColors[1];
                }
                if (!string.IsNullOrEmpty(iconColorClosestApproach))
                {
                    iconColorClosestApproachValue = ConfigNode.ParseColor32(iconColorClosestApproach);
                }

                startupComplete = true;
            }
            catch
            {
                JUtil.AnnoyUser(this);
                throw;
            }
        }

        private class Bounds
        {
            public double maxX;
            public double minX;
            public double maxY;
            public double minY;

            public Bounds(double radius)
            {
                Add(radius, radius);
                Add(-radius, -radius);
            }

            public void Add(double x, double y)
            {
                maxX = Math.Max(maxX, x);
                minX = Math.Min(minX, x);
                maxY = Math.Max(maxY, y);
                minY = Math.Min(minY, y);
            }
        }
    }
}
