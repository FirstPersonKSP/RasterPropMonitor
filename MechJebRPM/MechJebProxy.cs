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
using System.Reflection;
using System.Linq;
using MuMech;
using MechJebLib.FuelFlowSimulation;
using UnityEngine;

namespace JSI
{
    /// <summary>
    /// MechJebProxy provides a direct-access interface to MechJeb 2.15+.
    /// Uses Krafs.Publicizer for compile-time access to MechJeb internals.
    /// </summary>
    public static class MechJebProxy
    {
        #region Initialization State
        private static bool initialized = false;
        private static bool mjAvailable = false;
        private static string initError = null;

        public static bool IsAvailable { get { return mjAvailable; } }
        public static string InitializationError { get { return initError; } }
        #endregion

        #region Operation Cache
        public static OperationAdvancedTransfer OpAdvancedTransfer { get; private set; }
        public static OperationCircularize OpCircularize { get; private set; }
        public static OperationApoapsis OpChangeApoapsis { get; private set; }
        public static OperationPeriapsis OpChangePeriapsis { get; private set; }
        public static OperationSemiMajor OpChangeSemiMajorAxis { get; private set; }
        public static OperationInclination OpChangeInclination { get; private set; }
        public static OperationLan OpChangeLAN { get; private set; }
        public static OperationGeneric OpGeneric { get; private set; }
        public static OperationEllipticize OpEllipticize { get; private set; }
        public static OperationEccentricity OpEccentricity { get; private set; }
        public static OperationCourseCorrection OpCourseCorrection { get; private set; }
        public static OperationLambert OpLambert { get; private set; }
        public static OperationLongitude OpLongitude { get; private set; }
        public static OperationResonantOrbit OpResonantOrbit { get; private set; }
        public static OperationMoonReturn OpMoonReturn { get; private set; }
        public static OperationInterplanetaryTransfer OpInterplanetaryTransfer { get; private set; }
        public static OperationPlane OpMatchPlane { get; private set; }
        public static OperationKillRelVel OpMatchVelocity { get; private set; }

        private static Dictionary<string, Operation> operationsByName = new Dictionary<string, Operation>();
        #endregion

        #region Public Enum Copies

        /// <summary>SmartASS mode categories</summary>
        public enum Mode
        {
            ORBITAL = 0,
            SURFACE = 1,
            TARGET = 2,
            ADVANCED = 3,
            AUTO = 4,
        }

        /// <summary>Translatron modes</summary>
        public enum TranslatronMode
        {
            OFF,
            KEEP_OBT,
            KEEP_SURF,
            KEEP_VERT,
            KEEP_REL,
            DIRECT,
        }

        /// <summary>Mapping from Target to Mode</summary>
        public static readonly Mode[] Target2Mode = new Mode[]
        {
            Mode.ORBITAL, // OFF
            Mode.ORBITAL, // KILLROT
            Mode.ORBITAL, // NODE
            Mode.SURFACE, // SURFACE
            Mode.ORBITAL, // PROGRADE
            Mode.ORBITAL, // RETROGRADE
            Mode.ORBITAL, // NORMAL_PLUS
            Mode.ORBITAL, // NORMAL_MINUS
            Mode.ORBITAL, // RADIAL_PLUS
            Mode.ORBITAL, // RADIAL_MINUS
            Mode.TARGET,  // RELATIVE_PLUS
            Mode.TARGET,  // RELATIVE_MINUS
            Mode.TARGET,  // TARGET_PLUS
            Mode.TARGET,  // TARGET_MINUS
            Mode.TARGET,  // PARALLEL_PLUS
            Mode.TARGET,  // PARALLEL_MINUS
            Mode.ADVANCED,// ADVANCED
            Mode.AUTO,    // AUTO
            Mode.SURFACE, // SURFACE_PROGRADE
            Mode.SURFACE, // SURFACE_RETROGRADE
            Mode.SURFACE, // HORIZONTAL_PLUS
            Mode.SURFACE, // HORIZONTAL_MINUS
            Mode.SURFACE, // VERTICAL_PLUS
        };

        /// <summary>Display texts for modes</summary>
        public static readonly string[] ModeTexts = new string[]
        {
            "OBT",
            "SURF",
            "TGT",
            "ADV",
            "AUTO",
        };

        /// <summary>Display texts for targets</summary>
        public static readonly string[] TargetTexts = new string[]
        {
            "OFF",
            "KILL\nROT",
            "NODE",
            "SURF",
            "PRO\nGRAD",
            "RETR\nGRAD",
            "NML\n+",
            "NML\n-",
            "RAD\n+",
            "RAD\n-",
            "RVEL\n+",
            "RVEL\n-",
            "TGT\n+",
            "TGT\n-",
            "PAR\n+",
            "PAR\n-",
            "ADV",
            "AUTO",
            "SVEL\n+",
            "SVEL\n-",
            "HVEL\n+",
            "HVEL\n-",
            "UP",
        };

        /// <summary>Landing prediction outcomes</summary>
        public enum LandingOutcome
        {
            LANDED,
            AEROBRAKED,
            TIMED_OUT,
            NO_REENTRY,
            ERROR
        }
        #endregion

        #region Initialization
        public static void Initialize()
        {
            if (initialized) return;
            initialized = true;

            try
            {
                // Verify MechJeb is loaded
                bool found = false;
                foreach (AssemblyLoader.LoadedAssembly la in AssemblyLoader.loadedAssemblies)
                {
                    if (la.assembly.GetName().Name == "MechJeb2")
                    {
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    initError = "MechJeb2 assembly not found";
                    return;
                }

                // Cache operation instances
                foreach (var op in Operation.GetAvailableOperations())
                {
                    if (op is OperationAdvancedTransfer oat) OpAdvancedTransfer = oat;
                    else if (op is OperationCircularize oc) OpCircularize = oc;
                    else if (op is OperationApoapsis oa) OpChangeApoapsis = oa;
                    else if (op is OperationPeriapsis ope) OpChangePeriapsis = ope;
                    else if (op is OperationSemiMajor osm) OpChangeSemiMajorAxis = osm;
                    else if (op is OperationInclination oi) OpChangeInclination = oi;
                    else if (op is OperationLan ol) OpChangeLAN = ol;
                    else if (op is OperationGeneric og) OpGeneric = og;
                    else if (op is OperationEllipticize oe) OpEllipticize = oe;
                    else if (op is OperationEccentricity oec) OpEccentricity = oec;
                    else if (op is OperationCourseCorrection occ) OpCourseCorrection = occ;
                    else if (op is OperationLambert olm) OpLambert = olm;
                    else if (op is OperationLongitude olo) OpLongitude = olo;
                    else if (op is OperationResonantOrbit oro) OpResonantOrbit = oro;
                    else if (op is OperationMoonReturn omr) OpMoonReturn = omr;
                    else if (op is OperationInterplanetaryTransfer oit) OpInterplanetaryTransfer = oit;
                    else if (op is OperationPlane opl) OpMatchPlane = opl;
                    else if (op is OperationKillRelVel okr) OpMatchVelocity = okr;

                    operationsByName[op.GetName()] = op;
                }

                mjAvailable = true;
                JUtil.LogMessage(null, "MechJebProxy: Successfully initialized");
            }
            catch (Exception ex)
            {
                initError = ex.Message;
                JUtil.LogErrorMessage(null, "MechJebProxy initialization failed: {0}\n{1}", ex.Message, ex.StackTrace);
            }
        }
        #endregion

        #region Core Module Accessors

        public static MechJebModuleTargetController GetTargetController(MechJebCore core)
        {
            if (core == null) return null;
            return core.Target;
        }

        public static MechJebModuleNodeExecutor GetNodeExecutor(MechJebCore core)
        {
            if (core == null) return null;
            return core.Node;
        }

        public static MechJebModuleStagingController GetStagingController(MechJebCore core)
        {
            if (core == null) return null;
            return core.Staging;
        }
        #endregion

        #region SmartASS Methods

        public static string GetSmartASSAdvancedReferenceName(MechJebModuleSmartASS smartass)
        {
            if (smartass == null) return "N/A";
            return smartass.advReference.ToString();
        }

        public static void CycleSmartASSAdvancedReference(MechJebModuleSmartASS smartass, int direction)
        {
            if (smartass == null) return;
            var values = (AttitudeReference[])Enum.GetValues(typeof(AttitudeReference));
            int idx = Array.IndexOf(values, smartass.advReference);
            if (idx < 0) idx = 0;
            int next = (idx + direction + values.Length) % values.Length;
            smartass.advReference = values[next];
        }

        public static string GetSmartASSAdvancedDirectionName(MechJebModuleSmartASS smartass)
        {
            if (smartass == null) return "N/A";
            return smartass.advDirection.ToString();
        }

        public static void CycleSmartASSAdvancedDirection(MechJebModuleSmartASS smartass, int direction)
        {
            if (smartass == null) return;
            var values = (Vector6.Direction[])Enum.GetValues(typeof(Vector6.Direction));
            int idx = Array.IndexOf(values, smartass.advDirection);
            if (idx < 0) idx = 0;
            int next = (idx + direction + values.Length) % values.Length;
            smartass.advDirection = values[next];
        }
        #endregion

        #region Node Executor Methods

        public static void ExecuteOneNode(MechJebCore core, object controller)
        {
            var node = GetNodeExecutor(core);
            if (node == null) return;
            node.ExecuteOneNode(controller);
        }

        public static void AbortNode(MechJebCore core)
        {
            var node = GetNodeExecutor(core);
            if (node == null) return;
            node.Abort();
        }

        public static bool IsNodeExecutorRunning(MechJebCore core)
        {
            var node = GetNodeExecutor(core);
            return node != null && node.Enabled;
        }

        public static bool GetNodeAutowarp(MechJebCore core)
        {
            var node = GetNodeExecutor(core);
            if (node == null) return false;
            return node.Autowarp;
        }

        public static void SetNodeAutowarp(MechJebCore core, bool autowarp)
        {
            var node = GetNodeExecutor(core);
            if (node == null) return;
            node.Autowarp = autowarp;
        }
        #endregion

        #region Target Controller Methods

        public static bool PositionTargetExists(MechJebCore core)
        {
            var target = GetTargetController(core);
            if (target == null) return false;
            return target.PositionTargetExists;
        }

        public static double GetTargetLatitude(MechJebCore core)
        {
            return core.Target.targetLatitude;
        }

        public static double GetTargetLongitude(MechJebCore core)
        {
            return core.Target.targetLongitude;
        }

        public static void SetTargetLatitude(MechJebCore core, CelestialBody body, double latitude)
        {
            if (body == null) return;
            double lon = GetTargetLongitude(core);
            SetPositionTarget(core, body, latitude, lon);
        }

        public static void SetTargetLongitude(MechJebCore core, CelestialBody body, double longitude)
        {
            if (body == null) return;
            double lat = GetTargetLatitude(core);
            SetPositionTarget(core, body, lat, longitude);
        }

        public static void SetPositionTarget(MechJebCore core, CelestialBody body, double latitude, double longitude)
        {
            var target = GetTargetController(core);
            if (target == null) return;
            target.SetPositionTarget(body, latitude, longitude);
        }

        public static void PickPositionTargetOnMap(MechJebCore core)
        {
            var target = GetTargetController(core);
            if (target == null) return;
            target.PickPositionTargetOnMap();
        }
        #endregion

        #region Ascent Methods

        public static MechJebModuleAscentSettings GetAscentSettings(MechJebCore core)
        {
            if (core == null) return null;
            return core.AscentSettings;
        }

        public static MechJebModuleAscentBaseAutopilot GetAscentAutopilot(MechJebCore core)
        {
            var settings = GetAscentSettings(core);
            if (settings == null) return null;
            return settings.AscentAutopilot;
        }

        public static bool IsAscentAutopilotEngaged(MechJebCore core)
        {
            var autopilot = GetAscentAutopilot(core);
            return autopilot != null && autopilot.Enabled;
        }

        public static void SetAscentAutopilotEngaged(MechJebCore core, bool engaged, object controller = null)
        {
            var autopilot = GetAscentAutopilot(core);
            if (autopilot == null) return;

            if (controller == null) controller = core;

            if (engaged)
                autopilot.Users.Add(controller);
            else
                autopilot.Users.Remove(controller);
        }

        public static bool GetAscentAutowarp(MechJebCore core)
        {
            return GetNodeAutowarp(core);
        }

        public static void SetAscentAutowarp(MechJebCore core, bool autowarp)
        {
            SetNodeAutowarp(core, autowarp);
        }

        public static bool GetAscentAutoPath(MechJebCore core)
        {
            var settings = GetAscentSettings(core);
            if (settings == null) return false;
            return settings.AutoPath;
        }

        public static bool GetAscentSkipCircularization(MechJebCore core)
        {
            var settings = GetAscentSettings(core);
            if (settings == null) return false;
            return settings.SkipCircularization;
        }

        public static void SetAscentSkipCircularization(MechJebCore core, bool value)
        {
            var settings = GetAscentSettings(core);
            if (settings == null) return;
            settings.SkipCircularization = value;
        }
        #endregion

        #region Landing Methods

        public static MechJebModuleLandingAutopilot GetLandingAutopilot(MechJebCore core)
        {
            return core.Landing;
        }

        public static MechJebModuleLandingPredictions GetLandingPredictions(MechJebCore core)
        {
            return core.GetComputerModule<MechJebModuleLandingPredictions>();
        }

        public static void LandAtPositionTarget(MechJebCore core, object controller = null)
        {
            var landing = GetLandingAutopilot(core);
            if (landing == null) return;
            if (controller == null) controller = core.GetComputerModule<MechJebModuleLandingGuidance>();
            landing.LandAtPositionTarget(controller);
        }

        public static void LandUntargeted(MechJebCore core, object controller = null)
        {
            var landing = GetLandingAutopilot(core);
            if (landing == null) return;
            if (controller == null) controller = core.GetComputerModule<MechJebModuleLandingGuidance>();
            landing.LandUntargeted(controller);
        }

        public static void StopLanding(MechJebCore core)
        {
            var landing = GetLandingAutopilot(core);
            if (landing == null) return;
            landing.StopLanding();
        }

        public static bool IsLandingAutopilotEngaged(MechJebCore core)
        {
            var landing = GetLandingAutopilot(core);
            return landing != null && landing.Enabled;
        }

        public static ReentrySimulation.Result GetLandingPredictionResult(MechJebCore core)
        {
            var predictions = GetLandingPredictions(core);
            if (predictions == null) return null;
            return predictions.Result;
        }

        public static void GetLandingEndPosition(ReentrySimulation.Result result, out double latitude, out double longitude)
        {
            latitude = result.EndPosition.Latitude;
            longitude = result.EndPosition.Longitude;
        }

        public static double GetLandingEndUT(ReentrySimulation.Result result)
        {
            return result.EndUT;
        }

        public static double GetLandingMaxDragGees(ReentrySimulation.Result result)
        {
            if (result == null) return 0;
            return result.MaxDragGees;
        }

        public static bool GetLandingShowTrajectory(MechJebCore core)
        {
            var predictions = GetLandingPredictions(core);
            if (predictions == null) return false;
            return predictions.showTrajectory;
        }

        public static void SetLandingShowTrajectory(MechJebCore core, bool show)
        {
            var predictions = GetLandingPredictions(core);
            if (predictions == null) return;
            predictions.showTrajectory = show;
        }
        #endregion

        #region Stage Stats Methods

        public static MechJebModuleStageStats GetStageStats(MechJebCore core)
        {
            if (core == null) return null;
            return core.GetComputerModule<MechJebModuleStageStats>();
        }

        public static void RequestStageStatsUpdate(MechJebCore core, object controller = null)
        {
            var stats = GetStageStats(core);
            if (stats == null) return;
            stats.RequestUpdate();
        }

        public static List<FuelStats> GetVacuumStageStats(MechJebCore core)
        {
            var stats = GetStageStats(core);
            if (stats == null) return null;
            return stats.VacStats;
        }

        public static List<FuelStats> GetAtmoStageStats(MechJebCore core)
        {
            var stats = GetStageStats(core);
            if (stats == null) return null;
            return stats.AtmoStats;
        }

        public static double GetStageDeltaV(FuelStats fuelStats)
        {
            return fuelStats.DeltaV;
        }

        public static double GetTotalVacuumDeltaV(MechJebCore core)
        {
            var stats = GetVacuumStageStats(core);
            if (stats == null) return 0;

            double total = 0;
            foreach (var stage in stats)
                total += stage.DeltaV;
            return total;
        }

        public static double GetTotalAtmoDeltaV(MechJebCore core)
        {
            var stats = GetAtmoStageStats(core);
            if (stats == null) return 0;

            double total = 0;
            foreach (var stage in stats)
                total += stage.DeltaV;
            return total;
        }
        #endregion

        #region Staging Controller Methods

        public static void AutostageOnce(MechJebCore core, object controller = null)
        {
            var staging = GetStagingController(core);
            if (staging == null) return;
            staging.AutostageOnce(controller);
        }
        #endregion

        #region Docking Methods

        public static MechJebModuleDockingAutopilot GetDockingAutopilot(MechJebCore core)
        {
            return core.GetComputerModule<MechJebModuleDockingAutopilot>();
        }

        public static bool IsDockingAutopilotEngaged(MechJebCore core)
        {
            var docking = GetDockingAutopilot(core);
            return docking != null && docking.Enabled;
        }

        public static void SetDockingAutopilotEngaged(MechJebCore core, bool engaged)
        {
            var docking = GetDockingAutopilot(core);
            if (docking == null) return;
            docking.Enabled = engaged;
        }

        public static string GetDockingStatus(MechJebCore core)
        {
            var docking = GetDockingAutopilot(core);
            if (docking == null) return "";
            return docking.status;
        }
        #endregion

        #region Rendezvous Methods

        public static MechJebModuleRendezvousAutopilot GetRendezvousAutopilot(MechJebCore core)
        {
            return core.GetComputerModule<MechJebModuleRendezvousAutopilot>();
        }

        public static bool IsRendezvousAutopilotEngaged(MechJebCore core)
        {
            var rendezvous = GetRendezvousAutopilot(core);
            return rendezvous != null && rendezvous.Enabled;
        }

        public static void SetRendezvousAutopilotEngaged(MechJebCore core, bool engaged, object controller = null)
        {
            var rendezvous = GetRendezvousAutopilot(core);
            if (rendezvous == null) return;

            if (controller == null) controller = core.GetComputerModule<MechJebModuleRendezvousAutopilotWindow>();

            if (controller == null) return;

            if (engaged)
                rendezvous.Users.Add(controller);
            else
                rendezvous.Users.Remove(controller);
        }

        public static string GetRendezvousStatus(MechJebCore core)
        {
            var rendezvous = GetRendezvousAutopilot(core);
            if (rendezvous == null) return "";
            return rendezvous.status;
        }
        #endregion

        #region Translatron Methods

        public static MechJebModuleTranslatron GetTranslatron(MechJebCore core)
        {
            return core.GetComputerModule<MechJebModuleTranslatron>();
        }

        public static void SetTranslatronMode(MechJebCore core, TranslatronMode mode)
        {
            var translatron = GetTranslatron(core);
            if (translatron == null) return;
            translatron.SetMode((MechJebModuleThrustController.TMode)(int)mode);
        }

        public static bool GetTranslatronKillH(MechJebCore core)
        {
            if (core == null) return false;
            return core.Thrust.TransKillH;
        }

        public static void SetTranslatronKillH(MechJebCore core, bool killH)
        {
            if (core == null) return;
            core.Thrust.TransKillH = killH;
        }

        public static void PanicSwitch(MechJebCore core)
        {
            var translatron = GetTranslatron(core);
            if (translatron == null) return;
            translatron.PanicSwitch();
        }
        #endregion

        #region Rover Methods

        public static void DriveToTarget(MechJebCore core)
        {
            var rover = core.Rover;
            if (rover == null) return;
            rover.ControlHeading = true;
            rover.ControlSpeed = true;
        }

        public static void StopRover(MechJebCore core)
        {
            var rover = core.Rover;
            if (rover == null) return;
            rover.ControlHeading = false;
            rover.ControlSpeed = false;
        }

        public static void AddRoverWaypointAtCurrentPosition(MechJebCore core, Vessel vessel)
        {
            var rover = core.Rover;
            if (rover == null || vessel == null) return;
            rover.Waypoints.Add(new MechJebWaypoint(vessel.latitude, vessel.longitude));
        }

        public static void ClearRoverWaypoints(MechJebCore core)
        {
            var rover = core.Rover;
            if (rover == null) return;
            rover.Waypoints.Clear();
        }
        #endregion

        #region Spaceplane Autopilot Methods

        public static void SpaceplaneHoldHeadingAndAltitude(MechJebModuleSpaceplaneAutopilot sp)
        {
            if (sp == null) return;
            var airplane = sp.Autopilot;
            if (airplane == null) return;
            airplane.HeadingHoldEnabled = true;
            airplane.AltitudeHoldEnabled = true;
        }
        #endregion

        #region Warp Methods

        public static MechJebModuleWarpController GetWarpController(MechJebCore core)
        {
            if (core == null) return null;
            return core.GetComputerModule<MechJebModuleWarpController>();
        }

        public static void WarpToUT(MechJebCore core, double ut)
        {
            var warp = GetWarpController(core);
            if (warp == null) return;
            warp.WarpToUT(ut);
        }
        #endregion

        #region Maneuver Calculator Methods

        public static void PlaceManeuverNode(Vessel vessel, Orbit orbit, Vector3d dV, double UT)
        {
            if (vessel == null) return;
            vessel.PlaceManeuverNode(orbit, dV, UT);
        }

        public static void StartAdvancedTransferCompute(OperationAdvancedTransfer operation, Orbit orbit, double ut, MechJebModuleTargetController targetController)
        {
            if (operation == null || targetController == null) return;

            operation.selectionMode = OperationAdvancedTransfer.Mode.LIMITED_TIME;

            Orbit targetOrbit = targetController.TargetOrbit;
            if (targetOrbit == null) return;

            operation.ComputeTimes(orbit, targetOrbit, ut);
            operation.ComputeStuff(orbit, ut, targetController);
        }

        public static bool IsAdvancedTransferFinished(OperationAdvancedTransfer operation, out int progress)
        {
            progress = 0;
            if (operation == null) return false;

            var worker = operation.worker;
            if (worker == null) return false;

            progress = worker.Progress;
            return worker.Finished;
        }

        public static void SelectAdvancedTransferLowestDV(OperationAdvancedTransfer operation)
        {
            // BestDate/BestDuration on the worker already represent lowest DV
        }

        public static void SelectAdvancedTransferASAP(OperationAdvancedTransfer operation)
        {
            if (operation == null) return;

            var worker = operation.worker;
            if (worker == null) return;

            double[,] computed = worker.Computed;
            if (computed == null) return;

            int bestDuration = 0;
            int durationCount = computed.GetLength(1);
            for (int i = 1; i < durationCount; i++)
            {
                if (computed[0, bestDuration] > computed[0, i])
                    bestDuration = i;
            }

            worker.BestDate = 0;
            worker.BestDuration = bestDuration;
        }

        public static bool GetAdvancedTransferSelection(OperationAdvancedTransfer operation, out double departureUT, out double duration, out double deltaV)
        {
            departureUT = 0;
            duration = 0;
            deltaV = 0;

            if (operation == null) return false;

            var worker = operation.worker;
            if (worker == null) return false;

            int bestDateIdx = worker.BestDate;
            int bestDurIdx = worker.BestDuration;

            departureUT = worker.DateFromIndex(bestDateIdx);
            duration = worker.DurationFromIndex(bestDurIdx);

            double[,] computed = worker.Computed;
            if (computed != null && bestDateIdx < computed.GetLength(0) && bestDurIdx < computed.GetLength(1))
            {
                deltaV = computed[bestDateIdx, bestDurIdx];
            }

            return true;
        }

        public static bool CreateNodesFromOperation(Operation operation, Orbit orbit, double ut, MechJebModuleTargetController targetController, Vessel vessel)
        {
            if (operation == null || vessel == null) return false;

            // For AdvancedTransfer, ensure lastTargetCelestial is set
            if (operation is OperationAdvancedTransfer advTransfer)
            {
                CelestialBody targetBody = FlightGlobals.fetch.VesselTarget as CelestialBody;
                if (targetBody != null)
                {
                    advTransfer.lastTargetCelestial = targetBody;
                }
            }

            var nodes = operation.MakeNodes(orbit, ut, targetController);
            if (nodes == null) return false;

            foreach (var node in nodes)
            {
                if (node == null) continue;
                PlaceManeuverNode(vessel, orbit, node.dV, node.UT);
            }

            return true;
        }

        public static Operation GetOperationByName(string name)
        {
            return operationsByName.GetValueOrDefault(name);
        }

        public static TimeSelector GetTimeSelector(this Operation operation)
        {
            Type opType = operation.GetType();
            FieldInfo field = opType.GetField("_timeSelector",
                BindingFlags.NonPublic | BindingFlags.Static);
            if (field == null) return null;

            return field.GetValue(null) as TimeSelector;
        }

        public static bool ExecuteOperation(Operation operation, MechJebCore core, Vessel vessel)
        {
            if (operation == null || core == null || vessel == null) return false;

            Orbit orbit = vessel.orbit;
            double ut = Planetarium.GetUniversalTime();
            var targetController = GetTargetController(core);

            return CreateNodesFromOperation(operation, orbit, ut, targetController, vessel);
        }
        #endregion
    }
}
