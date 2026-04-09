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
using MuMech;
using MechJebLib.FuelFlowSimulation;
using UnityEngine;

namespace JSI
{
    /// <summary>
    /// MechJebProxy handles MechJeb initialization, operation caching,
    /// and complex multi-step MechJeb interactions.
    /// Simple one-liner accessors are called directly on MJ types from MechJebRPM.
    /// </summary>
    public static class MechJebProxy
    {
        #region Initialization
        private static bool initialized = false;
        private static bool mjAvailable = false;
        private static string initError = null;

        public static bool IsAvailable { get { return mjAvailable; } }
        public static string InitializationError { get { return initError; } }

        public static void Initialize()
        {
            if (initialized) return;
            initialized = true;

            try
            {
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

        #region Enums
        public enum TranslatronMode
        {
            OFF,
            KEEP_OBT,
            KEEP_SURF,
            KEEP_VERT,
            KEEP_REL,
            DIRECT,
        }
        #endregion

        #region Autopilot Engagement

        public static void SetAscentAutopilotEngaged(MechJebCore core, bool engaged, object controller = null)
        {
            var autopilot = core?.AscentSettings?.AscentAutopilot;
            if (autopilot == null) return;

            if (controller == null) controller = core;

            if (engaged)
                autopilot.Users.Add(controller);
            else
                autopilot.Users.Remove(controller);
        }

        public static void SetRendezvousAutopilotEngaged(MechJebCore core, bool engaged, object controller = null)
        {
            var rendezvous = core?.GetComputerModule<MechJebModuleRendezvousAutopilot>();
            if (rendezvous == null) return;

            if (controller == null) controller = core.GetComputerModule<MechJebModuleRendezvousAutopilotWindow>();
            if (controller == null) return;

            if (engaged)
                rendezvous.Users.Add(controller);
            else
                rendezvous.Users.Remove(controller);
        }

        public static void SetTranslatronMode(MechJebCore core, TranslatronMode mode)
        {
            var translatron = core?.GetComputerModule<MechJebModuleTranslatron>();
            if (translatron == null) return;
            translatron.SetMode((MechJebModuleThrustController.TMode)(int)mode);
        }
        #endregion

        #region Stage Stats

        public static List<FuelStats> GetVacuumStageStats(MechJebCore core)
        {
            return core?.GetComputerModule<MechJebModuleStageStats>()?.VacStats;
        }

        public static List<FuelStats> GetAtmoStageStats(MechJebCore core)
        {
            return core?.GetComputerModule<MechJebModuleStageStats>()?.AtmoStats;
        }

        public static double GetTotalVacuumDeltaV(MechJebCore core)
        {
            var stats = GetVacuumStageStats(core);
            if (stats == null) return 0;
            double total = 0;
            foreach (var stage in stats) total += stage.DeltaV;
            return total;
        }

        public static double GetTotalAtmoDeltaV(MechJebCore core)
        {
            var stats = GetAtmoStageStats(core);
            if (stats == null) return 0;
            double total = 0;
            foreach (var stage in stats) total += stage.DeltaV;
            return total;
        }
        #endregion

        #region Advanced Transfer

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
        #endregion

        #region Operation Execution

        public static bool CreateNodesFromOperation(Operation operation, Orbit orbit, double ut, MechJebModuleTargetController targetController, Vessel vessel)
        {
            if (operation == null || vessel == null) return false;

            if (operation is OperationAdvancedTransfer advTransfer)
            {
                CelestialBody targetBody = FlightGlobals.fetch.VesselTarget as CelestialBody;
                if (targetBody != null)
                    advTransfer.lastTargetCelestial = targetBody;
            }

            var nodes = operation.MakeNodes(orbit, ut, targetController);
            if (nodes == null) return false;

            foreach (var node in nodes)
            {
                if (node == null) continue;
                vessel.PlaceManeuverNode(orbit, node.dV, node.UT);
            }

            return true;
        }

        public static bool ExecuteOperation(Operation operation, MechJebCore core, Vessel vessel)
        {
            if (operation == null || core == null || vessel == null) return false;
            return CreateNodesFromOperation(operation, vessel.orbit, Planetarium.GetUniversalTime(), core.Target, vessel);
        }

        public static TimeSelector GetTimeSelector(this Operation operation)
        {
            FieldInfo field = operation.GetType().GetField("_timeSelector",
                BindingFlags.NonPublic | BindingFlags.Static);
            if (field == null) return null;
            return field.GetValue(null) as TimeSelector;
        }
        #endregion
    }
}
