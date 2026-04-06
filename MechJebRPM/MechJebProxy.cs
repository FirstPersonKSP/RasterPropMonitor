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

namespace JSI
{
    /// <summary>
    /// MechJebProxy provides a comprehensive reflection-based interface to MechJeb 2.15+
    /// This is a complete rewrite for full feature parity with MechJeb's IMGUI interface.
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

        #region Assembly References
        private static Assembly mechJebAssembly;
        private static Assembly mechJebLibAssembly;
        #endregion

        #region Type Cache
        // Core Types
        private static Type t_MechJebCore;
        private static Type t_ComputerModule;
        private static Type t_DisplayModule;
        private static Type t_VesselState;
        private static Type t_VesselExtensions;
        
        // Editable Types
        private static Type t_EditableDouble;
        private static Type t_EditableDoubleMult;
        private static Type t_EditableInt;
        private static Type t_EditableAngle;
        
        // Module Types
        private static Type t_SmartASS;
        private static Type t_AttitudeController;
        private static Type t_ThrustController;
        private static Type t_StagingController;
        private static Type t_NodeExecutor;
        private static Type t_TargetController;
        private static Type t_AscentSettings;
        private static Type t_AscentBaseAutopilot;
        private static Type t_AscentClassicAutopilot;
        private static Type t_AscentPSGAutopilot;
        private static Type t_AscentMenu;
        private static Type t_LandingAutopilot;
        private static Type t_LandingPredictions;
        private static Type t_LandingGuidance;
        private static Type t_ManeuverPlanner;
        private static Type t_RendezvousAutopilot;
        private static Type t_RendezvousGuidance;
        private static Type t_DockingAutopilot;
        private static Type t_DockingGuidance;
        private static Type t_Translatron;
        private static Type t_RoverController;
        private static Type t_AirplaneAutopilot;
        private static Type t_SpaceplaneAutopilot;
        private static Type t_StageStats;
        private static Type t_RCSBalancer;
        private static Type t_WarpController;
        private static Type t_Settings;

        // Maneuver/Operations Types
        private static Type t_OrbitalManeuverCalculator;
        private static Type t_Operation;
        private static Type t_OperationAdvancedTransfer;
        private static Type t_OperationCourseCorrection;
        private static Type t_ManeuverParameters;
        private static Type t_TransferCalculator;
        
        // Data Types
        private static Type t_AbsoluteVector;
        private static Type t_ReentryResult;
        private static Type t_FuelStats;
        private static Type t_UserPool;
        
        // Enum Types
        private static Type t_SmartASSTarget;
        private static Type t_SmartASSMode;
        private static Type t_AttitudeReference;
        private static Type t_TranslatronMode;
        #endregion

        #region Method/Field/Property Cache
        // VesselExtensions
        private static MethodInfo m_GetMasterMechJeb;
        private static MethodInfo m_PlaceManeuverNode;
        
        // MechJebCore
        private static MethodInfo m_GetComputerModule;
        private static FieldInfo f_Core_Target;
        private static FieldInfo f_Core_Node;
        private static FieldInfo f_Core_Attitude;
        private static FieldInfo f_Core_Thrust;
        private static FieldInfo f_Core_Staging;
        private static FieldInfo f_Core_VesselState;
        
        // ComputerModule
        private static PropertyInfo p_Module_Enabled;
        private static FieldInfo f_Module_Users;
        
        // UserPool
        private static MethodInfo m_UserPool_Add;
        private static MethodInfo m_UserPool_Remove;
        private static MethodInfo m_UserPool_Contains;
        
        // EditableDouble
        private static PropertyInfo p_EditableDouble_Val;
        
        // EditableDoubleMult
        private static PropertyInfo p_EditableDoubleMult_Val;
        
        // EditableInt
        private static PropertyInfo p_EditableInt_Val;
        
        // EditableAngle - converts to double
        private static MethodInfo m_EditableAngle_ToDouble;
        
        // AbsoluteVector
        private static FieldInfo f_AbsoluteVector_Latitude;
        private static FieldInfo f_AbsoluteVector_Longitude;
        private static MethodInfo m_AbsoluteVector_ToDouble;
        
        // VesselState
        private static MethodInfo m_VesselState_TerminalVelocity;
        
        // SmartASS
        private static FieldInfo f_SmartASS_Target;
        private static FieldInfo f_SmartASS_Mode;
        internal static FieldInfo f_SmartASS_ForceRol;
        private static FieldInfo f_SmartASS_Rol;
        private static FieldInfo f_SmartASS_SrfHdg;
        private static FieldInfo f_SmartASS_SrfPit;
        private static FieldInfo f_SmartASS_SrfRol;
        private static FieldInfo f_SmartASS_SrfVelYaw;
        private static FieldInfo f_SmartASS_SrfVelPit;
        private static FieldInfo f_SmartASS_SrfVelRol;
        private static FieldInfo f_SmartASS_AdvReference;
        private static FieldInfo f_SmartASS_AdvDirection;
        private static MethodInfo m_SmartASS_Engage;
        private static FieldInfo f_SmartASS_ModeTexts;
        private static FieldInfo f_SmartASS_TargetTexts;
        
        // NodeExecutor
        private static MethodInfo m_NodeExecutor_ExecuteOneNode;
        private static MethodInfo m_NodeExecutor_ExecuteAllNodes;
        private static MethodInfo m_NodeExecutor_Abort;
        private static FieldInfo f_NodeExecutor_Autowarp;
        private static FieldInfo f_NodeExecutor_LeadTime;
        
        // TargetController
        private static PropertyInfo p_Target_PositionTargetExists;
        private static PropertyInfo p_Target_NormalTargetExists;
        private static PropertyInfo p_Target_TargetOrbit;
        private static FieldInfo f_Target_TargetLatitude;
        private static FieldInfo f_Target_TargetLongitude;
        private static MethodInfo m_Target_SetPositionTarget;
        private static MethodInfo m_Target_PickPositionTargetOnMap;
        
        // AscentSettings
        private static FieldInfo f_Ascent_DesiredOrbitAltitude;
        private static FieldInfo f_Ascent_DesiredApoapsis;
        private static FieldInfo f_Ascent_DesiredInclination;
        private static FieldInfo f_Ascent_DesiredLAN;
        private static FieldInfo f_Ascent_LaunchPhaseAngle;
        private static FieldInfo f_Ascent_LaunchLANDifference;
        private static PropertyInfo p_Ascent_AscentAutopilot;
        private static FieldInfo f_Ascent_WarpCountDown;
        private static FieldInfo f_Ascent_SkipCircularization;
        internal static FieldInfo f_Ascent_ForceRoll;
        private static FieldInfo f_Ascent_VerticalRoll;
        private static FieldInfo f_Ascent_TurnRoll;
        private static FieldInfo f_Ascent_RollAltitude;
        internal static FieldInfo f_Ascent_LimitAoA;
        private static FieldInfo f_Ascent_MaxAoA;
        private static FieldInfo f_Ascent_AOALimitFadeoutPressure;
        internal static FieldInfo f_Ascent_CorrectiveSteering;
        private static FieldInfo f_Ascent_CorrectiveSteeringGain;
        private static FieldInfo f_Ascent_TurnStartAltitude;
        private static FieldInfo f_Ascent_TurnStartVelocity;
        private static FieldInfo f_Ascent_TurnEndAltitude;
        private static FieldInfo f_Ascent_TurnEndAngle;
        private static FieldInfo f_Ascent_TurnShapeExponent;
        internal static FieldInfo f_Ascent_AutoPath;
        private static FieldInfo f_Ascent_AutoTurnPerc;
        private static FieldInfo f_Ascent_AutoTurnSpdFactor;
        private static PropertyInfo p_Ascent_Autostage;
        internal static FieldInfo f_Ascent_Autostage;
        
        // AscentBaseAutopilot
        private static PropertyInfo p_AscentAP_Status;
        private static MethodInfo m_AscentAP_StartCountdown;
        
        // LandingAutopilot
        private static MethodInfo m_Landing_LandAtPositionTarget;
        private static MethodInfo m_Landing_LandUntargeted;
        private static MethodInfo m_Landing_StopLanding;
        private static FieldInfo f_Landing_TouchdownSpeed;
        internal static FieldInfo f_Landing_DeployGears;
        internal static FieldInfo f_Landing_DeployChutes;
        private static FieldInfo f_Landing_LimitGearsStage;
        private static FieldInfo f_Landing_LimitChutesStage;
        internal static FieldInfo f_Landing_UseRCS;
        private static PropertyInfo p_Landing_Status;
        
        // LandingPredictions
        private static MethodInfo m_Predictions_GetResult;
        private static PropertyInfo p_Predictions_ShowTrajectory;
        
        // ReentrySimulation.Result
        private static FieldInfo f_Result_Outcome;
        private static FieldInfo f_Result_EndPosition;
        private static FieldInfo f_Result_EndUT;
        private static FieldInfo f_Result_MaxDragGees;
        
        // ThrustController
        internal static FieldInfo f_Thrust_LimitToPreventOverheats;
        internal static FieldInfo f_Thrust_LimitToTerminalVelocity;
        internal static FieldInfo f_Thrust_LimitToMaxDynamicPressure;
        private static FieldInfo f_Thrust_MaxDynamicPressure;
        internal static FieldInfo f_Thrust_LimitAcceleration;
        private static FieldInfo f_Thrust_MaxAcceleration;
        internal static FieldInfo f_Thrust_LimitThrottle;
        private static FieldInfo f_Thrust_MaxThrottle;
        private static FieldInfo f_Thrust_LimiterMinThrottle;
        private static FieldInfo f_Thrust_MinThrottle;
        internal static FieldInfo f_Thrust_LimitToPreventFlameout;
        private static FieldInfo f_Thrust_FlameoutSafetyPct;
        internal static FieldInfo f_Thrust_SmoothThrottle;
        internal static FieldInfo f_Thrust_ManageIntakes;
        internal static FieldInfo f_Thrust_DifferentialThrottle;
        private static FieldInfo f_Thrust_DifferentialThrottleSuccess;
        
        // StagingController
        internal static FieldInfo f_Staging_Autostage;
        private static FieldInfo f_Staging_AutostageLimit;
        private static FieldInfo f_Staging_AutostagePreDelay;
        private static FieldInfo f_Staging_AutostagePostDelay;
        private static FieldInfo f_Staging_ClampAutoStageThrustPct;
        private static FieldInfo f_Staging_FairingMaxAerothermalFlux;
        private static FieldInfo f_Staging_FairingMaxDynamicPressure;
        private static FieldInfo f_Staging_FairingMinAltitude;
        private static FieldInfo f_Staging_HotStagingLeadTime;
        internal static FieldInfo f_Staging_DropSolids;
        private static FieldInfo f_Staging_DropSolidsLeadTime;
        private static MethodInfo m_Staging_AutostageOnce;
        
        // StageStats
        private static MethodInfo m_StageStats_RequestUpdate;
        private static FieldInfo f_StageStats_VacStats;
        private static FieldInfo f_StageStats_AtmoStats;
        
        // FuelStats
        private static FieldInfo f_FuelStats_DeltaV;
        private static FieldInfo f_FuelStats_DeltaTime;
        private static FieldInfo f_FuelStats_StartMass;
        private static FieldInfo f_FuelStats_EndMass;
        private static FieldInfo f_FuelStats_StartThrust;
        private static FieldInfo f_FuelStats_MaxAccel;
        private static FieldInfo f_FuelStats_Isp;

        // ManeuverParameters
        private static FieldInfo f_ManeuverParameters_dV;
        private static FieldInfo f_ManeuverParameters_UT;
        
        // Translatron
        private static PropertyInfo p_Translatron_TransSpd;
        private static FieldInfo f_Translatron_TransSpdAct;
        private static PropertyInfo p_Translatron_TransKillH;
        private static MethodInfo m_Translatron_SetMode;
        private static MethodInfo m_Translatron_PanicSwitch;
        
        // RendezvousAutopilot
        private static FieldInfo f_Rendezvous_DesiredDistance;
        private static FieldInfo f_Rendezvous_MaxPhasingOrbits;
        private static FieldInfo f_Rendezvous_MaxClosingSpeed;
        private static PropertyInfo p_Rendezvous_Status;
        
        // DockingAutopilot
        private static FieldInfo f_Docking_SpeedLimit;
        internal static FieldInfo f_Docking_ForceRoll;
        private static FieldInfo f_Docking_Roll;
        internal static FieldInfo f_Docking_OverrideSafeDistance;
        private static FieldInfo f_Docking_OverridenSafeDistance;
        internal static FieldInfo f_Docking_OverrideTargetSize;
        private static FieldInfo f_Docking_OverridenTargetSize;
        internal static FieldInfo f_Docking_DrawBoundingBox;
        private static PropertyInfo p_Docking_Status;
        private static PropertyInfo p_Docking_SASCalc_X;
        private static PropertyInfo p_Docking_SASCalc_Y;
        private static PropertyInfo p_Docking_SASCalc_Z;
        
        // RoverController
        internal static FieldInfo f_Rover_ControlHeading;
        internal static FieldInfo f_Rover_ControlSpeed;
        internal static FieldInfo f_Rover_Heading;
        private static FieldInfo f_Rover_Speed;
        private static FieldInfo f_Rover_HeadingError;
        private static FieldInfo f_Rover_SpeedError;
        internal static FieldInfo f_Rover_StabilityControl;
        internal static FieldInfo f_Rover_BrakeOnEject;
        internal static FieldInfo f_Rover_BrakeOnEnergyDepletion;
        internal static FieldInfo f_Rover_WarpToDaylight;
        private static MethodInfo m_Rover_DriveToTarget;
        private static MethodInfo m_Rover_Stop;
        private static MethodInfo m_Rover_AddWaypoint;
        private static MethodInfo m_Rover_ClearWaypoints;
        
        // AirplaneAutopilot
        private static FieldInfo f_Airplane_AltitudeHold;
        private static FieldInfo f_Airplane_AltitudeTarget;
        private static FieldInfo f_Airplane_VertSpeedHold;
        private static FieldInfo f_Airplane_VertSpeedTarget;
        private static FieldInfo f_Airplane_HeadingHold;
        private static FieldInfo f_Airplane_HeadingTarget;
        internal static FieldInfo f_Airplane_RollHold;
        private static FieldInfo f_Airplane_RollTarget;
        private static FieldInfo f_Airplane_RollMax;
        private static FieldInfo f_Airplane_SpeedHold;
        private static FieldInfo f_Airplane_SpeedTarget;
        private static FieldInfo f_Airplane_AccKp;
        private static FieldInfo f_Airplane_AccKi;
        private static FieldInfo f_Airplane_AccKd;
        private static FieldInfo f_Airplane_PitKp;
        private static FieldInfo f_Airplane_PitKi;
        private static FieldInfo f_Airplane_PitKd;
        private static FieldInfo f_Airplane_RolKp;
        private static FieldInfo f_Airplane_RolKi;
        private static FieldInfo f_Airplane_RolKd;
        private static FieldInfo f_Airplane_YawKi;
        private static FieldInfo f_Airplane_YawKd;
        
        // SpaceplaneAutopilot
        private static FieldInfo f_Spaceplane_Glideslope;
        private static FieldInfo f_Spaceplane_ApproachSpeed;
        private static FieldInfo f_Spaceplane_TouchdownSpeed;
        private static FieldInfo f_Spaceplane_Mode;
        private static MethodInfo m_Spaceplane_Autoland;
        private static MethodInfo m_Spaceplane_HoldHeadingAndAltitude;
        private static MethodInfo m_Spaceplane_AutopilotOff;
        
        // WarpController
        private static MethodInfo m_Warp_WarpToUT;
        
        // OrbitalManeuverCalculator (static methods)
        private static MethodInfo m_Calc_DeltaVToCircularize;
        private static MethodInfo m_Calc_DeltaVToChangeApoapsis;
        private static MethodInfo m_Calc_DeltaVToChangePeriapsis;
        private static MethodInfo m_Calc_DeltaVToMatchVelocities;
        private static MethodInfo m_Calc_DeltaVAndTimeForHohmannTransfer;
        private static MethodInfo m_Calc_DeltaVForSemiMajorAxis;
        private static MethodInfo m_Calc_DeltaVToChangeInclination;
        private static MethodInfo m_Calc_DeltaVToMatchPlanesAscending;
        private static MethodInfo m_Calc_DeltaVToMatchPlanesDescending;
        private static MethodInfo m_Calc_DeltaVToShiftLAN;
        private static MethodInfo m_Calc_DeltaVToEllipticize;

        // Operation (maneuver planner)
        private static MethodInfo m_Operation_MakeNodes;
        private static MethodInfo m_Operation_GetErrorMessage;
        private static MethodInfo m_Operation_GetName;

        // ManeuverPlanner static fields (wrapping MechJeb's actual instances)
        private static FieldInfo f_ManeuverPlanner_operation;  // static Operation[] _operation
        private static FieldInfo f_ManeuverPlanner_operationId;  // [Persistent] int _operationId

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

		private static Dictionary<string, Operation> operationsByName = new Dictionary<string, Operation>();


        // TimeSelector for operation timing options
        private static Type t_TimeSelector;
        private static MethodInfo m_TimeSelector_ComputeManeuverTime;
        private static FieldInfo f_TimeSelector_CurrentTimeRef;
        private static FieldInfo f_TimeSelector_CircularizeAltitude;
        private static FieldInfo f_TimeSelector_LeadTime;
        private static PropertyInfo p_TimeSelector_TimeReference;
        private static Type t_TimeReference;

        // OperationAdvancedTransfer internals
        private static FieldInfo f_AdvancedTransfer_SelectionMode;
        private static FieldInfo f_AdvancedTransfer_Worker;
        private static FieldInfo f_AdvancedTransfer_Plot;
        private static FieldInfo f_AdvancedTransfer_IncludeCaptureBurn;
        private static FieldInfo f_AdvancedTransfer_PeriapsisHeight;
        private static FieldInfo f_AdvancedTransfer_LastTargetCelestial;
        private static MethodInfo m_AdvancedTransfer_ComputeTimes;
        private static MethodInfo m_AdvancedTransfer_ComputeStuff;

        // OperationGeneric (Hohmann/bi-impulsive transfer) internals
        private static Type t_OperationGeneric;
        private static FieldInfo f_Generic_Capture;
        internal static FieldInfo f_Generic_PlanCapture;
        private static FieldInfo f_Generic_Rendezvous;
        internal static FieldInfo f_Generic_Coplanar;
        private static FieldInfo f_Generic_LagTime;

        internal static FieldInfo f_InterplanetaryTransfer_WaitForPhaseAngle;

        // OperationCourseCorrection internals
        private static FieldInfo f_CourseCorrection_TargetPe;
        private static PropertyInfo p_CourseCorrection_TargetPe;

        // PlotArea (porkchop plot)
        private static Type t_PlotArea;
        private static PropertyInfo p_PlotArea_SelectedPoint;

        // TransferCalculator internals
        private static FieldInfo f_TransferCalculator_Computed;
        private static FieldInfo f_TransferCalculator_BestDate;
        private static FieldInfo f_TransferCalculator_BestDuration;
        private static PropertyInfo p_TransferCalculator_Finished;
        private static PropertyInfo p_TransferCalculator_Progress;
        private static PropertyInfo p_TransferCalculator_ArrivalDate;
        private static MethodInfo m_TransferCalculator_DateFromIndex;
        private static MethodInfo m_TransferCalculator_DurationFromIndex;
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
        /// <summary>
        /// Initialize all reflection bindings. Called once at startup.
        /// </summary>
        public static void Initialize()
        {
            if (initialized) return;
            initialized = true;

            try
            {
                // Find MechJeb assembly
                foreach (AssemblyLoader.LoadedAssembly la in AssemblyLoader.loadedAssemblies)
                {
                    if (la.assembly.GetName().Name == "MechJeb2")
                    {
                        mechJebAssembly = la.assembly;
                    }
                    else if (la.assembly.GetName().Name == "MechJebLib")
                    {
                        mechJebLibAssembly = la.assembly;
                    }
                }

                if (mechJebAssembly == null)
                {
                    initError = "MechJeb2 assembly not found";
                    return;
                }

                // cache references to operations
                foreach (var op in Operation.GetAvailableOperations())
                {
                    if (op is OperationAdvancedTransfer opAdvancedTransfer) OpAdvancedTransfer = opAdvancedTransfer;
                    else if (op is OperationCircularize opCircularize) OpCircularize = opCircularize;
                    else if (op is OperationApoapsis opChangeApoapsis) OpChangeApoapsis = opChangeApoapsis;
                    else if (op is OperationPeriapsis opPeriapsis) OpChangePeriapsis = opPeriapsis;
                    else if (op is OperationSemiMajor operationSemiMajor) OpChangeSemiMajorAxis = operationSemiMajor;
                    else if (op is OperationInclination operationInclination) OpChangeInclination = operationInclination;
                    else if (op is OperationLan operationLan) OpChangeLAN = operationLan;
                    else if (op is OperationGeneric operationGeneric) OpGeneric = operationGeneric;
                    else if (op is OperationEllipticize operationEllipticize) OpEllipticize = operationEllipticize;
                    else if (op is OperationEccentricity operationEccentricity) OpEccentricity = operationEccentricity;
                    else if (op is OperationCourseCorrection operationCourseCorrection) OpCourseCorrection = operationCourseCorrection;
                    else if (op is OperationLambert operationLambert) OpLambert = operationLambert;
                    else if (op is OperationLongitude operationLongitude) OpLongitude = operationLongitude;
                    else if (op is OperationResonantOrbit operationResonantOrbit) OpResonantOrbit = operationResonantOrbit;
                    else if (op is OperationMoonReturn operationMoonReturn) OpMoonReturn = operationMoonReturn;
                    else if (op is OperationInterplanetaryTransfer operationInterplanetaryTransfer) OpInterplanetaryTransfer = operationInterplanetaryTransfer;



                    operationsByName[op.GetName()] = op; // review: GetName is localized, so this needs to change.  Maybe the type name?
                }

                // Initialize all types and members
                InitializeCoreTypes();
                InitializeEditableTypes();
                InitializeModuleTypes();
                InitializeDataTypes();
                InitializeEnumTypes();
                InitializeMethods();
                
                mjAvailable = true;
                JUtil.LogMessage(null, "MechJebProxy: Successfully initialized with full feature parity");
            }
            catch (Exception ex)
            {
                initError = ex.Message;
                JUtil.LogErrorMessage(null, "MechJebProxy initialization failed: {0}\n{1}", ex.Message, ex.StackTrace);
            }
        }

        private static void InitializeCoreTypes()
        {
            t_MechJebCore = mechJebAssembly.GetType("MuMech.MechJebCore");
            if (t_MechJebCore == null) throw new Exception("MechJebCore not found");

            t_ComputerModule = mechJebAssembly.GetType("MuMech.ComputerModule");
            if (t_ComputerModule == null) throw new Exception("ComputerModule not found");

            t_DisplayModule = mechJebAssembly.GetType("MuMech.DisplayModule");
            if (t_DisplayModule == null) throw new Exception("DisplayModule not found");

            t_VesselState = mechJebAssembly.GetType("MuMech.VesselState");
            if (t_VesselState == null) throw new Exception("VesselState not found");

            t_VesselExtensions = mechJebAssembly.GetType("MuMech.VesselExtensions");
            if (t_VesselExtensions == null) throw new Exception("VesselExtensions not found");
        }

        private static void InitializeEditableTypes()
        {
            t_EditableDouble = mechJebAssembly.GetType("MuMech.EditableDouble");
            t_EditableDoubleMult = mechJebAssembly.GetType("MuMech.EditableDoubleMult");
            t_EditableInt = mechJebAssembly.GetType("MuMech.EditableInt");
            t_EditableAngle = mechJebAssembly.GetType("MuMech.EditableAngle");

            if (t_EditableDouble != null)
            {
                p_EditableDouble_Val = t_EditableDouble.GetProperty("Val", BindingFlags.Public | BindingFlags.Instance);
            }
            if (t_EditableDoubleMult != null)
            {
                p_EditableDoubleMult_Val = t_EditableDoubleMult.GetProperty("Val", BindingFlags.Public | BindingFlags.Instance);
            }
            if (t_EditableInt != null)
            {
                p_EditableInt_Val = t_EditableInt.GetProperty("Val", BindingFlags.Public | BindingFlags.Instance);
            }
            if (t_EditableAngle != null)
            {
                // EditableAngle has implicit conversion to double
                m_EditableAngle_ToDouble = t_EditableAngle.GetMethods(BindingFlags.Public | BindingFlags.Static)
                    .FirstOrDefault(m => m.Name == "op_Implicit" && m.ReturnType == typeof(double));
            }
        }

        private static void InitializeModuleTypes()
        {
            // Get all module types
            t_SmartASS = mechJebAssembly.GetType("MuMech.MechJebModuleSmartASS");
            t_AttitudeController = mechJebAssembly.GetType("MuMech.MechJebModuleAttitudeController");
            t_ThrustController = mechJebAssembly.GetType("MuMech.MechJebModuleThrustController");
            t_StagingController = mechJebAssembly.GetType("MuMech.MechJebModuleStagingController");
            t_NodeExecutor = mechJebAssembly.GetType("MuMech.MechJebModuleNodeExecutor");
            t_TargetController = mechJebAssembly.GetType("MuMech.MechJebModuleTargetController");
            t_AscentSettings = mechJebAssembly.GetType("MuMech.MechJebModuleAscentSettings");
            t_AscentBaseAutopilot = mechJebAssembly.GetType("MuMech.MechJebModuleAscentBaseAutopilot");
            t_AscentClassicAutopilot = mechJebAssembly.GetType("MuMech.MechJebModuleAscentClassicAutopilot");
            t_AscentPSGAutopilot = mechJebAssembly.GetType("MuMech.MechJebModuleAscentPSGAutopilot");
            t_AscentMenu = mechJebAssembly.GetType("MuMech.MechJebModuleAscentMenu");
            t_LandingAutopilot = mechJebAssembly.GetType("MuMech.MechJebModuleLandingAutopilot");
            t_LandingPredictions = mechJebAssembly.GetType("MuMech.MechJebModuleLandingPredictions");
            t_LandingGuidance = mechJebAssembly.GetType("MuMech.MechJebModuleLandingGuidance");
            t_ManeuverPlanner = mechJebAssembly.GetType("MuMech.MechJebModuleManeuverPlanner");
            t_RendezvousAutopilot = mechJebAssembly.GetType("MuMech.MechJebModuleRendezvousAutopilot");
            t_RendezvousGuidance = mechJebAssembly.GetType("MuMech.MechJebModuleRendezvousGuidance");
            t_DockingAutopilot = mechJebAssembly.GetType("MuMech.MechJebModuleDockingAutopilot");
            t_DockingGuidance = mechJebAssembly.GetType("MuMech.MechJebModuleDockingGuidance");
            t_Translatron = mechJebAssembly.GetType("MuMech.MechJebModuleTranslatron");
            t_RoverController = mechJebAssembly.GetType("MuMech.MechJebModuleRoverController");
            t_AirplaneAutopilot = mechJebAssembly.GetType("MuMech.MechJebModuleAirplaneAutopilot");
            t_SpaceplaneAutopilot = mechJebAssembly.GetType("MuMech.MechJebModuleSpaceplaneAutopilot");
            t_StageStats = mechJebAssembly.GetType("MuMech.MechJebModuleStageStats");
            t_RCSBalancer = mechJebAssembly.GetType("MuMech.MechJebModuleRCSBalancer");
            t_WarpController = mechJebAssembly.GetType("MuMech.MechJebModuleWarpController");
            t_Settings = mechJebAssembly.GetType("MuMech.MechJebModuleSettings");

            // Maneuver / Operations
            t_OrbitalManeuverCalculator = mechJebAssembly.GetType("MuMech.OrbitalManeuverCalculator");
            t_Operation = mechJebAssembly.GetType("MuMech.Operation");
            t_OperationAdvancedTransfer = mechJebAssembly.GetType("MuMech.OperationAdvancedTransfer");
            t_OperationCourseCorrection = mechJebAssembly.GetType("MuMech.OperationCourseCorrection");
            t_ManeuverParameters = mechJebAssembly.GetType("MuMech.ManeuverParameters");
            t_TransferCalculator = mechJebAssembly.GetType("MuMech.TransferCalculator");
        }

        private static void InitializeDataTypes()
        {
            t_AbsoluteVector = mechJebAssembly.GetType("MuMech.AbsoluteVector");
            t_ReentryResult = mechJebAssembly.GetType("MuMech.ReentrySimulation+Result");
            t_UserPool = mechJebAssembly.GetType("MuMech.UserPool");

            // FuelStats is in MechJebLib
            if (mechJebLibAssembly != null)
            {
                t_FuelStats = mechJebLibAssembly.GetType("MechJebLib.FuelFlowSimulation.FuelStats");
            }
            
            // Initialize AbsoluteVector fields
            if (t_AbsoluteVector != null)
            {
                f_AbsoluteVector_Latitude = t_AbsoluteVector.GetField("Latitude", BindingFlags.Public | BindingFlags.Instance);
                f_AbsoluteVector_Longitude = t_AbsoluteVector.GetField("Longitude", BindingFlags.Public | BindingFlags.Instance);
                m_AbsoluteVector_ToDouble = t_AbsoluteVector.GetMethods(BindingFlags.Public | BindingFlags.Static)
                    .FirstOrDefault(m => m.Name == "op_Implicit" && m.ReturnType == typeof(double));
            }

            // Initialize ReentryResult fields
            if (t_ReentryResult != null)
            {
                f_Result_Outcome = t_ReentryResult.GetField("Outcome", BindingFlags.Public | BindingFlags.Instance);
                f_Result_EndPosition = t_ReentryResult.GetField("EndPosition", BindingFlags.Public | BindingFlags.Instance);
                f_Result_EndUT = t_ReentryResult.GetField("EndUT", BindingFlags.Public | BindingFlags.Instance);
                f_Result_MaxDragGees = t_ReentryResult.GetField("MaxDragGees", BindingFlags.Public | BindingFlags.Instance);
            }

            // Initialize FuelStats fields
            if (t_FuelStats != null)
            {
                f_FuelStats_DeltaV = t_FuelStats.GetField("DeltaV", BindingFlags.Public | BindingFlags.Instance);
                f_FuelStats_DeltaTime = t_FuelStats.GetField("DeltaTime", BindingFlags.Public | BindingFlags.Instance);
                f_FuelStats_StartMass = t_FuelStats.GetField("StartMass", BindingFlags.Public | BindingFlags.Instance);
                f_FuelStats_EndMass = t_FuelStats.GetField("EndMass", BindingFlags.Public | BindingFlags.Instance);
                f_FuelStats_StartThrust = t_FuelStats.GetField("StartThrust", BindingFlags.Public | BindingFlags.Instance);
                f_FuelStats_MaxAccel = t_FuelStats.GetField("MaxAccel", BindingFlags.Public | BindingFlags.Instance);
                f_FuelStats_Isp = t_FuelStats.GetField("Isp", BindingFlags.Public | BindingFlags.Instance);
            }

            // Initialize UserPool methods - need to specify parameter type due to 'new' keyword hiding
            if (t_UserPool != null)
            {
                m_UserPool_Add = t_UserPool.GetMethod("Add", BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly, null, new Type[] { typeof(object) }, null);
                m_UserPool_Remove = t_UserPool.GetMethod("Remove", BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly, null, new Type[] { typeof(object) }, null);
                m_UserPool_Contains = t_UserPool.GetMethod("Contains", BindingFlags.Public | BindingFlags.Instance);
            }
        }

        private static void InitializeEnumTypes()
        {
            t_SmartASSTarget = mechJebAssembly.GetType("MuMech.MechJebModuleSmartASS+Target");
            t_SmartASSMode = mechJebAssembly.GetType("MuMech.MechJebModuleSmartASS+Mode");
            t_AttitudeReference = mechJebAssembly.GetType("MuMech.AttitudeReference");
            t_TranslatronMode = mechJebAssembly.GetType("MuMech.MechJebModuleThrustController+TMode");
        }

        private static void InitializeMethods()
        {
            // VesselExtensions methods
            if (t_VesselExtensions != null)
            {
                m_GetMasterMechJeb = t_VesselExtensions.GetMethod("GetMasterMechJeb", 
                    BindingFlags.Public | BindingFlags.Static,
                    null, new Type[] { typeof(Vessel) }, null);
                m_PlaceManeuverNode = t_VesselExtensions.GetMethod("PlaceManeuverNode",
                    BindingFlags.Public | BindingFlags.Static);
            }

            // MechJebCore methods and fields
            if (t_MechJebCore != null)
            {
                m_GetComputerModule = t_MechJebCore.GetMethod("GetComputerModule",
                    new Type[] { typeof(string) });
                f_Core_Target = t_MechJebCore.GetField("Target", BindingFlags.Public | BindingFlags.Instance);
                f_Core_Node = t_MechJebCore.GetField("Node", BindingFlags.Public | BindingFlags.Instance);
                f_Core_Attitude = t_MechJebCore.GetField("Attitude", BindingFlags.Public | BindingFlags.Instance);
                f_Core_Thrust = t_MechJebCore.GetField("Thrust", BindingFlags.Public | BindingFlags.Instance);
                f_Core_Staging = t_MechJebCore.GetField("Staging", BindingFlags.Public | BindingFlags.Instance);
                f_Core_VesselState = t_MechJebCore.GetField("VesselState", BindingFlags.Public | BindingFlags.Instance);
            }

            // ComputerModule properties
            if (t_ComputerModule != null)
            {
                p_Module_Enabled = t_ComputerModule.GetProperty("Enabled", BindingFlags.Public | BindingFlags.Instance);
                f_Module_Users = t_ComputerModule.GetField("Users", BindingFlags.Public | BindingFlags.Instance);
            }

            // Initialize all module-specific methods and fields
            InitializeSmartASSMembers();
            InitializeNodeExecutorMembers();
            InitializeTargetControllerMembers();
            InitializeAscentMembers();
            InitializeLandingMembers();
            InitializeThrustMembers();
            InitializeStagingMembers();
            InitializeStageStatsMembers();
            InitializeTranslatronMembers();
            InitializeRendezvousMembers();
            InitializeDockingMembers();
            InitializeRoverMembers();
            InitializeAirplaneMembers();
            InitializeSpaceplaneMembers();
            InitializeManeuverCalculatorMembers();
            InitializeAdvancedTransferMembers();
            InitializeOperationMembers();
            InitializeWarpMembers();
        }

        private static void InitializeManeuverCalculatorMembers()
        {
            if (t_OrbitalManeuverCalculator == null) return;

            m_Calc_DeltaVToCircularize = t_OrbitalManeuverCalculator.GetMethod("DeltaVToCircularize",
                BindingFlags.Public | BindingFlags.Static);
            m_Calc_DeltaVToChangeApoapsis = t_OrbitalManeuverCalculator.GetMethod("DeltaVToChangeApoapsis",
                BindingFlags.Public | BindingFlags.Static);
            m_Calc_DeltaVToChangePeriapsis = t_OrbitalManeuverCalculator.GetMethod("DeltaVToChangePeriapsis",
                BindingFlags.Public | BindingFlags.Static);
            m_Calc_DeltaVToMatchVelocities = t_OrbitalManeuverCalculator.GetMethod("DeltaVToMatchVelocities",
                BindingFlags.Public | BindingFlags.Static);
            m_Calc_DeltaVAndTimeForHohmannTransfer = t_OrbitalManeuverCalculator.GetMethod("DeltaVAndTimeForHohmannTransfer",
                BindingFlags.Public | BindingFlags.Static);
            m_Calc_DeltaVForSemiMajorAxis = t_OrbitalManeuverCalculator.GetMethod("DeltaVForSemiMajorAxis",
                BindingFlags.Public | BindingFlags.Static);
            m_Calc_DeltaVToChangeInclination = t_OrbitalManeuverCalculator.GetMethod("DeltaVToChangeInclination",
                BindingFlags.Public | BindingFlags.Static);
            m_Calc_DeltaVToMatchPlanesAscending = t_OrbitalManeuverCalculator.GetMethod("DeltaVAndTimeToMatchPlanesAscending",
                BindingFlags.Public | BindingFlags.Static);
            m_Calc_DeltaVToMatchPlanesDescending = t_OrbitalManeuverCalculator.GetMethod("DeltaVAndTimeToMatchPlanesDescending",
                BindingFlags.Public | BindingFlags.Static);
            m_Calc_DeltaVToShiftLAN = t_OrbitalManeuverCalculator.GetMethod("DeltaVToShiftLAN",
                BindingFlags.Public | BindingFlags.Static);
            m_Calc_DeltaVToEllipticize = t_OrbitalManeuverCalculator.GetMethod("DeltaVToEllipticize",
                BindingFlags.Public | BindingFlags.Static);
        }

        private static void InitializeAdvancedTransferMembers()
        {
            // Operation base type bindings
            if (t_Operation != null)
            {
                m_Operation_MakeNodes = t_Operation.GetMethod("MakeNodes", BindingFlags.Public | BindingFlags.Instance);
                m_Operation_GetErrorMessage = t_Operation.GetMethod("GetErrorMessage", BindingFlags.Public | BindingFlags.Instance);
                m_Operation_GetName = t_Operation.GetMethod("GetName", BindingFlags.Public | BindingFlags.Instance);
            }

            // ManeuverPlanner static fields - these give us access to MechJeb's actual Operation instances
            if (t_ManeuverPlanner != null)
            {
                f_ManeuverPlanner_operation = t_ManeuverPlanner.GetField("_operation", 
                    BindingFlags.NonPublic | BindingFlags.Static);
                f_ManeuverPlanner_operationId = t_ManeuverPlanner.GetField("_operationId", 
                    BindingFlags.Public | BindingFlags.Instance);
            }

            // TimeSelector and TimeReference for operation timing
            t_TimeSelector = mechJebAssembly.GetType("MuMech.TimeSelector");
            if (t_TimeSelector != null)
            {
                m_TimeSelector_ComputeManeuverTime = t_TimeSelector.GetMethod("ComputeManeuverTime", 
                    BindingFlags.Public | BindingFlags.Instance);
                f_TimeSelector_CurrentTimeRef = t_TimeSelector.GetField("_currentTimeRef", 
                    BindingFlags.Public | BindingFlags.Instance);
                f_TimeSelector_CircularizeAltitude = t_TimeSelector.GetField("CircularizeAltitude", 
                    BindingFlags.Public | BindingFlags.Instance);
                f_TimeSelector_LeadTime = t_TimeSelector.GetField("LeadTime", 
                    BindingFlags.Public | BindingFlags.Instance);
                p_TimeSelector_TimeReference = t_TimeSelector.GetProperty("TimeReference", 
                    BindingFlags.Public | BindingFlags.Instance);
            }
            
            t_TimeReference = mechJebAssembly.GetType("MuMech.TimeReference");

            if (t_OperationAdvancedTransfer != null)
            {
                f_AdvancedTransfer_SelectionMode = t_OperationAdvancedTransfer.GetField("selectionMode", BindingFlags.NonPublic | BindingFlags.Instance);
                f_AdvancedTransfer_Worker = t_OperationAdvancedTransfer.GetField("worker", BindingFlags.NonPublic | BindingFlags.Instance);
                f_AdvancedTransfer_Plot = t_OperationAdvancedTransfer.GetField("plot", BindingFlags.NonPublic | BindingFlags.Instance);
                f_AdvancedTransfer_IncludeCaptureBurn = t_OperationAdvancedTransfer.GetField("includeCaptureBurn", BindingFlags.NonPublic | BindingFlags.Instance);
                f_AdvancedTransfer_PeriapsisHeight = t_OperationAdvancedTransfer.GetField("periapsisHeight", BindingFlags.NonPublic | BindingFlags.Instance);
                f_AdvancedTransfer_LastTargetCelestial = t_OperationAdvancedTransfer.GetField("lastTargetCelestial", BindingFlags.NonPublic | BindingFlags.Instance);
                m_AdvancedTransfer_ComputeTimes = t_OperationAdvancedTransfer.GetMethod("ComputeTimes", BindingFlags.NonPublic | BindingFlags.Instance);
                m_AdvancedTransfer_ComputeStuff = t_OperationAdvancedTransfer.GetMethod("ComputeStuff", BindingFlags.NonPublic | BindingFlags.Instance);
            }

            // OperationGeneric (Hohmann/bi-impulsive transfer)
            t_OperationGeneric = mechJebAssembly.GetType("MuMech.OperationGeneric");
            if (t_OperationGeneric != null)
            {
                f_Generic_Capture = t_OperationGeneric.GetField("Capture", BindingFlags.Public | BindingFlags.Instance);
                f_Generic_PlanCapture = t_OperationGeneric.GetField("PlanCapture", BindingFlags.Public | BindingFlags.Instance);
                f_Generic_Rendezvous = t_OperationGeneric.GetField("Rendezvous", BindingFlags.Public | BindingFlags.Instance);
                f_Generic_Coplanar = t_OperationGeneric.GetField("Coplanar", BindingFlags.Public | BindingFlags.Instance);
                f_Generic_LagTime = t_OperationGeneric.GetField("LagTime", BindingFlags.Public | BindingFlags.Instance);
            }

            Type t_OperatiionInterplanetaryTransfer = mechJebAssembly.GetType("MuMech.OperationInterplanetaryTransfer");
            if (t_OperatiionInterplanetaryTransfer != null)
            {
                f_InterplanetaryTransfer_WaitForPhaseAngle = t_OperatiionInterplanetaryTransfer.GetField("WaitForPhaseAngle", BindingFlags.Public | BindingFlags.Instance);
            }

            // PlotArea for porkchop selection
            t_PlotArea = mechJebAssembly.GetType("MuMech.PlotArea");
            if (t_PlotArea != null)
            {
                p_PlotArea_SelectedPoint = t_PlotArea.GetProperty("SelectedPoint", BindingFlags.Public | BindingFlags.Instance);
            }

            if (t_TransferCalculator != null)
            {
                f_TransferCalculator_Computed = t_TransferCalculator.GetField("Computed", BindingFlags.Public | BindingFlags.Instance);
                f_TransferCalculator_BestDate = t_TransferCalculator.GetField("BestDate", BindingFlags.Public | BindingFlags.Instance);
                f_TransferCalculator_BestDuration = t_TransferCalculator.GetField("BestDuration", BindingFlags.Public | BindingFlags.Instance);
                p_TransferCalculator_Finished = t_TransferCalculator.GetProperty("Finished", BindingFlags.Public | BindingFlags.Instance);
                p_TransferCalculator_Progress = t_TransferCalculator.GetProperty("Progress", BindingFlags.Public | BindingFlags.Instance);
                p_TransferCalculator_ArrivalDate = t_TransferCalculator.GetProperty("ArrivalDate", BindingFlags.Public | BindingFlags.Instance);
                m_TransferCalculator_DateFromIndex = t_TransferCalculator.GetMethod("DateFromIndex", BindingFlags.Public | BindingFlags.Instance);
                m_TransferCalculator_DurationFromIndex = t_TransferCalculator.GetMethod("DurationFromIndex", BindingFlags.Public | BindingFlags.Instance);
            }

            if (t_ManeuverParameters != null)
            {
                f_ManeuverParameters_dV = t_ManeuverParameters.GetField("dV", BindingFlags.Public | BindingFlags.Instance);
                f_ManeuverParameters_UT = t_ManeuverParameters.GetField("UT", BindingFlags.Public | BindingFlags.Instance);
            }
        }

        private static void InitializeOperationMembers()
        {
            if (t_OperationCourseCorrection == null) return;

            string[] candidates = new string[]
            {
                "targetPe", "targetPeA", "desiredPe", "desiredPeA", "periapsis", "periapsisAltitude", "PeA"
            };

            foreach (string name in candidates)
            {
                FieldInfo f = t_OperationCourseCorrection.GetField(name, BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
                if (f != null)
                {
                    f_CourseCorrection_TargetPe = f;
                    return;
                }

                PropertyInfo p = t_OperationCourseCorrection.GetProperty(name, BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
                if (p != null)
                {
                    p_CourseCorrection_TargetPe = p;
                    return;
                }
            }
        }

        private static void InitializeWarpMembers()
        {
            if (t_WarpController == null) return;
            m_Warp_WarpToUT = t_WarpController.GetMethod("WarpToUT", BindingFlags.Public | BindingFlags.Instance);
        }

        private static void InitializeSmartASSMembers()
        {
            if (t_SmartASS == null) return;

            f_SmartASS_Target = t_SmartASS.GetField("target", BindingFlags.Public | BindingFlags.Instance);
            f_SmartASS_Mode = t_SmartASS.GetField("mode", BindingFlags.Public | BindingFlags.Instance);
            f_SmartASS_ForceRol = t_SmartASS.GetField("forceRol", BindingFlags.Public | BindingFlags.Instance);
            f_SmartASS_Rol = t_SmartASS.GetField("rol", BindingFlags.Public | BindingFlags.Instance);
            f_SmartASS_SrfHdg = t_SmartASS.GetField("srfHdg", BindingFlags.Public | BindingFlags.Instance);
            f_SmartASS_SrfPit = t_SmartASS.GetField("srfPit", BindingFlags.Public | BindingFlags.Instance);
            f_SmartASS_SrfRol = t_SmartASS.GetField("srfRol", BindingFlags.Public | BindingFlags.Instance);
            f_SmartASS_SrfVelYaw = t_SmartASS.GetField("srfVelYaw", BindingFlags.Public | BindingFlags.Instance);
            f_SmartASS_SrfVelPit = t_SmartASS.GetField("srfVelPit", BindingFlags.Public | BindingFlags.Instance);
            f_SmartASS_SrfVelRol = t_SmartASS.GetField("srfVelRol", BindingFlags.Public | BindingFlags.Instance);
            f_SmartASS_AdvReference = t_SmartASS.GetField("advReference", BindingFlags.Public | BindingFlags.Instance);
            f_SmartASS_AdvDirection = t_SmartASS.GetField("advDirection", BindingFlags.Public | BindingFlags.Instance);
            m_SmartASS_Engage = t_SmartASS.GetMethod("Engage", BindingFlags.Public | BindingFlags.Instance);
            
            // Static fields for text arrays
            f_SmartASS_ModeTexts = t_SmartASS.GetField("ModeTexts", BindingFlags.Public | BindingFlags.Static);
            f_SmartASS_TargetTexts = t_SmartASS.GetField("TargetTexts", BindingFlags.Public | BindingFlags.Static);
        }

        private static void InitializeNodeExecutorMembers()
        {
            if (t_NodeExecutor == null) return;

            m_NodeExecutor_ExecuteOneNode = t_NodeExecutor.GetMethod("ExecuteOneNode", BindingFlags.Public | BindingFlags.Instance);
            m_NodeExecutor_ExecuteAllNodes = t_NodeExecutor.GetMethod("ExecuteAllNodes", BindingFlags.Public | BindingFlags.Instance);
            m_NodeExecutor_Abort = t_NodeExecutor.GetMethod("Abort", BindingFlags.Public | BindingFlags.Instance);
            f_NodeExecutor_Autowarp = t_NodeExecutor.GetField("Autowarp", BindingFlags.Public | BindingFlags.Instance);
            f_NodeExecutor_LeadTime = t_NodeExecutor.GetField("leadTime", BindingFlags.Public | BindingFlags.Instance);
        }

        private static void InitializeTargetControllerMembers()
        {
            if (t_TargetController == null) return;

            p_Target_PositionTargetExists = t_TargetController.GetProperty("PositionTargetExists", BindingFlags.Public | BindingFlags.Instance);
            p_Target_NormalTargetExists = t_TargetController.GetProperty("NormalTargetExists", BindingFlags.Public | BindingFlags.Instance);
            p_Target_TargetOrbit = t_TargetController.GetProperty("TargetOrbit", BindingFlags.Public | BindingFlags.Instance);
            f_Target_TargetLatitude = t_TargetController.GetField("targetLatitude", BindingFlags.Public | BindingFlags.Instance);
            f_Target_TargetLongitude = t_TargetController.GetField("targetLongitude", BindingFlags.Public | BindingFlags.Instance);
            m_Target_SetPositionTarget = t_TargetController.GetMethod("SetPositionTarget", BindingFlags.Public | BindingFlags.Instance);
            m_Target_PickPositionTargetOnMap = t_TargetController.GetMethod("PickPositionTargetOnMap", BindingFlags.Public | BindingFlags.Instance);
        }

        private static void InitializeAscentMembers()
        {
            if (t_AscentSettings == null) return;

            f_Ascent_DesiredOrbitAltitude = t_AscentSettings.GetField("DesiredOrbitAltitude", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_DesiredApoapsis = t_AscentSettings.GetField("DesiredApoapsis", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_DesiredInclination = t_AscentSettings.GetField("DesiredInclination", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_DesiredLAN = t_AscentSettings.GetField("DesiredLan", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_LaunchPhaseAngle = t_AscentSettings.GetField("LaunchPhaseAngle", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_LaunchLANDifference = t_AscentSettings.GetField("LaunchLANDifference", BindingFlags.Public | BindingFlags.Instance);
            p_Ascent_AscentAutopilot = t_AscentSettings.GetProperty("AscentAutopilot", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_WarpCountDown = t_AscentSettings.GetField("WarpCountDown", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_SkipCircularization = t_AscentSettings.GetField("SkipCircularization", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_ForceRoll = t_AscentSettings.GetField("ForceRoll", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_VerticalRoll = t_AscentSettings.GetField("VerticalRoll", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_TurnRoll = t_AscentSettings.GetField("TurnRoll", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_RollAltitude = t_AscentSettings.GetField("RollAltitude", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_LimitAoA = t_AscentSettings.GetField("LimitAoA", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_MaxAoA = t_AscentSettings.GetField("MaxAoA", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_AOALimitFadeoutPressure = t_AscentSettings.GetField("AOALimitFadeoutPressure", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_CorrectiveSteering = t_AscentSettings.GetField("CorrectiveSteering", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_CorrectiveSteeringGain = t_AscentSettings.GetField("CorrectiveSteeringGain", BindingFlags.Public | BindingFlags.Instance);

            f_Ascent_TurnStartAltitude = t_AscentSettings.GetField("TurnStartAltitude", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_TurnStartVelocity = t_AscentSettings.GetField("TurnStartVelocity", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_TurnEndAltitude = t_AscentSettings.GetField("TurnEndAltitude", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_TurnEndAngle = t_AscentSettings.GetField("TurnEndAngle", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_TurnShapeExponent = t_AscentSettings.GetField("TurnShapeExponent", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_AutoPath = t_AscentSettings.GetField("AutoPath", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_AutoTurnPerc = t_AscentSettings.GetField("AutoTurnPerc", BindingFlags.Public | BindingFlags.Instance);
            f_Ascent_AutoTurnSpdFactor = t_AscentSettings.GetField("AutoTurnSpdFactor", BindingFlags.Public | BindingFlags.Instance);

            p_Ascent_Autostage = t_AscentSettings.GetProperty("Autostage", BindingFlags.Public | BindingFlags.Instance);
            // In MJ 2.15.1, Autostage is a public field, not a private backing field
            f_Ascent_Autostage = t_AscentSettings.GetField("Autostage", BindingFlags.Public | BindingFlags.Instance);
            if (f_Ascent_Autostage == null)
            {
                // Fallback for older versions that might use a private backing field
                f_Ascent_Autostage = t_AscentSettings.GetField("_autostage", BindingFlags.NonPublic | BindingFlags.Instance);
            }

            if (t_AscentBaseAutopilot != null)
            {
                p_AscentAP_Status = t_AscentBaseAutopilot.GetProperty("Status", BindingFlags.Public | BindingFlags.Instance);
                m_AscentAP_StartCountdown = t_AscentBaseAutopilot.GetMethod("StartCountdown", BindingFlags.Public | BindingFlags.Instance);
            }
        }

        private static void InitializeLandingMembers()
        {
            if (t_LandingAutopilot == null) return;

            m_Landing_LandAtPositionTarget = t_LandingAutopilot.GetMethod("LandAtPositionTarget", BindingFlags.Public | BindingFlags.Instance);
            m_Landing_LandUntargeted = t_LandingAutopilot.GetMethod("LandUntargeted", BindingFlags.Public | BindingFlags.Instance);
            m_Landing_StopLanding = t_LandingAutopilot.GetMethod("StopLanding", BindingFlags.Public | BindingFlags.Instance);
            f_Landing_TouchdownSpeed = t_LandingAutopilot.GetField("TouchdownSpeed", BindingFlags.Public | BindingFlags.Instance);
            f_Landing_DeployGears = t_LandingAutopilot.GetField("DeployGears", BindingFlags.Public | BindingFlags.Instance);
            f_Landing_DeployChutes = t_LandingAutopilot.GetField("DeployChutes", BindingFlags.Public | BindingFlags.Instance);
            f_Landing_LimitGearsStage = t_LandingAutopilot.GetField("LimitGearsStage", BindingFlags.Public | BindingFlags.Instance);
            f_Landing_LimitChutesStage = t_LandingAutopilot.GetField("LimitChutesStage", BindingFlags.Public | BindingFlags.Instance);
            f_Landing_UseRCS = t_LandingAutopilot.GetField("RCSAdjustment", BindingFlags.Public | BindingFlags.Instance);
            p_Landing_Status = t_LandingAutopilot.GetProperty("Status", BindingFlags.Public | BindingFlags.Instance);

            if (t_LandingPredictions != null)
            {
                m_Predictions_GetResult = t_LandingPredictions.GetMethod("GetResult", BindingFlags.Public | BindingFlags.Instance);
                p_Predictions_ShowTrajectory = t_LandingPredictions.GetProperty("ShowTrajectory", BindingFlags.Public | BindingFlags.Instance);
            }
        }

        private static void InitializeThrustMembers()
        {
            if (t_ThrustController == null) return;

            f_Thrust_LimitToPreventOverheats = t_ThrustController.GetField("LimitToPreventOverheats", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_LimitToTerminalVelocity = t_ThrustController.GetField("LimitToTerminalVelocity", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_LimitToMaxDynamicPressure = t_ThrustController.GetField("LimitDynamicPressure", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_MaxDynamicPressure = t_ThrustController.GetField("MaxDynamicPressure", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_LimitAcceleration = t_ThrustController.GetField("LimitAcceleration", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_MaxAcceleration = t_ThrustController.GetField("MaxAcceleration", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_LimitThrottle = t_ThrustController.GetField("LimitThrottle", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_MaxThrottle = t_ThrustController.GetField("MaxThrottle", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_LimiterMinThrottle = t_ThrustController.GetField("LimiterMinThrottle", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_MinThrottle = t_ThrustController.GetField("MinThrottle", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_LimitToPreventFlameout = t_ThrustController.GetField("LimitToPreventFlameout", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_FlameoutSafetyPct = t_ThrustController.GetField("FlameoutSafetyPct", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_SmoothThrottle = t_ThrustController.GetField("SmoothThrottle", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_ManageIntakes = t_ThrustController.GetField("ManageIntakes", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_DifferentialThrottle = t_ThrustController.GetField("DifferentialThrottle", BindingFlags.Public | BindingFlags.Instance);
            f_Thrust_DifferentialThrottleSuccess = t_ThrustController.GetField("DifferentialThrottleSuccess", BindingFlags.Public | BindingFlags.Instance);
        }

        private static void InitializeStagingMembers()
        {
            if (t_StagingController == null) return;

            f_Staging_Autostage = t_StagingController.GetField("Autostage", BindingFlags.Public | BindingFlags.Instance);
            f_Staging_AutostageLimit = t_StagingController.GetField("AutostageLimit", BindingFlags.Public | BindingFlags.Instance);
            f_Staging_AutostagePreDelay = t_StagingController.GetField("AutostagePreDelay", BindingFlags.Public | BindingFlags.Instance);
            f_Staging_AutostagePostDelay = t_StagingController.GetField("AutostagePostDelay", BindingFlags.Public | BindingFlags.Instance);
            f_Staging_ClampAutoStageThrustPct = t_StagingController.GetField("ClampAutoStageThrustPct", BindingFlags.Public | BindingFlags.Instance);
            f_Staging_FairingMaxAerothermalFlux = t_StagingController.GetField("FairingMaxAerothermalFlux", BindingFlags.Public | BindingFlags.Instance);
            f_Staging_FairingMaxDynamicPressure = t_StagingController.GetField("FairingMaxDynamicPressure", BindingFlags.Public | BindingFlags.Instance);
            f_Staging_FairingMinAltitude = t_StagingController.GetField("FairingMinAltitude", BindingFlags.Public | BindingFlags.Instance);
            f_Staging_HotStagingLeadTime = t_StagingController.GetField("HotStagingLeadTime", BindingFlags.Public | BindingFlags.Instance);
            f_Staging_DropSolids = t_StagingController.GetField("DropSolids", BindingFlags.Public | BindingFlags.Instance);
            f_Staging_DropSolidsLeadTime = t_StagingController.GetField("DropSolidsLeadTime", BindingFlags.Public | BindingFlags.Instance);
            m_Staging_AutostageOnce = t_StagingController.GetMethod("AutostageOnce", BindingFlags.Public | BindingFlags.Instance);
        }

        private static void InitializeStageStatsMembers()
        {
            if (t_StageStats == null) return;

            m_StageStats_RequestUpdate = t_StageStats.GetMethod("RequestUpdate", BindingFlags.Public | BindingFlags.Instance);
            f_StageStats_VacStats = t_StageStats.GetField("VacStats", BindingFlags.Public | BindingFlags.Instance);
            f_StageStats_AtmoStats = t_StageStats.GetField("AtmoStats", BindingFlags.Public | BindingFlags.Instance);
        }

        private static void InitializeTranslatronMembers()
        {
            if (t_Translatron == null) return;

            p_Translatron_TransSpd = t_Translatron.GetProperty("trans_spd", BindingFlags.Public | BindingFlags.Instance);
            f_Translatron_TransSpdAct = t_Translatron.GetField("trans_spd_act", BindingFlags.Public | BindingFlags.Instance);
            p_Translatron_TransKillH = t_Translatron.GetProperty("trans_kill_h", BindingFlags.Public | BindingFlags.Instance);
            m_Translatron_SetMode = t_Translatron.GetMethod("SetMode", BindingFlags.Public | BindingFlags.Instance);
            m_Translatron_PanicSwitch = t_Translatron.GetMethod("PanicSwitch", BindingFlags.Public | BindingFlags.Instance);
        }

        private static void InitializeRendezvousMembers()
        {
            if (t_RendezvousAutopilot == null) return;

            f_Rendezvous_DesiredDistance = t_RendezvousAutopilot.GetField("desiredDistance", BindingFlags.Public | BindingFlags.Instance);
            f_Rendezvous_MaxPhasingOrbits = t_RendezvousAutopilot.GetField("maxPhasingOrbits", BindingFlags.Public | BindingFlags.Instance);
            f_Rendezvous_MaxClosingSpeed = t_RendezvousAutopilot.GetField("maxClosingSpeed", BindingFlags.Public | BindingFlags.Instance);
            p_Rendezvous_Status = t_RendezvousAutopilot.GetProperty("Status", BindingFlags.Public | BindingFlags.Instance);
        }

        private static void InitializeDockingMembers()
        {
            if (t_DockingAutopilot == null) return;

            f_Docking_SpeedLimit = t_DockingAutopilot.GetField("speedLimit", BindingFlags.Public | BindingFlags.Instance);
            f_Docking_ForceRoll = t_DockingAutopilot.GetField("forceRol", BindingFlags.Public | BindingFlags.Instance);
            f_Docking_Roll = t_DockingAutopilot.GetField("rol", BindingFlags.Public | BindingFlags.Instance);
            f_Docking_OverrideSafeDistance = t_DockingAutopilot.GetField("overrideSafeDistance", BindingFlags.Public | BindingFlags.Instance);
            f_Docking_OverridenSafeDistance = t_DockingAutopilot.GetField("overridenSafeDistance", BindingFlags.Public | BindingFlags.Instance);
            f_Docking_OverrideTargetSize = t_DockingAutopilot.GetField("overrideTargetSize", BindingFlags.Public | BindingFlags.Instance);
            f_Docking_OverridenTargetSize = t_DockingAutopilot.GetField("overridenTargetSize", BindingFlags.Public | BindingFlags.Instance);
            f_Docking_DrawBoundingBox = t_DockingAutopilot.GetField("drawBoundingBox", BindingFlags.Public | BindingFlags.Instance);
            p_Docking_Status = t_DockingAutopilot.GetProperty("Status", BindingFlags.Public | BindingFlags.Instance);
        }

        private static void InitializeRoverMembers()
        {
            if (t_RoverController == null) return;

            f_Rover_ControlHeading = t_RoverController.GetField("ControlHeading", BindingFlags.Public | BindingFlags.Instance);
            f_Rover_ControlSpeed = t_RoverController.GetField("ControlSpeed", BindingFlags.Public | BindingFlags.Instance);
            f_Rover_Heading = t_RoverController.GetField("heading", BindingFlags.Public | BindingFlags.Instance);
            f_Rover_Speed = t_RoverController.GetField("speed", BindingFlags.Public | BindingFlags.Instance);
            f_Rover_HeadingError = t_RoverController.GetField("headingErr", BindingFlags.Public | BindingFlags.Instance);
            f_Rover_SpeedError = t_RoverController.GetField("speedErr", BindingFlags.Public | BindingFlags.Instance);
            f_Rover_StabilityControl = t_RoverController.GetField("StabilityControl", BindingFlags.Public | BindingFlags.Instance);
            f_Rover_BrakeOnEject = t_RoverController.GetField("BrakeOnEject", BindingFlags.Public | BindingFlags.Instance);
            f_Rover_BrakeOnEnergyDepletion = t_RoverController.GetField("BrakeOnEnergyDepletion", BindingFlags.Public | BindingFlags.Instance);
            f_Rover_WarpToDaylight = t_RoverController.GetField("WarpToDaylight", BindingFlags.Public | BindingFlags.Instance);
            m_Rover_DriveToTarget = t_RoverController.GetMethod("DriveToTarget", BindingFlags.Public | BindingFlags.Instance);
            m_Rover_Stop = t_RoverController.GetMethod("Stop", BindingFlags.Public | BindingFlags.Instance);

            MethodInfo[] methods = t_RoverController.GetMethods(BindingFlags.Public | BindingFlags.Instance);
            foreach (MethodInfo m in methods)
            {
                if (m_Rover_AddWaypoint == null && (m.Name == "AddWaypoint" || m.Name == "AddWayPoint" || m.Name == "AddNewWaypoint"))
                {
                    m_Rover_AddWaypoint = m;
                }
                if (m_Rover_ClearWaypoints == null && (m.Name == "ClearWaypoints" || m.Name == "ClearAllWaypoints"))
                {
                    m_Rover_ClearWaypoints = m;
                }
            }
        }

        private static void InitializeAirplaneMembers()
        {
            if (t_AirplaneAutopilot == null) return;

            f_Airplane_AltitudeHold = t_AirplaneAutopilot.GetField("AltitudeHoldEnabled", BindingFlags.Public | BindingFlags.Instance);
            f_Airplane_AltitudeTarget = t_AirplaneAutopilot.GetField("AltitudeTarget", BindingFlags.Public | BindingFlags.Instance);
            f_Airplane_VertSpeedHold = t_AirplaneAutopilot.GetField("VertSpeedHoldEnabled", BindingFlags.Public | BindingFlags.Instance);
            f_Airplane_VertSpeedTarget = t_AirplaneAutopilot.GetField("VertSpeedTarget", BindingFlags.Public | BindingFlags.Instance);
            f_Airplane_HeadingHold = t_AirplaneAutopilot.GetField("HeadingHoldEnabled", BindingFlags.Public | BindingFlags.Instance);
            f_Airplane_HeadingTarget = t_AirplaneAutopilot.GetField("HeadingTarget", BindingFlags.Public | BindingFlags.Instance);
            f_Airplane_RollHold = t_AirplaneAutopilot.GetField("RollHoldEnabled", BindingFlags.Public | BindingFlags.Instance);
            f_Airplane_RollTarget = t_AirplaneAutopilot.GetField("RollTarget", BindingFlags.Public | BindingFlags.Instance);
            f_Airplane_RollMax = t_AirplaneAutopilot.GetField("BankAngle", BindingFlags.Public | BindingFlags.Instance);
            f_Airplane_SpeedHold = t_AirplaneAutopilot.GetField("SpeedHoldEnabled", BindingFlags.Public | BindingFlags.Instance);
            f_Airplane_SpeedTarget = t_AirplaneAutopilot.GetField("SpeedTarget", BindingFlags.Public | BindingFlags.Instance);
            // PID parameters - note: MJ 2.15 may have changed these names
            f_Airplane_AccKp = t_AirplaneAutopilot.GetField("AccKp", BindingFlags.Public | BindingFlags.Instance);
            f_Airplane_AccKi = t_AirplaneAutopilot.GetField("AccKi", BindingFlags.Public | BindingFlags.Instance);
            f_Airplane_AccKd = t_AirplaneAutopilot.GetField("AccKd", BindingFlags.Public | BindingFlags.Instance);
        }

        private static void InitializeSpaceplaneMembers()
        {
            if (t_SpaceplaneAutopilot == null) return;

            f_Spaceplane_Glideslope = t_SpaceplaneAutopilot.GetField("glideslope", BindingFlags.Public | BindingFlags.Instance);
            f_Spaceplane_ApproachSpeed = t_SpaceplaneAutopilot.GetField("approachSpeed", BindingFlags.Public | BindingFlags.Instance);
            f_Spaceplane_TouchdownSpeed = t_SpaceplaneAutopilot.GetField("touchdownSpeed", BindingFlags.Public | BindingFlags.Instance);
            f_Spaceplane_Mode = t_SpaceplaneAutopilot.GetField("mode", BindingFlags.Public | BindingFlags.Instance);
            m_Spaceplane_Autoland = t_SpaceplaneAutopilot.GetMethod("Autoland", BindingFlags.Public | BindingFlags.Instance);
            m_Spaceplane_HoldHeadingAndAltitude = t_SpaceplaneAutopilot.GetMethod("HoldHeadingAndAltitude", BindingFlags.Public | BindingFlags.Instance);
            m_Spaceplane_AutopilotOff = t_SpaceplaneAutopilot.GetMethod("AutopilotOff", BindingFlags.Public | BindingFlags.Instance);
        }
        #endregion

        #region Core Access Methods

        /// <summary>
        /// Get a computer module by name from the core
        /// </summary>
        public static object GetComputerModule(MechJebCore core, string moduleName)
        {
            if (core == null || m_GetComputerModule == null) return null;
            try
            {
                return m_GetComputerModule.Invoke(core, new object[] { moduleName });
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Get whether a module is enabled
        /// </summary>
        public static bool GetModuleEnabled(object module)
        {
            if (module == null || p_Module_Enabled == null) return false;
            try
            {
                return (bool)p_Module_Enabled.GetValue(module, null);
            }
            catch
            {
                return false;
            }
        }

        /// <summary>
        /// Set module enabled state
        /// </summary>
        public static void SetModuleEnabled(object module, bool enabled)
        {
            if (module == null || p_Module_Enabled == null) return;
            try
            {
                p_Module_Enabled.SetValue(module, enabled, null);
            }
            catch { }
        }

        /// <summary>
        /// Get the Users property of a module
        /// </summary>
        public static object GetModuleUsers(object module)
        {
            if (module == null || f_Module_Users == null) return null;
            try
            {
                return f_Module_Users.GetValue(module);
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Add a user to a module's user pool
        /// </summary>
        public static void AddUser(object users, object user)
        {
            if (users == null || user == null || m_UserPool_Add == null) return;
            try
            {
                m_UserPool_Add.Invoke(users, new object[] { user });
            }
            catch { }
        }

        /// <summary>
        /// Remove a user from a module's user pool
        /// </summary>
        public static void RemoveUser(object users, object user)
        {
            if (users == null || user == null || m_UserPool_Remove == null) return;
            try
            {
                m_UserPool_Remove.Invoke(users, new object[] { user });
            }
            catch { }
        }
        #endregion

        #region Editable Value Helpers
       

        private static object CreateAbsoluteVector(CelestialBody body, double latitude, double longitude, double altitude)
        {
            if (t_AbsoluteVector == null) return null;
            try
            {
                ConstructorInfo ctor = t_AbsoluteVector.GetConstructor(new Type[]
                {
                    typeof(CelestialBody), typeof(double), typeof(double), typeof(double)
                });
                if (ctor != null)
                {
                    return ctor.Invoke(new object[] { body, latitude, longitude, altitude });
                }

                ctor = t_AbsoluteVector.GetConstructor(new Type[]
                {
                    typeof(CelestialBody), typeof(double), typeof(double)
                });
                if (ctor != null)
                {
                    return ctor.Invoke(new object[] { body, latitude, longitude });
                }
            }
            catch { }
            return null;
        }
        #endregion

        #region Core Module Accessors
        /// <summary>
        /// Get the Target controller from the core
        /// </summary>
        public static MechJebModuleTargetController GetTargetController(MechJebCore core)
        {
            if (core == null || f_Core_Target == null) return null;
            return core.GetComponent<MechJebModuleTargetController>();
        }

        /// <summary>
        /// Get the Node executor from the core
        /// </summary>
        public static object GetNodeExecutor(MechJebCore core)
        {
            if (core == null || f_Core_Node == null) return null;
            try
            {
                return f_Core_Node.GetValue(core);
            }
            catch
            {
                return null;
            }
        }


        /// <summary>
        /// Get the Staging controller from the core
        /// </summary>
        public static object GetStagingController(MechJebCore core)
        {
            if (core == null || f_Core_Staging == null) return null;
            try
            {
                return f_Core_Staging.GetValue(core);
            }
            catch
            {
                return null;
            }
        }

        #endregion

        #region SmartASS Methods
      


        public static string GetSmartASSAdvancedReferenceName(object smartass)
        {
            if (smartass == null || f_SmartASS_AdvReference == null) return "N/A";
            try
            {
                object val = f_SmartASS_AdvReference.GetValue(smartass);
                return val != null ? val.ToString() : "N/A";
            }
            catch { return "N/A"; }
        }

        public static void CycleSmartASSAdvancedReference(object smartass, int direction)
        {
            if (smartass == null || f_SmartASS_AdvReference == null) return;
            try
            {
                Type enumType = f_SmartASS_AdvReference.FieldType;
                Array values = Enum.GetValues(enumType);
                object current = f_SmartASS_AdvReference.GetValue(smartass);
                int idx = Array.IndexOf(values, current);
                if (idx < 0) idx = 0;
                int next = (idx + direction + values.Length) % values.Length;
                f_SmartASS_AdvReference.SetValue(smartass, values.GetValue(next));
            }
            catch { }
        }

        public static string GetSmartASSAdvancedDirectionName(object smartass)
        {
            if (smartass == null || f_SmartASS_AdvDirection == null) return "N/A";
            try
            {
                object val = f_SmartASS_AdvDirection.GetValue(smartass);
                return val != null ? val.ToString() : "N/A";
            }
            catch { return "N/A"; }
        }

        public static void CycleSmartASSAdvancedDirection(object smartass, int direction)
        {
            if (smartass == null || f_SmartASS_AdvDirection == null) return;
            try
            {
                Type enumType = f_SmartASS_AdvDirection.FieldType;
                Array values = Enum.GetValues(enumType);
                object current = f_SmartASS_AdvDirection.GetValue(smartass);
                int idx = Array.IndexOf(values, current);
                if (idx < 0) idx = 0;
                int next = (idx + direction + values.Length) % values.Length;
                f_SmartASS_AdvDirection.SetValue(smartass, values.GetValue(next));
            }
            catch { }
        }
        #endregion

        #region Node Executor Methods
        /// <summary>
        /// Execute one maneuver node
        /// </summary>
        public static void ExecuteOneNode(MechJebCore core, object controller)
        {
            object node = GetNodeExecutor(core);
            if (node == null || m_NodeExecutor_ExecuteOneNode == null) return;
            try
            {
                m_NodeExecutor_ExecuteOneNode.Invoke(node, new object[] { controller });
            }
            catch { }
        }

        /// <summary>
        /// Abort node execution
        /// </summary>
        public static void AbortNode(MechJebCore core)
        {
            object node = GetNodeExecutor(core);
            if (node == null || m_NodeExecutor_Abort == null) return;
            try
            {
                m_NodeExecutor_Abort.Invoke(node, null);
            }
            catch { }
        }

        /// <summary>
        /// Check if node executor is running
        /// </summary>
        public static bool IsNodeExecutorRunning(MechJebCore core)
        {
            object node = GetNodeExecutor(core);
            return GetModuleEnabled(node);
        }

        /// <summary>
        /// Get node executor autowarp setting
        /// </summary>
        public static bool GetNodeAutowarp(MechJebCore core)
        {
            object node = GetNodeExecutor(core);
            if (node == null || f_NodeExecutor_Autowarp == null) return false;
            try
            {
                return (bool)f_NodeExecutor_Autowarp.GetValue(node);
            }
            catch
            {
                return false;
            }
        }

        /// <summary>
        /// Set node executor autowarp setting
        /// </summary>
        public static void SetNodeAutowarp(MechJebCore core, bool autowarp)
        {
            object node = GetNodeExecutor(core);
            if (node == null || f_NodeExecutor_Autowarp == null) return;
            try
            {
                f_NodeExecutor_Autowarp.SetValue(node, autowarp);
            }
            catch { }
        }

        #endregion

        #region Target Controller Methods
        /// <summary>
        /// Check if a position target exists
        /// </summary>
        public static bool PositionTargetExists(MechJebCore core)
        {
            object target = GetTargetController(core);
            if (target == null || p_Target_PositionTargetExists == null) return false;
            try
            {
                return (bool)p_Target_PositionTargetExists.GetValue(target, null);
            }
            catch
            {
                return false;
            }
        }


        /// <summary>
        /// Get target latitude
        /// </summary>
        public static double GetTargetLatitude(MechJebCore core)
        {
            return core.Target.targetLatitude;
        }

        /// <summary>
        /// Get target longitude
        /// </summary>
        public static double GetTargetLongitude(MechJebCore core)
        {
            return core.Target.targetLongitude;
        }
        /// <summary>
        /// Set target latitude using existing longitude
        /// </summary>
        public static void SetTargetLatitude(MechJebCore core, CelestialBody body, double latitude)
        {
            if (body == null) return;
            double lon = GetTargetLongitude(core);
            SetPositionTarget(core, body, latitude, lon);
        }

        /// <summary>
        /// Set target longitude using existing latitude
        /// </summary>
        public static void SetTargetLongitude(MechJebCore core, CelestialBody body, double longitude)
        {
            if (body == null) return;
            double lat = GetTargetLatitude(core);
            SetPositionTarget(core, body, lat, longitude);
        }

        /// <summary>
        /// Set position target
        /// </summary>
        public static void SetPositionTarget(MechJebCore core, CelestialBody body, double latitude, double longitude)
        {
            object target = GetTargetController(core);
            if (target == null || m_Target_SetPositionTarget == null) return;
            try
            {
                m_Target_SetPositionTarget.Invoke(target, new object[] { body, latitude, longitude });
            }
            catch { }
        }

        /// <summary>
        /// Open the map to pick a position target
        /// </summary>
        public static void PickPositionTargetOnMap(MechJebCore core)
        {
            object target = GetTargetController(core);
            if (target == null || m_Target_PickPositionTargetOnMap == null) return;
            try
            {
                m_Target_PickPositionTargetOnMap.Invoke(target, null);
            }
            catch { }
        }
        #endregion

        #region Ascent Methods
        /// <summary>
        /// Get ascent settings module
        /// </summary>
        public static object GetAscentSettings(MechJebCore core)
        {
            return GetComputerModule(core, "MechJebModuleAscentSettings");
        }

        /// <summary>
        /// Get the ascent autopilot from settings
        /// </summary>
        public static object GetAscentAutopilot(MechJebCore core)
        {
            object settings = GetAscentSettings(core);
            if (settings == null || p_Ascent_AscentAutopilot == null) return null;
            try
            {
                return p_Ascent_AscentAutopilot.GetValue(settings, null);
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Check if ascent autopilot is engaged
        /// </summary>
        public static bool IsAscentAutopilotEngaged(MechJebCore core)
        {
            object autopilot = GetAscentAutopilot(core);
            return GetModuleEnabled(autopilot);
        }

        /// <summary>
        /// Engage/disengage the ascent autopilot
        /// </summary>
        public static void SetAscentAutopilotEngaged(MechJebCore core, bool engaged, object controller = null)
        {
            object autopilot = GetAscentAutopilot(core);
            if (autopilot == null) return;

            object users = GetModuleUsers(autopilot);
            if (users == null) return;

            if (controller == null) controller = core;

            if (engaged)
            {
                AddUser(users, controller);
            }
            else
            {
                RemoveUser(users, controller);
            }
        }

        /// <summary>
        /// Get ascent autowarp setting
        /// </summary>
        public static bool GetAscentAutowarp(MechJebCore core)
        {
            return GetNodeAutowarp(core);
        }

        /// <summary>
        /// Set ascent autowarp
        /// </summary>
        public static void SetAscentAutowarp(MechJebCore core, bool autowarp)
        {
            SetNodeAutowarp(core, autowarp);
        }

        public static bool GetAscentAutoPath(MechJebCore core)
        {
            object settings = GetAscentSettings(core);
            if (settings == null || f_Ascent_AutoPath == null) return false;
            try
            {
                return (bool)f_Ascent_AutoPath.GetValue(settings);
            }
            catch { return false; }
        }

        public static bool GetAscentSkipCircularization(MechJebCore core)
        {
            object settings = GetAscentSettings(core);
            if (settings == null || f_Ascent_SkipCircularization == null) return false;
            try { return (bool)f_Ascent_SkipCircularization.GetValue(settings); }
            catch { return false; }
        }

        public static void SetAscentSkipCircularization(MechJebCore core, bool value)
        {
            object settings = GetAscentSettings(core);
            if (settings == null || f_Ascent_SkipCircularization == null) return;
            try { f_Ascent_SkipCircularization.SetValue(settings, value); }
            catch { }
        }


        #endregion

        #region Landing Methods
        /// <summary>
        /// Get landing autopilot module
        /// </summary>
        public static MechJebModuleLandingAutopilot GetLandingAutopilot(MechJebCore core)
        {
            return core.Landing;
        }

        /// <summary>
        /// Get landing predictions module
        /// </summary>
        public static MechJebModuleLandingPredictions GetLandingPredictions(MechJebCore core)
        {
            return core.GetComputerModule<MechJebModuleLandingPredictions>();
        }

        /// <summary>
        /// Start landing at position target
        /// </summary>
        public static void LandAtPositionTarget(MechJebCore core, object controller = null)
        {
            object landing = GetLandingAutopilot(core);
            if (landing == null || m_Landing_LandAtPositionTarget == null) return;

            if (controller == null)
            {
                controller = GetComputerModule(core, "MechJebModuleLandingGuidance");
            }

            try
            {
                m_Landing_LandAtPositionTarget.Invoke(landing, new object[] { controller });
            }
            catch { }
        }

        /// <summary>
        /// Start landing untargeted
        /// </summary>
        public static void LandUntargeted(MechJebCore core, object controller = null)
        {
            object landing = GetLandingAutopilot(core);
            if (landing == null || m_Landing_LandUntargeted == null) return;

            if (controller == null)
            {
                controller = GetComputerModule(core, "MechJebModuleLandingGuidance");
            }

            try
            {
                m_Landing_LandUntargeted.Invoke(landing, new object[] { controller });
            }
            catch { }
        }

        /// <summary>
        /// Stop landing
        /// </summary>
        public static void StopLanding(MechJebCore core)
        {
            object landing = GetLandingAutopilot(core);
            if (landing == null || m_Landing_StopLanding == null) return;
            try
            {
                m_Landing_StopLanding.Invoke(landing, null);
            }
            catch { }
        }

        /// <summary>
        /// Check if landing autopilot is engaged
        /// </summary>
        public static bool IsLandingAutopilotEngaged(MechJebCore core)
        {
            object landing = GetLandingAutopilot(core);
            return GetModuleEnabled(landing);
        }

        /// <summary>
        /// Get landing prediction result
        /// </summary>
        public static ReentrySimulation.Result GetLandingPredictionResult(MechJebCore core)
        {
            var predictions = GetLandingPredictions(core);
            return predictions.Result;
        }

        /// <summary>
        /// Get landing prediction end position
        /// </summary>
        public static void GetLandingEndPosition(ReentrySimulation.Result result, out double latitude, out double longitude)
        {
            latitude = result.EndPosition.Latitude;
            longitude = result.EndPosition.Longitude;
        }

        /// <summary>
        /// Get landing prediction end UT
        /// </summary>
        public static double GetLandingEndUT(ReentrySimulation.Result result)
        {
            return result.EndUT;
        }

        /// <summary>
        /// Get max drag gees from prediction
        /// </summary>
        public static double GetLandingMaxDragGees(object result)
        {
            if (result == null || f_Result_MaxDragGees == null) return 0;
            try
            {
                return (double)f_Result_MaxDragGees.GetValue(result);
            }
            catch
            {
                return 0;
            }
        }

        public static bool GetLandingShowTrajectory(MechJebCore core)
        {
            object predictions = GetLandingPredictions(core);
            if (predictions == null || p_Predictions_ShowTrajectory == null) return false;
            try
            {
                return (bool)p_Predictions_ShowTrajectory.GetValue(predictions, null);
            }
            catch
            {
                return false;
            }
        }

        public static void SetLandingShowTrajectory(MechJebCore core, bool show)
        {
            object predictions = GetLandingPredictions(core);
            if (predictions == null || p_Predictions_ShowTrajectory == null) return;
            try
            {
                p_Predictions_ShowTrajectory.SetValue(predictions, show, null);
            }
            catch { }
        }

        #endregion

        #region Stage Stats Methods
        /// <summary>
        /// Get the stage stats module
        /// </summary>
        public static object GetStageStats(MechJebCore core)
        {
            return GetComputerModule(core, "MechJebModuleStageStats");
        }

        /// <summary>
        /// Request an update of stage stats
        /// </summary>
        public static void RequestStageStatsUpdate(MechJebCore core, object controller = null)
        {
            object stats = GetStageStats(core);
            if (stats == null || m_StageStats_RequestUpdate == null) return;
            try
            {
                m_StageStats_RequestUpdate.Invoke(stats, new object[] { controller, false });
            }
            catch { }
        }

        /// <summary>
        /// Get vacuum stage stats list
        /// </summary>
        public static System.Collections.IList GetVacuumStageStats(MechJebCore core)
        {
            object stats = GetStageStats(core);
            if (stats == null || f_StageStats_VacStats == null) return null;
            try
            {
                return f_StageStats_VacStats.GetValue(stats) as System.Collections.IList;
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Get atmospheric stage stats list
        /// </summary>
        public static System.Collections.IList GetAtmoStageStats(MechJebCore core)
        {
            object stats = GetStageStats(core);
            if (stats == null || f_StageStats_AtmoStats == null) return null;
            try
            {
                return f_StageStats_AtmoStats.GetValue(stats) as System.Collections.IList;
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Get delta-V for a specific stage
        /// </summary>
        public static double GetStageDeltaV(object fuelStats)
        {
            if (fuelStats == null || f_FuelStats_DeltaV == null) return 0;
            try
            {
                return (double)f_FuelStats_DeltaV.GetValue(fuelStats);
            }
            catch
            {
                return 0;
            }
        }

        /// <summary>
        /// Get total vacuum delta-V
        /// </summary>
        public static double GetTotalVacuumDeltaV(MechJebCore core)
        {
            var stats = GetVacuumStageStats(core);
            if (stats == null) return 0;

            double total = 0;
            foreach (var stage in stats)
            {
                total += GetStageDeltaV(stage);
            }
            return total;
        }

        /// <summary>
        /// Get total atmospheric delta-V
        /// </summary>
        public static double GetTotalAtmoDeltaV(MechJebCore core)
        {
            var stats = GetAtmoStageStats(core);
            if (stats == null) return 0;

            double total = 0;
            foreach (var stage in stats)
            {
                total += GetStageDeltaV(stage);
            }
            return total;
        }
        #endregion


        #region Staging Controller Methods


        /// <summary>
        /// Autostage once
        /// </summary>
        public static void AutostageOnce(MechJebCore core, object controller = null)
        {
            object staging = GetStagingController(core);
            if (staging == null || m_Staging_AutostageOnce == null) return;
            try
            {
                m_Staging_AutostageOnce.Invoke(staging, new object[] { controller });
            }
            catch { }
        }
        #endregion

        #region Docking Methods
        /// <summary>
        /// Get docking autopilot module
        /// </summary>
        public static MechJebModuleDockingAutopilot GetDockingAutopilot(MechJebCore core)
        {
            return core.GetComputerModule<MechJebModuleDockingAutopilot>();
        }


        /// <summary>
        /// Check if docking autopilot is engaged
        /// </summary>
        public static bool IsDockingAutopilotEngaged(MechJebCore core)
        {
            object docking = GetDockingAutopilot(core);
            return GetModuleEnabled(docking);
        }

        /// <summary>
        /// Set docking autopilot engaged
        /// </summary>
        public static void SetDockingAutopilotEngaged(MechJebCore core, bool engaged)
        {
            object docking = GetDockingAutopilot(core);
            if (docking == null) return;
            
            SetModuleEnabled(docking, engaged);
        }

        /// <summary>
        /// Get docking status
        /// </summary>
        public static string GetDockingStatus(MechJebCore core)
        {
            object docking = GetDockingAutopilot(core);
            if (docking == null || p_Docking_Status == null) return "";
            try
            {
                return (string)p_Docking_Status.GetValue(docking, null);
            }
            catch
            {
                return "";
            }
        }

        #endregion

        #region Rendezvous Methods
        /// <summary>
        /// Get rendezvous autopilot module
        /// </summary>
        public static MechJebModuleRendezvousAutopilot GetRendezvousAutopilot(MechJebCore core)
        {
            return core.GetComputerModule<MechJebModuleRendezvousAutopilot>();
        }

        /// <summary>
        /// Check if rendezvous autopilot is engaged
        /// </summary>
        public static bool IsRendezvousAutopilotEngaged(MechJebCore core)
        {
            object rendezvous = GetRendezvousAutopilot(core);
            return GetModuleEnabled(rendezvous);
        }

        /// <summary>
        /// Set rendezvous autopilot engaged
        /// </summary>
        public static void SetRendezvousAutopilotEngaged(MechJebCore core, bool engaged, object controller = null)
        {
            object rendezvous = GetRendezvousAutopilot(core);
            if (rendezvous == null) return;

            object users = GetModuleUsers(rendezvous);
            if (users == null) return;

            if (controller == null)
            {
                controller = GetComputerModule(core, "MechJebModuleRendezvousAutopilotWindow");
            }

            if (controller == null) return;

            if (engaged)
            {
                AddUser(users, controller);
            }
            else
            {
                RemoveUser(users, controller);
            }
        }


        /// <summary>
        /// Get rendezvous status
        /// </summary>
        public static string GetRendezvousStatus(MechJebCore core)
        {
            object rendezvous = GetRendezvousAutopilot(core);
            if (rendezvous == null || p_Rendezvous_Status == null) return "";
            try
            {
                return (string)p_Rendezvous_Status.GetValue(rendezvous, null);
            }
            catch
            {
                return "";
            }
        }

        #endregion

        #region Translatron Methods
        /// <summary>
        /// Get translatron module
        /// </summary>
        public static MechJebModuleTranslatron GetTranslatron(MechJebCore core)
        {
            return core.GetComputerModule<MechJebModuleTranslatron>();
        }

        /// <summary>
        /// Set translatron mode
        /// </summary>
        public static void SetTranslatronMode(MechJebCore core, TranslatronMode mode)
        {
            object translatron = GetTranslatron(core);
            if (translatron == null || m_Translatron_SetMode == null) return;
            try
            {
                object mjMode = Enum.ToObject(t_TranslatronMode, (int)mode);
                m_Translatron_SetMode.Invoke(translatron, new object[] { mjMode });
            }
            catch { }
        }


        /// <summary>
        /// Get kill horizontal velocity setting
        /// </summary>
        public static bool GetTranslatronKillH(MechJebCore core)
        {
            object translatron = GetTranslatron(core);
            if (translatron == null || p_Translatron_TransKillH == null) return false;
            try
            {
                return (bool)p_Translatron_TransKillH.GetValue(translatron, null);
            }
            catch
            {
                return false;
            }
        }

        /// <summary>
        /// Set kill horizontal velocity
        /// </summary>
        public static void SetTranslatronKillH(MechJebCore core, bool killH)
        {
            object translatron = GetTranslatron(core);
            if (translatron == null || p_Translatron_TransKillH == null) return;
            try
            {
                p_Translatron_TransKillH.SetValue(translatron, killH, null);
            }
            catch { }
        }

        /// <summary>
        /// Trigger PANIC button
        /// </summary>
        public static void PanicSwitch(MechJebCore core)
        {
            object translatron = GetTranslatron(core);
            if (translatron == null || m_Translatron_PanicSwitch == null) return;
            try
            {
                m_Translatron_PanicSwitch.Invoke(translatron, null);
            }
            catch { }
        }
        #endregion

        #region Rover Methods


        public static void DriveToTarget(MechJebCore core)
        {
            object rover = core.Rover;
            if (rover == null || m_Rover_DriveToTarget == null) return;
            try { m_Rover_DriveToTarget.Invoke(rover, null); }
            catch { }
        }

        public static void StopRover(MechJebCore core)
        {
            object rover = core.Rover;
            if (rover == null || m_Rover_Stop == null) return;
            try { m_Rover_Stop.Invoke(rover, null); }
            catch { }
        }

        public static void AddRoverWaypointAtCurrentPosition(MechJebCore core, Vessel vessel)
        {
            object rover = core.Rover;
            if (rover == null || vessel == null || m_Rover_AddWaypoint == null) return;

            try
            {
                ParameterInfo[] parms = m_Rover_AddWaypoint.GetParameters();
                if (parms.Length == 0)
                {
                    m_Rover_AddWaypoint.Invoke(rover, null);
                    return;
                }

                if (parms.Length == 2 && parms[0].ParameterType == typeof(double) && parms[1].ParameterType == typeof(double))
                {
                    m_Rover_AddWaypoint.Invoke(rover, new object[] { vessel.latitude, vessel.longitude });
                    return;
                }

                if (parms.Length == 1 && t_AbsoluteVector != null && parms[0].ParameterType == t_AbsoluteVector)
                {
                    object absVec = CreateAbsoluteVector(vessel.mainBody, vessel.latitude, vessel.longitude, vessel.altitude);
                    if (absVec != null)
                    {
                        m_Rover_AddWaypoint.Invoke(rover, new object[] { absVec });
                    }
                }
            }
            catch { }
        }

        public static void ClearRoverWaypoints(MechJebCore core)
        {
            object rover = core.Rover;
            if (rover == null || m_Rover_ClearWaypoints == null) return;
            try { m_Rover_ClearWaypoints.Invoke(rover, null); }
            catch { }
        }
        #endregion

        #region Spaceplane Autopilot Methods
        public static MechJebModuleSpaceplaneAutopilot GetSpaceplaneAutopilot(MechJebCore core)
        {
            return core.GetComputerModule<MechJebModuleSpaceplaneAutopilot>();
        }

        public static void SpaceplaneHoldHeadingAndAltitude(MechJebCore core)
        {
            object sp = GetSpaceplaneAutopilot(core);
            if (sp == null || m_Spaceplane_HoldHeadingAndAltitude == null) return;
            try { m_Spaceplane_HoldHeadingAndAltitude.Invoke(sp, null); }
            catch { }
        }

        #endregion

        #region Warp Methods
        public static object GetWarpController(MechJebCore core)
        {
            return GetComputerModule(core, "MechJebModuleWarpController");
        }

        public static void WarpToUT(MechJebCore core, double ut)
        {
            object warp = GetWarpController(core);
            if (warp == null || m_Warp_WarpToUT == null) return;
            try
            {
                m_Warp_WarpToUT.Invoke(warp, new object[] { ut });
            }
            catch { }
        }
        #endregion

        #region Maneuver Calculator Methods
        // These would call OrbitalManeuverCalculator static methods
        // Implementing a few key ones

        /// <summary>
        /// Place a maneuver node
        /// </summary>
        public static void PlaceManeuverNode(Vessel vessel, Orbit orbit, Vector3d dV, double UT)
        {
            if (m_PlaceManeuverNode == null) return;
            try
            {
                m_PlaceManeuverNode.Invoke(null, new object[] { vessel, orbit, dV, UT });
            }
            catch { }
        }

        public static OperationCourseCorrection CreateCourseCorrectionOperation()
        {
            if (t_OperationCourseCorrection == null) return null;
            try
            {
                return Activator.CreateInstance(t_OperationCourseCorrection) as OperationCourseCorrection;
            }
            catch { return null; }
        }

        public static void SetCourseCorrectionTargetPe(OperationCourseCorrection operation, double peKm)
        {
            if (operation == null) return;

            operation.CourseCorrectFinalPeA.Val = peKm; // review: is this correct?
        }

        static EditableDouble GetPeriapsisEditable(this OperationAdvancedTransfer operation)
        {
            // Note: this field is private, so we either need to keep using reflection or publicize it
            return f_AdvancedTransfer_PeriapsisHeight.GetValue(operation) as EditableDouble;
        }

        public static void StartAdvancedTransferCompute(OperationAdvancedTransfer operation, Orbit orbit, double ut, object targetController, bool includeCaptureBurn, double periapsisKm)
        {
            if (operation == null || targetController == null) return;

            try
            {
                if (f_AdvancedTransfer_IncludeCaptureBurn != null)
                {
                    f_AdvancedTransfer_IncludeCaptureBurn.SetValue(operation, includeCaptureBurn);
                }

                

                operation.GetPeriapsisEditable().Val = periapsisKm;

                if (f_AdvancedTransfer_SelectionMode != null)
                {
                    Type enumType = f_AdvancedTransfer_SelectionMode.FieldType;
                    object limitedTime = Enum.Parse(enumType, "LIMITED_TIME", true);
                    f_AdvancedTransfer_SelectionMode.SetValue(operation, limitedTime);
                }

                Orbit targetOrbit = GetTargetOrbitFromController(targetController);
                if (targetOrbit == null) return;

                if (m_AdvancedTransfer_ComputeTimes != null)
                {
                    m_AdvancedTransfer_ComputeTimes.Invoke(operation, new object[] { orbit, targetOrbit, ut });
                }

                if (m_AdvancedTransfer_ComputeStuff != null)
                {
                    m_AdvancedTransfer_ComputeStuff.Invoke(operation, new object[] { orbit, ut, targetController });
                }
            }
            catch { }
        }

        /// <summary>
        /// Gets the includeCaptureBurn field from OperationAdvancedTransfer
        /// </summary>
        public static bool GetAdvancedTransferIncludeCapture(object operation)
        {
            if (operation == null || f_AdvancedTransfer_IncludeCaptureBurn == null) return false;
            try
            {
                return (bool)f_AdvancedTransfer_IncludeCaptureBurn.GetValue(operation);
            }
            catch { return false; }
        }

        /// <summary>
        /// Sets the includeCaptureBurn field on OperationAdvancedTransfer
        /// </summary>
        public static void SetAdvancedTransferIncludeCapture(object operation, bool value)
        {
            if (operation == null || f_AdvancedTransfer_IncludeCaptureBurn == null) return;
            try
            {
                f_AdvancedTransfer_IncludeCaptureBurn.SetValue(operation, value);
            }
            catch { }
        }

        /// <summary>
        /// Gets the periapsisHeight (in km) from OperationAdvancedTransfer
        /// </summary>
        public static double GetAdvancedTransferPeriapsisKm(OperationAdvancedTransfer operation)
        {
            return operation.GetPeriapsisEditable().Val;
        }

        /// <summary>
        /// Sets the periapsisHeight (in km) on OperationAdvancedTransfer
        /// </summary>
        public static void SetAdvancedTransferPeriapsisKm(OperationAdvancedTransfer operation, double valueKm)
        {
            operation.GetPeriapsisEditable().Val = valueKm;
        }

        public static bool IsAdvancedTransferFinished(OperationAdvancedTransfer operation, out int progress)
        {
            progress = 0;
            if (operation == null || f_AdvancedTransfer_Worker == null) return false;
            try
            {
                object worker = f_AdvancedTransfer_Worker.GetValue(operation);
                if (worker == null || p_TransferCalculator_Finished == null) return false;
                progress = p_TransferCalculator_Progress != null ? (int)p_TransferCalculator_Progress.GetValue(worker, null) : 0;
                return (bool)p_TransferCalculator_Finished.GetValue(worker, null);
            }
            catch { return false; }
        }

        public static void SelectAdvancedTransferLowestDV(OperationAdvancedTransfer operation)
        {
            if (operation == null || f_AdvancedTransfer_Worker == null) return;
            try
            {
                object worker = f_AdvancedTransfer_Worker.GetValue(operation);
                if (worker == null) return;
                // BestDate/BestDuration already represent lowest DV
            }
            catch { }
        }

        public static void SelectAdvancedTransferASAP(OperationAdvancedTransfer operation)
        {
            if (operation == null || f_AdvancedTransfer_Worker == null) return;
            try
            {
                object worker = f_AdvancedTransfer_Worker.GetValue(operation);
                if (worker == null || f_TransferCalculator_Computed == null || f_TransferCalculator_BestDate == null || f_TransferCalculator_BestDuration == null) return;

                double[,] computed = f_TransferCalculator_Computed.GetValue(worker) as double[,];
                if (computed == null) return;

                int bestDuration = 0;
                int durationCount = computed.GetLength(1);
                for (int i = 1; i < durationCount; i++)
                {
                    if (computed[0, bestDuration] > computed[0, i])
                        bestDuration = i;
                }

                f_TransferCalculator_BestDate.SetValue(worker, 0);
                f_TransferCalculator_BestDuration.SetValue(worker, bestDuration);
            }
            catch { }
        }

        /// <summary>
        /// Sets the lastTargetCelestial field on OperationAdvancedTransfer before calling MakeNodes
        /// </summary>
        private static void SetAdvancedTransferTargetCelestial(object operation, CelestialBody target)
        {
            if (operation == null || target == null || f_AdvancedTransfer_LastTargetCelestial == null) return;
            try
            {
                f_AdvancedTransfer_LastTargetCelestial.SetValue(operation, target);
            }
            catch { }
        }

        /// <summary>
        /// Gets the selected departure time and duration from the worker
        /// </summary>
        public static bool GetAdvancedTransferSelection(object operation, out double departureUT, out double duration, out double deltaV)
        {
            departureUT = 0;
            duration = 0;
            deltaV = 0;

            if (operation == null || f_AdvancedTransfer_Worker == null) return false;

            try
            {
                object worker = f_AdvancedTransfer_Worker.GetValue(operation);
                if (worker == null) return false;

                if (f_TransferCalculator_BestDate == null || f_TransferCalculator_BestDuration == null) return false;

                int bestDateIdx = (int)f_TransferCalculator_BestDate.GetValue(worker);
                int bestDurIdx = (int)f_TransferCalculator_BestDuration.GetValue(worker);

                if (m_TransferCalculator_DateFromIndex != null)
                    departureUT = (double)m_TransferCalculator_DateFromIndex.Invoke(worker, new object[] { bestDateIdx });

                if (m_TransferCalculator_DurationFromIndex != null)
                    duration = (double)m_TransferCalculator_DurationFromIndex.Invoke(worker, new object[] { bestDurIdx });

                // Get DV from computed array
                if (f_TransferCalculator_Computed != null)
                {
                    double[,] computed = f_TransferCalculator_Computed.GetValue(worker) as double[,];
                    if (computed != null && bestDateIdx < computed.GetLength(0) && bestDurIdx < computed.GetLength(1))
                    {
                        deltaV = computed[bestDateIdx, bestDurIdx];
                    }
                }

                return true;
            }
            catch { return false; }
        }

        public static bool CreateNodesFromOperation(object operation, Orbit orbit, double ut, object targetController, Vessel vessel)
        {
            if (operation == null || m_Operation_MakeNodes == null || vessel == null) return false;
            try
            {
                // For AdvancedTransfer, ensure lastTargetCelestial is set
                if (t_OperationAdvancedTransfer != null && t_OperationAdvancedTransfer.IsInstanceOfType(operation))
                {
                    CelestialBody targetBody = FlightGlobals.fetch.VesselTarget as CelestialBody;
                    if (targetBody != null)
                    {
                        SetAdvancedTransferTargetCelestial(operation, targetBody);
                    }
                }

                object nodeList = m_Operation_MakeNodes.Invoke(operation, new object[] { orbit, ut, targetController });
                if (nodeList == null) return false;

                System.Collections.IEnumerable nodes = nodeList as System.Collections.IEnumerable;
                if (nodes == null) return false;

                foreach (object node in nodes)
                {
                    if (node == null || f_ManeuverParameters_dV == null || f_ManeuverParameters_UT == null) continue;
                    Vector3d dV = (Vector3d)f_ManeuverParameters_dV.GetValue(node);
                    double nodeUT = (double)f_ManeuverParameters_UT.GetValue(node);
                    PlaceManeuverNode(vessel, orbit, dV, nodeUT);
                }

                return true;
            }
            catch { return false; }
        }

        #region ManeuverPlanner Operation Wrapper

        public static Operation GetOperationByName(string name)
        {
            return operationsByName.GetValueOrDefault(name);
        }

        /// <summary>
        /// Gets the TimeSelector from an operation (for timing options)
        /// </summary>
        public static object GetOperationTimeSelector(object operation)
        {
            if (operation == null || t_TimeSelector == null) return null;
            try
            {
                Type opType = operation.GetType();
                // Look for static _timeSelector field
                FieldInfo field = opType.GetField("_timeSelector", 
                    BindingFlags.NonPublic | BindingFlags.Static);
                if (field == null) return null;
                
                return field.GetValue(null);
            }
            catch { return null; }
        }


        /// <summary>
        /// Sets the current TimeReference index on a TimeSelector
        /// </summary>
        public static void SetTimeSelectorCurrentTimeRef(object timeSelector, int timeRefIndex)
        {
            if (timeSelector == null || f_TimeSelector_CurrentTimeRef == null) return;
            try
            {
                f_TimeSelector_CurrentTimeRef.SetValue(timeSelector, timeRefIndex);
            }
            catch { }
        }

        /// <summary>
        /// Gets the CircularizeAltitude from a TimeSelector (in meters)
        /// </summary>
        public static double GetTimeSelectorCircularizeAltitude(object timeSelector)
        {
            if (timeSelector == null || f_TimeSelector_CircularizeAltitude == null || p_EditableDoubleMult_Val == null) return 0;
            try
            {
                object altField = f_TimeSelector_CircularizeAltitude.GetValue(timeSelector);
                if (altField == null) return 0;
                return (double)p_EditableDoubleMult_Val.GetValue(altField, null);
            }
            catch { return 0; }
        }

        /// <summary>
        /// Sets the CircularizeAltitude on a TimeSelector (in meters)
        /// </summary>
        public static void SetTimeSelectorCircularizeAltitude(object timeSelector, double altitudeMeters)
        {
            if (timeSelector == null || f_TimeSelector_CircularizeAltitude == null || p_EditableDoubleMult_Val == null) return;
            try
            {
                object altField = f_TimeSelector_CircularizeAltitude.GetValue(timeSelector);
                if (altField == null) return;
                p_EditableDoubleMult_Val.SetValue(altField, altitudeMeters, null);
            }
            catch { }
        }

        /// <summary>
        /// Gets the LeadTime from a TimeSelector (in seconds)
        /// </summary>
        public static double GetTimeSelectorLeadTime(object timeSelector)
        {
            if (timeSelector == null || f_TimeSelector_LeadTime == null || p_EditableDouble_Val == null) return 0;
            try
            {
                object leadField = f_TimeSelector_LeadTime.GetValue(timeSelector);
                if (leadField == null) return 0;
                return (double)p_EditableDouble_Val.GetValue(leadField, null);
            }
            catch { return 0; }
        }

        /// <summary>
        /// Sets the LeadTime on a TimeSelector (in seconds)
        /// </summary>
        public static void SetTimeSelectorLeadTime(object timeSelector, double seconds)
        {
            if (timeSelector == null || f_TimeSelector_LeadTime == null || p_EditableDouble_Val == null) return;
            try
            {
                object leadField = f_TimeSelector_LeadTime.GetValue(timeSelector);
                if (leadField == null) return;
                p_EditableDouble_Val.SetValue(leadField, seconds, null);
            }
            catch { }
        }


        /// <summary>
        /// Calls MakeNodes on an operation and creates the maneuver nodes
        /// </summary>
        public static bool ExecuteOperation(object operation, MechJebCore core, Vessel vessel)
        {
            if (operation == null || core == null || vessel == null) return false;
            
            Orbit orbit = vessel.orbit;
            double ut = Planetarium.GetUniversalTime();
            object targetController = GetTargetController(core);
            
            return CreateNodesFromOperation(operation, orbit, ut, targetController, vessel);
        }
        #endregion

        #endregion

        private static Orbit GetTargetOrbitFromController(object targetController)
        {
            if (targetController == null || p_Target_TargetOrbit == null) return null;
            try
            {
                return (Orbit)p_Target_TargetOrbit.GetValue(targetController, null);
            }
            catch { return null; }
        }

        #region Additional Operation Types
        // Operation type cache for other maneuvers
        private static Type t_OperationEccentricity;
        private static Type t_OperationLongitude;
        private static Type t_OperationCourseCorrection_FineTune;
        private static Type t_OperationLambert;
        private static Type t_OperationResonantOrbit;
        private static Type t_OperationMoonReturn;
        private static Type t_OperationInterplanetaryTransfer;

        #endregion

    }
}
