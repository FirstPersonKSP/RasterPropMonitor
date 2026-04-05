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
using KSP.UI.Screens;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;
using System.Text;
using UnityEngine;

namespace JSI
{
    public partial class RasterPropMonitorComputer : PartModule
    {
        internal delegate object VariableEvaluator(RPMVesselComputer comp);
        internal delegate double NumericVariableEvaluator(RPMVesselComputer comp);

        private NumericVariableEvaluator sideSlipEvaluator;
        internal float Sideslip
        {
            get
            {
                if (sideSlipEvaluator == null)
                {
                    sideSlipEvaluator = SideSlip();
                }
                RPMVesselComputer comp = RPMVesselComputer.Instance(vid);
                return sideSlipEvaluator(comp).MassageToFloat();
            }
        }

        private NumericVariableEvaluator angleOfAttackEvaluator;
        internal float AbsoluteAoA
        {
            get
            {
                if (angleOfAttackEvaluator == null)
                {
                    angleOfAttackEvaluator = AngleOfAttack();
                }

                RPMVesselComputer comp = RPMVesselComputer.Instance(vessel);
                return ((comp.RotationVesselSurface.eulerAngles.x > 180.0f) ? (360.0f - comp.RotationVesselSurface.eulerAngles.x) : -comp.RotationVesselSurface.eulerAngles.x) - angleOfAttackEvaluator(comp).MassageToFloat();
            }
        }

        #region evaluator
        //static // uncomment this to make sure there's no non-static methods being generated
        internal VariableEvaluator GetEvaluator(string input, out VariableUpdateType updateType)
        {
            updateType = VariableUpdateType.PerFrame;

            // handle literals
            if (input[0] == '$')
            {
                updateType = VariableUpdateType.Constant;
                input = input.Substring(1).Trim();
                return (RPMVesselComputer comp) => input;
            }

			// plugins get first crack at variables
			try
			{
				object result;
				if (plugins.ProcessVariable(input, out result, out bool cacheable))
				{
                    updateType = cacheable ? VariableUpdateType.PerFrame: VariableUpdateType.Volatile;
					// It's a plugin variable.
					return (RPMVesselComputer comp) =>
					{
						object o;
						bool b;
						// Ignore return value - we already checked it
						plugins.ProcessVariable(input, out o, out b);
						return o;
					};
				}
			}
			catch { }

			if (input.IndexOf("_", StringComparison.Ordinal) > -1)
            {
                string[] tokens = input.Split('_');

                switch (tokens[0])
                {
                    case "LISTR":
                        {
                            ushort resourceID = Convert.ToUInt16(tokens[1]);
                            if (tokens[2] == "NAME")
                            {
                                return (RPMVesselComputer comp) => comp.resources.GetActiveResourceByIndex(resourceID);
                            }
                            // the numeric variable handler should have covered the rest
                            return null;
                        }

                    case "PROPELLANTR":
                        if (tokens[2] == "NAME")
                        {
                            ushort propellantIndex = Convert.ToUInt16(tokens[1]);
                            return (RPMVesselComputer comp) => comp.resources.GetPropellantResourceName(propellantIndex);
                        }
                        return null;

                    case "CREWLOCAL":
                        int crewSeatID = Convert.ToInt32(tokens[1]);
                        return (RPMVesselComputer comp) =>
                        {
                            return CrewListElement(tokens[2], crewSeatID, localCrew, localCrewMedical);
                        };

                    case "CREW":
                        int vesselCrewSeatID = Convert.ToInt32(tokens[1]);
                        return (RPMVesselComputer comp) =>
                        {
                            return CrewListElement(tokens[2], vesselCrewSeatID, comp.vesselCrew, comp.vesselCrewMedical);
                        };

                    case "STOREDSTRING":
                        int storedStringNumber;
                        updateType = VariableUpdateType.Constant;
                        if (int.TryParse(tokens[1], out storedStringNumber))
                        {
                            return (RPMVesselComputer comp) => 
                            {
                                if (storedStringNumber >= 0 && storedStringNumber < storedStringsArray.Count)
                                {
                                    return storedStringsArray[storedStringNumber];
                                }
                                else
                                {
                                    return "";
                                }
                            };
                        }
                        else
                        {
                            return null;
                        }

                    case "PLUGIN":
                        Delegate pluginMethod = GetInternalMethod(tokens[1]);
                        if (pluginMethod != null)
                        {
                            MethodInfo mi = pluginMethod.Method;
                            if (mi.ReturnType == typeof(string))
                            {
                                Func<string> method = (Func<string>)pluginMethod;
                                return (RPMVesselComputer comp) => { return method(); };
                            }
                        }
                        JUtil.LogErrorMessage(this, "Unable to create a plugin handler for {0}", tokens[1]);
                        return null;
                }
            }

            if (input.StartsWith("AGMEMO", StringComparison.Ordinal))
            {
                updateType = VariableUpdateType.Constant;
                if (uint.TryParse(input.Substring("AGMEMO".Length), out uint groupID))
                {
                    // if the memo contains a pipe character, the string changes depending on the state of the action group
                    string[] tokens;
                    if (actionGroupMemo[groupID].IndexOf('|') > 1 && (tokens = actionGroupMemo[groupID].Split('|')).Length == 2)
                    {
                        return (RPMVesselComputer comp) =>
                        {
                            return vessel.ActionGroups.groups[RPMVesselComputer.actionGroupID[groupID]]
                                ? tokens[0]
                                : tokens[1];
                        };
                    }
                    else
                    {
                        return (RPMVesselComputer comp) => actionGroupMemo[groupID];
                    }
                }
                else
                {
                    return null;
                }
            }

            // Handle many/most variables
            switch (input)
            {
                // Meta.
                case "RPMVERSION":
                    updateType = VariableUpdateType.Constant;
                    return (RPMVesselComputer comp) => { return FileVersionInfo.GetVersionInfo(Assembly.GetExecutingAssembly().Location).FileVersion; };

                // Orbital parameters
                case "ORBITBODY":
                    return (RPMVesselComputer comp) => { return vessel.orbit.referenceBody.name; };

                case "ENCOUNTERBODY":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                        {
                            switch (vessel.orbit.patchEndTransition)
                            {
                                case Orbit.PatchTransitionType.ENCOUNTER:
                                    return vessel.orbit.nextPatch.referenceBody.bodyName;
                                case Orbit.PatchTransitionType.ESCAPE:
                                    return vessel.mainBody.referenceBody.bodyName;
                            }
                        }
                        return string.Empty;
                    };

                // Names!
                case "NAME":
                    return (RPMVesselComputer comp) => { return vessel.vesselName; };
                case "VESSELTYPE":
                    return (RPMVesselComputer comp) => { return vessel.vesselType.ToString(); };
                case "TARGETTYPE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetVessel != null)
                        {
                            return comp.targetVessel.vesselType.ToString();
                        }
                        if (comp.targetDockingNode != null)
                        {
                            return "Port";
                        }
                        if (comp.targetBody != null)
                        {
                            return "Celestial";
                        }
                        return "Position";
                    };
                case "SITUATION":
                    return (RPMVesselComputer comp) => { return SituationString(vessel.situation); };

                // Targeting
                case "TARGETNAME":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null)
                            return string.Empty;
                        if (comp.target is Vessel || comp.target is CelestialBody || comp.target is ModuleDockingNode)
                            return comp.target.GetName();
                        // What remains is MechJeb's ITargetable implementations, which also can return a name,
                        // but the newline they return in some cases needs to be removed.
                        return comp.target.GetName().Replace('\n', ' ');
                    };
                case "TARGETORBITBODY":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.targetOrbit != null)
                            return comp.targetOrbit.referenceBody.name;
                        return string.Empty;
                    };
                case "TARGETSITUATION":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target is Vessel)
                            return SituationString(comp.target.GetVessel().situation);
                        return string.Empty;
                    };
                case "TARGETSIGNALSTRENGTHCAPTION":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetVessel != null && comp.targetVessel.DiscoveryInfo.Level != DiscoveryLevels.Owned && comp.targetVessel.DiscoveryInfo.HaveKnowledgeAbout(DiscoveryLevels.Presence))
                        {
                            return DiscoveryInfo.GetSignalStrengthCaption(comp.targetVessel.DiscoveryInfo.GetSignalStrength(comp.targetVessel.DiscoveryInfo.lastObservedTime));
                        }
                        else
                        {
                            return "";
                        }
                    };
                case "TARGETSIZECLASS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetVessel != null && comp.targetVessel.DiscoveryInfo.Level != DiscoveryLevels.Owned && comp.targetVessel.DiscoveryInfo.HaveKnowledgeAbout(DiscoveryLevels.Presence))
                        {
                            return comp.targetVessel.DiscoveryInfo.objectSize;
                        }
                        else
                        {
                            return "";
                        }
                    };

                // Thermal
                case "HOTTESTPARTNAME":
                    return (RPMVesselComputer comp) => comp.hottestPartName;

                // SCIENCE
                case "BIOMENAME":
                    return (RPMVesselComputer comp) =>
                    {
                        return vessel.CurrentBiome();
                    };
                case "BIOMEID":
                    return (RPMVesselComputer comp) =>
                    {
                        return ScienceUtil.GetExperimentBiome(vessel.mainBody, vessel.latitude, vessel.longitude);
                    };
                case "PATCHENCOUNTERBODY":
                    return (RPMVesselComputer comp) =>
                    {
                        if (!JUtil.OrbitMakesSense(vessel)) return string.Empty;
                        Orbit patch = GetSelectedPatchOrbit();
                        switch (patch.patchEndTransition)
                        {
                            case Orbit.PatchTransitionType.ENCOUNTER:
                                return patch.nextPatch.referenceBody.bodyName;
                            case Orbit.PatchTransitionType.ESCAPE:
                                return patch.referenceBody.bodyName;
                        }
                        return string.Empty;
                    };
                case "PATCHORBITBODY":
                    return (RPMVesselComputer comp) => GetSelectedPatchOrbit().referenceBody.name;
            }

            return null;
        }
        
        // cacheable: whether the value for this variable can calculated once per frame.  random variables are the only things that aren't cacheable right now
        // constant: whether the value is constant for the lifetime of this part
        // isUpdateable: whether the variable system should recalculate the value of this variable once per frame
        internal NumericVariableEvaluator GetNumericEvaluator(string input, out VariableUpdateType updateType)
        {
            updateType = VariableUpdateType.PerFrame;
            
            // handle literals
            if (double.TryParse(input, out double value))
            {
                updateType = VariableUpdateType.Constant;
                return (RPMVesselComputer comp) => value;
            }

            if (input.IndexOf("_", StringComparison.Ordinal) > -1)
            {
                string[] tokens = input.Split('_');

                switch (tokens[0])
                {
                    case "ISLOADED":
                        string assemblyname = input.Substring(input.IndexOf("_", StringComparison.Ordinal) + 1);

                        updateType = VariableUpdateType.Constant;

                        if (RPMGlobals.knownLoadedAssemblies.Contains(assemblyname))
                        {
                            return (RPMVesselComputer comp) => { return 1.0f; };
                        }
                        else
                        {
                            return (RPMVesselComputer comp) => { return 0.0f; };
                        }
                    case "SYSR":
                        {
                            string resourceName = ResourceDataStorage.ParseResourceQuery(tokens[1], out var valueType, out bool stage);

                            if (RPMGlobals.systemNamedResources.ContainsKey(resourceName))
                            {
                                return (RPMVesselComputer comp) => comp.resources.ListSYSElement(resourceName, valueType, stage);
                            }
                            else
                            {
                                return null;
                            }
                        }
                    case "PROPELLANTR":
                        {
                            if (tokens.Length == 3 && uint.TryParse(tokens[1], out uint propellantIndex))
                            {
                                if (tokens[2] == "NAME") return null;
                                ResourceDataStorage.ParseResourceQuery(tokens[2], out var valueType, out bool stage);
                                return (RPMVesselComputer comp) => comp.resources.GetPropellantResourceValue(propellantIndex, valueType, stage);
                            }
                            else
                            {
                                JUtil.LogErrorMessage(this, "incorrect format for variable: {0}", input);
                                return null;
                            }
                        }
                    case "LISTR":
                        {
                            if (tokens[2] == "NAME") return null;

                            ushort resourceID = Convert.ToUInt16(tokens[1]);

                            ResourceDataStorage.ParseResourceQuery(tokens[2], out var valueType, out bool stage);

                            return (RPMVesselComputer comp) =>
                            {
                                string resourceName = comp.resources.GetActiveResourceByIndex(resourceID);

                                if (string.IsNullOrEmpty(resourceName))
                                {
                                    return 0d;
                                }
                                else
                                {
                                    return comp.resources.ListElement(resourceName, valueType, stage);
                                }
                            };
                        }
                    case "CREWLOCAL":
                        int crewSeatID = Convert.ToInt32(tokens[1]);
                        if (CrewListElementIsNumeric(tokens[2]))
                        {
                            return (RPMVesselComputer comp) =>
                            {
                                return CrewListNumericElement(tokens[2], crewSeatID, localCrew, localCrewMedical);
                            };
                        }
                        else
                        {
                            return null;
                        }

                    case "CREW":
                        int vesselCrewSeatID = Convert.ToInt32(tokens[1]);
                        if (CrewListElementIsNumeric(tokens[2]))
                        {
                            return (RPMVesselComputer comp) =>
                            {
                                return CrewListNumericElement(tokens[2], vesselCrewSeatID, comp.vesselCrew, comp.vesselCrewMedical);
                            };
                        }
                        else
                        {
                            return null;
                        }
                    case "PERIODRANDOM":
                        int periodrandom;
                        if (int.TryParse(tokens[1], out periodrandom))
                        {
                            PeriodicRandomValue v = periodicRandomVals.Find(x => x.period == periodrandom);
                            if (v == null)
                            {
                                v = new PeriodicRandomValue(periodrandom);
                                periodicRandomVals.Add(v);
                            }
                            return (RPMVesselComputer comp) =>
                            {
                                return v.value;
                            };
                        }
                        else
                        {
                            return null;
                        }

                    case "PERIOD":
                        if (tokens[1].Substring(tokens[1].Length - 2) == "HZ")
                        {
                            double period;
                            if (double.TryParse(tokens[1].Substring(0, tokens[1].Length - 2), out period) && period > 0.0)
                            {
                                double invPeriod = 1.0 / period;

                                return (RPMVesselComputer comp) =>
                                {
                                    double remainder = Planetarium.GetUniversalTime() % invPeriod;

                                    return (remainder > invPeriod * 0.5).GetHashCode();
                                };
                            }
                        }

                        return null;

                    case "CUSTOM":
                    case "MAPPED":
                    case "MATH":
                    case "SELECT":
                        if (RPMGlobals.customVariables.ContainsKey(input))
                        {
                            IComplexVariable var;
                            if (!customVariables.ContainsKey(input))
                            {
                                ConfigNode cn = RPMGlobals.customVariables[input];
                                var = JUtil.InstantiateComplexVariable(cn, this);
                                customVariables.Add(input, var);
                            }
                            else
                            {
                                var = customVariables[input];
                            }
                            return (RPMVesselComputer comp) => { return var.Evaluate(); };
                        }
                        else
                        {
                            return null;
                        }

                    case "PERSISTENT":
                        {
                            string substring = input.Substring("PERSISTENT".Length + 1);
                            updateType = VariableUpdateType.Pushed;
                            return (RPMVesselComputer comp) =>
                            {
                                return HasPersistentVariable(substring, false)
                                    ? GetPersistentVariable(substring, -1.0, false)
                                    : GetPersistentVariable(substring, -1.0, true);
                            };
                        }
                    case "PLUGIN":
                        Delegate pluginMethod = GetInternalMethod(tokens[1]);
                        if (pluginMethod != null)
                        {
                            MethodInfo mi = pluginMethod.Method;
                            if (mi.ReturnType == typeof(bool))
                            {
                                Func<bool> method = (Func<bool>)pluginMethod;
                                return (RPMVesselComputer comp) => { return method().GetHashCode(); };
                            }
                            else if (mi.ReturnType == typeof(double))
                            {
                                Func<double> method = (Func<double>)pluginMethod;
                                return (RPMVesselComputer comp) => { return method(); };
                            }
                            else
                            {
                                return null;

                            }
                        }

                        string[] internalModule = tokens[1].Split(':');
                        if (internalModule.Length != 2)
                        {
                            JUtil.LogErrorMessage(this, "Badly-formed plugin name in {0}", input);
                            return null;
                        }

                        InternalProp propToUse = null;
                        foreach (InternalProp thisProp in part.internalModel.props)
                        {
                            foreach (InternalModule module in thisProp.internalModules)
                            {
                                if (module != null && module.ClassName == internalModule[0])
                                {
                                    propToUse = thisProp;
                                    break;
                                }
                            }
                        }

                        if (propToUse == null)
                        {
                            updateType = VariableUpdateType.Constant;
                            JUtil.LogErrorMessage(this, $"Could not find InternalModule for {tokens[1]}");
                            return (RPMVesselComputer comp) => { return -1; };
                        }
                        else
                        {
                            Func<bool> pluginCall = (Func<bool>)JUtil.GetMethod(tokens[1], propToUse, typeof(Func<bool>));
                            if (pluginCall == null)
                            {
                                Func<double> pluginNumericCall = (Func<double>)JUtil.GetMethod(tokens[1], propToUse, typeof(Func<double>));
                                if (pluginNumericCall != null)
                                {
                                    return (RPMVesselComputer comp) => { return pluginNumericCall(); };
                                }
                                else
                                {
                                    updateType = VariableUpdateType.Constant;
                                    // Doesn't exist -- return nothing
                                    return (RPMVesselComputer comp) => { return -1; };
                                }
                            }
                            else
                            {
                                return (RPMVesselComputer comp) => { return pluginCall().GetHashCode(); };
                            }
                        }
                }
            }

            // Action group state.
            if (input.StartsWith("AGSTATE", StringComparison.Ordinal))
            {
                string groupName = input.Substring("AGSTATE".Length);
                uint groupID;
                if (uint.TryParse(groupName, out groupID) && groupID < 10)
                {
                    return (RPMVesselComputer comp) => vessel.ActionGroups.groups[RPMVesselComputer.actionGroupID[groupID]].GetHashCode();
                }
                else if (groupName == "ABORT")
                {
                    return (RPMVesselComputer comp) => vessel.ActionGroups.groups[RPMVesselComputer.abortGroupNumber].GetHashCode();
                }
                else if (groupName == "STAGE")
                {
                    return (RPMVesselComputer comp) => vessel.ActionGroups.groups[RPMVesselComputer.stageGroupNumber].GetHashCode();
                }
                else
                {
                    return null;
                }
            }

            // Handle many/most variables
            switch (input)
            {
                // Conversion constants
                case "MetersToFeet":
                    updateType = VariableUpdateType.Constant;
                    return (RPMVesselComputer comp) => RPMGlobals.MetersToFeet;
                case "MetersPerSecondToKnots":
                    updateType = VariableUpdateType.Constant;
                    return (RPMVesselComputer comp) => RPMGlobals.MetersPerSecondToKnots;
                case "MetersPerSecondToFeetPerMinute":
                    updateType = VariableUpdateType.Constant;
                    return (RPMVesselComputer comp) => RPMGlobals.MetersPerSecondToFeetPerMinute;

                // Speeds.
                case "VERTSPEED":
                    return (RPMVesselComputer comp) => comp.speedVertical;
                case "VERTSPEEDLOG10":
                    return (RPMVesselComputer comp) => JUtil.PseudoLog10(comp.speedVertical);
                case "VERTSPEEDROUNDED":
                    return (RPMVesselComputer comp) => comp.speedVerticalRounded;
                case "RADARALTVERTSPEED":
                    return (RPMVesselComputer comp) => comp.radarAltitudeRate;
                case "TERMINALVELOCITY":
                    return (RPMVesselComputer comp) => TerminalVelocity(comp);
                case "SURFSPEED":
                    return (RPMVesselComputer comp) => vessel.srfSpeed;
                case "SURFSPEEDMACH":
                    // Mach number wiggles around 1e-7 when sitting in launch
                    // clamps before launch, so pull it down to zero if it's close.
                    return (RPMVesselComputer comp) => { return (vessel.mach < 0.001) ? 0.0 : vessel.mach; };
                case "ORBTSPEED":
                    return (RPMVesselComputer comp) => { return vessel.orbit.GetVel().magnitude; };
                case "TRGTSPEED":
                    return (RPMVesselComputer comp) => comp.velocityRelativeTarget.magnitude;
                case "HORZVELOCITY":
                    return (RPMVesselComputer comp) => comp.speedHorizontal;
                case "HORZVELOCITYFORWARD":
                    // Negate it, since this is actually movement on the Z axis,
                    // and we want to treat it as a 2D projection on the surface
                    // such that moving "forward" has a positive value.
                    return (RPMVesselComputer comp) => -Vector3d.Dot(vessel.srf_velocity, comp.SurfaceForward);
                case "HORZVELOCITYRIGHT":
                    return (RPMVesselComputer comp) => Vector3d.Dot(vessel.srf_velocity, comp.SurfaceRight);
                case "EASPEED":
                    return (RPMVesselComputer comp) =>
                    {
                        double densityRatio = (AeroExtensions.GetCurrentDensity(vessel) / 1.225);
                        return vessel.srfSpeed * Math.Sqrt(densityRatio);
                    };
                case "IASPEED":
                    return (RPMVesselComputer comp) =>
                    {
                        double densityRatio = (AeroExtensions.GetCurrentDensity(vessel) / 1.225);
                        double pressureRatio = AeroExtensions.StagnationPressureCalc(vessel.mainBody, vessel.mach);
                        return vessel.srfSpeed * Math.Sqrt(densityRatio) * pressureRatio;
                    };
                case "APPROACHSPEED":
                    return (RPMVesselComputer comp) => comp.approachSpeed;
                case "SELECTEDSPEED":
                    return (RPMVesselComputer comp) =>
                    {
                        switch (FlightGlobals.speedDisplayMode)
                        {
                            case FlightGlobals.SpeedDisplayModes.Orbit:
                                return vessel.orbit.GetVel().magnitude;
                            case FlightGlobals.SpeedDisplayModes.Surface:
                                return vessel.srfSpeed;
                            case FlightGlobals.SpeedDisplayModes.Target:
                                return comp.velocityRelativeTarget.magnitude;
                        }
                        return double.NaN;
                    };
                case "TGTRELX":
                    return (RPMVesselComputer comp) =>
                    {
                        if (FlightGlobals.fetch.VesselTarget != null)
                        {
                            return Vector3d.Dot(FlightGlobals.ship_tgtVelocity, FlightGlobals.ActiveVessel.ReferenceTransform.right);
                        }
                        else
                        {
                            return 0.0;
                        }
                    };

                case "TGTRELY":
                    return (RPMVesselComputer comp) =>
                    {
                        if (FlightGlobals.fetch.VesselTarget != null)
                        {
                            return Vector3d.Dot(FlightGlobals.ship_tgtVelocity, FlightGlobals.ActiveVessel.ReferenceTransform.forward);
                        }
                        else
                        {
                            return 0.0;
                        }
                    };
                case "TGTRELZ":
                    return (RPMVesselComputer comp) =>
                    {
                        if (FlightGlobals.fetch.VesselTarget != null)
                        {
                            return Vector3d.Dot(FlightGlobals.ship_tgtVelocity, FlightGlobals.ActiveVessel.ReferenceTransform.up);
                        }
                        else
                        {
                            return 0.0;
                        }
                    };

                case "TIMETOIMPACTSECS":
                    return (RPMVesselComputer comp) => TimeToImpact(comp);
                case "SPEEDATIMPACT":
                    return (RPMVesselComputer comp) => comp.SpeedAtImpact(comp.totalCurrentThrust);
                case "BESTSPEEDATIMPACT":
                    return (RPMVesselComputer comp) => comp.SpeedAtImpact(comp.totalLimitedMaximumThrust);
                case "SUICIDEBURNSTARTSECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (vessel.orbit.PeA > 0.0)
                        {
                            return double.NaN;
                        }
                        else
                        {
                            return comp.SuicideBurnCountdown();
                        }
                    };

                case "LATERALBRAKEDISTANCE":
                    // (-(SHIP:SURFACESPEED)^2)/(2*(ship:maxthrust/ship:mass)) 
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.totalLimitedMaximumThrust <= 0.0)
                        {
                            // It should be impossible for wet mass to be zero.
                            return -1.0;
                        }
                        return (comp.speedHorizontal * comp.speedHorizontal) / (2.0 * comp.totalLimitedMaximumThrust / comp.totalShipWetMass);
                    };

                // Altitudes
                case "ALTITUDE":
                    return (RPMVesselComputer comp) => comp.altitudeASL;
                case "ALTITUDELOG10":
                    return (RPMVesselComputer comp) => JUtil.PseudoLog10(comp.altitudeASL);
                case "RADARALT":
                    return (RPMVesselComputer comp) => comp.altitudeTrue;
                case "RADARALTLOG10":
                    return (RPMVesselComputer comp) => JUtil.PseudoLog10(comp.altitudeTrue);
                case "RADARALTOCEAN":
                    return (RPMVesselComputer comp) =>
                    {
                        if (vessel.mainBody.ocean)
                        {
                            return Math.Min(comp.altitudeASL, comp.altitudeTrue);
                        }
                        return comp.altitudeTrue;
                    };
                case "RADARALTOCEANLOG10":
                    return (RPMVesselComputer comp) =>
                    {
                        if (vessel.mainBody.ocean)
                        {
                            return JUtil.PseudoLog10(Math.Min(comp.altitudeASL, comp.altitudeTrue));
                        }
                        return JUtil.PseudoLog10(comp.altitudeTrue);
                    };
                case "ALTITUDEBOTTOM":
                    return (RPMVesselComputer comp) => comp.altitudeBottom;
                case "ALTITUDEBOTTOMLOG10":
                    return (RPMVesselComputer comp) => JUtil.PseudoLog10(comp.altitudeBottom);
                case "TERRAINHEIGHT":
                    return (RPMVesselComputer comp) => vessel.terrainAltitude;
                case "TERRAINDELTA":
                    return (RPMVesselComputer comp) => comp.terrainDelta;
                case "TERRAINHEIGHTLOG10":
                    return (RPMVesselComputer comp) => JUtil.PseudoLog10(vessel.terrainAltitude);
                case "DISTTOATMOSPHERETOP":
                    return (RPMVesselComputer comp) => vessel.orbit.referenceBody.atmosphereDepth - comp.altitudeASL;

                // Atmospheric values
                case "ATMPRESSURE":
                    return (RPMVesselComputer comp) => vessel.staticPressurekPa * PhysicsGlobals.KpaToAtmospheres;
                case "ATMDENSITY":
                    return (RPMVesselComputer comp) => vessel.atmDensity;
                case "DYNAMICPRESSURE":
                    return DynamicPressure();
                case "ATMOSPHEREDEPTH":
                    return (RPMVesselComputer comp) =>
                    {
                        if (vessel.mainBody.atmosphere)
                        {
                            return Math.Pow(FlightGlobals.ActiveVessel.atmDensity / vessel.mainBody.GetDensity(vessel.mainBody.GetPressure(0.0), vessel.mainBody.GetTemperature(0.0)), 0.25);
                        }
                        else
                        {
                            return 0.0;
                        }
                    };

                // Masses.
                case "MASSDRY":
                    return (RPMVesselComputer comp) => comp.totalShipDryMass;
                case "MASSWET":
                    return (RPMVesselComputer comp) => comp.totalShipWetMass;
                case "MASSRESOURCES":
                    return (RPMVesselComputer comp) => comp.totalShipWetMass - comp.totalShipDryMass;
                case "MASSPROPELLANT":
                    return (RPMVesselComputer comp) => comp.resources.PropellantMass(false);
                case "MASSPROPELLANTSTAGE":
                    return (RPMVesselComputer comp) => comp.resources.PropellantMass(true);

                // The delta V calculation.
                case "DELTAV":
                    return DeltaV();
                case "DELTAVSTAGE":
                    return DeltaVStage();

                // Thrust and related
                case "THRUST":
                    return (RPMVesselComputer comp) => comp.totalCurrentThrust;
                case "THRUSTMAX":
                    return (RPMVesselComputer comp) => comp.totalLimitedMaximumThrust;
                case "THRUSTMAXRAW":
                    return (RPMVesselComputer comp) => comp.totalRawMaximumThrust;
                case "THRUSTLIMIT":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.totalRawMaximumThrust > 0.0f) ? comp.totalLimitedMaximumThrust / comp.totalRawMaximumThrust : 0.0f;
                    };
                case "TWR":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.totalCurrentThrust / (comp.totalShipWetMass * comp.localGeeASL));
                    };
                case "TWRMAX":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.totalLimitedMaximumThrust / (comp.totalShipWetMass * comp.localGeeASL));
                    };
                case "ACCEL":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.totalCurrentThrust / comp.totalShipWetMass);
                    };
                case "MAXACCEL":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.totalLimitedMaximumThrust / comp.totalShipWetMass);
                    };
                case "GFORCE":
                    return (RPMVesselComputer comp) => vessel.geeForce_immediate;
                case "EFFECTIVEACCEL":
                    return (RPMVesselComputer comp) => vessel.acceleration.magnitude;
                case "REALISP":
                    return (RPMVesselComputer comp) => comp.actualAverageIsp;
                case "MAXISP":
                    return (RPMVesselComputer comp) => comp.actualMaxIsp;
                case "ACTIVEENGINECOUNT":
                    return (RPMVesselComputer comp) => comp.activeEngineCount;
                case "ENGINECOUNT":
                    return (RPMVesselComputer comp) => comp.currentEngineCount;
                case "CURRENTINTAKEAIRFLOW":
                    return (RPMVesselComputer comp) => comp.currentAirFlow;
                case "CURRENTENGINEFUELFLOW":
                    return (RPMVesselComputer comp) => comp.currentEngineFuelFlow;
                case "MAXENGINEFUELFLOW":
                    return (RPMVesselComputer comp) => comp.maxEngineFuelFlow;
                case "HOVERPOINT":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.localGeeDirect / (comp.totalLimitedMaximumThrust / comp.totalShipWetMass)).Clamp(0.0f, 1.0f);
                    };
                case "HOVERPOINTEXISTS":
                    return (RPMVesselComputer comp) =>
                    {
                        return ((comp.localGeeDirect / (comp.totalLimitedMaximumThrust / comp.totalShipWetMass)) > 1.0f) ? -1.0 : 1.0;
                    };
                case "EFFECTIVERAWTHROTTLE":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.totalRawMaximumThrust > 0.0f) ? (comp.totalCurrentThrust / comp.totalRawMaximumThrust) : 0.0f;
                    };
                case "EFFECTIVETHROTTLE":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.totalLimitedMaximumThrust > 0.0f) ? (comp.totalCurrentThrust / comp.totalLimitedMaximumThrust) : 0.0f;
                    };
                case "DRAG":
                    return DragForce();
                case "DRAGACCEL":
                    return DragAccel();
                case "LIFT":
                    return LiftForce();
                case "LIFTACCEL":
                    return LiftAccel();
                case "ACCELPROGRADE":
                    return (RPMVesselComputer comp) =>
                    {
                        return Vector3.Dot(vessel.acceleration, comp.prograde);
                    };
                case "ACCELRADIAL":
                    return (RPMVesselComputer comp) =>
                    {
                        return Vector3.Dot(vessel.acceleration, comp.radialOut);
                    };
                case "ACCELNORMAL":
                    return (RPMVesselComputer comp) =>
                    {
                        return Vector3.Dot(vessel.acceleration, comp.normalPlus);
                    };
                case "ACCELSURFPROGRADE":
                    return (RPMVesselComputer comp) => { return Vector3.Dot(vessel.acceleration, vessel.srf_velocity.normalized); };
                case "ACCELFORWARD":
                    return (RPMVesselComputer comp) =>
                    {
                        return Vector3.Dot(vessel.acceleration, comp.forward);
                    };
                case "ACCELRIGHT":
                    return (RPMVesselComputer comp) =>
                    {
                        return Vector3.Dot(vessel.acceleration, comp.right);
                    };
                case "ACCELTOP":
                    return (RPMVesselComputer comp) =>
                    {
                        return Vector3.Dot(vessel.acceleration, comp.top);
                    };

                // Power production rates
                case "ELECOUTPUTALTERNATOR":
                    return (RPMVesselComputer comp) => comp.alternatorOutput;
                case "ELECOUTPUTFUELCELL":
                    return (RPMVesselComputer comp) => comp.fuelcellOutput;
                case "ELECOUTPUTGENERATOR":
                    return (RPMVesselComputer comp) => comp.generatorOutput;
                case "ELECOUTPUTSOLAR":
                    return (RPMVesselComputer comp) => comp.solarOutput;

                // Maneuvers
                case "MNODETIMESECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (node != null)
                        {
                            return -(node.UT - Planetarium.GetUniversalTime());
                        }
                        return double.NaN;
                    };
                case "MNODEDV":
                    return (RPMVesselComputer comp) =>
                    {
                        if (node != null)
                        {
                            return node.GetBurnVector(vessel.orbit).magnitude;
                        }
                        return 0d;
                    };
                case "MNODEBURNTIMESECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (node != null)
                        {
                            return JSIInternalRPMButtons.GetBurnDuration((float)node.DeltaV.magnitude);
                        }
                        return double.NaN;
                    };
                case "MNODEEXISTS":
                    return (RPMVesselComputer comp) =>
                    {
                        return node == null ? -1d : 1d;
                    };

                case "MNODEDVPROGRADE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (node != null)
                        {
                            Vector3d burnVector = node.GetBurnVector(vessel.orbit);
                            return Vector3d.Dot(burnVector, vessel.orbit.Prograde(node.UT));
                        }
                        return 0.0;
                    };
                case "MNODEDVNORMAL":
                    return (RPMVesselComputer comp) =>
                    {
                        if (node != null)
                        {
                            Vector3d burnVector = node.GetBurnVector(vessel.orbit);
                            // NormalPlus seems to be backwards...
                            return -Vector3d.Dot(burnVector, vessel.orbit.Normal(node.UT));
                        }
                        return 0.0;
                    };
                case "MNODEDVRADIAL":
                    return (RPMVesselComputer comp) =>
                    {
                        if (node != null)
                        {
                            Vector3d burnVector = node.GetBurnVector(vessel.orbit);
                            return Vector3d.Dot(burnVector, vessel.orbit.Radial(node.UT));
                        }
                        return 0.0;
                    };

                case "MNODEPERIAPSIS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (node != null && node.nextPatch != null)
                        {
                            return node.nextPatch.PeA;
                        }
                        return double.NaN;
                    };
                case "MNODEAPOAPSIS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (node != null && node.nextPatch != null)
                        {
                            return node.nextPatch.ApA;
                        }
                        return double.NaN;
                    };
                case "MNODEINCLINATION":
                    return (RPMVesselComputer comp) =>
                    {
                        if (node != null && node.nextPatch != null)
                        {
                            return node.nextPatch.inclination;
                        }
                        return double.NaN;
                    };
                case "MNODEECCENTRICITY":
                    return (RPMVesselComputer comp) =>
                    {
                        if (node != null && node.nextPatch != null)
                        {
                            return node.nextPatch.eccentricity;
                        }
                        return double.NaN;
                    };

                case "MNODETARGETCLOSESTAPPROACHTIME":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null || comp.targetOrbit == null || node == null || node.nextPatch == null)
                        {
                            return double.NaN;
                        }
                        else
                        {
                            double approachTime, approachDistance;
                            approachDistance = JUtil.GetClosestApproach(node.nextPatch, comp.target, out approachTime);
                            return approachTime - Planetarium.GetUniversalTime();
                        }
                    };
                case "MNODETARGETCLOSESTAPPROACHDISTANCE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null || comp.targetOrbit == null || node == null || node.nextPatch == null)
                        {
                            return double.NaN;
                        }
                        else
                        {
                            double approachTime;
                            return JUtil.GetClosestApproach(node.nextPatch, comp.target, out approachTime);
                        }
                    };
                case "MNODERELATIVEINCLINATION":
                    // MechJeb's targetables don't have orbits.
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null || comp.targetOrbit == null || node == null || node.nextPatch == null)
                        {
                            return double.NaN;
                        }
                        else
                        {
                            return comp.targetOrbit.referenceBody != node.nextPatch.referenceBody ?
                                -1d :
                                node.nextPatch.RelativeInclination_DEG(comp.targetOrbit);
                        }
                    };

                // Orbital parameters
                case "PERIAPSIS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                            return vessel.orbit.PeA;
                        return double.NaN;
                    };
                case "APOAPSIS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                        {
                            return vessel.orbit.ApA;
                        }
                        return double.NaN;
                    };
                case "INCLINATION":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                        {
                            return vessel.orbit.inclination;
                        }
                        return double.NaN;
                    };
                case "ECCENTRICITY":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                        {
                            return vessel.orbit.eccentricity;
                        }
                        return double.NaN;
                    };
                case "SEMIMAJORAXIS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                        {
                            return vessel.orbit.semiMajorAxis;
                        }
                        return double.NaN;
                    };

                case "ORBPERIODSECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                            return vessel.orbit.period;
                        return double.NaN;
                    };
                case "TIMETOAPSECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                            return vessel.orbit.timeToAp;
                        return double.NaN;
                    };
                case "TIMETOPESECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                        {
                            return vessel.orbit.timeToPe;
                        }
                        return double.NaN;
                    };
                case "TIMESINCELASTAP":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                            return vessel.orbit.period - vessel.orbit.timeToAp;
                        return double.NaN;
                    };
                case "TIMESINCELASTPE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                        {
                            return (vessel.orbit.eccentricity < 1.0) ?
                                vessel.orbit.period - vessel.orbit.timeToPe :
                                -vessel.orbit.timeToPe;
                        }
                        return double.NaN;
                    };
                case "TIMETONEXTAPSIS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                        {
                            double apsisType = NextApsisType(vessel);
                            if (apsisType < 0.0)
                            {
                                return vessel.orbit.timeToPe;
                            }
                            return vessel.orbit.timeToAp;
                        }
                        return 0.0;
                    };
                case "NEXTAPSIS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                        {
                            double apsisType = NextApsisType(vessel);
                            if (apsisType < 0.0)
                            {
                                return vessel.orbit.PeA;
                            }
                            if (apsisType > 0.0)
                            {
                                return vessel.orbit.ApA;
                            }
                        }
                        return double.NaN;
                    };
                case "NEXTAPSISTYPE":
                    return (RPMVesselComputer comp) =>
                    {
                        return NextApsisType(vessel);
                    };
                case "ORBITMAKESSENSE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                            return 1d;
                        return -1d;
                    };
                case "TIMETOANEQUATORIAL":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility && vessel.orbit.AscendingNodeEquatorialExists())
                            return vessel.orbit.TimeOfAscendingNodeEquatorial(Planetarium.GetUniversalTime()) - Planetarium.GetUniversalTime();
                        return double.NaN;
                    };
                case "TIMETODNEQUATORIAL":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility && vessel.orbit.DescendingNodeEquatorialExists())
                            return vessel.orbit.TimeOfDescendingNodeEquatorial(Planetarium.GetUniversalTime()) - Planetarium.GetUniversalTime();
                        return double.NaN;
                    };
                case "TIMETOATMOSPHERESECS":
                    return (RPMVesselComputer comp) =>
                    {
                        double timeToAtm = 0.0;
                        if (orbitSensibility && vessel.orbit.referenceBody.atmosphere == true)
                        {
                            try
                            {
                                double now = Planetarium.GetUniversalTime();
                                timeToAtm = vessel.orbit.GetNextTimeOfRadius(now, vessel.orbit.referenceBody.atmosphereDepth + vessel.orbit.referenceBody.Radius) - now;
                                timeToAtm = Math.Max(timeToAtm, 0.0);
                            }
                            catch
                            {
                                //...
                            }
                        }
                        return timeToAtm;
                    };

                // SOI changes in orbits.
                case "ENCOUNTEREXISTS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility)
                        {
                            switch (vessel.orbit.patchEndTransition)
                            {
                                case Orbit.PatchTransitionType.ESCAPE:
                                    return -1d;
                                case Orbit.PatchTransitionType.ENCOUNTER:
                                    return 1d;
                            }
                        }
                        return 0d;
                    };
                case "ENCOUNTERTIME":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility &&
                            (vessel.orbit.patchEndTransition == Orbit.PatchTransitionType.ENCOUNTER ||
                            vessel.orbit.patchEndTransition == Orbit.PatchTransitionType.ESCAPE))
                        {
                            return vessel.orbit.UTsoi - Planetarium.GetUniversalTime();
                        }
                        return 0.0;
                    };

                // Time
                case "UTSECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (GameSettings.KERBIN_TIME)
                        {
                            return Planetarium.GetUniversalTime() + 426 * 6 * 60 * 60;
                        }
                        return Planetarium.GetUniversalTime() + 365 * 24 * 60 * 60;
                    };
                case "TIMEOFDAYSECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (GameSettings.KERBIN_TIME)
                        {
                            return Planetarium.GetUniversalTime() % (6.0 * 60.0 * 60.0);
                        }
                        else
                        {
                            return Planetarium.GetUniversalTime() % (24.0 * 60.0 * 60.0);
                        }
                    };
                case "METSECS":
                    return (RPMVesselComputer comp) => { return vessel.missionTime; };

                // Coordinates.
                case "LATITUDE":
                    return (RPMVesselComputer comp) =>
                    {
                        return vessel.mainBody.GetLatitude(comp.CoM);
                    };
                case "LONGITUDE":
                    return (RPMVesselComputer comp) =>
                    {
                        return JUtil.ClampDegrees180(vessel.mainBody.GetLongitude(comp.CoM));
                    };
                case "TARGETLATITUDE":
                case "LATITUDETGT":
                    return (RPMVesselComputer comp) =>
                    { // These targetables definitely don't have any coordinates.
                        if (comp.target == null || comp.target is CelestialBody)
                        {
                            return double.NaN;
                        }
                        // These definitely do.
                        if (comp.target is Vessel || comp.target is ModuleDockingNode)
                        {
                            return comp.target.GetVessel().mainBody.GetLatitude(comp.target.GetTransform().position);
                        }
                        // We're going to take a guess here and expect MechJeb's PositionTarget and DirectionTarget,
                        // which don't have vessel structures but do have a transform.
                        return vessel.mainBody.GetLatitude(comp.target.GetTransform().position);
                    };
                case "TARGETLONGITUDE":
                case "LONGITUDETGT":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null || comp.target is CelestialBody)
                        {
                            return double.NaN;
                        }
                        if (comp.target is Vessel || comp.target is ModuleDockingNode)
                        {
                            return JUtil.ClampDegrees180(comp.target.GetVessel().mainBody.GetLongitude(comp.target.GetTransform().position));
                        }
                        return vessel.mainBody.GetLongitude(comp.target.GetTransform().position);
                    };

                // Orientation
                case "HEADING":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.rotationVesselSurface.eulerAngles.y;
                    };
                case "PITCH":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.rotationVesselSurface.eulerAngles.x > 180.0f) ? (360.0f - comp.rotationVesselSurface.eulerAngles.x) : -comp.rotationVesselSurface.eulerAngles.x;
                    };
                case "ROLL":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.rotationVesselSurface.eulerAngles.z > 180.0f) ? (360.0f - comp.rotationVesselSurface.eulerAngles.z) : -comp.rotationVesselSurface.eulerAngles.z;
                    };
                case "PITCHRATE":
                    return (RPMVesselComputer comp) => { return -vessel.angularVelocity.x * Mathf.Rad2Deg; };
                case "ROLLRATE":
                    return (RPMVesselComputer comp) => { return -vessel.angularVelocity.y * Mathf.Rad2Deg; };
                case "YAWRATE":
                    return (RPMVesselComputer comp) => { return -vessel.angularVelocity.z * Mathf.Rad2Deg; };
                case "ANGLEOFATTACK":
                    return AngleOfAttack();
                case "SIDESLIP":
                    return SideSlip();

                case "PITCHSURFPROGRADE":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativePitch(vessel.srf_velocity.normalized);
                    };
                case "PITCHSURFRETROGRADE":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativePitch(-vessel.srf_velocity.normalized);
                    };
                case "PITCHPROGRADE":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativePitch(comp.prograde);
                    };
                case "PITCHRETROGRADE":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativePitch(-comp.prograde);
                    };
                case "PITCHRADIALIN":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativePitch(-comp.radialOut);
                    };
                case "PITCHRADIALOUT":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativePitch(comp.radialOut);
                    };
                case "PITCHNORMALPLUS":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativePitch(comp.normalPlus);
                    };
                case "PITCHNORMALMINUS":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativePitch(-comp.normalPlus);
                    };
                case "PITCHNODE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (node != null)
                        {
                            return comp.GetRelativePitch(node.GetBurnVector(vessel.orbit).normalized);
                        }
                        else
                        {
                            return 0.0;
                        }
                    };
                case "PITCHTARGET":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null)
                        {
                            return comp.GetRelativePitch(-comp.targetSeparation.normalized);
                        }
                        else
                        {
                            return 0.0;
                        }
                    };
                case "PITCHTARGETRELPLUS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.velocityRelativeTarget.sqrMagnitude > 0.0)
                        {
                            return comp.GetRelativePitch(comp.velocityRelativeTarget.normalized);
                        }
                        else
                        {
                            return 0.0;
                        }
                    };
                case "PITCHTARGETRELMINUS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.velocityRelativeTarget.sqrMagnitude > 0.0)
                        {
                            return comp.GetRelativePitch(-comp.velocityRelativeTarget.normalized);
                        }
                        else
                        {
                            return 0.0;
                        }
                    };
                case "YAWSURFPROGRADE":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativeYaw(vessel.srf_velocity.normalized);
                    };
                case "YAWSURFRETROGRADE":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativeYaw(-vessel.srf_velocity.normalized);
                    };
                case "YAWPROGRADE":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativeYaw(comp.prograde);
                    };
                case "YAWRETROGRADE":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativeYaw(-comp.prograde);
                    };
                case "YAWRADIALIN":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativeYaw(-comp.radialOut);
                    };
                case "YAWRADIALOUT":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativeYaw(comp.radialOut);
                    };
                case "YAWNORMALPLUS":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativeYaw(comp.normalPlus);
                    };
                case "YAWNORMALMINUS":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.GetRelativeYaw(-comp.normalPlus);
                    };
                case "YAWNODE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (node != null)
                        {
                            return comp.GetRelativeYaw(node.GetBurnVector(vessel.orbit).normalized);
                        }
                        else
                        {
                            return 0.0;
                        }
                    };
                case "YAWTARGET":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null)
                        {
                            return comp.GetRelativeYaw(-comp.targetSeparation.normalized);
                        }
                        else
                        {
                            return 0.0;
                        }
                    };
                case "YAWTARGETRELPLUS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.velocityRelativeTarget.sqrMagnitude > 0.0)
                        {
                            return comp.GetRelativeYaw(comp.velocityRelativeTarget.normalized);
                        }
                        else
                        {
                            return 0.0;
                        }
                    };
                case "YAWTARGETRELMINUS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.velocityRelativeTarget.sqrMagnitude > 0.0)
                        {
                            return comp.GetRelativeYaw(-comp.velocityRelativeTarget.normalized);
                        }
                        else
                        {
                            return 0.0;
                        }
                    };


                // comp.targeting. Probably the most finicky bit right now.
                case "TARGETSAMESOI":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null)
                        {
                            return 0;
                        }
                        else
                        {
                            return (comp.targetOrbit.referenceBody == vessel.orbit.referenceBody).GetHashCode();
                        }
                    };
                case "TARGETDISTANCE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null)
                            return comp.targetDistance;
                        return -1d;
                    };
                case "TARGETGROUNDDISTANCE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null)
                        {
                            Vector3d targetGroundPos = comp.target.ProjectPositionOntoSurface(vessel.mainBody);
                            if (targetGroundPos != Vector3d.zero)
                            {
                                return Vector3d.Distance(targetGroundPos, vessel.ProjectPositionOntoSurface());
                            }
                        }
                        return -1d;
                    };
                case "RELATIVEINCLINATION":
                    // MechJeb's comp.targetables don't have orbits.
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.targetOrbit != null)
                        {
                            return comp.targetOrbit.referenceBody != vessel.orbit.referenceBody ?
                                -1d :
                                vessel.GetOrbit().RelativeInclination_DEG(comp.targetOrbit);
                        }
                        return double.NaN;
                    };
                case "ANYTARGETEXISTS":
                    return (RPMVesselComputer comp) => comp.target == null ? -1d : 1d;
                case "TARGETEXISTS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null)
                            return -1d;
                        if (comp.target is Vessel)
                            return 1d;
                        return 0d;
                    };
                case "TARGETISDOCKINGPORT":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null)
                            return -1d;
                        if (comp.target is ModuleDockingNode)
                            return 1d;
                        return 0d;
                    };
                case "TARGETISVESSELORPORT":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null)
                            return -1d;
                        if (comp.target is ModuleDockingNode || comp.target is Vessel)
                            return 1d;
                        return 0d;
                    };
                case "TARGETISCELESTIAL":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null)
                            return -1d;
                        if (comp.target is CelestialBody)
                            return 1d;
                        return 0d;
                    };
                case "TARGETISPOSITION":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null)
                        {
                            return -1d;
                        }
                        else if (comp.target is PositionTarget)
                        {
                            return 1d;
                        }
                        else
                        {
                            return 0d;
                        }
                    };
                case "TARGETALTITUDE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null)
                        {
                            return -1d;
                        }
                        if (comp.target is CelestialBody)
                        {
                            if (comp.targetBody == vessel.mainBody || comp.targetBody == Planetarium.fetch.Sun)
                            {
                                return 0d;
                            }
                            else
                            {
                                return comp.targetBody.referenceBody.GetAltitude(comp.targetBody.position);
                            }
                        }
                        if (comp.target is Vessel || comp.target is ModuleDockingNode)
                        {
                            return comp.target.GetVessel().mainBody.GetAltitude(comp.target.GetVessel().CoM);
                        }
                        else
                        {
                            return vessel.mainBody.GetAltitude(comp.target.GetTransform().position);
                        }
                    };
                // MOARdV: I don't think these are needed - I don't remember why we needed comp.targetOrbit
                //if (comp.targetOrbit != null)
                //{
                //    return comp.targetOrbit.altitude;
                //}
                //return -1d;
                case "TARGETSEMIMAJORAXIS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null)
                            return double.NaN;
                        if (comp.targetOrbit != null)
                            return comp.targetOrbit.semiMajorAxis;
                        return double.NaN;
                    };
                case "TIMETOANWITHTARGETSECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null)
                            return double.NaN;
                        return vessel.GetOrbit().TimeOfAscendingNode(comp.target.GetOrbit(), Planetarium.GetUniversalTime()) - Planetarium.GetUniversalTime();
                    };
                case "TIMETODNWITHTARGETSECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null)
                            return double.NaN;
                        return vessel.GetOrbit().TimeOfDescendingNode(comp.target.GetOrbit(), Planetarium.GetUniversalTime()) - Planetarium.GetUniversalTime();
                    };
                case "TARGETCLOSESTAPPROACHTIME":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null || orbitSensibility == false)
                        {
                            return double.NaN;
                        }
                        else
                        {
                            double approachTime, approachDistance;
                            approachDistance = JUtil.GetClosestApproach(vessel.GetOrbit(), comp.target, out approachTime);
                            return approachTime - Planetarium.GetUniversalTime();
                        }
                    };
                case "TARGETCLOSESTAPPROACHDISTANCE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target == null || orbitSensibility == false)
                        {
                            return double.NaN;
                        }
                        else
                        {
                            double approachTime;
                            return JUtil.GetClosestApproach(vessel.GetOrbit(), comp.target, out approachTime);
                        }
                    };

                // Space Objects (asteroid) specifics
                case "TARGETSIGNALSTRENGTH":
                    // MOARdV:
                    // Based on observation, it appears the discovery
                    // level bitfield is basically unused - either the
                    // craft is Owned (-1) or Unowned (29 - which is the
                    // OR of all the bits).  However, maybe career mode uses
                    // the bits, so I will make a guess on what knowledge is
                    // appropriate here.
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetVessel != null && comp.targetVessel.DiscoveryInfo.Level != DiscoveryLevels.Owned && comp.targetVessel.DiscoveryInfo.HaveKnowledgeAbout(DiscoveryLevels.Presence))
                        {
                            return comp.targetVessel.DiscoveryInfo.GetSignalStrength(comp.targetVessel.DiscoveryInfo.lastObservedTime);
                        }
                        else
                        {
                            return -1.0;
                        }
                    };

                case "TARGETLASTOBSERVEDTIMEUT":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetVessel != null && comp.targetVessel.DiscoveryInfo.Level != DiscoveryLevels.Owned && comp.targetVessel.DiscoveryInfo.HaveKnowledgeAbout(DiscoveryLevels.Presence))
                        {
                            return comp.targetVessel.DiscoveryInfo.lastObservedTime;
                        }
                        else
                        {
                            return -1.0;
                        }
                    };

                case "TARGETLASTOBSERVEDTIMESECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetVessel != null && comp.targetVessel.DiscoveryInfo.Level != DiscoveryLevels.Owned && comp.targetVessel.DiscoveryInfo.HaveKnowledgeAbout(DiscoveryLevels.Presence))
                        {
                            return Math.Max(Planetarium.GetUniversalTime() - comp.targetVessel.DiscoveryInfo.lastObservedTime, 0.0);
                        }
                        else
                        {
                            return -1.0;
                        }
                    };

                case "TARGETDISTANCEX":    //distance to comp.target along the yaw axis (j and l rcs keys)
                    return (RPMVesselComputer comp) =>
                    {
                        return Vector3d.Dot(comp.targetSeparation, vessel.GetTransform().right);
                    };
                case "TARGETDISTANCEY":   //distance to comp.target along the pitch axis (i and k rcs keys)
                    return (RPMVesselComputer comp) =>
                    {
                        return Vector3d.Dot(comp.targetSeparation, vessel.GetTransform().forward);
                    };
                case "TARGETDISTANCEZ":  //closure distance from comp.target - (h and n rcs keys)
                    return (RPMVesselComputer comp) =>
                    {
                        return -Vector3d.Dot(comp.targetSeparation, vessel.GetTransform().up);
                    };

                case "TARGETDISTANCESCALEDX":    //scaled and clamped version of comp.targetDISTANCEX.  Returns a number between 100 and -100, with precision increasing as distance decreases.
                    return (RPMVesselComputer comp) =>
                    {
                        double scaledX = Vector3d.Dot(comp.targetSeparation, vessel.GetTransform().right);
                        double zdist = -Vector3d.Dot(comp.targetSeparation, vessel.GetTransform().up);
                        if (zdist < .1)
                            scaledX = scaledX / (0.1 * Math.Sign(zdist));
                        else
                            scaledX = ((scaledX + zdist) / (zdist + zdist)) * (100) - 50;
                        if (scaledX > 100) scaledX = 100;
                        if (scaledX < -100) scaledX = -100;
                        return scaledX;
                    };


                case "TARGETDISTANCESCALEDY":  //scaled and clamped version of comp.targetDISTANCEY.  These two numbers will control the position needles on a docking port alignment gauge.
                    return (RPMVesselComputer comp) =>
                    {
                        double scaledY = Vector3d.Dot(comp.targetSeparation, vessel.GetTransform().forward);
                        double zdist2 = -Vector3d.Dot(comp.targetSeparation, vessel.GetTransform().up);
                        if (zdist2 < .1)
                            scaledY = scaledY / (0.1 * Math.Sign(zdist2));
                        else
                            scaledY = ((scaledY + zdist2) / (zdist2 + zdist2)) * (100) - 50;
                        if (scaledY > 100) scaledY = 100;
                        if (scaledY < -100) scaledY = -100;
                        return scaledY;
                    };

                // TODO: I probably should return something else for vessels. But not sure what exactly right now.
                case "TARGETANGLEX":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null)
                        {
                            if (comp.targetDockingNode != null)
                                return JUtil.NormalAngle(-comp.targetDockingNode.GetTransform().forward, FlightGlobals.ActiveVessel.ReferenceTransform.up, FlightGlobals.ActiveVessel.ReferenceTransform.forward);
                            if (comp.target is Vessel)
                                return JUtil.NormalAngle(-comp.target.GetFwdVector(), comp.forward, comp.up);
                            return 0d;
                        }
                        return 0d;
                    };
                case "TARGETANGLEY":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null)
                        {
                            if (comp.targetDockingNode != null)
                                return JUtil.NormalAngle(-comp.targetDockingNode.GetTransform().forward, FlightGlobals.ActiveVessel.ReferenceTransform.up, -FlightGlobals.ActiveVessel.ReferenceTransform.right);
                            if (comp.target is Vessel)
                            {
                                JUtil.NormalAngle(-comp.target.GetFwdVector(), comp.forward, -comp.right);
                            }
                            return 0d;
                        }
                        return 0d;
                    };
                case "TARGETANGLEZ":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null)
                        {
                            if (comp.targetDockingNode != null)
                                return (360 - (JUtil.NormalAngle(-comp.targetDockingNode.GetTransform().up, FlightGlobals.ActiveVessel.ReferenceTransform.forward, FlightGlobals.ActiveVessel.ReferenceTransform.up))) % 360;
                            if (comp.target is Vessel)
                            {
                                return JUtil.NormalAngle(comp.target.GetTransform().up, comp.up, -comp.forward);
                            }
                            return 0d;
                        }
                        return 0d;
                    };
                case "TARGETANGLEDEV":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null)
                        {
                            return Vector3d.Angle(vessel.ReferenceTransform.up, FlightGlobals.fetch.vesselTargetDirection);
                        }
                        return 180d;
                    };

                case "TARGETAPOAPSIS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.targetOrbitSensibility)
                            return comp.targetOrbit.ApA;
                        return double.NaN;
                    };
                case "TARGETPERIAPSIS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.targetOrbitSensibility)
                            return comp.targetOrbit.PeA;
                        return double.NaN;
                    };
                case "TARGETINCLINATION":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.targetOrbitSensibility)
                            return comp.targetOrbit.inclination;
                        return double.NaN;
                    };
                case "TARGETECCENTRICITY":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.targetOrbitSensibility)
                            return comp.targetOrbit.eccentricity;
                        return double.NaN;
                    };
                case "TARGETORBITALVEL":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.targetOrbitSensibility)
                            return comp.targetOrbit.orbitalSpeed;
                        return double.NaN;
                    };
                case "TARGETTIMETOAPSECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.targetOrbitSensibility)
                            return comp.targetOrbit.timeToAp;
                        return double.NaN;
                    };
                case "TARGETORBPERIODSECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.targetOrbit != null && comp.targetOrbitSensibility)
                            return comp.targetOrbit.period;
                        return double.NaN;
                    };
                case "TARGETTIMETOPESECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.targetOrbitSensibility)
                        {
                            return comp.targetOrbit.timeToPe;
                        }
                        return double.NaN;
                    };
                case "TARGETLAUNCHTIMESECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetVessel != null && comp.targetVessel.mainBody == vessel.mainBody && (vessel.situation == Vessel.Situations.LANDED || vessel.situation == Vessel.Situations.PRELAUNCH || vessel.situation == Vessel.Situations.SPLASHED))
                        {
                            // MOARdV TODO: Make phase angle a variable?
                            return TimeToPhaseAngle(12.7, vessel.mainBody, vessel.longitude, comp.target.GetOrbit());
                        }
                        else
                        {
                            return 0.0;
                        }
                    };
                case "TARGETPLANELAUNCHTIMESECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetVessel != null && comp.targetVessel.mainBody == vessel.mainBody && (vessel.situation == Vessel.Situations.LANDED || vessel.situation == Vessel.Situations.PRELAUNCH || vessel.situation == Vessel.Situations.SPLASHED))
                        {
                            return TimeToPlane(vessel.mainBody, vessel.latitude, vessel.longitude, comp.target.GetOrbit());
                        }
                        else
                        {
                            return 0.0;
                        }
                    };

                // Protractor-type values (phase angle, ejection angle)
                case "TARGETBODYPHASEANGLE":
                    // comp.targetOrbit is always null if comp.targetOrbitSensibility is false,
                    // so no need to test if the orbit makes sense.
                    return (RPMVesselComputer comp) =>
                    {
                        Protractor.Update(vessel, comp.altitudeASL, comp.targetOrbit);
                        return Protractor.PhaseAngle;
                    };
                case "TARGETBODYPHASEANGLESECS":
                    return (RPMVesselComputer comp) =>
                    {
                        Protractor.Update(vessel, comp.altitudeASL, comp.targetOrbit);
                        return Protractor.TimeToPhaseAngle;
                    };
                case "TARGETBODYEJECTIONANGLE":
                    return (RPMVesselComputer comp) =>
                    {
                        Protractor.Update(vessel, comp.altitudeASL, comp.targetOrbit);
                        return Protractor.EjectionAngle;
                    };
                case "TARGETBODYEJECTIONANGLESECS":
                    return (RPMVesselComputer comp) =>
                    {
                        Protractor.Update(vessel, comp.altitudeASL, comp.targetOrbit);
                        return Protractor.TimeToEjectionAngle;
                    };
                case "TARGETBODYCLOSESTAPPROACH":
                    return (RPMVesselComputer comp) =>
                    {
                        if (orbitSensibility == true)
                        {
                            double approachTime;
                            return JUtil.GetClosestApproach(vessel.GetOrbit(), comp.target, out approachTime);
                        }
                        else
                        {
                            return -1.0;
                        }
                    };
                case "TARGETBODYMOONEJECTIONANGLE":
                    return (RPMVesselComputer comp) =>
                    {
                        Protractor.Update(vessel, comp.altitudeASL, comp.targetOrbit);
                        return Protractor.MoonEjectionAngle;
                    };
                case "TARGETBODYEJECTIONALTITUDE":
                    return (RPMVesselComputer comp) =>
                    {
                        Protractor.Update(vessel, comp.altitudeASL, comp.targetOrbit);
                        return Protractor.EjectionAltitude;
                    };
                case "TARGETBODYDELTAV":
                    return (RPMVesselComputer comp) =>
                    {
                        Protractor.Update(vessel, comp.altitudeASL, comp.targetOrbit);
                        return Protractor.TargetBodyDeltaV;
                    };
                case "PREDICTEDLANDINGALTITUDE":
                    return LandingAltitude();
                case "PREDICTEDLANDINGLATITUDE":
                    return LandingLatitude();
                case "PREDICTEDLANDINGLONGITUDE":
                    return LandingLongitude();
                case "PREDICTEDLANDINGERROR":
                    return LandingError();

                // Flight control status
                case "THROTTLE":
                    return (RPMVesselComputer comp) => { return vessel.ctrlState.mainThrottle; };
                case "STICKPITCH":
                    return (RPMVesselComputer comp) => { return vessel.ctrlState.pitch; };
                case "STICKROLL":
                    return (RPMVesselComputer comp) => { return vessel.ctrlState.roll; };
                case "STICKYAW":
                    return (RPMVesselComputer comp) => { return vessel.ctrlState.yaw; };
                case "STICKPITCHTRIM":
                    return (RPMVesselComputer comp) => { return vessel.ctrlState.pitchTrim; };
                case "STICKROLLTRIM":
                    return (RPMVesselComputer comp) => { return vessel.ctrlState.rollTrim; };
                case "STICKYAWTRIM":
                    return (RPMVesselComputer comp) => { return vessel.ctrlState.yawTrim; };
                case "STICKRCSX":
                    return (RPMVesselComputer comp) => { return vessel.ctrlState.X; };
                case "STICKRCSY":
                    return (RPMVesselComputer comp) => { return vessel.ctrlState.Y; };
                case "STICKRCSZ":
                    return (RPMVesselComputer comp) => { return vessel.ctrlState.Z; };
                case "PRECISIONCONTROL":
                    return (RPMVesselComputer comp) => { return (FlightInputHandler.fetch.precisionMode).GetHashCode(); };
                case "WHEELSTEER":
                    return (RPMVesselComputer comp) => { return vessel.ctrlState.wheelSteer; };
                case "WHEELTHROTTLE":
                    return (RPMVesselComputer comp) => { return vessel.ctrlState.wheelThrottle; };

                // Staging and other stuff
                case "STAGE":
                    return (RPMVesselComputer comp) => { return StageManager.CurrentStage; };
                case "STAGEREADY":
                    return (RPMVesselComputer comp) => { return (StageManager.CanSeparate && InputLockManager.IsUnlocked(ControlTypes.STAGING)).GetHashCode(); };
                case "RANDOM":
                    updateType = VariableUpdateType.Volatile;
                    return (RPMVesselComputer comp) => { return UnityEngine.Random.value; };
                case "RANDOMNORMAL":
                    updateType = VariableUpdateType.Volatile;
                    return (RPMVesselComputer comp) =>
                    {
                        // Box-Muller method tweaked to prevent a 0 in u.
                        float u = UnityEngine.Random.Range(0.0009765625f, 1.0f);
                        float v = UnityEngine.Random.Range(0.0f, 2.0f * Mathf.PI);
                        float x = Mathf.Sqrt(-2.0f * Mathf.Log(u)) * Mathf.Cos(v);
                        // TODO: verify the stddev - I believe it is 1; mean is 0.
                        return x;
                    };

                // Thermals
                case "PODTEMPERATURE":
                    return (RPMVesselComputer comp) => { return (part != null) ? (part.temperature + RPMGlobals.KelvinToCelsius) : 0.0; };
                case "PODTEMPERATUREKELVIN":
                    return (RPMVesselComputer comp) => { return (part != null) ? (part.temperature) : 0.0; };
                case "PODSKINTEMPERATURE":
                    return (RPMVesselComputer comp) => { return (part != null) ? (part.skinTemperature + RPMGlobals.KelvinToCelsius) : 0.0; };
                case "PODSKINTEMPERATUREKELVIN":
                    return (RPMVesselComputer comp) => { return (part != null) ? (part.skinTemperature) : 0.0; };
                case "PODMAXSKINTEMPERATURE":
                    return (RPMVesselComputer comp) => { return (part != null) ? (part.skinMaxTemp + RPMGlobals.KelvinToCelsius) : 0.0; };
                case "PODMAXSKINTEMPERATUREKELVIN":
                    return (RPMVesselComputer comp) => { return (part != null) ? (part.skinMaxTemp) : 0.0; };
                case "PODMAXTEMPERATURE":
                    return (RPMVesselComputer comp) => { return (part != null) ? (part.maxTemp + RPMGlobals.KelvinToCelsius) : 0.0; };
                case "PODMAXTEMPERATUREKELVIN":
                    return (RPMVesselComputer comp) => { return (part != null) ? (part.maxTemp) : 0.0; };
                case "PODNETFLUX":
                    return (RPMVesselComputer comp) => { return (part != null) ? (part.thermalConductionFlux + part.thermalConvectionFlux + part.thermalInternalFlux + part.thermalRadiationFlux) : 0.0; };
                case "EXTERNALTEMPERATURE":
                    return (RPMVesselComputer comp) => { return vessel.externalTemperature + RPMGlobals.KelvinToCelsius; };
                case "EXTERNALTEMPERATUREKELVIN":
                    return (RPMVesselComputer comp) => { return vessel.externalTemperature; };
                case "AMBIENTTEMPERATURE":
                    return (RPMVesselComputer comp) => { return vessel.atmosphericTemperature + RPMGlobals.KelvinToCelsius; };
                case "AMBIENTTEMPERATUREKELVIN":
                    return (RPMVesselComputer comp) => { return vessel.atmosphericTemperature; };
                case "HEATSHIELDTEMPERATURE":
                    return (RPMVesselComputer comp) =>
                    {
                        return (double)comp.heatShieldTemperature + RPMGlobals.KelvinToCelsius;
                    };
                case "HEATSHIELDTEMPERATUREKELVIN":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.heatShieldTemperature;
                    };
                case "HEATSHIELDTEMPERATUREFLUX":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.heatShieldFlux;
                    };
                case "HOTTESTPARTTEMP":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.hottestPartTemperature;
                    };
                case "HOTTESTPARTMAXTEMP":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.hottestPartMaxTemperature;
                    };
                case "HOTTESTPARTTEMPRATIO":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.hottestPartMaxTemperature > 0.0f) ? (comp.hottestPartTemperature / comp.hottestPartMaxTemperature) : 0.0f;
                    };
                case "HOTTESTENGINETEMP":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.hottestEngineTemperature;
                    };
                case "HOTTESTENGINEMAXTEMP":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.hottestEngineMaxTemperature;
                    };
                case "HOTTESTENGINETEMPRATIO":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.hottestEngineMaxTemperature > 0.0f) ? (comp.hottestEngineTemperature / comp.hottestEngineMaxTemperature) : 0.0f;
                    };

                case "SLOPEANGLE":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.slopeAngle;
                    };
                case "SPEEDDISPLAYMODE":
                    return (RPMVesselComputer comp) =>
                    {
                        switch (FlightGlobals.speedDisplayMode)
                        {
                            case FlightGlobals.SpeedDisplayModes.Orbit:
                                return 1d;
                            case FlightGlobals.SpeedDisplayModes.Surface:
                                return 0d;
                            case FlightGlobals.SpeedDisplayModes.Target:
                                return -1d;
                        }
                        return double.NaN;
                    };
                case "ISONKERBINTIME":
                    return (RPMVesselComputer comp) => { return GameSettings.KERBIN_TIME.GetHashCode(); };
                case "ISDOCKINGPORTREFERENCE":
                    return (RPMVesselComputer comp) =>
                    {
                        ModuleDockingNode thatPort = null;
                        Part referencePart = vessel.GetReferenceTransformPart();
                        if (referencePart != null)
                        {
                            foreach (PartModule thatModule in referencePart.Modules)
                            {
                                thatPort = thatModule as ModuleDockingNode;
                                if (thatPort != null)
                                    break;
                            }
                        }
                        if (thatPort != null)
                            return 1d;
                        return 0d;
                    };
                case "ISCLAWREFERENCE":
                    return (RPMVesselComputer comp) =>
                    {
                        ModuleGrappleNode thatClaw = null;
                        Part referencePart = vessel.GetReferenceTransformPart();
                        if (referencePart != null)
                        {
                            foreach (PartModule thatModule in referencePart.Modules)
                            {
                                thatClaw = thatModule as ModuleGrappleNode;
                                if (thatClaw != null)
                                    break;
                            }
                        }
                        if (thatClaw != null)
                            return 1d;
                        return 0d;
                    };
                case "ISREMOTEREFERENCE":
                    return (RPMVesselComputer comp) =>
                    {
                        ModuleCommand thatPod = null;
                        Part referencePart = vessel.GetReferenceTransformPart();
                        if (referencePart != null)
                        {
                            foreach (PartModule thatModule in referencePart.Modules)
                            {
                                thatPod = thatModule as ModuleCommand;
                                if (thatPod != null)
                                    break;
                            }
                        }
                        if (thatPod == null)
                            return 1d;
                        return 0d;
                    };
                case "FLIGHTUIMODE":
                    return (RPMVesselComputer comp) =>
                    {
                        switch (FlightUIModeController.Instance.Mode)
                        {
                            case FlightUIMode.DOCKING:
                                return 1d;
                            case FlightUIMode.STAGING:
                                return -1d;
                            case FlightUIMode.MAPMODE:
                                return 0d;
                            case FlightUIMode.MANEUVER_EDIT:
                                return 2d;
                            case FlightUIMode.MANEUVER_INFO:
                                return 3d;
                        }
                        return double.NaN;
                    };

                // Meta.
                case "MECHJEBAVAILABLE":
                    updateType = VariableUpdateType.Constant;
                    return MechJebAvailable();
                case "TIMEWARPPHYSICS":
                    return (RPMVesselComputer comp) => { return (TimeWarp.CurrentRate > 1.0f && TimeWarp.WarpMode == TimeWarp.Modes.LOW).GetHashCode(); };
                case "TIMEWARPNONPHYSICS":
                    return (RPMVesselComputer comp) => { return (TimeWarp.CurrentRate > 1.0f && TimeWarp.WarpMode == TimeWarp.Modes.HIGH).GetHashCode(); };
                case "TIMEWARPACTIVE":
                    return (RPMVesselComputer comp) => { return (TimeWarp.CurrentRate > 1.0f).GetHashCode(); };
                case "TIMEWARPCURRENT":
                    return (RPMVesselComputer comp) => { return TimeWarp.CurrentRate; };


                // Compound variables which exist to stave off the need to parse logical and arithmetic expressions. :)
                case "GEARALARM":
                    // Returns 1 if vertical speed is negative, gear is not extended, and radar altitude is less than 50m.
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.speedVerticalRounded < 0.0 && !vessel.ActionGroups.groups[RPMVesselComputer.gearGroupNumber] && comp.altitudeBottom < 100.0).GetHashCode();
                    };
                case "GROUNDPROXIMITYALARM":
                    // Returns 1 if, at maximum acceleration, in the time remaining until ground impact, it is impossible to get a vertical speed higher than -10m/s.
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.SpeedAtImpact(comp.totalLimitedMaximumThrust) < -10d).GetHashCode();
                    };
                case "TUMBLEALARM":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.speedVerticalRounded < 0.0 && comp.altitudeBottom < 100.0 && comp.speedHorizontal > 5.0).GetHashCode();
                    };
                case "SLOPEALARM":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.speedVerticalRounded < 0.0 && comp.altitudeBottom < 100.0 && comp.slopeAngle > 15.0f).GetHashCode();
                    };
                case "DOCKINGANGLEALARM":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.targetDockingNode != null && comp.targetDistance < 10.0 && comp.approachSpeed > 0.0f &&
                            (Math.Abs(JUtil.NormalAngle(-comp.targetDockingNode.GetFwdVector(), comp.forward, comp.up)) > 1.5 ||
                            Math.Abs(JUtil.NormalAngle(-comp.targetDockingNode.GetFwdVector(), comp.forward, -comp.right)) > 1.5)).GetHashCode();
                    };
                case "DOCKINGSPEEDALARM":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.targetDockingNode != null && comp.approachSpeed > 2.5f && comp.targetDistance < 15.0).GetHashCode();
                    };
                case "ALTITUDEALARM":
                    return (RPMVesselComputer comp) =>
                    {
                        return (comp.speedVerticalRounded < 0.0 && comp.altitudeBottom < 150.0).GetHashCode();
                    };
                case "PODTEMPERATUREALARM":
                    return (RPMVesselComputer comp) =>
                    {
                        if (part != null)
                        {
                            double tempRatio = part.temperature / part.maxTemp;
                            if (tempRatio > 0.85d)
                            {
                                return 1d;
                            }
                            else if (tempRatio > 0.75d)
                            {
                                return 0d;
                            }
                        }
                        return -1d;
                    };
                // Well, it's not a compound but it's an alarm...
                case "ENGINEOVERHEATALARM":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.anyEnginesOverheating.GetHashCode();
                    };
                case "ENGINEFLAMEOUTALARM":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.anyEnginesFlameout.GetHashCode();
                    };
                case "IMPACTALARM":
                    return (RPMVesselComputer comp) =>
                    {
                        return (part != null && vessel.srfSpeed > part.crashTolerance).GetHashCode();
                    };

                // SCIENCE!!
                case "SCIENCEDATA":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.totalDataAmount;
                    };
                case "SCIENCECOUNT":
                    return (RPMVesselComputer comp) =>
                    {
                        return comp.totalExperimentCount;
                    };

                // Some of the new goodies in 0.24.
                case "REPUTATION":
                    return (RPMVesselComputer comp) => { return Reputation.Instance != null ? Reputation.CurrentRep : 0.0f; };
                case "FUNDS":
                    return (RPMVesselComputer comp) => { return Funding.Instance != null ? Funding.Instance.Funds : 0.0; };

                // CommNet
                case "COMMNETCONNECTED":
                    return (RPMVesselComputer comp) => {
                        return ((vessel.connection != null) && (vessel.connection.IsConnected)) ? 1.0 : 0.0;
                    };
                case "COMMNETSIGNALSTRENGTH":
                    return (RPMVesselComputer comp) => {
                        return (vessel.connection != null) ? (float)vessel.connection.SignalStrength : 0.0;
                    };
                case "COMMNETVESSELCONTROLSTATE":
                    return (RPMVesselComputer comp) => {
                        if (vessel.connection == null)
                            return 0.0;

                        switch (vessel.connection.ControlState)
                        {
                            case CommNet.VesselControlState.None: return 0.0;
                            case CommNet.VesselControlState.Probe: return 2.0;
                            //case CommNet.VesselControlState.ProbeNone: return 2.0;
                            case CommNet.VesselControlState.Kerbal: return 4.0;
                            //case CommNet.VesselControlState.KerbalNone: return 4.0;
                            case CommNet.VesselControlState.Partial: return 8.0;
                            case CommNet.VesselControlState.ProbePartial: return 10.0;
                            case CommNet.VesselControlState.KerbalPartial: return 12.0;
                            case CommNet.VesselControlState.Full: return 16.0;
                            case CommNet.VesselControlState.ProbeFull: return 18.0;
                            case CommNet.VesselControlState.KerbalFull: return 20.0;
                            default: return 0.0;
                        }
                    };

                // Action group flags. To properly format those, use this format:
                // {0:on;0;OFF}
                case "GEAR":
                    return (RPMVesselComputer comp) => { return vessel.ActionGroups.groups[RPMVesselComputer.gearGroupNumber].GetHashCode(); };
                case "BRAKES":
                    return (RPMVesselComputer comp) => { return vessel.ActionGroups.groups[RPMVesselComputer.brakeGroupNumber].GetHashCode(); };
                case "SAS":
                    return (RPMVesselComputer comp) => { return vessel.ActionGroups.groups[RPMVesselComputer.sasGroupNumber].GetHashCode(); };
                case "LIGHTS":
                    return (RPMVesselComputer comp) => { return vessel.ActionGroups.groups[RPMVesselComputer.lightGroupNumber].GetHashCode(); };
                case "RCS":
                    return (RPMVesselComputer comp) => { return vessel.ActionGroups.groups[RPMVesselComputer.rcsGroupNumber].GetHashCode(); };

                // 0.90 SAS mode fields:
                case "SASMODESTABILITY":
                    return (RPMVesselComputer comp) =>
                    {
                        var sasMode = vessel.Autopilot.GetActualMode();
                        return (sasMode == VesselAutopilot.AutopilotMode.StabilityAssist) ? 1.0 : 0.0;
                    };
                case "SASMODEPROGRADE":
                    return (RPMVesselComputer comp) =>
                    {
                        var sasMode = vessel.Autopilot.GetActualMode();
                        return (sasMode == VesselAutopilot.AutopilotMode.Prograde) ? 1.0 :
                            (sasMode == VesselAutopilot.AutopilotMode.Retrograde) ? -1.0 : 0.0;
                    };
                case "SASMODENORMAL":
                    return (RPMVesselComputer comp) =>
                    {
                        var sasMode = vessel.Autopilot.GetActualMode();
                        return (sasMode == VesselAutopilot.AutopilotMode.Normal) ? 1.0 :
                            (sasMode == VesselAutopilot.AutopilotMode.Antinormal) ? -1.0 : 0.0;
                    };
                case "SASMODERADIAL":
                    return (RPMVesselComputer comp) =>
                    {
                        var sasMode = vessel.Autopilot.GetActualMode();
                        return (sasMode == VesselAutopilot.AutopilotMode.RadialOut) ? 1.0 :
                            (sasMode == VesselAutopilot.AutopilotMode.RadialIn) ? -1.0 : 0.0;
                    };
                case "SASMODETARGET":
                    return (RPMVesselComputer comp) =>
                    {
                        var sasMode = vessel.Autopilot.GetActualMode();
                        return (sasMode == VesselAutopilot.AutopilotMode.Target) ? 1.0 :
                            (sasMode == VesselAutopilot.AutopilotMode.AntiTarget) ? -1.0 : 0.0;
                    };
                case "SASMODEMANEUVER":
                    return (RPMVesselComputer comp) =>
                    {
                        var sasMode = vessel.Autopilot.GetActualMode();
                        return (sasMode == VesselAutopilot.AutopilotMode.Maneuver) ? 1.0 : 0.0;
                    };


                // Database information about planetary bodies.
                case "ORBITBODYINDEX":
                    return (RPMVesselComputer comp) => { return vessel.orbit.referenceBody.flightGlobalsIndex; };
                case "TARGETBODYINDEX":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.target != null && comp.targetBody != null)
                        {
                            return comp.targetBody.flightGlobalsIndex;
                        }
                        return -1;
                    };
                case "ORBITBODYATMOSPHERE":
                    return (RPMVesselComputer comp) => { return vessel.orbit.referenceBody.atmosphere ? 1d : -1d; };
                case "TARGETBODYATMOSPHERE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                            return comp.targetBody.atmosphere ? 1d : -1d;
                        return 0d;
                    };
                case "ORBITBODYOXYGEN":
                    return (RPMVesselComputer comp) => { return vessel.orbit.referenceBody.atmosphereContainsOxygen ? 1d : -1d; };
                case "TARGETBODYOXYGEN":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                            return comp.targetBody.atmosphereContainsOxygen ? 1d : -1d;
                        return -1d;
                    };
                case "ORBITBODYSCALEHEIGHT":
                    return (RPMVesselComputer comp) => { return vessel.orbit.referenceBody.atmosphereDepth; };
                case "TARGETBODYSCALEHEIGHT":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                            return comp.targetBody.atmosphereDepth;
                        return -1d;
                    };
                case "ORBITBODYRADIUS":
                    return (RPMVesselComputer comp) => { return vessel.orbit.referenceBody.Radius; };
                case "TARGETBODYRADIUS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                            return comp.targetBody.Radius;
                        return -1d;
                    };
                case "ORBITBODYMASS":
                    return (RPMVesselComputer comp) => { return vessel.orbit.referenceBody.Mass; };
                case "TARGETBODYMASS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                            return comp.targetBody.Mass;
                        return -1d;
                    };
                case "ORBITBODYROTATIONPERIOD":
                    return (RPMVesselComputer comp) => { return vessel.orbit.referenceBody.rotationPeriod; };
                case "TARGETBODYROTATIONPERIOD":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                            return comp.targetBody.rotationPeriod;
                        return -1d;
                    };
                case "ORBITBODYSOI":
                    return (RPMVesselComputer comp) => { return vessel.orbit.referenceBody.sphereOfInfluence; };
                case "TARGETBODYSOI":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                            return comp.targetBody.sphereOfInfluence;
                        return -1d;
                    };
                case "ORBITBODYGEEASL":
                    return (RPMVesselComputer comp) => { return vessel.orbit.referenceBody.GeeASL; };
                case "TARGETBODYGEEASL":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                            return comp.targetBody.GeeASL;
                        return -1d;
                    };
                case "ORBITBODYGM":
                    return (RPMVesselComputer comp) => { return vessel.orbit.referenceBody.gravParameter; };
                case "TARGETBODYGM":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                            return comp.targetBody.gravParameter;
                        return -1d;
                    };
                case "ORBITBODYATMOSPHERETOP":
                    return (RPMVesselComputer comp) => { return vessel.orbit.referenceBody.atmosphereDepth; };
                case "TARGETBODYATMOSPHERETOP":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                            return comp.targetBody.atmosphereDepth;
                        return -1d;
                    };
                case "ORBITBODYESCAPEVEL":
                    return (RPMVesselComputer comp) => { return Math.Sqrt(2 * vessel.orbit.referenceBody.gravParameter / vessel.orbit.referenceBody.Radius); };
                case "TARGETBODYESCAPEVEL":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                            return Math.Sqrt(2 * comp.targetBody.gravParameter / comp.targetBody.Radius);
                        return -1d;
                    };
                case "ORBITBODYAREA":
                    return (RPMVesselComputer comp) => { return 4.0 * Math.PI * vessel.orbit.referenceBody.Radius * vessel.orbit.referenceBody.Radius; };
                case "TARGETBODYAREA":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                            return 4 * Math.PI * comp.targetBody.Radius * comp.targetBody.Radius;
                        return -1d;
                    };
                case "ORBITBODYSYNCORBITALTITUDE":
                    return (RPMVesselComputer comp) =>
                    {
                        double syncRadius = Math.Pow(vessel.orbit.referenceBody.gravParameter / Math.Pow(2.0 * Math.PI / vessel.orbit.referenceBody.rotationPeriod, 2.0), 1.0 / 3.0);
                        return syncRadius > vessel.orbit.referenceBody.sphereOfInfluence ? double.NaN : syncRadius - vessel.orbit.referenceBody.Radius;
                    };
                case "TARGETBODYSYNCORBITALTITUDE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                        {
                            double syncRadiusT = Math.Pow(comp.targetBody.gravParameter / Math.Pow(2 * Math.PI / comp.targetBody.rotationPeriod, 2), 1 / 3d);
                            return syncRadiusT > comp.targetBody.sphereOfInfluence ? double.NaN : syncRadiusT - comp.targetBody.Radius;
                        }
                        return -1d;
                    };
                case "ORBITBODYSYNCORBITVELOCITY":
                    return (RPMVesselComputer comp) =>
                    {
                        return (2 * Math.PI / vessel.orbit.referenceBody.rotationPeriod) *
                            Math.Pow(vessel.orbit.referenceBody.gravParameter / Math.Pow(2.0 * Math.PI / vessel.orbit.referenceBody.rotationPeriod, 2), 1.0 / 3.0d);
                    };
                case "TARGETBODYSYNCORBITVELOCITY":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                        {
                            return (2 * Math.PI / comp.targetBody.rotationPeriod) *
                            Math.Pow(comp.targetBody.gravParameter / Math.Pow(2 * Math.PI / comp.targetBody.rotationPeriod, 2), 1 / 3d);
                        }
                        return -1d;
                    };
                case "ORBITBODYSYNCORBITCIRCUMFERENCE":
                    return (RPMVesselComputer comp) => { return 2 * Math.PI * Math.Pow(vessel.orbit.referenceBody.gravParameter / Math.Pow(2 * Math.PI / vessel.orbit.referenceBody.rotationPeriod, 2), 1 / 3d); };
                case "TARGETBODYSYNCORBICIRCUMFERENCE":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                        {
                            return 2 * Math.PI * Math.Pow(comp.targetBody.gravParameter / Math.Pow(2 * Math.PI / comp.targetBody.rotationPeriod, 2), 1 / 3d);
                        }
                        return -1d;
                    };
                case "ORBITBODYSURFACETEMP":
                    return (RPMVesselComputer comp) => { return FlightGlobals.currentMainBody.atmosphereTemperatureSeaLevel + RPMGlobals.KelvinToCelsius; };
                case "TARGETBODYSURFACETEMP":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                        {
                            return comp.targetBody.atmosphereTemperatureSeaLevel + RPMGlobals.KelvinToCelsius;
                        }
                        return -1d;
                    };
                case "ORBITBODYSURFACETEMPKELVIN":
                    return (RPMVesselComputer comp) => { return FlightGlobals.currentMainBody.atmosphereTemperatureSeaLevel; };
                case "TARGETBODYSURFACETEMPKELVIN":
                    return (RPMVesselComputer comp) =>
                    {
                        if (comp.targetBody != null)
                        {
                            return comp.targetBody.atmosphereTemperatureSeaLevel;
                        }
                        return -1d;
                    };
                case "PATCHINDEX":
                    return (RPMVesselComputer comp) =>
                    {
                        (int index, Orbit _) = GetSelectedPatch();
                        return index + 1;
                    };
                case "PATCHCOUNT":
                    return (RPMVesselComputer comp) =>
                    {
                        (int index, Orbit _) = GetLastPatch();
                        return index + 1;
                    };
                case "PATCHALTITUDE":
                    return (RPMVesselComputer comp) => GetSelectedPatchOrbit().referenceBody.GetAltitude(vessel.CoM);
                case "PATCHORBTSPEED":
                    return (RPMVesselComputer comp) => GetSelectedPatchOrbit().GetVel().magnitude;
                case "PATCHAPOAPSIS":
                    return (RPMVesselComputer comp) => JUtil.OrbitMakesSense(vessel) ? GetSelectedPatchOrbit().ApA : double.NaN;
                case "PATCHPERIAPSIS":
                    return (RPMVesselComputer comp) => JUtil.OrbitMakesSense(vessel) ? GetSelectedPatchOrbit().PeA : double.NaN;
                case "PATCHINCLINATION":
                    return (RPMVesselComputer comp) => JUtil.OrbitMakesSense(vessel) ? GetSelectedPatchOrbit().inclination : double.NaN;
                case "PATCHECCENTRICITY":
                    return (RPMVesselComputer comp) => JUtil.OrbitMakesSense(vessel) ? GetSelectedPatchOrbit().eccentricity : double.NaN;
                case "PATCHTIMETOAPSECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (!JUtil.OrbitMakesSense(vessel)) return double.NaN;
                        Orbit patch = GetSelectedPatchOrbit();
                        // When the tracking station is not upgraded, StartUT will not be updated to the current time.
                        if (patch.StartUT == 0.0) return patch.timeToAp;
                        return patch.timeToAp + patch.StartUT - Planetarium.GetUniversalTime();
                    };
                case "PATCHTIMETOPESECS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (!JUtil.OrbitMakesSense(vessel)) return double.NaN;
                        Orbit patch = GetSelectedPatchOrbit();
                        // When the tracking station is not upgraded, StartUT will not be updated to the current time.
                        if (patch.StartUT == 0.0) return patch.timeToPe;
                        return patch.timeToPe + patch.StartUT - Planetarium.GetUniversalTime();
                    };
                case "PATCHORBPERIODSECS":
                    return (RPMVesselComputer comp) => JUtil.OrbitMakesSense(vessel) ? GetSelectedPatchOrbit().period : double.NaN;
                case "PATCHTIMETOANEQUATORIAL":
                    return (RPMVesselComputer comp) =>
                    {
                        Orbit patch = GetSelectedPatchOrbit();
                        if (!JUtil.OrbitMakesSense(vessel) || !patch.AscendingNodeEquatorialExists())
                        {
                            return double.NaN;
                        }
                        return patch.TimeOfAscendingNodeEquatorial(Planetarium.GetUniversalTime()) - Planetarium.GetUniversalTime();
                    };
                case "PATCHTIMETODNEQUATORIAL":
                    return (RPMVesselComputer comp) =>
                    {
                        Orbit patch = GetSelectedPatchOrbit();
                        if (!JUtil.OrbitMakesSense(vessel) || !patch.DescendingNodeEquatorialExists())
                        {
                            return double.NaN;
                        }
                        return patch.TimeOfDescendingNodeEquatorial(Planetarium.GetUniversalTime()) - Planetarium.GetUniversalTime();
                    };
                case "PATCHFIRST":
                    return (RPMVesselComputer comp) =>
                    {
                        (int index, Orbit _) = GetSelectedPatch();
                        return index == 0 ? 1.0d : 0.0d;
                    };
                case "PATCHLAST":
                    return (RPMVesselComputer comp) =>
                    {
                        (int selected, Orbit _) = GetSelectedPatch();
                        (int last, Orbit _) = GetLastPatch();
                        return selected == last ? 1.0d : 0.0d;
                    };
                case "PATCHNEXTAPSISTYPE":
                    return (RPMVesselComputer comp) =>
                    {
                        Orbit patch = GetSelectedPatchOrbit();
                        if (patch.eccentricity < 1.0)
                        {
                            // Which one will we reach first?
                            return (patch.timeToPe < patch.timeToAp) ? -1.0 : 1.0;
                        }

                        // Ship is hyperbolic.  There is no Ap.  Have we already
                        // passed Pe?
                        return (patch.timeToPe > 0.0) ? -1.0 : 0.0;
                    };
                case "PATCHNEXT_ANDN_EQUATORIAL":
                    return (RPMVesselComputer comp) =>
                    {
                        Orbit patch = GetSelectedPatchOrbit();
                        double universalTime = Planetarium.GetUniversalTime();
                        if (!JUtil.OrbitMakesSense(vessel)) return 0.0;

                        double dnTime = patch.DescendingNodeEquatorialExists() ? patch.TimeOfDescendingNodeEquatorial(universalTime) : double.NaN;
                        double anTime = patch.AscendingNodeEquatorialExists() ? patch.TimeOfAscendingNodeEquatorial(universalTime) : double.NaN;

                        if (double.IsNaN(anTime)) return -1.0;
                        if (double.IsNaN(dnTime)) return 1.0;
                        return Math.Max(0.0, anTime) < Math.Max(dnTime, 0.0) ? 1.0 : -1.0;
                    };
                case "PATCHENCOUNTEREXISTS":
                    return (RPMVesselComputer comp) =>
                    {
                        if (!JUtil.OrbitMakesSense(vessel)) return 0.0;
                        Orbit patch = GetSelectedPatchOrbit();
                        switch (patch.patchEndTransition)
                        {
                            case Orbit.PatchTransitionType.ESCAPE:
                                return -1d;
                            case Orbit.PatchTransitionType.ENCOUNTER:
                                return 1d;
                            default:
                                return 0.0d;
                        }
                    };
                case "PATCHENCOUNTERTIME":
                    return (RPMVesselComputer comp) =>
                    {
                        if (!JUtil.OrbitMakesSense(vessel)) return 0.0;
                        Orbit patch = GetSelectedPatchOrbit();
                        if (patch.patchEndTransition == Orbit.PatchTransitionType.ENCOUNTER ||
                            patch.patchEndTransition == Orbit.PatchTransitionType.ESCAPE)
                        {
                            return patch.UTsoi - Planetarium.GetUniversalTime();
                        }
                        return 0.0;
                    };
            }

            return null;
        }
        
        #endregion

        #region eval helpers

        private static bool CrewListElementIsNumeric(string element)
        {
            switch (element)
            {

                case "PRESENT":
                case "EXISTS":
                case "STUPIDITY":
                case "COURAGE":
                case "BADASS":
                case "PANIC":
                case "WHEE":
                case "LEVEL":
                case "EXPERIENCE":
                    return true;
                default:
                    return false;
            }
        }

        private static double CrewListNumericElement(string element, int seatID, IList<ProtoCrewMember> crewList, IList<kerbalExpressionSystem> crewMedical)
        {
            bool exists = (crewList != null) && (seatID < crewList.Count);
            bool valid = exists && crewList[seatID] != null;
            switch (element)
            {
                case "PRESENT":
                    return valid ? 1d : -1d;
                case "EXISTS":
                    return exists ? 1d : -1d;
                case "STUPIDITY":
                    return valid ? crewList[seatID].stupidity : -1d;
                case "COURAGE":
                    return valid ? crewList[seatID].courage : -1d;
                case "BADASS":
                    return valid ? crewList[seatID].isBadass.GetHashCode() : -1d;
                case "PANIC":
                    return (valid && crewMedical[seatID] != null) ? crewMedical[seatID].panicLevel : -1d;
                case "WHEE":
                    return (valid && crewMedical[seatID] != null) ? crewMedical[seatID].wheeLevel : -1d;
                case "LEVEL":
                    return valid ? (float)crewList[seatID].experienceLevel : -1d;
                case "EXPERIENCE":
                    return valid ? crewList[seatID].experience : -1d;
                default:
                    return -1d;
            }
        }

        private static string CrewListElement(string element, int seatID, IList<ProtoCrewMember> crewList, IList<kerbalExpressionSystem> crewMedical)
        {
            bool exists = (crewList != null) && (seatID < crewList.Count);
            bool valid = exists && crewList[seatID] != null;
            switch (element)
            {
                case "FIRST":
                    return valid ? crewList[seatID].name.Split()[0] : string.Empty;
                case "LAST":
                    return valid ? crewList[seatID].name.Split()[1] : string.Empty;
                case "FULL":
                    return valid ? crewList[seatID].name : string.Empty;
                case "TITLE":
                    return valid ? crewList[seatID].experienceTrait.Title : string.Empty;
                default:
                    return "???!";
            }
        }

        /// <summary>
        /// According to C# specification, switch-case is compiled to a constant hash table.
        /// So this is actually more efficient than a dictionary, who'd have thought.
        /// </summary>
        /// <param name="situation"></param>
        /// <returns></returns>
        private static string SituationString(Vessel.Situations situation)
        {
            switch (situation)
            {
                case Vessel.Situations.FLYING:
                    return "Flying";
                case Vessel.Situations.SUB_ORBITAL:
                    return "Sub-orbital";
                case Vessel.Situations.ESCAPING:
                    return "Escaping";
                case Vessel.Situations.LANDED:
                    return "Landed";
                case Vessel.Situations.DOCKED:
                    return "Docked"; // When does this ever happen exactly, I wonder?
                case Vessel.Situations.PRELAUNCH:
                    return "Ready to launch";
                case Vessel.Situations.ORBITING:
                    return "Orbiting";
                case Vessel.Situations.SPLASHED:
                    return "Splashed down";
            }
            return "??!";
        }

        /// <summary>
        /// Returns a number identifying the next apsis type
        /// </summary>
        /// <returns></returns>
        private static double NextApsisType(Vessel vessel)
        {
            if (JUtil.OrbitMakesSense(vessel))
            {
                if (vessel.orbit.eccentricity < 1.0)
                {
                    // Which one will we reach first?
                    return (vessel.orbit.timeToPe < vessel.orbit.timeToAp) ? -1.0 : 1.0;
                } 	// Ship is hyperbolic.  There is no Ap.  Have we already
                // passed Pe?
                return (vessel.orbit.timeToPe > 0.0) ? -1.0 : 0.0;
            }

            return 0.0;
        }

        /// <summary>
        /// Originally from MechJeb
        /// Computes the time until the phase angle between the launchpad and the target equals the given angle.
        /// The convention used is that phase angle is the angle measured starting at the target and going east until
        /// you get to the launchpad. 
        /// The time returned will not be exactly accurate unless the target is in an exactly circular orbit. However,
        /// the time returned will go to exactly zero when the desired phase angle is reached.
        /// </summary>
        /// <param name="phaseAngle"></param>
        /// <param name="launchBody"></param>
        /// <param name="launchLongitude"></param>
        /// <param name="target"></param>
        /// <returns></returns>
        private static double TimeToPhaseAngle(double phaseAngle, CelestialBody launchBody, double launchLongitude, Orbit target)
        {
            double launchpadAngularRate = 360 / launchBody.rotationPeriod;
            double targetAngularRate = 360.0 / target.period;
            if (Vector3d.Dot(-target.GetOrbitNormal().SwizzleXZY().normalized, launchBody.angularVelocity) < 0) targetAngularRate *= -1; //retrograde target

            Vector3d currentLaunchpadDirection = launchBody.GetSurfaceNVector(0, launchLongitude);
            Vector3d currentTargetDirection = target.SwappedRelativePositionAtUT(Planetarium.GetUniversalTime());
            currentTargetDirection = Vector3d.Exclude(launchBody.angularVelocity, currentTargetDirection);

            double currentPhaseAngle = Math.Abs(Vector3d.Angle(currentLaunchpadDirection, currentTargetDirection));
            if (Vector3d.Dot(Vector3d.Cross(currentTargetDirection, currentLaunchpadDirection), launchBody.angularVelocity) < 0)
            {
                currentPhaseAngle = 360 - currentPhaseAngle;
            }

            double phaseAngleRate = launchpadAngularRate - targetAngularRate;

            double phaseAngleDifference = JUtil.ClampDegrees360(phaseAngle - currentPhaseAngle);

            if (phaseAngleRate < 0)
            {
                phaseAngleRate *= -1;
                phaseAngleDifference = 360 - phaseAngleDifference;
            }


            return phaseAngleDifference / phaseAngleRate;
        }

        /// <summary>
        /// Originally from MechJeb
        /// Computes the time required for the given launch location to rotate under the target orbital plane. 
        /// If the latitude is too high for the launch location to ever actually rotate under the target plane,
        /// returns the time of closest approach to the target plane.
        /// I have a wonderful proof of this formula which this comment is too short to contain.
        /// </summary>
        /// <param name="launchBody"></param>
        /// <param name="launchLatitude"></param>
        /// <param name="launchLongitude"></param>
        /// <param name="target"></param>
        /// <returns></returns>
        private static double TimeToPlane(CelestialBody launchBody, double launchLatitude, double launchLongitude, Orbit target)
        {
            double inc = Math.Abs(Vector3d.Angle(-target.GetOrbitNormal().SwizzleXZY().normalized, launchBody.angularVelocity));
            Vector3d b = Vector3d.Exclude(launchBody.angularVelocity, -target.GetOrbitNormal().SwizzleXZY().normalized).normalized; // I don't understand the sign here, but this seems to work
            b *= launchBody.Radius * Math.Sin(Math.PI / 180 * launchLatitude) / Math.Tan(Math.PI / 180 * inc);
            Vector3d c = Vector3d.Cross(-target.GetOrbitNormal().SwizzleXZY().normalized, launchBody.angularVelocity).normalized;
            double cMagnitudeSquared = Math.Pow(launchBody.Radius * Math.Cos(Math.PI / 180 * launchLatitude), 2) - b.sqrMagnitude;
            if (cMagnitudeSquared < 0) cMagnitudeSquared = 0;
            c *= Math.Sqrt(cMagnitudeSquared);
            Vector3d a1 = b + c;
            Vector3d a2 = b - c;

            Vector3d longitudeVector = launchBody.GetSurfaceNVector(0, launchLongitude);

            double angle1 = Math.Abs(Vector3d.Angle(longitudeVector, a1));
            if (Vector3d.Dot(Vector3d.Cross(longitudeVector, a1), launchBody.angularVelocity) < 0) angle1 = 360 - angle1;
            double angle2 = Math.Abs(Vector3d.Angle(longitudeVector, a2));
            if (Vector3d.Dot(Vector3d.Cross(longitudeVector, a2), launchBody.angularVelocity) < 0) angle2 = 360 - angle2;

            double angle = Math.Min(angle1, angle2);
            return (angle / 360) * launchBody.rotationPeriod;
        }
        #endregion

        #region delegation
        /// <summary>
        /// Get a plugin or internal method.
        /// </summary>
        /// <param name="packedMethod">The method to fetch in the format ModuleName:MethodName</param>
        /// <param name="internalProp">The internal prop that should be used to instantiate InternalModule plugin methods.</param>
        /// <param name="delegateType">The expected signature of the method.</param>
        /// <returns></returns>
        public Delegate GetMethod(string packedMethod, InternalProp internalProp, Type delegateType)
        {
            Delegate returnValue = GetInternalMethod(packedMethod, delegateType);
            if (returnValue == null && internalProp != null)
            {
                returnValue = JUtil.GetMethod(packedMethod, internalProp, delegateType);
            }

            return returnValue;
        }

        /// <summary>
        /// Creates a new PluginEvaluator object for the method supplied (if
        /// the method exists), attached to an IJSIModule.
        /// </summary>
        /// <param name="packedMethod"></param>
        /// <returns></returns>
        internal Delegate GetInternalMethod(string packedMethod)
        {
            string[] tokens = packedMethod.Split(':');
            if (tokens.Length != 2 || string.IsNullOrEmpty(tokens[0]) || string.IsNullOrEmpty(tokens[1]))
            {
                JUtil.LogErrorMessage(this, "Bad format on {0}", packedMethod);
                throw new ArgumentException("stateMethod incorrectly formatted");
            }

            // Backwards compatibility:
            if (tokens[0] == "MechJebRPMButtons")
            {
                tokens[0] = "JSIMechJeb";
            }
            else if (tokens[0] == "JSIGimbal")
            {
                tokens[0] = "JSIInternalRPMButtons";
            }
            IJSIModule jsiModule = null;
            foreach (IJSIModule module in installedModules)
            {
                if (module.GetType().Name == tokens[0])
                {
                    jsiModule = module;
                    break;
                }
            }

            //JUtil.LogMessage(this, "searching for {0} : {1}", tokens[0], tokens[1]);
            Delegate pluginEval = null;
            if (jsiModule != null)
            {
                foreach (MethodInfo m in jsiModule.GetType().GetMethods())
                {
                    if (m.Name == tokens[1])
                    {
                        //JUtil.LogMessage(this, "Found method {1}: return type is {0}, IsStatic is {2}, with {3} parameters", m.ReturnType, tokens[1],m.IsStatic, m.GetParameters().Length);
                        ParameterInfo[] parms = m.GetParameters();
                        if (parms.Length > 0)
                        {
                            JUtil.LogErrorMessage(this, "GetInternalMethod failed: {1} parameters in plugin method {0}", packedMethod, parms.Length);
                            return null;
                        }

                        if (m.ReturnType == typeof(bool))
                        {
                            try
                            {
                                pluginEval = (m.IsStatic) ? Delegate.CreateDelegate(typeof(Func<bool>), m) : Delegate.CreateDelegate(typeof(Func<bool>), jsiModule, m);
                            }
                            catch (Exception e)
                            {
                                JUtil.LogErrorMessage(this, "Failed creating a delegate for {0}: {1}", packedMethod, e);
                            }
                        }
                        else if (m.ReturnType == typeof(double))
                        {
                            try
                            {
                                pluginEval = (m.IsStatic) ? Delegate.CreateDelegate(typeof(Func<double>), m) : Delegate.CreateDelegate(typeof(Func<double>), jsiModule, m);
                            }
                            catch (Exception e)
                            {
                                JUtil.LogErrorMessage(this, "Failed creating a delegate for {0}: {1}", packedMethod, e);
                            }
                        }
                        else if (m.ReturnType == typeof(string))
                        {
                            try
                            {
                                pluginEval = (m.IsStatic) ? Delegate.CreateDelegate(typeof(Func<string>), m) : Delegate.CreateDelegate(typeof(Func<string>), jsiModule, m);
                            }
                            catch (Exception e)
                            {
                                JUtil.LogErrorMessage(this, "Failed creating a delegate for {0}: {1}", packedMethod, e);
                            }
                        }
                        else
                        {
                            JUtil.LogErrorMessage(this, "I need to support a return type of {0}", m.ReturnType);
                            throw new Exception("Not Implemented");
                        }
                    }
                }

                if (pluginEval == null)
                {
                    JUtil.LogErrorMessage(this, "I failed to find the method for {0}:{1}", tokens[0], tokens[1]);
                }
            }

            return pluginEval;
        }

        /// <summary>
        /// Get an internal method (one that is built into an IJSIModule)
        /// </summary>
        /// <param name="packedMethod"></param>
        /// <param name="delegateType"></param>
        /// <returns></returns>
        public Delegate GetInternalMethod(string packedMethod, Type delegateType)
        {
            string[] tokens = packedMethod.Split(':');
            if (tokens.Length != 2)
            {
                JUtil.LogErrorMessage(this, "Bad format on {0}", packedMethod);
                throw new ArgumentException("stateMethod incorrectly formatted");
            }

            // Backwards compatibility:
            if (tokens[0] == "MechJebRPMButtons")
            {
                tokens[0] = "JSIMechJeb";
            }
            IJSIModule jsiModule = null;
            foreach (IJSIModule module in installedModules)
            {
                if (module.GetType().Name == tokens[0])
                {
                    jsiModule = module;
                    break;
                }
            }

            Delegate stateCall = null;
            if (jsiModule != null)
            {
                var methodInfo = delegateType.GetMethod("Invoke");
                Type returnType = methodInfo.ReturnType;
                foreach (MethodInfo m in jsiModule.GetType().GetMethods())
                {
                    if (!string.IsNullOrEmpty(tokens[1]) && m.Name == tokens[1] && IsEquivalent(m, methodInfo))
                    {
                        if (m.IsStatic)
                        {
                            stateCall = Delegate.CreateDelegate(delegateType, m);
                        }
                        else
                        {
                            stateCall = Delegate.CreateDelegate(delegateType, jsiModule, m);
                        }
                    }
                }
            }

            return stateCall;
        }

        /// <summary>
        /// Returns whether two methods are effectively equal
        /// </summary>
        /// <param name="method1"></param>
        /// <param name="method2"></param>
        /// <returns></returns>
        private static bool IsEquivalent(MethodInfo method1, MethodInfo method2)
        {
            if (method1.ReturnType == method2.ReturnType)
            {
                var m1Parms = method1.GetParameters();
                var m2Parms = method2.GetParameters();
                if (m1Parms.Length == m2Parms.Length)
                {
                    for (int i = 0; i < m1Parms.Length; ++i)
                    {
                        if (m1Parms[i].GetType() != m2Parms[i].GetType())
                        {
                            return false;
                        }
                    }
                    return true;
                }
            }

            return false;
        }
        #endregion

        #region pluginevaluators
        private Func<double> evaluateTerminalVelocity;
        private bool evaluateTerminalVelocityReady;
        private Func<double> evaluateTimeToImpact;
        private bool evaluateTimeToImpactReady;

        private NumericVariableEvaluator AngleOfAttack()
        {
            Func<double> accessor = null;

            if (JSIFAR.farFound)
            {
                accessor = (Func<double>)GetInternalMethod("JSIFAR:GetAngleOfAttack", typeof(Func<double>));
                if (accessor != null)
                {
                    double value = accessor();
                    if (double.IsNaN(value))
                    {
                        accessor = null;
                    }
                }
            }

            if (accessor == null)
            {
                return (RPMVesselComputer comp) =>
                {
                    return comp.FallbackEvaluateAngleOfAttack();
                };
            }
            else
            {
                return (RPMVesselComputer comp) => { return accessor(); };
            }
        }

        private NumericVariableEvaluator DeltaV()
        {
            Func<double> accessor = null;

            accessor = (Func<double>)GetInternalMethod("JSIMechJeb:GetDeltaV", typeof(Func<double>));
            if (accessor != null)
            {
                double value = accessor();
                if (double.IsNaN(value))
                {
                    accessor = null;
                }
            }

            if (accessor == null)
            {
                return (RPMVesselComputer comp) =>
                {
                    // TODO: use the stock deltav calculator instead
                    if (comp?.vessel?.VesselDeltaV != null)
                    {
                        return comp.vessel.VesselDeltaV.TotalDeltaVActual;
                    }
                    return (comp.actualAverageIsp * RPMGlobals.gee) * Math.Log(comp.totalShipWetMass / (comp.totalShipWetMass - comp.resources.PropellantMass(false)));
                };
            }
            else
            {
                return (RPMVesselComputer comp) => { return accessor(); };
            }
        }

        private NumericVariableEvaluator DeltaVStage()
        {
            Func<double> accessor = null;

            accessor = (Func<double>)GetInternalMethod("JSIMechJeb:GetStageDeltaV", typeof(Func<double>));
            if (accessor != null)
            {
                double value = accessor();
                if (double.IsNaN(value))
                {
                    accessor = null;
                }
            }

            if (accessor == null)
            {
                return (RPMVesselComputer comp) =>
                {
                    var stageInfo = comp?.vessel?.VesselDeltaV?.GetStage(comp.vessel.currentStage);
                    if (stageInfo != null)
                    {
                        return stageInfo.deltaVActual;
                    }
                    return (comp.actualAverageIsp * RPMGlobals.gee) * Math.Log(comp.totalShipWetMass / (comp.totalShipWetMass - comp.resources.PropellantMass(true)));
                };
            }
            else
            {
                return (RPMVesselComputer comp) => { return accessor(); };
            }
        }

        private NumericVariableEvaluator DragAccel()
        {
            Func<double> accessor = null;

            if (JSIFAR.farFound)
            {
                accessor = (Func<double>)GetInternalMethod("JSIFAR:GetDragForce", typeof(Func<double>));
                if (accessor != null)
                {
                    double value = accessor();
                    if (double.IsNaN(value))
                    {
                        accessor = null;
                    }
                }
            }

            if (accessor == null)
            {
                return (RPMVesselComputer comp) =>
                {
                    return comp.FallbackEvaluateDragForce() / comp.totalShipWetMass;
                };
            }
            else
            {
                return (RPMVesselComputer comp) =>
                {
                    return accessor() / comp.totalShipWetMass;
                };
            }
        }

        private NumericVariableEvaluator DragForce()
        {
            Func<double> accessor = null;

            if (JSIFAR.farFound)
            {
                accessor = (Func<double>)GetInternalMethod("JSIFAR:GetDragForce", typeof(Func<double>));
                if (accessor != null)
                {
                    double value = accessor();
                    if (double.IsNaN(value))
                    {
                        accessor = null;
                    }
                }
            }

            if (accessor == null)
            {
                return (RPMVesselComputer comp) =>
                {
                    return comp.FallbackEvaluateDragForce();
                };
            }
            else
            {
                return (RPMVesselComputer comp) => { return accessor(); };
            }
        }

        private NumericVariableEvaluator DynamicPressure()
        {
            Func<double> accessor = null;

            if (JSIFAR.farFound)
            {
                accessor = (Func<double>)GetInternalMethod("JSIFAR:GetDynamicPressure", typeof(Func<double>));
                if (accessor != null)
                {
                    double value = accessor();
                    if (double.IsNaN(value))
                    {
                        accessor = null;
                    }
                }
            }

            if (accessor == null)
            {
                return (RPMVesselComputer comp) => { return vessel.dynamicPressurekPa; };
            }
            else
            {
                return (RPMVesselComputer comp) => { return accessor(); };
            }
        }

        private NumericVariableEvaluator LandingError()
        {
            Func<double> accessor = (Func<double>)GetInternalMethod("JSIMechJeb:GetLandingError", typeof(Func<double>));

            return (RPMVesselComputer comp) => { return accessor(); };
        }

        private NumericVariableEvaluator LandingAltitude()
        {
            Func<double> accessor = (Func<double>)GetInternalMethod("JSIMechJeb:GetLandingAltitude", typeof(Func<double>));

            return (RPMVesselComputer comp) =>
            {
                double est = accessor();
                return (est == 0.0) ? comp.estLandingAltitude : est;
            };
        }

        private NumericVariableEvaluator LandingLatitude()
        {
            Func<double> accessor = (Func<double>)GetInternalMethod("JSIMechJeb:GetLandingLatitude", typeof(Func<double>));

            return (RPMVesselComputer comp) =>
            {
                double est = accessor();
                return (est == 0.0) ? comp.estLandingLatitude : est;
            };
        }

        private NumericVariableEvaluator LandingLongitude()
        {
            Func<double> accessor = (Func<double>)GetInternalMethod("JSIMechJeb:GetLandingLongitude", typeof(Func<double>));

            return (RPMVesselComputer comp) =>
            {
                double est = accessor();
                return (est == 0.0) ? comp.estLandingLongitude : est;
            };
        }

        private NumericVariableEvaluator LiftAccel()
        {
            Func<double> accessor = null;

            if (JSIFAR.farFound)
            {
                accessor = (Func<double>)GetInternalMethod("JSIFAR:GetLiftForce", typeof(Func<double>));
                if (accessor != null)
                {
                    double value = accessor();
                    if (double.IsNaN(value))
                    {
                        accessor = null;
                    }
                }
            }

            if (accessor == null)
            {
                return (RPMVesselComputer comp) =>
                {
                    return comp.FallbackEvaluateLiftForce() / comp.totalShipWetMass;
                };
            }
            else
            {
                return (RPMVesselComputer comp) =>
                {
                    return accessor() / comp.totalShipWetMass;
                };
            }
        }

        private NumericVariableEvaluator LiftForce()
        {
            Func<double> accessor = null;

            if (JSIFAR.farFound)
            {
                accessor = (Func<double>)GetInternalMethod("JSIFAR:GetLiftForce", typeof(Func<double>));
                if (accessor != null)
                {
                    double value = accessor();
                    if (double.IsNaN(value))
                    {
                        accessor = null;
                    }
                }
            }

            if (accessor == null)
            {
                return (RPMVesselComputer comp) =>
                {
                    return comp.FallbackEvaluateLiftForce();
                };
            }
            else
            {
                return (RPMVesselComputer comp) => { return accessor(); };
            }
        }

        private NumericVariableEvaluator MechJebAvailable()
        {
            Func<bool> accessor = null;

            accessor = (Func<bool>)GetInternalMethod("JSIMechJeb:GetMechJebAvailable", typeof(Func<bool>));

            if (accessor == null)
            {
                return (RPMVesselComputer comp) => { return 0; };
            }
            else
            {
                return (RPMVesselComputer comp) => { return accessor().GetHashCode(); };
            }
        }

        private NumericVariableEvaluator SideSlip()
        {
            Func<double> accessor = null;

            if (JSIFAR.farFound)
            {
                accessor = (Func<double>)GetInternalMethod("JSIFAR:GetSideSlip", typeof(Func<double>));
                if (accessor != null)
                {
                    double value = accessor();
                    if (double.IsNaN(value))
                    {
                        accessor = null;
                    }
                }
            }

            if (accessor == null)
            {
                return (RPMVesselComputer comp) =>
                {
                    return comp.FallbackEvaluateSideSlip();
                };
            }
            else
            {
                return (RPMVesselComputer comp) => { return accessor(); };
            }
        }

        internal double TerminalVelocity(RPMVesselComputer comp)
        {
            if (evaluateTerminalVelocityReady == false)
            {
                Func<double> accessor = null;

                if (JSIFAR.farFound)
                {
                    accessor = (Func<double>)GetInternalMethod("JSIFAR:GetTerminalVelocity", typeof(Func<double>));
                    if (accessor != null)
                    {
                        double value = accessor();
                        if (value < 0.0)
                        {
                            accessor = null;
                        }
                    }
                }

                if (accessor == null)
                {
                    accessor = (Func<double>)GetInternalMethod("JSIMechJeb:GetTerminalVelocity", typeof(Func<double>));
                    
                    if (accessor != null && double.IsNaN(accessor()))
                    {
                        accessor = null;
                    }
                }

                evaluateTerminalVelocity = accessor;
                evaluateTerminalVelocityReady = true;
            }

            if (evaluateTerminalVelocity == null)
            {
                return comp.FallbackEvaluateTerminalVelocity();
            }
            else
            {
                return evaluateTerminalVelocity();
            }
        }

        private double TimeToImpact(RPMVesselComputer comp)
        {
            double timeToImpact = double.NaN;

            if (evaluateTimeToImpactReady == false)
            {
                Func<double> accessor = (Func<double>)GetInternalMethod("JSIMechJeb:GetLandingTime", typeof(Func<double>));

                if (accessor != null && double.IsNaN(accessor()))
                {
                    accessor = null;
                }

                evaluateTimeToImpact = accessor;
                evaluateTimeToImpactReady = true;
            }

            if (evaluateTimeToImpact != null)
            {
                timeToImpact = evaluateTimeToImpact();
            }
            else
            {
                timeToImpact = comp.FallbackEvaluateTimeToImpact();
            }

            if (double.IsNaN(timeToImpact) || timeToImpact > 365.0 * 24.0 * 60.0 * 60.0 || timeToImpact < 0.0)
            {
                timeToImpact = -1.0;
            }
            else if (timeToImpact == 0.0)
            {
                return comp.estLandingUT - Planetarium.GetUniversalTime();
            }
            return timeToImpact;
        }
        #endregion
    }
}
