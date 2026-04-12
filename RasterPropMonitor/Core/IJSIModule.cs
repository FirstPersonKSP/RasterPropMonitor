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
using Log = KSPCommunityLib.Logging.Log;

namespace JSI
{
    /// <summary>
    /// This class exists to provide a base class that RasterPropMonitorComputer
    /// manages for tracking various built-in plugin action handlers.
    /// </summary>
    public class IJSIModule
    {
        public IJSIModule(Vessel v)
        {
            this.vessel = v;
        }

        public Vessel vessel;

        #region Module Registration

        static List<Type> x_registeredTypes = new List<Type>();

        internal static void CreateJSIModules(List<IJSIModule> modules, Vessel v)
        {
            object[] constructorArgs = new[] { v };
            foreach (Type t in x_registeredTypes)
            {
                try
                {
                    modules.Add((IJSIModule)Activator.CreateInstance(t, constructorArgs));
                }
                catch (Exception e)
                {
                    Log.Error("Error creating JSI module of type " + t.Name + ": ");
                    Log.Exception(e);
                }
            }
        }

        public static void RegisterModule(Type jsiModuleType)
        {
            if (x_registeredTypes.IndexOf(jsiModuleType) != -1) return;
            if (!typeof(IJSIModule).IsAssignableFrom(jsiModuleType))
            {
                Log.Error($"Tried to register an ISJIModuleType {jsiModuleType.Name} that does not inherit from IJSIModule");
                return;
            }
            x_registeredTypes.Add(jsiModuleType);
        }

        // A place to register known modules that might not otherwise have their static constructors called
        static IJSIModule()
        {
            RegisterModule(typeof(JSIParachute));
            RegisterModule(typeof(JSIInternalRPMButtons));
            if (JSIChatterer.chattererFound) RegisterModule(typeof(JSIChatterer));
            if (JSIFAR.farFound) RegisterModule(typeof(JSIFAR));
            if (JSIKAC.kacFound) RegisterModule(typeof(JSIKAC));
            if (JSIPilotAssistant.paFound) RegisterModule(typeof(JSIPilotAssistant));
        }

        #endregion
    }
}
