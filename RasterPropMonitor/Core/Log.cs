using System;
using System.Reflection;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace KSPBuildTools
{
    interface ILogContextProvider {
        string context();
    }
	internal static class Log
	{
		internal static string ModPrefix = "[" + Assembly.GetExecutingAssembly().GetName().Name + "]";

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private static string ObjPrefix(this UnityEngine.Object obj)
		{
			return "[" + obj.name + "]";
		}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static string ObjPrefix(this ILogContextProvider obj)
        {
            return "[" + obj.context() + "]";
        }

		[System.Diagnostics.Conditional("DEBUG")]
		internal static void Debug(string message, string prefix = "") {
            UnityEngine.Debug.Log("[DEBUG]" + ModPrefix + prefix + " " + message);
        }

		[System.Diagnostics.Conditional("DEBUG")]
		internal static void LogDebug(this UnityEngine.Object obj, string message, string prefix = "") {
            UnityEngine.Debug.Log( "[DEBUG]" + ModPrefix + obj.ObjPrefix() + prefix +  " " + message, obj);
        }

        [System.Diagnostics.Conditional("DEBUG")]
        internal static void LogDebug(this ILogContextProvider obj, string message, string prefix = "") {
            UnityEngine.Debug.Log("[DEBUG]" + ModPrefix + obj.ObjPrefix() + prefix +  " " + message);
        }

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static void Message(string message, string prefix = "")
		{
			UnityEngine.Debug.Log(ModPrefix + prefix + " " + message);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void LogMessage(this UnityEngine.Object obj, string message, string prefix = "")
		{
            UnityEngine.Debug.Log(ModPrefix + obj.ObjPrefix() + prefix + " " + message, obj);
		}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void LogMessage(this ILogContextProvider obj, string message, string prefix = "")
        {
            UnityEngine.Debug.Log(ModPrefix + obj.ObjPrefix() + prefix + " " + message);
        }

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static void Warning(string message, string prefix = "")
		{
			UnityEngine.Debug.LogWarning(ModPrefix + prefix + " " + message);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static void LogWarning(this UnityEngine.Object obj, string message, string prefix = "")
		{
            UnityEngine.Debug.LogWarning(ModPrefix + obj.ObjPrefix() + prefix + " " + message, obj);
		}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void LogWarning(this ILogContextProvider obj, string message, string prefix = "")
        {
            UnityEngine.Debug.LogWarning(ModPrefix + obj.ObjPrefix() + prefix + " " + message);
        }

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static void Error(string message, string prefix = "")
		{
			UnityEngine.Debug.LogError(ModPrefix + prefix + " " + message);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static void LogError(UnityEngine.Object obj, string message, string prefix = "")
		{
			UnityEngine.Debug.LogError(ModPrefix + obj.ObjPrefix() + prefix + " " + message, obj);
		}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void LogError(this ILogContextProvider obj, string message, string prefix = "")
        {
            UnityEngine.Debug.LogError(ModPrefix + obj.ObjPrefix() + prefix + " " + message);
        }

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static void Exception(Exception ex)
		{
			UnityEngine.Debug.LogException(ex);
		}
	}
}