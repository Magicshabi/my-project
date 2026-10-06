param([Parameter(Mandatory = $true)][string]$Path)

$source = @'
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Runtime.InteropServices.ComTypes;

public static class RestartManager {
    private const int ERROR_MORE_DATA = 234;
    private const int CCH_RM_SESSION_KEY = 32;
    private const int CCH_RM_MAX_APP_NAME = 255;
    private const int CCH_RM_MAX_SVC_NAME = 63;

    [StructLayout(LayoutKind.Sequential)]
    private struct RM_UNIQUE_PROCESS {
        public int dwProcessId;
        public FILETIME ProcessStartTime;
    }

    private enum RM_APP_TYPE {
        RmUnknownApp = 0,
        RmMainWindow = 1,
        RmOtherWindow = 2,
        RmService = 3,
        RmExplorer = 4,
        RmConsole = 5,
        RmCritical = 1000
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    private struct RM_PROCESS_INFO {
        public RM_UNIQUE_PROCESS Process;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = CCH_RM_MAX_APP_NAME + 1)]
        public string strAppName;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = CCH_RM_MAX_SVC_NAME + 1)]
        public string strServiceShortName;
        public RM_APP_TYPE ApplicationType;
        public uint AppStatus;
        public uint TSSessionId;
        [MarshalAs(UnmanagedType.Bool)]
        public bool bRestartable;
    }

    [DllImport("rstrtmgr.dll", CharSet = CharSet.Unicode)]
    private static extern int RmStartSession(out uint sessionHandle, int sessionFlags, string sessionKey);

    [DllImport("rstrtmgr.dll", CharSet = CharSet.Unicode)]
    private static extern int RmRegisterResources(uint sessionHandle, uint fileCount, string[] fileNames,
        uint applicationCount, IntPtr applications, uint serviceCount, string[] serviceNames);

    [DllImport("rstrtmgr.dll")]
    private static extern int RmGetList(uint sessionHandle, out uint processInfoNeeded,
        ref uint processInfoCount, [In, Out] RM_PROCESS_INFO[] processInfo, ref uint rebootReasons);

    [DllImport("rstrtmgr.dll")]
    private static extern int RmEndSession(uint sessionHandle);

    public static string[] GetLockers(string path) {
        uint handle;
        string key = Guid.NewGuid().ToString("N").Substring(0, CCH_RM_SESSION_KEY);
        int result = RmStartSession(out handle, 0, key);
        if (result != 0) throw new Exception("RmStartSession failed: " + result);
        try {
            string[] resources = new[] { path };
            result = RmRegisterResources(handle, 1, resources, 0, IntPtr.Zero, 0, null);
            if (result != 0) throw new Exception("RmRegisterResources failed: " + result);

            uint needed = 0;
            uint count = 0;
            uint reasons = 0;
            result = RmGetList(handle, out needed, ref count, null, ref reasons);
            if (result == 0) return new string[0];
            if (result != ERROR_MORE_DATA) throw new Exception("RmGetList(size) failed: " + result);

            var info = new RM_PROCESS_INFO[needed];
            count = needed;
            result = RmGetList(handle, out needed, ref count, info, ref reasons);
            if (result != 0) throw new Exception("RmGetList(data) failed: " + result);

            var output = new List<string>();
            for (int i = 0; i < count; i++) {
                output.Add(info[i].Process.dwProcessId + "|" + info[i].strAppName + "|" + info[i].ApplicationType);
            }
            return output.ToArray();
        }
        finally {
            RmEndSession(handle);
        }
    }
}
'@

Add-Type -TypeDefinition $source
[RestartManager]::GetLockers((Resolve-Path -LiteralPath $Path).Path)
