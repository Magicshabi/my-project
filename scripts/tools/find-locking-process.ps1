param([Parameter(Mandatory=$true)][string]$Path)

$source = @'
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

public static class RestartManagerLocks
{
    const int ERROR_MORE_DATA = 234;

    [StructLayout(LayoutKind.Sequential)]
    struct RM_UNIQUE_PROCESS
    {
        public int dwProcessId;
        public System.Runtime.InteropServices.ComTypes.FILETIME ProcessStartTime;
    }

    enum RM_APP_TYPE
    {
        RmUnknownApp = 0, RmMainWindow = 1, RmOtherWindow = 2,
        RmService = 3, RmExplorer = 4, RmConsole = 5,
        RmCritical = 1000
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    struct RM_PROCESS_INFO
    {
        public RM_UNIQUE_PROCESS Process;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
        public string strAppName;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
        public string strServiceShortName;
        public RM_APP_TYPE ApplicationType;
        public uint AppStatus;
        public uint TSSessionId;
        [MarshalAs(UnmanagedType.Bool)]
        public bool bRestartable;
    }

    [DllImport("rstrtmgr.dll", CharSet = CharSet.Unicode)]
    static extern int RmStartSession(out uint handle, int flags, StringBuilder key);
    [DllImport("rstrtmgr.dll")]
    static extern int RmEndSession(uint handle);
    [DllImport("rstrtmgr.dll", CharSet = CharSet.Unicode)]
    static extern int RmRegisterResources(uint handle, uint fileCount,
        string[] fileNames, uint appCount, IntPtr applications,
        uint serviceCount, string[] serviceNames);
    [DllImport("rstrtmgr.dll")]
    static extern int RmGetList(uint handle, out uint needed, ref uint count,
        [In, Out] RM_PROCESS_INFO[] affectedApps, ref uint rebootReasons);

    public static string[] Find(string path)
    {
        uint handle;
        var key = new StringBuilder(32);
        int result = RmStartSession(out handle, 0, key);
        if (result != 0) throw new Exception("RmStartSession=" + result);
        try
        {
            result = RmRegisterResources(handle, 1, new[] { path }, 0,
                IntPtr.Zero, 0, null);
            if (result != 0) throw new Exception("RmRegisterResources=" + result);
            uint needed = 0, count = 0, reasons = 0;
            result = RmGetList(handle, out needed, ref count, null, ref reasons);
            if (result == 0) return new string[0];
            if (result != ERROR_MORE_DATA) throw new Exception("RmGetList(size)=" + result);
            var infos = new RM_PROCESS_INFO[needed];
            count = needed;
            result = RmGetList(handle, out needed, ref count, infos, ref reasons);
            if (result != 0) throw new Exception("RmGetList(data)=" + result);
            var output = new List<string>();
            for (int i = 0; i < count; i++)
            {
                int pid = infos[i].Process.dwProcessId;
                string processName = "<exited>";
                try { processName = Process.GetProcessById(pid).ProcessName; } catch { }
                output.Add(pid + "\t" + processName + "\t" + infos[i].strAppName + "\t" + infos[i].strServiceShortName);
            }
            return output.ToArray();
        }
        finally { RmEndSession(handle); }
    }
}
'@

Add-Type -TypeDefinition $source
[RestartManagerLocks]::Find($Path)
