param([Parameter(Mandatory = $true)][string]$Path)

$source = @"
using System;
using System.Runtime.InteropServices;

public static class RestartManagerLocks
{
    const int CCH_RM_SESSION_KEY = 32;
    const int CCH_RM_MAX_APP_NAME = 255;
    const int CCH_RM_MAX_SVC_NAME = 63;
    const int ERROR_MORE_DATA = 234;

    [StructLayout(LayoutKind.Sequential)]
    struct RM_UNIQUE_PROCESS
    {
        public int dwProcessId;
        public System.Runtime.InteropServices.ComTypes.FILETIME ProcessStartTime;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    struct RM_PROCESS_INFO
    {
        public RM_UNIQUE_PROCESS Process;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = CCH_RM_MAX_APP_NAME + 1)]
        public string strAppName;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = CCH_RM_MAX_SVC_NAME + 1)]
        public string strServiceShortName;
        public uint ApplicationType;
        public uint AppStatus;
        public uint TSSessionId;
        [MarshalAs(UnmanagedType.Bool)]
        public bool bRestartable;
    }

    [DllImport("rstrtmgr.dll", CharSet = CharSet.Unicode)]
    static extern int RmStartSession(out uint handle, int flags, string key);

    [DllImport("rstrtmgr.dll")]
    static extern int RmEndSession(uint handle);

    [DllImport("rstrtmgr.dll", CharSet = CharSet.Unicode)]
    static extern int RmRegisterResources(uint handle, uint fileCount, string[] files,
        uint applicationCount, IntPtr applications, uint serviceCount, string[] services);

    [DllImport("rstrtmgr.dll")]
    static extern int RmGetList(uint handle, out uint needed, ref uint count,
        [In, Out] RM_PROCESS_INFO[] affectedApps, ref uint rebootReasons);

    public static string[] Find(string path)
    {
        uint handle;
        string key = Guid.NewGuid().ToString("N").Substring(0, CCH_RM_SESSION_KEY);
        int result = RmStartSession(out handle, 0, key);
        if (result != 0) throw new InvalidOperationException("RmStartSession: " + result);
        try
        {
            result = RmRegisterResources(handle, 1, new[] { path }, 0, IntPtr.Zero, 0, null);
            if (result != 0) throw new InvalidOperationException("RmRegisterResources: " + result);
            uint needed = 0, count = 0, reasons = 0;
            result = RmGetList(handle, out needed, ref count, null, ref reasons);
            if (result == 0) return new string[0];
            if (result != ERROR_MORE_DATA) throw new InvalidOperationException("RmGetList: " + result);
            var info = new RM_PROCESS_INFO[needed];
            count = needed;
            result = RmGetList(handle, out needed, ref count, info, ref reasons);
            if (result != 0) throw new InvalidOperationException("RmGetList(2): " + result);
            var output = new string[count];
            for (int i = 0; i < count; ++i)
                output[i] = info[i].Process.dwProcessId + "\t" + info[i].strAppName + "\t" + info[i].strServiceShortName;
            return output;
        }
        finally { RmEndSession(handle); }
    }
}
"@

Add-Type -TypeDefinition $source
[RestartManagerLocks]::Find((Resolve-Path -LiteralPath $Path).Path)
