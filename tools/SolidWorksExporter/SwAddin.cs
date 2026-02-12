using System;
using System.Runtime.InteropServices;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swpublished;
using SolidWorks.Interop.swconst;

namespace SolidWorksURDFExporter
{
    [Guid("D1037E90-1234-4567-89AB-CDEF01234567"), ComVisible(true)]
    [SwAddin(
        Description = "Export SolidWorks Assembly to ROS URDF",
        Title = "SW2URDF Exporter",
        LoadAtStartup = true
        )]
    public class SwAddin : ISwAddin
    {
        private ISldWorks iSwApp;
        private ICommandManager iCmdMgr;
        private int addinID;

        public bool ConnectToSW(object ThisSW, int Cookie)
        {
            iSwApp = (ISldWorks)ThisSW;
            addinID = Cookie;

            // Setup callbacks
            iSwApp.SetAddinCallbackInfo(0, this, addinID);

            // Register Command Manager
            iCmdMgr = iSwApp.GetCommandManager(Cookie);
            AddCommandManager();

            return true;
        }

        public bool DisconnectFromSW()
        {
            RemoveCommandManager();
            System.Runtime.InteropServices.Marshal.ReleaseComObject(iCmdMgr);
            iCmdMgr = null;
            System.Runtime.InteropServices.Marshal.ReleaseComObject(iSwApp);
            iSwApp = null;
            // The addin _must_ call GC.Collect() to gracefully disconnect
            GC.Collect();
            return true;
        }

        private void AddCommandManager()
        {
            // Implementation of menu setup
            // Calls ExportCommand.Export()
        }

        private void RemoveCommandManager()
        {
            // Implementation of menu removal
        }
    }
}
