using System.Windows;
using TeachPendant_WPF.Models;

namespace TeachPendant_WPF
{
    public partial class App : Application
    {
        protected override void OnStartup(StartupEventArgs e)
        {
            base.OnStartup(e);

            // Ensure the SQLite database is created in local AppData
            using (var db = new AppDbContext())
            {
                db.Database.EnsureCreated();
                System.Diagnostics.Debug.WriteLine("Database Initialized.");
            }
        }
    }
}
