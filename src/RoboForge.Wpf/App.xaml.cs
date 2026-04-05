using System.Windows;
using Microsoft.Extensions.DependencyInjection;
using RoboForge.Wpf.ViewModels;

namespace RoboForge.Wpf
{
    public partial class App : Application
    {
        public static IServiceProvider ServiceProvider { get; private set; }

        private void OnStartup(object sender, StartupEventArgs e)
        {
            var services = new ServiceCollection();
            
            // Register ViewModels
            services.AddSingleton<MainViewModel>();
            services.AddSingleton<WorkspaceViewModel>();
            services.AddSingleton<ProgramTreeViewModel>();
            services.AddSingleton<BlockEditorViewModel>();
            services.AddSingleton<CodeEditorViewModel>();
            services.AddSingleton<Robot3DViewModel>();
            services.AddSingleton<JogControlViewModel>();
            services.AddSingleton<PropertiesPanelViewModel>();
            services.AddSingleton<ConsoleViewModel>();
            services.AddSingleton<SettingsViewModel>();
            services.AddSingleton<ProjectManagerViewModel>();

            ServiceProvider = services.BuildServiceProvider();

            var vm = ServiceProvider.GetRequiredService<MainViewModel>();
            var mainWindow = new MainWindow {
                DataContext = vm
            };
            mainWindow.Loaded += (_, _) => vm.Initialize();
            mainWindow.Show();
        }
    }
}
