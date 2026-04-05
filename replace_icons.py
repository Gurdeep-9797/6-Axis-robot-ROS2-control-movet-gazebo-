import re
import os

path = r"d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\RoboForge_WPF\Views\MainWindow.xaml"
with open(path, 'r', encoding='utf-8') as f:
    txt = f.read()

replacements = {
    "📄": "Icon.New",
    "📂": "Icon.Open",
    "💾": "Icon.Save",
    "📤": "Icon.Export",
    
    "↩": "Icon.Undo",
    "↪": "Icon.Redo",
    "✂": "Icon.Cut",
    "📋": "Icon.Copy",
    "📌": "Icon.Paste",
    
    "🔨": "Icon.Compile",
    "▶": "Icon.Run",
    "⏸": "Icon.Pause",
    "⏹": "Icon.Stop",
    "⏭": "Icon.Step",
    
    # Sidebar
    "📜": "Icon.Program",
    "🧩": "Icon.Blocks",
    "🎯": "Icon.Motion",
    "⚡": "Icon.IO",
    "🔍": "Icon.Debug",
    "🤖": "Icon.Config",
    "⚙": "Icon.Settings",
    "📁": "Icon.Project",

    # Pipeline
    "💻": "Icon.Pipeline",
    "🌍": "Icon.Robot",
    "💾": "Icon.Database", # Used 'save' above, but ok
}

for emoji, resource in replacements.items():
    if "Sidebar" in resource or resource in ["Icon.Program", "Icon.Blocks", "Icon.Motion", "Icon.IO", "Icon.Debug", "Icon.Config", "Icon.Settings", "Icon.Project"]:
        # Sidebar formatting
        txt = re.sub(
            f'<TextBlock Text="{emoji}" FontSize="18"/>',
            f'<Image Source="{{StaticResource {resource}}}" Width="24" Height="24" RenderOptions.BitmapScalingMode="HighQuality"/>',
            txt
        )
    else:
        # Ribbon formatting
        txt = re.sub(
            f'<TextBlock Text="{emoji}" FontSize="18"(.*?)\/>',
            f'<Image Source="{{StaticResource {resource}}}" Width="20" Height="20" \\1 RenderOptions.BitmapScalingMode="HighQuality"/>',
            txt
        )

# Fix background borders to use AcrylicPanel
txt = txt.replace('Style="{StaticResource PanelHeader}"', 'Style="{StaticResource PanelHeader}"') # header stays same
txt = re.sub(r'<Border[^>]*Background="\{StaticResource Bg.Panel\}"[^>]*BorderBrush="\{StaticResource Border.Default\}"[^>]*>', '<Border Style="{StaticResource AcrylicPanel}">', txt)

# Fix Pipeline status colors
txt = txt.replace('Foreground="{StaticResource Text.Ok}"', 'Foreground="{StaticResource Status.Ok}"')
txt = txt.replace('Foreground="{StaticResource Text.Warning}"', 'Foreground="{StaticResource Status.Warning}"')

with open(path, 'w', encoding='utf-8') as f:
    f.write(txt)

print("Replacement complete.")
