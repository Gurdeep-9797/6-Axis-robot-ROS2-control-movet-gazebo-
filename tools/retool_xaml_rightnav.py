import re

filepath = r"d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\TeachPendant_WPF\Views\MainWindow.xaml"
with open(filepath, 'r', encoding='utf-8') as f:
    xaml = f.read()

# Replace column definitions
old_defs = """            <Grid.ColumnDefinitions>
                <ColumnDefinition Width="64"/> <!-- Col 0: Left Nav -->
                <ColumnDefinition Width="220"/> <!-- Col 1: Palette -->
                <ColumnDefinition Width="5"/>   <!-- Col 2: Splitter -->
                <ColumnDefinition Width="350" MinWidth="250"/> <!-- Col 3: WorkTree -->
                <ColumnDefinition Width="5"/>   <!-- Col 4: Splitter -->
                <ColumnDefinition Width="*" />   <!-- Col 5: Right Master Panel -->
            </Grid.ColumnDefinitions>"""

new_defs = """            <Grid.ColumnDefinitions>
                <ColumnDefinition Width="220"/> <!-- Col 0: Palette -->
                <ColumnDefinition Width="5"/>   <!-- Col 1: Splitter -->
                <ColumnDefinition Width="350" MinWidth="250"/> <!-- Col 2: WorkTree -->
                <ColumnDefinition Width="5"/>   <!-- Col 3: Splitter -->
                <ColumnDefinition Width="*"/>   <!-- Col 4: Master Panel -->
                <ColumnDefinition Width="64"/>  <!-- Col 5: Right Nav -->
            </Grid.ColumnDefinitions>"""

xaml = xaml.replace(
    '<ColumnDefinition Width="64"/> <!-- Col 0: Left Nav -->\n                <ColumnDefinition Width="220"/> <!-- Col 1: Palette -->\n                <ColumnDefinition Width="5"/>   <!-- Col 2: Splitter -->\n                <ColumnDefinition Width="350" MinWidth="250"/> <!-- Col 3: WorkTree -->\n                <ColumnDefinition Width="5"/>   <!-- Col 4: Splitter -->\n                <ColumnDefinition Width="*"/>   <!-- Col 5: Right Master Panel -->',
    '<ColumnDefinition Width="220"/> <!-- Col 0: Palette -->\n                <ColumnDefinition Width="5"/>   <!-- Col 1: Splitter -->\n                <ColumnDefinition Width="350" MinWidth="250"/> <!-- Col 2: WorkTree -->\n                <ColumnDefinition Width="5"/>   <!-- Col 3: Splitter -->\n                <ColumnDefinition Width="*"/>   <!-- Col 4: Master Panel -->\n                <ColumnDefinition Width="64"/>  <!-- Col 5: Right Nav -->'
)

# Reassign Grid columns for children of Grid.Row="1"
xaml = xaml.replace('<!-- COL 0: Vertical Navigation -->\n            <Border Grid.Column="0" Background="{StaticResource Bg.Deepest}" BorderBrush="{StaticResource Border.Default}" BorderThickness="0,0,1,0">',
                    '<!-- COL 5: Vertical Navigation (Moved to Right) -->\n            <Border Grid.Column="5" Background="{StaticResource Bg.Deepest}" BorderBrush="{StaticResource Border.Default}" BorderThickness="1,0,0,0">')

xaml = xaml.replace('<!-- COL 1: Command Toolbox -->\n            <Border Grid.Column="1"',
                    '<!-- COL 0: Command Toolbox -->\n            <Border Grid.Column="0"')

xaml = xaml.replace('<!-- SPLITTER 1 -->\n            <GridSplitter Grid.Column="2"',
                    '<!-- SPLITTER 1 -->\n            <GridSplitter Grid.Column="1"')

xaml = xaml.replace('<!-- COL 3: Logic WorkTree -->\n            <Border Grid.Column="3"',
                    '<!-- COL 2: Logic WorkTree -->\n            <Border Grid.Column="2"')

xaml = xaml.replace('<!-- SPLITTER 2 -->\n            <GridSplitter Grid.Column="4"',
                    '<!-- SPLITTER 2 -->\n            <GridSplitter Grid.Column="3"')

xaml = xaml.replace('<!-- COL 5: Right Master Panel (Viewer & Jog Tabs) -->\n            <Border Grid.Column="5"',
                    '<!-- COL 4: Master Panel (Viewer & Jog Tabs) -->\n            <Border Grid.Column="4"')

# Fix rounded corner styling on tabs or blocks to look more professional
xaml = xaml.replace('CornerRadius="0"', 'CornerRadius="4"')

with open(filepath, 'w', encoding='utf-8') as f:
    f.write(xaml)

print("XAML columns successfully swapped to Right-side Navigation.")
