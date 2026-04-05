import re
import os

filepath = r"d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\TeachPendant_WPF\Views\MainWindow.xaml"
with open(filepath, 'r', encoding='utf-8') as f:
    xaml = f.read()

# 1. Isolate the Logic Tree Component (the entire TreeView tree xml)
tree_pattern = re.compile(r'(<TreeView Grid\.Row="1" ItemsSource="\{Binding WorkTree\.RootNodes\}".*?<\/TreeView>)', re.DOTALL)
tree_match = tree_pattern.search(xaml)
if not tree_match:
    print("Error: Could not find Logic Tree")
    exit(1)
tree_xml = tree_match.group(1)

# 2. Isolate the 3D Workspace
workspace_pattern = re.compile(r'(<hx:HelixViewport3D.*?<\/hx:HelixViewport3D>)', re.DOTALL)
workspace_match = workspace_pattern.search(xaml)
workspace_xml = workspace_match.group(1)

# 3. Isolate the Viewport Overlays (TCP Readout and Active Line)
overlay1_pattern = re.compile(r'(<!-- Viewport overlay: TCP readout -->.*?<\/Border>)', re.DOTALL)
overlay1_xml = overlay1_pattern.search(xaml).group(1)

overlay2_pattern = re.compile(r'(<!-- Viewport overlay: Active Line -->.*?<\/Border>)', re.DOTALL)
overlay2_xml = overlay2_pattern.search(xaml).group(1)

# 4. Isolate the Machine Control panel contents (the StackPanel with sliders and Cartesian inputs)
# Look for <StackPanel Margin="16"> ... </StackPanel> inside the machine control scrollviewer
machine_pattern = re.compile(r'(<ScrollViewer VerticalScrollBarVisibility="Auto">\s*<StackPanel Margin="16">.*?<\/StackPanel>\s*<\/ScrollViewer>)', re.DOTALL)
machine_xml = machine_pattern.search(xaml).group(1)

# Build the New Grid
new_grid = f"""        <!-- ═══════════════════════════════════════════════════════════ -->
        <!-- 4-PANE FR-HMI TABLET EXPLORER                               -->
        <!-- ═══════════════════════════════════════════════════════════ -->
        <Grid Grid.Row="1">
            <Grid.ColumnDefinitions>
                <ColumnDefinition Width="64"/> <!-- Col 0: Left Nav -->
                <ColumnDefinition Width="220"/> <!-- Col 1: Palette -->
                <ColumnDefinition Width="5"/>   <!-- Col 2: Splitter -->
                <ColumnDefinition Width="350" MinWidth="250"/> <!-- Col 3: WorkTree -->
                <ColumnDefinition Width="5"/>   <!-- Col 4: Splitter -->
                <ColumnDefinition Width="*"/>   <!-- Col 5: Right Master Panel -->
            </Grid.ColumnDefinitions>

            <!-- COL 0: Vertical Navigation -->
            <Border Grid.Column="0" Background="{{StaticResource Bg.Deepest}}" BorderBrush="{{StaticResource Border.Default}}" BorderThickness="0,0,1,0">
                <StackPanel Margin="0,16,0,0">
                    <RadioButton Style="{{StaticResource FRToggleButtonStyle}}" Content="Prog " IsChecked="True" Height="64" Margin="0,0,0,8"/>
                    <RadioButton Style="{{StaticResource FRToggleButtonStyle}}" Content="Stat " Height="64" Margin="0,0,0,8"/>
                    <RadioButton Style="{{StaticResource FRToggleButtonStyle}}" Content="Aux " Height="64" Margin="0,0,0,8"/>
                    <RadioButton Style="{{StaticResource FRToggleButtonStyle}}" Content="Set " Height="64"/>
                </StackPanel>
            </Border>

            <!-- COL 1: Command Toolbox -->
            <Border Grid.Column="1" Background="#801E1E1E" BorderBrush="{{StaticResource Border.Default}}" BorderThickness="0,0,1,0">
                <ScrollViewer>
                    <StackPanel Margin="10,16,10,10">
                        <TextBlock Text="Logic Command" FontWeight="Bold" Foreground="{{StaticResource Text.Muted}}" FontSize="12" Margin="0,0,0,12"/>
                        <UniformGrid Columns="2">
                            <Button Style="{{StaticResource FRButtonStyle}}" Command="{{Binding AddInstructionCommand}}" CommandParameter="WAIT" Margin="2,2">
                                <StackPanel><TextBlock Text="⏱" Foreground="#D19A66" FontSize="18" HorizontalAlignment="Center"/><TextBlock Text="Wait" Foreground="{{StaticResource Text.Primary}}" HorizontalAlignment="Center"/></StackPanel>
                            </Button>
                            <Button Style="{{StaticResource FRButtonStyle}}" Command="{{Binding AddInstructionCommand}}" CommandParameter="SETDO" Margin="2,2">
                                <StackPanel><TextBlock Text="⚡" Foreground="#98C379" FontSize="18" HorizontalAlignment="Center"/><TextBlock Text="IO Set" Foreground="{{StaticResource Text.Primary}}" HorizontalAlignment="Center"/></StackPanel>
                            </Button>
                            <Button Style="{{StaticResource FRButtonStyle}}" Command="{{Binding AddInstructionCommand}}" CommandParameter="WHILE" Margin="2,2">
                                <StackPanel><TextBlock Text="🔃" Foreground="#61AFEF" FontSize="18" HorizontalAlignment="Center"/><TextBlock Text="Loop" Foreground="{{StaticResource Text.Primary}}" HorizontalAlignment="Center"/></StackPanel>
                            </Button>
                            <Button Style="{{StaticResource FRButtonStyle}}" Command="{{Binding AddInstructionCommand}}" CommandParameter="DOFILE" Margin="2,2">
                                <StackPanel><TextBlock Text="📄" Foreground="#E5C07B" FontSize="18" HorizontalAlignment="Center"/><TextBlock Text="Sub Call" Foreground="{{StaticResource Text.Primary}}" HorizontalAlignment="Center"/></StackPanel>
                            </Button>
                        </UniformGrid>

                        <Rectangle Height="1" Fill="{{StaticResource Border.Default}}" Margin="0,16"/>

                        <TextBlock Text="Motion Command" FontWeight="Bold" Foreground="{{StaticResource Text.Muted}}" FontSize="12" Margin="0,0,0,12"/>
                        <UniformGrid Columns="2">
                            <Button Style="{{StaticResource FRButtonStyle}}" Command="{{Binding AddInstructionCommand}}" CommandParameter="LIN" Margin="2,2">
                                <StackPanel><TextBlock Text="↗" Foreground="#56B6C2" FontSize="18" HorizontalAlignment="Center"/><TextBlock Text="Lin" Foreground="{{StaticResource Text.Primary}}" HorizontalAlignment="Center"/></StackPanel>
                            </Button>
                            <Button Style="{{StaticResource FRButtonStyle}}" Command="{{Binding AddInstructionCommand}}" CommandParameter="PTP" Margin="2,2">
                                <StackPanel><TextBlock Text="⤿" Foreground="#C678DD" FontSize="18" HorizontalAlignment="Center"/><TextBlock Text="Ptp" Foreground="{{StaticResource Text.Primary}}" HorizontalAlignment="Center"/></StackPanel>
                            </Button>
                            <Button Style="{{StaticResource FRButtonStyle}}" Command="{{Binding AddInstructionCommand}}" CommandParameter="ARC" Margin="2,2">
                                <StackPanel><TextBlock Text="⭕" Foreground="#E06C75" FontSize="18" HorizontalAlignment="Center"/><TextBlock Text="Arc" Foreground="{{StaticResource Text.Primary}}" HorizontalAlignment="Center"/></StackPanel>
                            </Button>
                            <Button Style="{{StaticResource FRButtonStyle}}" Command="{{Binding CommandReg.ExecuteCommand}}" CommandParameter="LoadURDF" Margin="2,2">
                                <StackPanel><TextBlock Text="🤖" Foreground="{{StaticResource Accent.Motion}}" FontSize="18" HorizontalAlignment="Center"/><TextBlock Text="URDF" Foreground="{{StaticResource Text.Primary}}" HorizontalAlignment="Center"/></StackPanel>
                            </Button>
                        </UniformGrid>
                    </StackPanel>
                </ScrollViewer>
            </Border>

            <!-- SPLITTER 1 -->
            <GridSplitter Grid.Column="2" Width="5" HorizontalAlignment="Center" VerticalAlignment="Stretch" Background="{{StaticResource Border.Default}}"/>

            <!-- COL 3: Logic WorkTree -->
            <Border Grid.Column="3" Background="#801E1E1E" CornerRadius="0" BorderBrush="{{StaticResource Border.Default}}" BorderThickness="0,0,1,0">
                <Grid>
                    <Grid.RowDefinitions>
                        <RowDefinition Height="Auto"/>
                        <RowDefinition Height="*"/>
                    </Grid.RowDefinitions>
                    <Border Grid.Row="0" Background="{{StaticResource Bg.Surface2}}" BorderBrush="{{StaticResource Border.Default}}" BorderThickness="0,0,0,1" Padding="12,8">
                        <TextBlock Text="LOGIC SEQUENCE" FontWeight="Bold" Foreground="{{StaticResource Text.Muted}}" FontSize="11"/>
                    </Border>
                    {tree_xml}
                </Grid>
            </Border>

            <!-- SPLITTER 2 -->
            <GridSplitter Grid.Column="4" Width="5" HorizontalAlignment="Center" VerticalAlignment="Stretch" Background="{{StaticResource Border.Default}}"/>

            <!-- COL 5: Right Master Panel (Viewer & Jog Tabs) -->
            <Border Grid.Column="5" Background="{{StaticResource Bg.Surface2}}">
                <TabControl Background="Transparent" BorderThickness="0">
                    <TabControl.Resources>
                        <Style TargetType="TabItem">
                            <Setter Property="Background" Value="Transparent"/>
                            <Setter Property="Foreground" Value="{{StaticResource Text.Muted}}"/>
                            <Setter Property="FontSize" Value="13"/>
                            <Setter Property="FontWeight" Value="SemiBold"/>
                            <Setter Property="Padding" Value="16,8"/>
                            <Setter Property="BorderThickness" Value="0,0,0,2"/>
                            <Setter Property="BorderBrush" Value="Transparent"/>
                            <Setter Property="Template">
                                <Setter.Value>
                                    <ControlTemplate TargetType="TabItem">
                                        <Border Background="{{TemplateBinding Background}}" BorderBrush="{{TemplateBinding BorderBrush}}" BorderThickness="{{TemplateBinding BorderThickness}}" Padding="{{TemplateBinding Padding}}">
                                            <ContentPresenter HorizontalAlignment="Center" VerticalAlignment="Center" ContentSource="Header"/>
                                        </Border>
                                        <ControlTemplate.Triggers>
                                            <Trigger Property="IsSelected" Value="True">
                                                <Setter Property="Foreground" Value="{{StaticResource Text.Primary}}"/>
                                                <Setter Property="BorderBrush" Value="{{StaticResource Accent.Motion}}"/>
                                            </Trigger>
                                            <Trigger Property="IsMouseOver" Value="True">
                                                <Setter Property="Foreground" Value="{{StaticResource Text.Primary}}"/>
                                            </Trigger>
                                        </ControlTemplate.Triggers>
                                    </ControlTemplate>
                                </Setter.Value>
                            </Setter>
                        </Style>
                    </TabControl.Resources>
                    
                    <TabItem Header="3D Simulation Viewer">
                        <Grid>
                            {workspace_xml}
                            {overlay1_xml}
                            {overlay2_xml}
                        </Grid>
                    </TabItem>
                    
                    <TabItem Header="Manual System Jog">
                        {machine_xml}
                    </TabItem>
                </TabControl>
            </Border>
        </Grid>"""

# Replace the giant grid starting around line 137 up to the bottom console
split_point1 = xaml.find('<!-- 3-PANE EXPLORER')
split_point2 = xaml.find('<!-- BOTTOM CONSOLE (Telemetry)')
if split_point1 == -1 or split_point2 == -1:
    print("Could not find insertion bounds")
    exit(1)

new_xaml = xaml[:split_point1] + new_grid + "\n\n        <!-- ═══════════════════════════════════════════════════════════ -->\n        " + xaml[split_point2:]

with open(filepath, 'w', encoding='utf-8') as f:
    f.write(new_xaml)

print("Wrote updated MainWindow.xaml successfully.")
