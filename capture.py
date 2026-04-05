import time
import os
from PIL import ImageGrab

time.sleep(15) # Wait for WPF and Gazebo to appear

# Capture screenshot
img = ImageGrab.grab()
path = r"C:\Users\Admin\.gemini\antigravity\brain\5722c9e3-ecba-4746-b0e6-9711ce2bec80\roboforge_mac_theme_check.png"
img.save(path)
print("Screenshot saved to:", path)
