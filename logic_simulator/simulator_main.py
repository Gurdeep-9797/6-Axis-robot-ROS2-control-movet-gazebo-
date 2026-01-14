"""
Logic Simulator Orchestrator

Runs the full validation harness:
1. Instantiates Bus
2. Starts Controller
3. Starts Bridge
4. Starts Task
5. Injects Faults (Optional Test)
"""

import time
import sys
from bus import default_bus
import fake_controller
import hardware_bridge
import pick_and_place
import messages

def main():
    print("=== ROBOT LOGIC SIMULATOR STARTING ===")
    
    # 1. Components
    controller = fake_controller.FakeController()
    bridge = hardware_bridge.HardwareBridge()
    task = pick_and_place.PickAndPlaceTask()
    
    # Monitor Loop
    try:
        while task.running:
            # Optional: Inject a random fault at Cycle 5 to test recovery?
            # For now, let's keep it clean to prove successful path first.
            if task.cycle_count == 5 and not hasattr(task, 'fault_tested'):
                print("\n>>> INJECTING TEST FAULT...")
                controller.trigger_fault()
                time.sleep(2)
                print(">>> CLEARING FAULT...")
                controller.clear_fault()
                task.fault_tested = True
                
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Stopping...")
        
    print(f"=== SIMULATION ENDED. Completed Cycles: {task.cycle_count} ===")

if __name__ == "__main__":
    main()
