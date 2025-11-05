#!/usr/bin/env python3
"""
Minimal test: Can we import DARP from the submodule?
Run this BEFORE building with colcon to test Python imports.
"""
import sys
sys.path.insert(0, '/home/ubuntu/Documents/ros-task/workspace/darp_ros2')

try:
    from darp_ros2.darp_lib import multiRobotPathPlanner
    print("✓ SUCCESS: Imported multiRobotPathPlanner from darp_lib")

    from darp_ros2.darp_lib.multiRobotPathPlanner import MultiRobotPathPlanner
    print("✓ SUCCESS: Imported MultiRobotPathPlanner class")

    # Try to instantiate
    planner = MultiRobotPathPlanner(
        nx=5, ny=5,
        notEqualPortions=False,
        initial_positions=[0, 4],
        portions=[0.5, 0.5],
        obs_pos=[],
        visualization=False
    )
    print(f"✓ SUCCESS: DARP ran in {planner.execution_time:.2f}s")
    print(f"  Turns: {planner.best_case.turns}")

except ImportError as e:
    print(f"✗ IMPORT FAILED: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
except Exception as e:
    print(f"✗ EXECUTION FAILED: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n✓ All tests passed!")
