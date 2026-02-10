"""Service wrapper for mission result queries using std_srvs/Trigger.

Since custom ROS2 service definitions require a full package build,
we use std_srvs/srv/Trigger and encode the mission result as JSON
in the response message field.

Response format (JSON in message field):
{
    "result": "pending" | "success" | "failure_timeout" | "failure_physicalassistance" | "failure_foodspoiled",
    "mission_number": int,
    "distance_to_goal": float,
    "in_progress": bool,
    "traveled_distance": float,
    "elapsed_time": float,
    "food_enabled": bool,
    "food_initial_pieces": int,
    "food_final_pieces": int,
    "food_loss_fraction": float,
    "food_spoiled": bool
}

Result values:
- "pending": Mission not yet started or in progress
- "success": Robot reached goal within tolerance
- "failure_timeout": Timeout reached before reaching goal
- "failure_physicalassistance": Robot fell down (bad orientation, requires physical assistance)
- "failure_foodspoiled": Food spoiled during delivery
"""

from std_srvs.srv import Trigger

# Re-export Trigger as GetMissionResult for cleaner imports
GetMissionResult = Trigger
