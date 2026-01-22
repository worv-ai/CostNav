"""Service wrapper for mission result queries using std_srvs/Trigger.

Since custom ROS2 service definitions require a full package build,
we use std_srvs/srv/Trigger and encode the mission result as JSON
in the response message field.

Response format (JSON in message field):
{
    "result": "pending" | "success" | "failure",
    "mission_number": int,
    "distance_to_goal": float,
    "in_progress": bool,
    "people_collision_count": int
}
"""

from std_srvs.srv import Trigger

# Re-export Trigger as GetMissionResult for cleaner imports
GetMissionResult = Trigger
