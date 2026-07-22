# Runtime state of the bottom-layer autonomous obstacle avoidance stack.
std_msgs/Header header
bool enabled
string state
bool replanning
float32 nearest_obstacle_m
bool takeoff_clearance_valid
float32 takeoff_clearance_m
string takeoff_clearance_source
float32 target_distance_m
string active_strategy
bool dynamic_risk_active
string dynamic_object_id
float32 dynamic_clearance_m
float32 dynamic_ttc_sec
string message
