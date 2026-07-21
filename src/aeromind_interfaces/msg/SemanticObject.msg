# 融合后的语义世界对象
string id
string class_name
float32 confidence
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 size
bool position_valid
bool dynamic
# Row-major 3x3 position covariance in frame_id. Zero when position is invalid.
float32[9] position_covariance
float32 depth_m
float32 age_sec
string state
string evidence_type
string[] sources
uint32 hit_count
uint32 miss_count
