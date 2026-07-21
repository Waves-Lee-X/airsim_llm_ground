# Runtime health of semantic fusion and object tracking.
std_msgs/Header header
bool healthy
string state
bool detections_fresh
bool depth_fresh
bool camera_info_valid
bool tf_available
float32 sync_delta_sec
uint32 observation_count
uint32 track_count
uint32 confirmed_track_count
string message
