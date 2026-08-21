# TrackerState.msg
std_msgs/Header header

string state

uint32 detection_count
uint32 lost_count

int32 last_pitch
int32 last_yaw

float32 flight_time
float32 armor_distance
float32 compen_height
