#TargetSpinTop.msg
std_msgs/Header header

bool tracking
uint8 id

int32 armors_count			
geometry_msgs/Point position		#This is the center points
geometry_msgs/Vector3 velocity
float64 yaw
float64 w_yaw
float64 r1
float64 r2
float64 dz

float64 yaw_diff
float64 position_diff
