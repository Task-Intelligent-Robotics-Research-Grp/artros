int32 NO_RES=0             # no responses to ROS
int32 SWEEP_RES=1          # specify sweep endpoint
int32 RECAPTURE_RES=2      # force ROS to recapture 3D point cloud

std_msgs/Header header
int32 pointing_state       # NO_RES|SWEEP_RES|RECAPTURE_RES
geometry_msgs/Point point  # sweep endpoint
