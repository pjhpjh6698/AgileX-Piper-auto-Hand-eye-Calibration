# Current state of the Piper robot as seen by the calibration adapter.
# tcp_pose is base_T_gripper (gripper/flange expressed in the robot base frame).
std_msgs/Header header
bool connected
bool enabled
bool moving
geometry_msgs/Pose tcp_pose      # base_T_gripper
float64[] joint_positions        # radians
string error_message             # empty when healthy
