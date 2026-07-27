# Detection of the calibration target marker in a single camera frame.
# pose is camera_T_target (target expressed in the camera optical frame).
std_msgs/Header header
bool detected
int32 marker_id
geometry_msgs/Pose pose          # camera_T_target
float64 reprojection_error       # pixels
float64 stability_score          # 0..1, higher = more stable across recent frames
string rejection_reason          # empty when accepted
