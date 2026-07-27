# High level status of the hand-eye calibration state machine.
std_msgs/Header header
string state                     # IDLE, CHECKING_SYSTEM, MOVING, SETTLING, ...
int32 current_sample_count
int32 target_sample_count
int32 current_pose_index
float32 progress                 # 0.0 .. 1.0
string message
float64 translation_rms          # meters (0 until VALIDATING)
float64 rotation_rms_deg         # degrees (0 until VALIDATING)
