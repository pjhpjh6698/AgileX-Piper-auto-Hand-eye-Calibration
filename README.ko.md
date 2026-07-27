# Piper 자동 Hand-Eye 캘리브레이션 (Eye-in-Hand, ArUco)

**AgileX Piper** 로봇 팔과 엔드이펙터에 장착된 **Intel RealSense** 카메라를 이용해
고정된 **ArUco** 마커를 관측하는 **Eye-in-Hand** 방식의 자동 hand-eye 캘리브레이션
시스템입니다. 로봇을 여러 자세로 자동으로 이동시키며
`(base_T_gripper, camera_T_target)` 쌍을 수집하고, OpenCV의
`calibrateHandEye()`를 이용해 `gripper_T_camera`를 계산합니다.

> **상태:** MVP — 고정된 단일 ArUco 마커 타겟만 지원합니다. ChArUco/GridBoard는
> 확장 지점으로 문서화되어 있으나(`aruco_detector_node` 참고) 아직 구현되지
> 않았습니다.

---

## 1. 이 프로젝트가 하는 일

- 카메라는 **로봇 플랜지(`link6`)에 고정**되어 있고, ArUco 마커는 **작업 공간에
  고정**되어 있습니다. 이것이 *Eye-in-Hand* 구성입니다.
- 로봇이 **완전히 정지했을 때만** 샘플을 수집하며, 자세마다 여러 프레임의 마커
  관측값을 평균 내고, 잘못된 검출은 걸러낸 뒤 계산을 수행하고, 폐루프
  타겟-일관성 검사로 결과를 **검증**합니다.
- 결과를 **정적 TF** `link6 → camera_color_optical_frame`로 퍼블리시합니다.
- 목(mock) 로봇과 합성(synthetic) 마커를 이용해 **하드웨어 없이도** 동작하며,
  알려진 변환값을 정확히 복원할 수 있습니다.

모든 위치는 **미터(m)** 단위이며, 회전은 내부적으로 모두 **회전 행렬 / 쿼터니언**
으로 처리됩니다. 오일러 각은 **GUI/로그 표시용으로만** 사용됩니다.

---

## 2. 패키지 구성

| 패키지 | 타입 | 내용 |
|---|---|---|
| `auto_handeye_interfaces` | ament_cmake | 메시지(`MarkerDetection`, `RobotState`, `CalibrationStatus`), 서비스(`ResetCalibration`, `SaveCalibration`, `LoadCalibration`, `PublishCalibrationTf`, `AddManualSample`), 액션(`RunCalibration`, `MoveToCalibrationPose`) |
| `piper_auto_handeye` | ament_python | 전체 노드 + ROS 비의존 수학 코어 + 설정 + launch + 테스트 |
| `piper_auto_handeye_gui` | ament_python | rqt GUI 플러그인 |

### 노드
- `aruco_detector_node` — 타겟 마커를 검출하고 `camera_T_target`을 퍼블리시합니다.
- `piper_control_node` — 기존 Piper 드라이버(`piper_ctrl_single_node`)에 대한
  **어댑터**로, `RobotState`를 퍼블리시하고 안전 검증이 적용된
  `MoveToCalibrationPose` 액션을 제공합니다.
- `handeye_calibration_node` — 상태 머신 + `RunCalibration` 액션 + 서비스들.
- `calibration_tf_publisher_node` — 계산된 정적 결과 TF를 브로드캐스트합니다.
- `mock_robot_node` — 하드웨어 없는 테스트를 위해 Piper 드라이버 토픽을
  에뮬레이션합니다.
- `synthetic_marker_publisher_node` — 카메라 없이 테스트할 수 있는 마커
  더블(알려진 정답값으로부터 `camera_T_target`을 생성).

### ROS 비의존 코어 (단위 테스트 가능, 순수 numpy)
`transform_utils.py`, `calibration_solver.py`, `calibration_validator.py`,
`pose_filter.py`, `safety_validator.py`, `calibration_io.py`.

---

## 3. 설계 결정: 기존 Piper 드라이버 재사용

이 워크스페이스에는 이미 동작하는 드라이버 `Piper_ros/src/piper`가 있으며,
CAN을 통해 `piper_sdk`를 감싸 다음을 제공합니다:

| 용도 | 토픽 | 타입 |
|---|---|---|
| 자세 읽기 (`base_T_gripper`) | `/end_pose_stamped` | `geometry_msgs/PoseStamped` |
| 상태 읽기 | `/arm_status` | `piper_msgs/PiperStatusMsg` |
| 조인트 읽기 | `/joint_states_single` | `sensor_msgs/JointState` |
| 이동 (Cartesian moveL) | `/pos_cmd` | `piper_msgs/PosCmd` (x,y,z m; roll,pitch,yaw rad; `mode2=1`) |
| 인에이블 | `/enable_flag` | `std_msgs/Bool` |

`piper_control_node`는 SDK를 다시 감싸는 대신 이 토픽들과 직접 통신합니다.
**동일한** 어댑터가 실제 드라이버와 `mock_robot_node` 양쪽에서 모두 동작합니다.
(원시 SDK 호출인 `GetArmEndPoseMsgs` / `EndPoseCtrl` / `EnableArm`은
`piper_control_node`에 향후 `control_backend: sdk` 옵션으로 문서화되어
있습니다.)

---

## 4. 변환 규약 (반드시 읽으세요)

프레임 방향 오류는 hand-eye 캘리브레이션 실패의 **가장 큰 원인**입니다.
명명 규칙: `A_T_B` = "A 기준으로 표현된 B" = B의 점을 A로 매핑
(`p_A = A_T_B @ p_B`).

- `base_T_gripper` — 로봇 베이스 기준 그리퍼/플랜지 (`/end_pose_stamped`에서
  가져옴)
- `camera_T_target` — 카메라 광학 프레임 기준 마커 (검출 노드에서 가져옴)
- `gripper_T_camera` — **캘리브레이션 결과** (그리퍼 기준 카메라)
- `base_T_target` — 검증용으로만 사용

**OpenCV `calibrateHandEye` (Eye-in-Hand) 매핑:**
```
R/t_gripper2base  == base_T_gripper     (샘플별 리스트)
R/t_target2cam    == camera_T_target    (샘플별 리스트)
반환값 cam2gripper == gripper_T_camera (퍼블리시하는 정적 TF)
```

**쿼터니언은 `transform_utils`의 모든 공개 함수에서 ROS 순서 `(x, y, z, w)`**를
사용합니다. `(w, x, y, z)`로의 변환은 라이브러리 경계에서
`quaternion_ros_to_wxyz` / `quaternion_wxyz_to_ros`를 통해서만 수행하세요.

**퍼블리시되는 TF:** 부모 `link6` → 자식 `camera_color_optical_frame` =
`gripper_T_camera`.

---

## 5. 프레임 & 토픽 (기본값, 모두 설정 파일에서 변경 가능)

- base=`base_link`, gripper=`link6`, camera=`camera_color_optical_frame`, target=`calibration_target`
- image=`/camera/camera/color/image_raw`, camera_info=`/camera/camera/color/camera_info`

시스템에 맞게 `config/*.yaml`에서 변경하세요.

---

## 6. 의존성 & 빌드

```bash
# ROS 2 Humble, Ubuntu 22.04
sudo apt install ros-humble-realsense2-camera ros-humble-cv-bridge \
                 ros-humble-tf2-ros ros-humble-rqt-gui-py
pip install opencv-contrib-python numpy pyyaml   # aruco에는 contrib 필요

# 빌드 (워크스페이스 루트에서)
cd ~/piper_ros2_ws
colcon build --symlink-install \
  --packages-select auto_handeye_interfaces piper_auto_handeye piper_auto_handeye_gui
source install/setup.bash
```
실제 로봇 제어를 위해서는 `piper_msgs`(`Piper_ros` 소속)가 빌드/소싱되어
있어야 합니다. 없을 경우 노드는 (mock 전용으로) 우아하게 축소 동작합니다.

---

## 7. 빠른 시작 — 하드웨어 없이 (먼저 권장)

목(mock) 로봇과, 알려진 `gripper_T_camera` 값을 인코딩한 **카메라 없는** 합성
마커로 전체 상태 머신을 실행합니다. 파이프라인은 이 값을 복원해야 합니다.

```bash
ros2 launch piper_auto_handeye mock_calibration.launch.py
# 다른 터미널에서 트리거:
ros2 action send_goal /run_calibration auto_handeye_interfaces/action/RunCalibration \
  "{target_sample_count: 10, auto_move: true, calibration_method: PARK,
    settle_time: 0.3, observations_per_pose: 3, save_on_success: true}" --feedback
```
결과는 `~/.ros/piper_auto_handeye/handeye_park_*.yaml`에 저장됩니다.
퍼블리시 + 검증:
```bash
ros2 run piper_auto_handeye calibration_tf_publisher_node \
  --ros-args -p calibration_file:=$(ls -t ~/.ros/piper_auto_handeye/handeye_park_*.yaml | grep -v samples | head -1)
ros2 run tf2_ros tf2_echo link6 camera_color_optical_frame
```

---

## 8. 실제 하드웨어

### 8a. 카메라 시작 및 확인
```bash
ros2 launch realsense2_camera rs_launch.py enable_color:=true
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/camera_info --once   # 내부 파라미터가 있는지 확인
```

### 8b. ArUco 마커
설정된 딕셔너리(`DICT_4X4_50`)와 설정된 ID(`target_marker_id: 1`)로 마커를
출력하고, **인쇄된 한 변의 실제 길이를 측정**하세요. 측정값을
`config/aruco.yaml`의 `marker_length`(미터 단위)에 설정합니다. 마커를 모든
캘리브레이션 자세에서 보이도록 작업 공간에 견고하게 고정하세요.

### 8c. Piper 드라이버 시작 (CAN 오픈)
```bash
# Piper_ros에서 실행; /end_pose_stamped, /pos_cmd, /arm_status, /enable_flag를 기동
ros2 run piper piper_ctrl_single_node ...    # (Piper_ros 안내에 따름; can0 설정)
```

### 8d. 검출기 + 캘리브레이션 스택 (먼저 DRY-RUN!)
```bash
ros2 launch piper_auto_handeye detection.launch.py            # aruco 검출기
ros2 launch piper_auto_handeye calibration.launch.py dry_run:=true
```
dry-run 상태에서 모든 자세가 검증되는지(움직임 없이) 확인하세요. 그런 다음,
mock + RViz에서 도달 범위/충돌/마커 가시성을 확인하고 비상 정지 버튼 옆에
대기한 **후에만**:
```bash
ros2 launch piper_auto_handeye calibration.launch.py dry_run:=false
```

또는 한 번에 모두 실행 (저수준 드라이버는 시작하지 **않음**):
```bash
ros2 launch piper_auto_handeye bringup.launch.py \
  use_realsense:=true dry_run:=true use_gui:=true
```

---

## 9. 자동 캘리브레이션
```bash
ros2 action send_goal /run_calibration auto_handeye_interfaces/action/RunCalibration \
  "{target_sample_count: 15, auto_move: true, calibration_method: PARK,
    save_on_success: true}" --feedback
```
흐름: 시스템 점검 → 각 자세마다: 이동 → 정착(settle) → 마커 안정화 대기 →
N개 프레임 평균 → `(base_T_gripper, camera_T_target)` 저장 → 반복 → 계산 →
검증(선택적 이상치 제거) → 저장 → (서비스/노드를 통해 TF 퍼블리시).

## 10. 수동 캘리브레이션 (`auto_move:=false`)
로봇을 직접(teach/teleop) 움직인 뒤, 현재 쌍을 캡처합니다:
```bash
ros2 action send_goal /run_calibration auto_handeye_interfaces/action/RunCalibration \
  "{target_sample_count: 15, auto_move: false}" --feedback &
# 로봇이 새 자세에서 정지할 때마다:
ros2 service call /add_manual_sample auto_handeye_interfaces/srv/AddManualSample "{}"
```

## 11. GUI
```bash
rqt --standalone piper_auto_handeye_gui
```
카메라 뷰, 마커/로봇 상태, 시작/일시정지/취소/리셋/샘플추가, 방법 + 목표
샘플 수, 실시간 진행 상황, 결과(`gripper_T_camera`의 이동/쿼터니언/RPY +
RMS), 저장/불러오기/TF 퍼블리시/로그. GUI는 **오직** ROS 토픽/서비스/액션을
통해서만 통신하며, GUI를 닫아도 캘리브레이션 매니저는 멈추지 않습니다.

---

## 12. 서비스
```bash
ros2 service call /reset_calibration  auto_handeye_interfaces/srv/ResetCalibration "{}"
ros2 service call /save_calibration   auto_handeye_interfaces/srv/SaveCalibration "{path: ''}"
ros2 service call /load_calibration   auto_handeye_interfaces/srv/LoadCalibration "{path: ''}"
ros2 service call /publish_calibration_tf auto_handeye_interfaces/srv/PublishCalibrationTf "{path: '', publish: true}"
```

## 13. 출력
- 결과: `~/.ros/piper_auto_handeye/handeye_<method>_<timestamp>.yaml`
  (이동, 쿼터니언, 4×4 행렬, 검증 결과, 소스 포함).
- 원시 샘플: `..._samples.yaml` (샘플별 자세 + 재투영 오차, 안정성,
  로봇-마커 Δt, 수락/거절 사유).

## 14. RViz에서 검증
```bash
rviz2   # Fixed Frame = base_link; TF 디스플레이 추가; 카메라 프레임이 플랜지에 위치하는지 확인
ros2 run tf2_ros tf2_echo link6 camera_color_optical_frame
```

---

## 15. 검증 및 품질
`base_T_target_i = base_T_gripper_i @ gripper_T_camera @ camera_T_target_i`는
모든 i에 대해 (거의) 동일해야 합니다. 검증기는 이동/회전 RMS 및 최댓값을
보고하고, 이상치를 표시하며, 최악의 샘플을 (제한된 범위 내에서) 반복적으로
제거할 수 있습니다. `config/handeye.yaml`의 임계값
(`maximum_translation_rms_m`, `maximum_rotation_rms_deg`)이
SUCCESS/WARNING/FAILED를 결정합니다. **2축 이상**의 회전과 이동 다양성을
포함한 **15–25개 샘플**을 권장하며, `minimum_samples` 하한은 10입니다.

## 16. 테스트
```bash
# ROS 비의존 수학 (합성 hand-eye 복원, 노이즈, 가드, 필터):
cd src/autoCali/piper_auto_handeye && python3 -m pytest test/ -q
# 또는 colcon으로:
colcon test --packages-select piper_auto_handeye
```

---

## 17. 문제 해결
- **`target_id_N_not_seen`** — `target_marker_id`/딕셔너리가 잘못되었거나
  마커가 시야에 없음.
- **`reproj_error_*`** — `marker_length`가 잘못되었거나, 모션 블러, 또는
  잘못된 내부 파라미터.
- **`waiting_for_camera_info`** — camera_info 토픽 이름이 잘못되었거나 카메라가
  켜져 있지 않음.
- **이동이 "not enabled"로 거부됨** — 어댑터는 실제 이동 전에 자동으로
  인에이블합니다(`auto_enable: true`). `/arm_status`와 CAN을 확인하세요.
- **이동이 "outside workspace"로 거부됨** — `config/piper.yaml`의
  `workspace_min/max`를 조정하세요.
- **회전 다양성 부족 경고** — 서로 다른 축으로 회전하는 자세를 추가하세요.
- **매니저가 이동할 수 없음** — `piper_control_node`(또는 `mock_robot_node`)가
  실행 중이고 `move_to_calibration_pose` 액션 서버가 떠 있는지
  (`ros2 action list`) 확인하세요.

## 18. ⚠️ 안전
- `dry_run`은 모든 곳에서 기본값이 **true**입니다. 실제 이동을 위해서는 제어
  노드에 `dry_run:=false`를 설정하고 **그리고** 각 goal마다
  `dry_run=false`를 설정해야 합니다.
- `config/calibration_poses.yaml`의 자세들은 **자리표시자(placeholder)**이며
  로봇에 맞게 검증되지 않았습니다. `dry_run:=false` **전에** mock + RViz에서
  도달 범위/충돌을 검증하세요.
- 작업 공간 경계, 최대 스텝 거리, 최대 속도, NaN 거부, 타임아웃은
  `safety_validator.py`에서 강제됩니다. 천천히 시작하고 비상 정지 버튼
  옆에서 대기하세요.
