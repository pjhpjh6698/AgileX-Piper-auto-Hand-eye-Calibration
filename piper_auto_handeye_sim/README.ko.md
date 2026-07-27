# Gazebo Hand-Eye 검증 환경

Piper + 플랜지 장착 카메라를 Gazebo에서 시뮬레이션해 hand-eye 캘리브레이션을
**정답을 아는 상태로** 검증합니다.

## 왜 시뮬레이션인가

실기에서는 결과가 얼마나 **정확한지** 알 방법이 없습니다. 측정할 수 있는 건
자기일관성(RMS)뿐이고, 그건 "샘플들이 서로 모순되지 않는다"는 뜻일 뿐
"정답에 가깝다"는 뜻이 아닙니다. 프레임 방향이 통째로 뒤집혀도 RMS는 낮게
나올 수 있습니다.

Gazebo에서는 카메라 장착 변환이 URDF에 적혀 있으므로 **참값을 압니다.**
따라서 진짜 오차를 잴 수 있습니다.

## 핵심 설계: 하드웨어 스택을 그대로 검증

```
        [실기]                              [Gazebo]
  piper_ctrl_single_node            gazebo_piper_driver_node
   (CAN 버스)                        (FK + IK, ros2_control)
        │                                    │
        └──── /end_pose_stamped, /pos_cmd ───┘   ← 동일한 토픽 인터페이스
                        │
        ┌───────────────┴───────────────┐
        │  piper_control_node           │
        │  aruco_detector_node          │  ← 전부 수정 없이 그대로
        │  handeye_calibration_node     │
        └───────────────────────────────┘
```

`mock_robot_node`와 같은 발상이지만, 이번엔 **실제 물리 + 실제 렌더링 + 실제
ArUco 검출**이 붙습니다. 즉 시뮬레이션에서 검증한 코드가 곧 실기에서 도는
코드입니다.

## 설치

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
                    ros-humble-ros2-control ros-humble-ros2-controllers \
                    ros-humble-xacro
cd ~/piper_ros2_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  piper_msgs piper_description auto_handeye_interfaces \
  piper_auto_handeye piper_auto_handeye_gui piper_auto_handeye_sim
source install/setup.bash
```

## 실행

```bash
# 터미널 1 — Gazebo + 전체 스택
ros2 launch piper_auto_handeye_sim gazebo_handeye.launch.py

# 터미널 2 — 캘리브레이션 실행
ros2 action send_goal /run_calibration \
  auto_handeye_interfaces/action/RunCalibration \
  "{target_sample_count: 14, auto_move: true, calibration_method: PARK,
    save_on_success: true}" --feedback

# 터미널 3 — 진짜 오차 확인
ros2 run piper_auto_handeye_sim ground_truth_reporter_node
```

`ground_truth_reporter_node`가 TF에서 정답을 읽어 결과와 비교합니다:

```
GROUND TRUTH  gripper_T_camera (link6 -> camera_color_optical_frame)
  t   (m)  : [+0.055000, -0.032000, +0.048000]
ESTIMATE      from handeye_park_*.yaml
  t   (m)  : [...]
TRUE ERROR vs ground truth
  translation :    0.xxxx mm
  rotation    :    0.xxxxx deg
  VERDICT     : EXCELLENT
```

## 구성 요소

| 파일 | 역할 |
|---|---|
| `urdf/piper_handeye_gazebo.xacro` | Piper + 카메라. **카메라 장착 변환 = 정답** |
| `worlds/handeye_calibration.world` | ArUco 마커가 바닥에 놓인 월드 |
| `models/aruco_marker/` | 마커 모델 (검은 마커 0.10 m, 판 0.15 m) |
| `piper_auto_handeye_sim/gazebo_piper_driver_node.py` | 실기 드라이버 토픽 에뮬레이션 |
| `piper_auto_handeye_sim/urdf_kdl.py` | URDF→PyKDL 체인, 다중 재시작 IK |
| `config/sim_calibration_poses.yaml` | **생성된** 자세 14개 (수정 금지) |
| `scripts/generate_poses.py` | 도달 가능 집합에서 자세 생성 |
| `scripts/validate_poses.py` | 오프라인 도달성/가시성 검증 |
| `scripts/generate_aruco_texture.py` | 마커 텍스처 생성 |

## 설계 과정에서 발견한 것들

### 1. 업스트림 `ros2_control`의 관절 한계가 틀려 있음

`piper_description_gazebo.xacro`는 모든 관절 명령을 `[-1, 1]` rad로 묶는데,
실제 URDF 한계는 다릅니다:

| 관절 | 실제 범위 | 업스트림 명령 제한 |
|---|---|---|
| joint2 | `[0.000, 3.140]` | `[-1, 1]` |
| joint3 | `[-2.967, 0.000]` | `[-1, 1]` |

작업영역 절반 이상이 막히고, 하필 캘리브레이션에 필요한 전방-하향 자세가
그 안에 있습니다. 그래서 이 패키지는 순수 기구학 URDF
(`piper_no_gripper_description.urdf`) 위에 **올바른 한계로 자체
`ros2_control`을 선언**합니다.

### 2. 카메라는 link6의 `+x`를 봐야 함 (`+z`가 아니라)

관절 공간 30만 개를 샘플링해 확인한 결과, 전방 영역에서
link6의 z축이 아래를 향하는 경우는 **0%**, x축이 아래를 향하는 경우는
**25.8%**였습니다. 그래서 카메라는 `+x` 방향을 봅니다.

### 3. 카르테시안 자세를 손으로 쓰면 안 됨

처음에 직관으로 쓴 14개 자세는 **전부 도달 불가**였습니다. 6축 팔에서는
"이 위치에 이 자세"가 대부분 불가능합니다. 그래서 `generate_poses.py`가
정기구학 해에서 자세를 **생성**합니다 — 도달성이 구조적으로 보장됩니다.

### 4. IK 단일 시드는 실패함 → 다중 재시작 필수

FK 해에서 만든(=반드시 도달 가능한) 자세인데도 단일 시드 LMA IK는 14개 중
**13개를 못 찾았습니다.** 팔에는 여러 IK 분기(팔꿈치 위/아래, 손목 뒤집힘)가
있고 LMA는 국소 최적화이기 때문입니다. `solve_ik()`가 관절 한계 안에서
무작위 재시작을 하며, 성공 판정은 솔버 반환값이 아니라 **FK로 직접 확인**합니다.

이 발견이 없었다면 Gazebo에서 "이동 거절 → 샘플 0개"로 나타났을 것입니다.

## 튜닝

정답값을 바꾸려면 xacro의 `cam_*` 프로퍼티만 고치면 됩니다. 나머지는 따라옵니다:

```bash
# 1) 마운트 수정 후 자세 재생성
python3 scripts/generate_poses.py --urdf <piper_no_gripper_description.urdf 경로>
# 2) 출력된 marker 위치를 worlds/handeye_calibration.world 에 반영
# 3) 검증
python3 scripts/validate_poses.py --urdf <같은 경로>
```

`ground_truth_reporter_node`는 TF에서 읽으므로 하드코딩 갱신이 필요 없습니다.

## 알려진 제약

- **충돌 회피 없음.** 자세는 도달성과 가시성만 검증하며, 자기충돌은 보지
  않습니다. Gazebo에서 눈으로 확인하세요.
- 마커가 로봇 베이스에서 x=0.247 m로 가깝습니다. 팔이 마커를 가릴 수 있으니
  실행 중 카메라 영상을 확인하세요:
  `ros2 run rqt_image_view rqt_image_view /camera/camera/color/debug_image`
- Gazebo 렌더러의 에일리어싱 때문에 `maximum_reprojection_error`를 실기(3.0)보다
  느슨한 4.0으로 두었습니다.

관련 문서: [../ARCHITECTURE.ko.md](../ARCHITECTURE.ko.md)
