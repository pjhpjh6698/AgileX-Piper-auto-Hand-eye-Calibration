# AgileX Piper 자동 Hand-Eye 캘리브레이션

손목에 RealSense 카메라를 단 AgileX Piper 팔을 위한 자동 eye-in-hand
캘리브레이션입니다. Start를 누르면 팔이 스스로 여러 자세를 돌며 고정된 마커를
관측하고, 그리퍼 기준 카메라 위치를 계산합니다. 손으로 팔을 옮기거나 자세를
받아적을 필요가 없습니다.

English documentation: [README.md](README.md)

## 특징

- launch 파일 하나로 팔, 카메라, 검출기, 솔버, GUI, RViz가 모두 뜹니다.
- 팔이 자세 스위프를 스스로 계획하고 이동합니다. 사용자는 Start만 누릅니다.
- ArUco 마커와 ChArUco 보드를 지원하며, 실행 중 GUI에서 전환합니다.
- 실시간 결과: 샘플이 하나 수락될 때마다 다시 풀어서 화면에 갱신합니다.
- 그리퍼가 마커를 가리면 자동으로 복구합니다.
- OpenCV 기반 솔버 5종(Tsai-Lenz, Park, Horaud, Andreff, Daniilidis).
- 한국어/영어 GUI.
- 로봇에 필요한 것이 전부 벤더링되어 있어 새 컴퓨터에서도 `colcon build` 하나면 끝납니다.

## 동작 방식: eye-in-hand

카메라는 플랜지에 고정되어 팔과 함께 움직이고, 마커는 작업 공간에 가만히
있습니다. 구하려는 값은 플랜지에서 카메라까지의 고정 변환
`gripper_T_camera`입니다.

각 자세마다 팔이 보고하는 플랜지 위치(`base_T_gripper`)와 카메라가 보는 마커
위치(`camera_T_target`)를 기록합니다. 마커가 움직이지 않는다는 것만으로 카메라
장착 위치를 풀기에 충분하며, 마커 자신의 위치는 수식에서 상쇄되므로 측정할
필요가 없습니다.

표기: `A_T_B`는 "A 기준으로 표현된 B"이며 `p_A = A_T_B @ p_B`입니다.

## 설치

Ubuntu 22.04 + ROS 2 Humble이 필요합니다.

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-cv-bridge \
                 ros-humble-tf2-ros ros-humble-rqt-gui-py
pip install opencv-contrib-python numpy pyyaml   # aruco에는 contrib 필요

mkdir -p ~/piper_ros2_ws && cd ~/piper_ros2_ws
git clone https://github.com/pjhpjh6698/AgileX-Piper-auto-Hand-eye-Calibration.git autoCali
cd autoCali
colcon build
source install/setup.bash
```

`--symlink-install`은 쓰지 마세요. 두 설치 방식이 섞이면 console script 래퍼가
패키지 메타데이터를 찾지 못합니다.

## 사용법

### 1. 타겟을 인쇄하고 실측한다

`DICT_4X4_50`의 ID 1번 ArUco 마커나 ChArUco 보드를 인쇄합니다. 그리고 인쇄물을
자로 재서, 생성기에 입력한 값이 아니라 실제로 잰 값을 넣으세요.

이 단계가 보기보다 중요합니다. 타겟 크기는 자세 추정이 가진 유일한 스케일이라,
크기가 10% 틀리면 카메라 위치도 작업 거리의 10%만큼 어긋납니다. 그런데 회전은
정확하게 나오기 때문에 아무 데서도 이상이 드러나지 않습니다. 프린터는 조용히
배율을 바꿉니다. 이 프로젝트도 133.3%로 인쇄된 보드 때문에 15mm 체커가 20mm가
되어 하루를 날렸습니다.

ChArUco 보드는 한 칸을 재지 말고 전체 폭을 재서 나누세요.

```
가로 전체 폭 ÷ 열 개수  ==  세로 전체 높이 ÷ 행 개수
```

두 값이 일치해야 합니다. 그 값을 GUI나 `config/aruco.yaml`에 입력합니다.

타겟은 모든 자세에서 카메라에 보이도록 작업 공간에 단단히 고정하고, 팔 앞쪽
20~30cm 정도에 두세요.

### 2. CAN을 올리고 팔이 응답하는지 확인한다

```bash
bash piper_auto_handeye/scripts/can_setup.sh          # 발견되는 모든 CAN 어댑터
ros2 run piper_auto_handeye agx_arm_check             # 읽기 전용 점검
```

`agx_arm_check`는 팔을 인에이블하지도, 모션을 명령하지도 않습니다. 링크가 살아
있고, 프레임이 들어오며, 팔이 결함 없이 자세를 디코딩할 때만 0으로 종료합니다.
`NOT READY`가 나오면 그대로 진행하지 마세요.

launch 파일의 기본값은 이 프로젝트를 개발한 장비 기준이므로, 사용 환경의 값을
직접 넘기세요. 인터페이스 이름은 `ip -br link show type can`으로 확인합니다.

```bash
ros2 launch piper_auto_handeye real_calibration.launch.py can_port:=can0
```

### 3. 캘리브레이션한다

```bash
ros2 launch piper_auto_handeye real_calibration.launch.py
```

이 launch는 로봇을 실제로 움직입니다. 첫 실행에서는 비상 정지 버튼에 손을 두세요.

GUI에서: CAN 패널이 초록인지 확인 → 타겟 종류 선택 후 실측 크기 입력 → RViz
시작 → Start. 팔이 자세를 순회하다가 샘플이 충분히 모이면 스스로 멈춥니다.
결과는 `gripper_T_camera` 패널에 나오고 샘플마다 갱신됩니다.

모션 없이 전체 스위프만 확인하려면:

```bash
ros2 launch piper_auto_handeye real_calibration.launch.py dry_run:=true
```

### 4. 결과를 확인한다

Publish TF를 누르면 내장 RViz의 손목 위에 카메라 프레임이 나타납니다. 팔을
움직여 보세요. 제대로 된 캘리브레이션은 카메라 프레임이 실제 카메라가 있는
자리에 붙어서 따라다니고, 잘못된 값은 그리퍼에서 떠 있거나 엉뚱한 방향을
가리킵니다.

따로 보려면:

```bash
ros2 launch piper_auto_handeye view_calibration.launch.py
ros2 run tf2_ros tf2_echo link6 camera_color_optical_frame
```

결과는 `~/.ros/piper_auto_handeye/handeye_<방법>_<시각>.yaml`에 저장되며, 원본
샘플과 수락/거부 사유가 담긴 `_samples.yaml`이 함께 생성됩니다.

## Launch 인자

| 인자 | 기본값 | 의미 |
|---|---|---|
| `dry_run` | `false` | `true`면 로봇을 움직이지 않고 자세만 검증 |
| `gui_lang` | `ko` | GUI 언어. 영어는 `en` |
| `can_port` | | 팔이 물린 socketcan 인터페이스. 사용 환경에 맞게 지정 |
| `wrist_camera_serial` | | 손목 카메라 시리얼. 카메라가 2대 이상일 때만 필요 |
| `calibration_method` | `TSAI` | `TSAI`, `PARK`, `HORAUD`, `ANDREFF`, `DANIILIDIS` |
| `use_gui` | `true` | rqt GUI 실행 |
| `use_realsense` | `true` | 카메라 실행 |
| `use_piper_driver` | `true` | 벤더링된 AgileX 드라이버 실행 |

전체 목록은 `ros2 launch piper_auto_handeye real_calibration.launch.py --show-args`
로 볼 수 있습니다.

카메라가 1대라면 시리얼을 비워서 넘기면 연결된 카메라를 그대로 사용합니다.

```bash
ros2 launch piper_auto_handeye real_calibration.launch.py wrist_camera_serial:=''
```

2대 이상이면 손목 카메라를 시리얼로 고정하세요. 앞의 밑줄을 반드시 유지해야
합니다. realsense2_camera가 숫자만 있는 시리얼을 정수로 해석해 매칭에 실패하기
때문입니다.

```bash
ros2 launch piper_auto_handeye real_calibration.launch.py \
  wrist_camera_serial:=_123456789012        # rs-enumerate-devices -s 로 조회
```

GUI 언어는 launch 없이도 지정할 수 있습니다.

```bash
HANDEYE_GUI_LANG=en rqt --standalone piper_auto_handeye_gui.handeye_gui_plugin.HandeyeGuiPlugin
```

## 설정

프레임과 임계값은 `piper_auto_handeye/config/`에 있습니다. 기본 프레임은
`base_link`, `link6`, `camera_color_optical_frame`, `calibration_target`입니다.

| 파일 | 내용 |
|---|---|
| `aruco.yaml` | 타겟 종류, 마커 ID와 크기, 보드 규격, 검출 튜닝 |
| `handeye.yaml` | 프레임, 솔버, 샘플 수, 자세 스위프, 복구, 임계값 |
| `piper.yaml` | 작업 공간 경계, 스텝/속도 제한 |

알아둘 점:

- `target_samples`(30)는 솔브에 들어가는 (로봇자세, 마커자세) 쌍의 개수이고,
  `observations_per_pose`(10)는 그 쌍 하나를 만들기 위해 평균내는 카메라 프레임
  수입니다. N장을 평균내면 랜덤 검출 노이즈가 약 `sqrt(N)`배 줄지만, 10장을
  넘어서면 남는 오차는 계통 오차라 평균으로 지워지지 않습니다.
- 퍼블리시되는 TF는 `camera_link`로 재타겟팅됩니다(`retarget_frame`). RealSense
  드라이버가 이미 `camera_color_optical_frame`의 부모를 갖고 있고 하나의
  프레임은 부모를 하나만 가질 수 있기 때문입니다. `base_link`에서 광학
  프레임까지의 체인은 그대로 유지됩니다.

## 자주 묻는 질문

"회전 다양성 부족"으로 솔브가 실패합니다.
: 자세들이 너무 비슷합니다. Hand-eye는 최소 두 축에 대한 회전이 필요합니다.
  내장 스위프는 이미 이를 만족하므로, 직접 자세를 넣었다면 다른 축으로 회전하는
  자세를 추가하세요.

ChArUco와 ArUco 결과가 몇 센티미터 차이 납니다.
: 거의 항상 타겟 크기가 틀린 경우입니다. 1단계를 보세요. 스케일 오차는 회전은
  정확한 채 이동값에만 나타나기 때문에 발견하기가 유난히 어렵습니다.

마커가 잘 보이는데 계속 거부됩니다.
: GUI 상태 패널의 reprojection error를 보세요. 마커가 보이는데 값이 크면
  `marker_length`가 틀렸거나 카메라 내부 파라미터가 나쁩니다. 값은 정상인데
  stability가 낮다면 팔이 아직 흔들리는 것이므로 `settle_time`을 올리세요.

팔이 "outside workspace"라며 움직이지 않습니다.
: 목표가 `config/piper.yaml`의 `workspace_min`/`workspace_max` 밖입니다.

그리퍼가 자꾸 마커를 가립니다.
: eye-in-hand에서는 정상이며 이미 처리됩니다. 손목을 조금 움직여 다시 촬영하는
  `RECOVERING` 상태로 들어갑니다. `config/handeye.yaml`의 `marker_recovery_*`로
  조절합니다.

## 안전

이 launch는 실제 모션을 명령합니다. 실행을 보호하는 것은 `config/piper.yaml`의
제한값들, 즉 직교 작업 공간 경계, 한 번의 이동 최대 거리, 속도 상한입니다.
정지는 두 종류입니다. `/stop_motion`은 현재 자세를 유지하며 GUI의 STOP 버튼이
호출하는 안전한 정지이고, `/hard_stop`은 구동 전원을 끊어 팔이 떨어지므로 그것이
덜 나쁜 상황에서만 쓰세요.

새 환경에서는 움직이게 하기 전에 `dry_run:=true`로 한 번 돌려 보세요.

## 테스트

```bash
colcon test --packages-select piper_auto_handeye piper_auto_handeye_gui
```

수학 코어(`transform_utils`, `calibration_solver`, `calibration_validator`,
`pose_filter`, `safety_validator`)는 ROS에 의존하지 않으며, 정답을 아는 합성
hand-eye 문제로 검증합니다.

## 패키지 구성

| 패키지 | 내용 |
|---|---|
| `piper_auto_handeye` | 노드, 수학 코어, 설정, launch, 테스트 |
| `piper_auto_handeye_gui` | rqt GUI 플러그인 |
| `auto_handeye_interfaces` | 메시지, 서비스, 액션 |
| `piper_auto_handeye_sim` | Gazebo 검증 리그 |
| `agx_arm_description` | RViz 로봇 모델용 벤더링 URDF와 메시 |
| `agx_arm_ctrl`, `agx_arm_msgs`, `pyagxarm_vendor` | 벤더링된 AgileX 드라이버와 SDK |

`piper_control_node`는 CAN 버스를 직접 열지 않고, 버스를 소유한 AgileX
드라이버와 대화합니다. 덕분에 캘리브레이션 정책(안전 한계, 목표 검증, 도달
판정)이 "팔에 어떻게 닿는지"와 무관한 한 곳에 모입니다.

## 벤더링된 서드파티 코드

전체 경로가 `colcon build` 한 번으로 빌드되도록 워크스페이스에 복사해 두었으며,
`pip install`이나 `PYTHONPATH` 수정이 필요 없습니다. 출처 표기는 라이선스
조건입니다.

| 디렉터리 | 업스트림 | 버전 | 라이선스 |
|---|---|---|---|
| `agx_arm_ctrl`, `agx_arm_msgs`, `agx_arm_description` | [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) (`ros2` 브랜치) | commit `91e6b2e`, 복사 2026-07-27 | Apache-2.0 |
| `pyagxarm_vendor/pyAgxArm` | [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm) | v1.0.0, commit `cc498c0`, 복사 2026-07-27 | MIT |

업스트림 `.py` 파일은 수정하지 않았습니다. `agx_arm_description`에만 메시
관련 수정 두 가지가 있습니다. Collada 비주얼을 옆에 함께 배포된 STL로
교체했고(Humble의 RViz에서 `.dae`가 로드되지 않음), 각 비주얼에 회색 재질을
선언했습니다. STL에는 색 정보가 없어 재질이 없으면 RViz가 모델 전체를 빨간색
으로 그리기 때문입니다.

## 참고 문헌

R. Tsai and R. Lenz, "A new technique for fully autonomous and efficient 3D
robotics hand/eye calibration", IEEE Transactions on Robotics and Automation,
1989.

GUI와 작업 흐름은
[easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2)의 구성을
참고했습니다.
