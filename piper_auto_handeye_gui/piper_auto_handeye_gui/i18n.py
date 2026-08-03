#!/usr/bin/env python3
"""Language selection for the calibration GUI.

The GUI ships Korean text; ``HANDEYE_GUI_LANG=en`` switches it to English.
real_calibration.launch.py exposes this as ``gui_lang:=en`` and puts it in the
rqt process environment, which is the only channel available: rqt loads the
plugin itself, so a ROS parameter would not reach the widget before it builds
its UI.

The Korean source text is the lookup key rather than an abstract id. That keeps
the widget readable in the language it was written in, and it fails safe -- a
missing entry shows Korean instead of a bare key like ``btn.start``. Strings
that interpolate values are stored as ``str.format`` templates with positional
``{}`` holes, so word order can differ between the two languages.
"""

import os

LANG = os.environ.get("HANDEYE_GUI_LANG", "ko").strip().lower()
if LANG not in ("ko", "en"):
    LANG = "ko"

_EN = {
    # ---- banners and status ----
    "dry_run 상태 확인 중...":
        "checking dry_run state...",
    "⚠  LIVE — dry_run=false : 로봇이 실제로 움직입니다. 비상정지 대기!":
        "⚠  LIVE — dry_run=false : the robot WILL move. Keep e-stop within reach!",
    "DRY-RUN — dry_run=true : 로봇은 움직이지 않습니다 (검증만)":
        "DRY-RUN — dry_run=true : the robot will not move (validation only)",

    # ---- RViz panel ----
    "RViz (로봇 + 캘리브레이션 TF)": "RViz (robot + calibration TF)",
    "RViz가 실행되지 않았습니다.\n[RViz 시작]을 누르면 이 안에 표시됩니다.":
        "RViz is not running.\nPress [Start RViz] to show it in this panel.",
    "RViz 시작": "Start RViz",
    "RViz는 이미 실행 중입니다": "RViz is already running",
    "FAIL: rviz2 실행 파일을 찾을 수 없습니다": "FAIL: rviz2 executable not found",
    "RViz 시작... 창을 GUI 안으로 가져옵니다":
        "starting RViz... adopting its window into the GUI",
    "RViz 창을 기다리는 중...": "waiting for the RViz window...",
    "RViz 프로세스가 종료되었습니다. 다시 시작하세요.":
        "the RViz process exited. Start it again.",
    "RViz 창을 찾지 못해 별도 창으로 둡니다 (기능은 동일)":
        "RViz window not found; leaving it as a separate window (same functionality)",
    "FAIL: RViz 내장 실패 ({}); 별도 창으로 사용하세요":
        "FAIL: could not embed RViz ({}); use it as a separate window",
    "RViz 실행 중 (내장됨)": "RViz running (embedded)",
    "OK: RViz를 GUI 패널 안에 내장했습니다. Publish TF를 누르면 카메라 프레임이 손목 위에 나타납니다.":
        "OK: RViz is embedded in the GUI panel. Press Publish TF and the camera "
        "frame appears on the wrist.",
    "WARN: RViz가 패널 위치와 어긋나 있습니다 (rviz@({},{}) vs 패널@({},{}))":
        "WARN: RViz is offset from its panel (rviz@({},{}) vs panel@({},{}))",

    # ---- camera panel ----
    "영상 없음\n(aruco_detector_node 미실행 또는 카메라 없음)":
        "no video\n(aruco_detector_node not running, or no camera)",

    # ---- CAN panel ----
    "로봇 연결 (CAN)": "Robot link (CAN)",
    "링크: 확인 중...": "link: checking...",
    "팔: 확인 중...": "arm: checking...",
    "CAN 연결": "Connect CAN",
    "CAN 연결됨": "CAN connected",
    "상태 확인": "Refresh",
    "링크: ● {} UP (1 Mbit/s)": "link: ● {} UP (1 Mbit/s)",
    "링크: ▲ {} UP, bitrate={} (1000000 이어야 함)":
        "link: ▲ {} UP, bitrate={} (should be 1000000)",
    "링크: ○ {} {}": "link: ○ {} {}",
    "팔: ● 피드백 수신 중": "arm: ● receiving feedback",
    "팔: ○ 피드백 없음 (전원/케이블 확인)":
        "arm: ○ no feedback (check power and cabling)",
    "연결 중...": "connecting...",
    "ERROR: can_setup.sh를 찾을 수 없습니다. piper_auto_handeye 패키지가 빌드되었는지 확인하세요.":
        "ERROR: can_setup.sh not found. Check that the piper_auto_handeye package "
        "is built.",
    "pkexec가 없어 GUI에서 권한 상승을 할 수 없습니다. 터미널에서 실행하세요:":
        "pkexec is missing, so the GUI cannot elevate privileges. Run this in a "
        "terminal instead:",
    "CAN 연결 시도: {} {} (관리자 권한 요청)":
        "bringing up CAN: {} {} (asking for admin privileges)",
    "OK: CAN 링크 활성화": "OK: CAN link up",
    "FAIL: CAN 링크 활성화 실패 (위 로그 확인)":
        "FAIL: could not bring the CAN link up (see the log above)",

    # ---- run configuration ----
    "한 자세에 멈춰 있는 동안 평균낼 마커 관측 프레임 수.\n"
    "N장을 평균내면 검출 노이즈가 약 √N배 줄어듭니다 (10장 ≈ 3배).\n"
    "Target samples와는 다른 축입니다: 이 N장이 모여 샘플 1개가 됩니다.\n"
    "너무 크게 잡으면 marker_timeout 안에 못 채워 자세를 건너뜁니다.":
        "Marker frames averaged while the arm sits still at one pose.\n"
        "Averaging N frames cuts detection noise by about sqrt(N) (10 frames "
        "= 3x).\n"
        "A different axis from Target samples: these N frames make ONE sample.\n"
        "Set it too high and the pose is skipped for missing marker_timeout.",
    "최종 솔브에 넣을 (로봇자세, 마커자세) 쌍의 개수.":
        "How many (robot pose, marker pose) pairs go into the final solve.",
    "자동 이동 (auto_move)": "Auto move (auto_move)",
    "성공 시 자동 저장": "Save automatically on success",
    "ArUco 마커": "ArUco marker",
    "ChArUco 보드": "ChArUco board",
    "마커 설정 적용": "Apply marker settings",
    "STOP 해제": "Clear STOP",
    "아직 결과 없음": "no result yet",

    # ---- ChArUco board panel ----
    "ChArUco 보드 설정 (calib.io 규격)": "ChArUco board (calib.io fields)",
    "보드 설정 적용": "Apply board settings",
    "Marker size (실측)": "Marker size (measured)",
    "calib.io의 Board width/height는 종이 크기라 입력 불필요.\n"
    "Marker size는 인쇄물의 검은 마커 한 변을 자로 재서 입력.":
        "calib.io's Board width/height is the paper size, so it is not needed "
        "here.\nMeasure one black marker edge on the print for Marker size.",

    # ---- action / service plumbing ----
    "ERROR: run_calibration 액션 서버 없음 (handeye_calibration_node 실행 중인지 확인)":
        "ERROR: no run_calibration action server (is handeye_calibration_node "
        "running?)",
    "ERROR: goal 전송 실패: {}": "ERROR: could not send the goal: {}",
    "goal이 거절되었습니다.": "the goal was rejected.",
    "goal 수락됨. 캘리브레이션 진행 중...": "goal accepted. Calibration running...",
    "ERROR: 결과 수신 실패: {}": "ERROR: could not receive the result: {}",
    "완료: {} — {}": "finished: {} — {}",
    "저장됨: {}": "saved: {}",
    "취소할 goal이 없습니다.": "there is no goal to cancel.",
    "취소 요청 전송": "cancel request sent",
    "재개": "resume",
    "일시정지": "pause",
    "캘리브레이션 YAML 선택": "Select a calibration YAML",
    "ERROR: 서비스 '{}' 사용 불가 (노드 미실행?)":
        "ERROR: service '{}' unavailable (node not running?)",
    "ERROR: {} 호출 실패: {}": "ERROR: {} call failed: {}",
    "ERROR: {} 실패: {}": "ERROR: {} failed: {}",

    # ---- detector parameter round-trips ----
    "FAIL: 검출기 파라미터 서비스에 연결할 수 없음 (마커 설정)":
        "FAIL: cannot reach the detector parameter service (marker settings)",
    "FAIL: 마커 설정 적용 실패: {}": "FAIL: could not apply marker settings: {}",
    "FAIL: 마커 설정 거부됨: {}": "FAIL: marker settings rejected: {}",
    "OK: 마커 설정 적용 -> ID {}, {} mm": "OK: marker settings applied -> ID {}, {} mm",
    "FAIL: 검출기 파라미터 서비스에 연결할 수 없음 (보드 설정)":
        "FAIL: cannot reach the detector parameter service (board settings)",
    "FAIL: 마커 크기는 체커 한 칸보다 작아야 합니다":
        "FAIL: the marker must be smaller than one checker square",
    "FAIL: Start ID {} + 마커 {}개 = 최대 ID {} 가 {} 용량({})을 넘습니다":
        "FAIL: Start ID {} + {} markers = highest ID {}, which exceeds {} "
        "(capacity {})",
    "FAIL: 보드 설정 적용 실패: {}": "FAIL: could not apply board settings: {}",
    "FAIL: 보드 설정 거부됨: {}": "FAIL: board settings rejected: {}",
    "OK: ChArUco 보드 -> {}x{}, 체커 {}mm, 마커 {}mm, Start ID {}, {}":
        "OK: ChArUco board -> {}x{}, checker {}mm, marker {}mm, Start ID {}, {}",
    "FAIL: 검출기 파라미터 서비스에 연결할 수 없음 (target={})":
        "FAIL: cannot reach the detector parameter service (target={})",
    "FAIL: target_type 변경 실패: {}": "FAIL: could not change target_type: {}",
    "OK: 캘리브레이션 타겟 -> {}": "OK: calibration target -> {}",
    "FAIL: target_type 거부됨: {}": "FAIL: target_type rejected: {}",

    # ---- live result panel ----
    "state   : LIVE (진행 중 · 자동 갱신)\n":
        "state   : LIVE (running, updates automatically)\n",
    "saved   : -  (아직 저장 전)": "saved   : -  (not saved yet)",
}


def tr(text):
    """Return ``text`` in the selected language.

    Untranslated strings fall through unchanged, so adding UI text never breaks
    the English build -- it just shows that one string in Korean.
    """
    if LANG == "en":
        return _EN.get(text, text)
    return text
