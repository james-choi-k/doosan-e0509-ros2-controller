#!/usr/bin/env python3
"""
Doosan E0509 Robot Arm Controller — ROS2 + PyQt5
"""

import sys
import threading
import time
import math
from typing import List, Optional, Tuple

# ── ROS2 imports (graceful fallback) ──────────────────
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[WARNING] ROS2 not available — simulation mode.")

try:
    from dsr_msgs2.srv import MoveLine, MoveJoint, MoveStop
    DSR_AVAILABLE = True
except ImportError:
    DSR_AVAILABLE = False

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QLineEdit, QPushButton,
    QCheckBox, QTextEdit, QComboBox, QDoubleSpinBox,
    QScrollArea, QFrame, QSplitter, QMessageBox, QProgressBar
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QTextCursor

# ── Constants ──────────────────────────────────────────
JOINT_NAMES = ['joint1','joint2','joint3','joint4','joint5','joint6']
MAX_VEL = 1.0   # m/s
MAX_ACC = 1.5   # m/s²

# ── Dark Theme QSS ─────────────────────────────────────
QSS = """
QMainWindow, QWidget {
    background-color: #1e1e2e; color: #cdd6f4;
    font-family: 'Noto Sans CJK KR', 'NanumGothic', 'DejaVu Sans', sans-serif;
    font-size: 11px;
}
QGroupBox {
    border: 1px solid #45475a; border-radius: 6px;
    margin-top: 10px; padding-top: 8px;
    font-weight: bold; color: #89b4fa;
}
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
QLineEdit, QDoubleSpinBox, QComboBox {
    background-color: #313244; border: 1px solid #45475a;
    border-radius: 4px; padding: 4px 8px; color: #cdd6f4;
}
QLineEdit:focus, QDoubleSpinBox:focus { border: 1px solid #89b4fa; }
QLineEdit:disabled { background-color: #1e1e2e; color: #585b70; }
QPushButton {
    background-color: #89b4fa; color: #1e1e2e;
    border: none; border-radius: 5px; padding: 7px 16px; font-weight: bold;
}
QPushButton:hover { background-color: #b4befe; }
QPushButton:pressed { background-color: #74c7ec; }
QPushButton:disabled { background-color: #45475a; color: #585b70; }
QPushButton#stopBtn { background-color: #f38ba8; }
QPushButton#stopBtn:hover { background-color: #eba0ac; }
QPushButton#addBtn { background-color: #a6e3a1; color: #1e1e2e; }
QPushButton#removeBtn { background-color: #fab387; color: #1e1e2e; }
QCheckBox { color: #cdd6f4; spacing: 6px; }
QCheckBox::indicator {
    width: 14px; height: 14px; border: 1px solid #45475a;
    border-radius: 3px; background-color: #313244;
}
QCheckBox::indicator:checked { background-color: #89b4fa; border-color: #89b4fa; }
QComboBox::drop-down { border: none; }
QComboBox QAbstractItemView {
    background-color: #313244; selection-background-color: #89b4fa; selection-color: #1e1e2e;
}
QTextEdit {
    background-color: #181825; border: 1px solid #45475a;
    border-radius: 4px; color: #a6e3a1;
    font-family: 'Consolas', 'Courier New', monospace; font-size: 10px;
}
QScrollArea { border: none; }
QProgressBar {
    background-color: #313244; border: 1px solid #45475a;
    border-radius: 3px; color: #1e1e2e; font-size: 9px;
}
QProgressBar::chunk { background-color: #89b4fa; border-radius: 3px; }
"""


# ════════════════════════════════════════════════════════
#  Robot Node
# ════════════════════════════════════════════════════════
class DoosanRobotNode(Node if ROS_AVAILABLE else object):
    """
    ROS2 Node for Doosan E0509.
    Provides joint state feedback and motion commands.
    Falls back to kinematic simulation when ROS2 is unavailable.
    """
    def __init__(self):
        if ROS_AVAILABLE:
            super().__init__('doosan_e0509_controller')
            self._joint_sub = self.create_subscription(
                JointState, '/dsr01/joint_states', self._joint_cb, 10)
            if DSR_AVAILABLE:
                self._move_line_client = self.create_client(
                    MoveLine, '/dsr01/motion/move_line')
                self._move_stop_client = self.create_client(
                    MoveStop, '/dsr01/motion/move_stop')
            self.get_logger().info('DoosanRobotNode initialized')
        self._joints  = [0.0] * 6            # degrees
        self._pose    = [450.0, 0.0, 200.0, 0.0, 0.0, 0.0]  # mm
        self._moving  = False
        self._connected = ROS_AVAILABLE
        self._cbs = {}
        if not ROS_AVAILABLE:
            threading.Thread(target=self._sim_loop, daemon=True).start()
            self._connected = True

    # ── ROS callbacks ───────────────────────────────────
    def _joint_cb(self, msg):
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self._joints[i] = math.degrees(msg.position[idx])
        self._fk(); self._fire('joint_state')

    # ── Simulation ──────────────────────────────────────
    def _sim_loop(self):
        t0 = time.time()
        while True:
            t = time.time() - t0
            for i in range(6):
                self._joints[i] = 5.0 * math.sin(0.3 * t + i * 0.5)
            self._fk(); self._fire('joint_state')
            time.sleep(0.05)

    def _fk(self):
        """Simplified FK for E0509 (DH approximation)."""
        j = [math.radians(a) for a in self._joints]
        L1, L2, L3 = 0.409, 0.041, 0.367
        c0, s0 = math.cos(j[0]), math.sin(j[0])
        r = (L1 * math.cos(j[1]) +
             L2 * math.cos(j[1]+j[2]) +
             L3 * math.cos(j[1]+j[2]+j[4]))
        self._pose[0] = round(c0 * r * 1000, 2)
        self._pose[1] = round(s0 * r * 1000, 2)
        self._pose[2] = round((0.1555 + L1*math.sin(j[1]) +
                               L2*math.sin(j[1]+j[2])) * 1000, 2)

    def _fire(self, event):
        if event in self._cbs:
            self._cbs[event](list(self._joints), list(self._pose))

    def register_callback(self, event, cb):
        self._cbs[event] = cb

    # ── Motion ──────────────────────────────────────────
    def move_to_targets(self, targets, velocity, acceleration, relative):
        self._moving = True
        try:
            for x, y, z in targets:
                if not self._moving:
                    return False
                if relative:
                    x += self._pose[0]; y += self._pose[1]; z += self._pose[2]
                ok = (self._moveit_move(x, y, z, velocity, acceleration)
                      if (ROS_AVAILABLE and  DSR_AVAILABLE)
                      else self._sim_move(x, y, z, velocity))
                if not ok:
                    return False
        finally:
            self._moving = False
        return True

    def _moveit_move(self, x, y, z, vel, acc):
            if not DSR_AVAILABLE:
                return self._sim_move(x, y, z, vel)

            # 서비스 준비 확인 (executor 외부에서 직접 확인)
            import time as _time
            deadline = _time.time() + 10.0
            while not self._move_line_client.service_is_ready():
                if _time.time() > deadline:
                    self.get_logger().warn('move_line 서비스 타임아웃 — 시뮬레이션으로 대체')
                    return self._sim_move(x, y, z, vel)
                _time.sleep(0.1)

            self.get_logger().info(f'MoveLine 요청: ({x:.1f},{y:.1f},{z:.1f})')
            req = MoveLine.Request()
            req.pos = [x, y, z, 0.0, 0.0, 0.0]
            req.vel = [vel * 1000, vel * 1000]
            req.acc = [acc * 1000, acc * 1000]
            req.time = 0.0
            req.radius = 0.0
            req.ref = 0
            req.mode = 0
            req.blend_type = 0
            req.sync_type = 0

            future = self._move_line_client.call_async(req)
            deadline = _time.time() + 30.0
            while not future.done():
                if _time.time() > deadline:
                    self.get_logger().error('MoveLine 응답 타임아웃')
                    return False
                _time.sleep(0.05)

            if future.result() is not None:
                self.get_logger().info(f'MoveLine 완료!')
                return True
            self.get_logger().error('MoveLine 실패')
            return False

    def _sim_move(self, tx, ty, tz, velocity):
        sx, sy, sz = self._pose[0], self._pose[1], self._pose[2]
        dist = math.sqrt((tx-sx)**2 + (ty-sy)**2 + (tz-sz)**2)
        duration = max(0.5, dist / max(velocity * 1000, 1))
        steps = max(int(duration / 0.04), 1)
        for i in range(steps + 1):
            if not self._moving:
                return False
            t = i / steps
            s = t * t * (3 - 2 * t)  # smoothstep
            self._pose[0] = sx + (tx - sx) * s
            self._pose[1] = sy + (ty - sy) * s
            self._pose[2] = sz + (tz - sz) * s
            self._fire('joint_state')
            time.sleep(0.04)
        return True

    def stop(self): self._moving = False

    @property
    def is_connected(self): return self._connected
    @property
    def is_moving(self):    return self._moving
    @property
    def joint_states(self): return list(self._joints)
    @property
    def current_pose(self): return list(self._pose)


# ════════════════════════════════════════════════════════
#  Motion Worker Thread
# ════════════════════════════════════════════════════════
class MotionWorker(QThread):
    log_sig      = pyqtSignal(str, str)
    status_sig   = pyqtSignal(bool)
    progress_sig = pyqtSignal(int, int)
    done_sig     = pyqtSignal(bool)

    def __init__(self, robot, targets, velocity, acceleration, relative):
        super().__init__()
        self._robot   = robot
        self._targets = targets
        self._vel     = velocity
        self._acc     = acceleration
        self._rel     = relative
        self._stop    = False

    def run(self):
        self.status_sig.emit(True)
        mode = "상대" if self._rel else "절대"
        self.log_sig.emit(
            f"▶ {len(self._targets)}개 목표 이동 시작 "
            f"({mode}, V={self._vel:.2f}m/s, A={self._acc:.2f}m/s²)", "info")
        ok = True
        for i, (x, y, z) in enumerate(self._targets):
            if self._stop:
                self.log_sig.emit("■ 중단됨", "warn"); ok = False; break
            self.progress_sig.emit(i, len(self._targets))
            self.log_sig.emit(
                f"→ 목표 {i+1}/{len(self._targets)}: ({x:.1f},{y:.1f},{z:.1f})mm", "info")
            if self._robot.move_to_targets([(x,y,z)], self._vel, self._acc, self._rel):
                p = self._robot.current_pose
                self.log_sig.emit(
                    f"✔ 도달: ({p[0]:.1f},{p[1]:.1f},{p[2]:.1f})mm", "success")
            else:
                self.log_sig.emit(f"✖ 실패: 목표 {i+1}", "error"); ok = False; break
        self.progress_sig.emit(len(self._targets), len(self._targets))
        self.status_sig.emit(False); self.done_sig.emit(ok)

    def request_stop(self):
        self._stop = True; self._robot.stop()


# ════════════════════════════════════════════════════════
#  Target Row Widget
# ════════════════════════════════════════════════════════
class TargetRow(QWidget):
    removed = pyqtSignal(object)

    def __init__(self, idx, parent=None):
        super().__init__(parent)
        self._idx = idx
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0,2,0,2); lay.setSpacing(5)
        self._lbl = QLabel(f"#{idx+1}")
        self._lbl.setFixedWidth(26); self._lbl.setAlignment(Qt.AlignCenter)
        self._lbl.setStyleSheet("color:#89b4fa;font-weight:bold;")
        self._x = QLineEdit("0.0"); self._x.setFixedWidth(80)
        self._y = QLineEdit("0.0"); self._y.setFixedWidth(80)
        self._z = QLineEdit("0.0"); self._z.setFixedWidth(80)
        btn = QPushButton("✕"); btn.setObjectName("removeBtn")
        btn.setFixedSize(24,24); btn.clicked.connect(lambda: self.removed.emit(self))
        for w, lbl in zip([self._x,self._y,self._z],["X:","Y:","Z:"]):
            lay.addWidget(QLabel(lbl)); lay.addWidget(w)
        lay.addWidget(self._lbl); lay.addWidget(btn); lay.addStretch()

    def values(self):
        try:
            return float(self._x.text()), float(self._y.text()), float(self._z.text())
        except ValueError:
            raise ValueError(f"목표지점 #{self._idx+1}: 숫자를 입력해주세요.")

    def set_index(self, i):
        self._idx = i; self._lbl.setText(f"#{i+1}")


# ════════════════════════════════════════════════════════
#  Main Window
# ════════════════════════════════════════════════════════
class MainWindow(QMainWindow):
    def __init__(self, robot: DoosanRobotNode):
        super().__init__()
        self._robot  = robot
        self._worker: Optional[MotionWorker] = None
        self._rows:   List[TargetRow] = []
        self.setWindowTitle("Doosan E0509 — ROS2 Robot Arm Controller")
        self.setMinimumSize(1080, 720); self.resize(1200, 780)
        self.setStyleSheet(QSS)
        self._build()
        self._robot.register_callback('joint_state', self._on_robot_data)
        self._timer = QTimer(self); self._timer.timeout.connect(self._refresh)
        self._timer.start(100)

    # ── Build ────────────────────────────────────────────
    def _build(self):
        c = QWidget(); self.setCentralWidget(c)
        root = QVBoxLayout(c); root.setContentsMargins(12,12,12,8); root.setSpacing(8)
        title = QLabel("🤖  Doosan E0509 — Robot Arm Controller")
        title.setStyleSheet("font-size:15px;font-weight:bold;color:#89b4fa;padding:4px 0;")
        root.addWidget(title)
        sp = QSplitter(Qt.Horizontal); sp.setHandleWidth(5)
        sp.setStyleSheet("QSplitter::handle{background-color:#45475a;}")
        sp.addWidget(self._left()); sp.addWidget(self._right()); sp.setSizes([500,580])
        root.addWidget(sp, 1)

    def _left(self):
        w = QWidget(); lay = QVBoxLayout(w)
        lay.setContentsMargins(0,0,6,0); lay.setSpacing(10)
        lay.addWidget(self._grp_coord())
        lay.addWidget(self._grp_targets())
        lay.addWidget(self._grp_params())
        lay.addWidget(self._grp_actions())
        lay.addStretch(); return w

    def _grp_coord(self):
        g = QGroupBox("📍 좌표 기준 설정"); lay = QHBoxLayout(g)
        self._coord_cb = QComboBox()
        self._coord_cb.addItems([
            "절대 좌표 (Absolute) — Base 프레임 기준",
            "상대 좌표 (Relative) — 현재 위치 기준 증분"])
        lay.addWidget(QLabel("좌표계:")); lay.addWidget(self._coord_cb, 1)
        return g

    def _grp_targets(self):
        g = QGroupBox("🎯 목표지점 목록  [mm]"); lay = QVBoxLayout(g)
        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        scroll.setMinimumHeight(150); scroll.setMaximumHeight(240)
        scroll.setFrameShape(QFrame.NoFrame)
        self._rows_container = QWidget()
        self._rows_lay = QVBoxLayout(self._rows_container)
        self._rows_lay.setContentsMargins(4,4,4,4); self._rows_lay.setSpacing(3)
        self._rows_lay.addStretch()
        scroll.setWidget(self._rows_container); lay.addWidget(scroll)
        btns = QHBoxLayout()
        add = QPushButton("＋  목표지점 추가"); add.setObjectName("addBtn")
        add.clicked.connect(self._add_row)
        clr = QPushButton("전체 삭제"); clr.setObjectName("removeBtn")
        clr.clicked.connect(self._clear_rows)
        btns.addWidget(add); btns.addWidget(clr); btns.addStretch()
        lay.addLayout(btns); self._add_row(); return g

    def _grp_params(self):
        g = QGroupBox("⚙️  이동 파라미터"); lay = QGridLayout(g); lay.setSpacing(8)
        self._vel_chk = QCheckBox("최대 속도"); self._vel_chk.setChecked(True)
        self._vel_sp = QDoubleSpinBox(); self._vel_sp.setRange(0.01, MAX_VEL)
        self._vel_sp.setValue(0.3); self._vel_sp.setSuffix("  m/s")
        self._acc_chk = QCheckBox("최대 가속도"); self._acc_chk.setChecked(True)
        self._acc_sp = QDoubleSpinBox(); self._acc_sp.setRange(0.01, MAX_ACC)
        self._acc_sp.setValue(0.5); self._acc_sp.setSuffix("  m/s²")
        self._vel_chk.stateChanged.connect(lambda s: self._vel_sp.setEnabled(bool(s)))
        self._acc_chk.stateChanged.connect(lambda s: self._acc_sp.setEnabled(bool(s)))
        lay.addWidget(self._vel_chk,0,0); lay.addWidget(self._vel_sp,0,1)
        lay.addWidget(self._acc_chk,1,0); lay.addWidget(self._acc_sp,1,1)
        return g

    def _grp_actions(self):
        g = QGroupBox("▶  실행 제어"); lay = QVBoxLayout(g); lay.setSpacing(8)
        row = QHBoxLayout()
        self._exec_btn = QPushButton("▶  실행 (Execute)"); self._exec_btn.setFixedHeight(38)
        self._exec_btn.clicked.connect(self._execute)
        self._stop_btn = QPushButton("■  정지 (Stop)"); self._stop_btn.setObjectName("stopBtn")
        self._stop_btn.setFixedHeight(38); self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._stop)
        row.addWidget(self._exec_btn,2); row.addWidget(self._stop_btn,1); lay.addLayout(row)
        self._prog = QProgressBar(); self._prog.setFixedHeight(12); self._prog.setValue(0)
        self._prog.setFormat(""); lay.addWidget(self._prog)
        return g

    def _right(self):
        w = QWidget(); lay = QVBoxLayout(w)
        lay.setContentsMargins(6,0,0,0); lay.setSpacing(10)
        lay.addWidget(self._grp_status())
        lay.addWidget(self._grp_pose())
        lay.addWidget(self._grp_joints())
        lay.addWidget(self._grp_log(), 1)
        return w

    def _grp_status(self):
        g = QGroupBox("🔌 연결 / 동작 상태"); lay = QGridLayout(g)
        lay.addWidget(QLabel("로봇 연결:"),0,0)
        self._conn_lbl = QLabel("● 연결됨"); lay.addWidget(self._conn_lbl,0,1)
        lay.addWidget(QLabel("동작 상태:"),0,2)
        self._motion_lbl = QLabel("● 정지"); lay.addWidget(self._motion_lbl,0,3)
        lay.addWidget(QLabel("구동 모드:"),1,0)
        mode = ("🟡 시뮬레이션 모드 (ROS2 미연결)"
                if not ROS_AVAILABLE else "🟢 ROS2 실제 연결됨")
        ml = QLabel(mode)
        ml.setStyleSheet("color:#f9e2af;" if not ROS_AVAILABLE else "color:#a6e3a1;")
        lay.addWidget(ml,1,1,1,3); return g

    def _grp_pose(self):
        g = QGroupBox("📌 현재 End-Effector 위치  (Base 절대 좌표)")
        lay = QGridLayout(g); lay.setSpacing(8); self._pose_lbl = {}
        for col, axis in enumerate(['X','Y','Z']):
            n = QLabel(f"{axis}:"); n.setStyleSheet("color:#89b4fa;font-weight:bold;")
            lay.addWidget(n,0,col*2)
            v = QLabel("0.00 mm")
            v.setStyleSheet("color:#cdd6f4;background:#313244;border-radius:3px;padding:2px 8px;")
            v.setMinimumWidth(95); lay.addWidget(v,0,col*2+1)
            self._pose_lbl[axis] = v
        return g

    def _grp_joints(self):
        g = QGroupBox("🔩 현재 Joint 각도  [°]")
        lay = QGridLayout(g); lay.setSpacing(6); self._joint_lbl = {}
        for i in range(6):
            r,c = divmod(i,3)
            n = QLabel(f"J{i+1}:"); n.setStyleSheet("color:#89b4fa;font-weight:bold;")
            lay.addWidget(n,r,c*2)
            v = QLabel("0.00°")
            v.setStyleSheet("color:#cdd6f4;background:#313244;border-radius:3px;padding:2px 6px;")
            v.setMinimumWidth(70); lay.addWidget(v,r,c*2+1); self._joint_lbl[i] = v
        return g

    def _grp_log(self):
        g = QGroupBox("📋 실시간 로그"); lay = QVBoxLayout(g)
        self._log = QTextEdit(); self._log.setReadOnly(True)
        self._log.setMinimumHeight(160); lay.addWidget(self._log)
        clr = QPushButton("로그 초기화"); clr.setFixedWidth(100)
        clr.clicked.connect(self._log.clear)
        lay.addWidget(clr, alignment=Qt.AlignRight); return g

    # ── Target rows ─────────────────────────────────────
    def _add_row(self):
        r = TargetRow(len(self._rows), self); r.removed.connect(self._del_row)
        self._rows_lay.insertWidget(self._rows_lay.count()-1, r)
        self._rows.append(r)
        if hasattr(self, '_log'):
            self._log_msg(f"목표지점 #{len(self._rows)} 추가", "info")

    def _del_row(self, r):
        if len(self._rows) <= 1:
            QMessageBox.warning(self,"경고","최소 1개의 목표지점이 필요합니다."); return
        self._rows_lay.removeWidget(r); r.deleteLater(); self._rows.remove(r)
        for i, row in enumerate(self._rows): row.set_index(i)
        self._log_msg("목표지점 삭제됨", "warn")

    def _clear_rows(self):
        while len(self._rows) > 1:
            r = self._rows.pop(); self._rows_lay.removeWidget(r); r.deleteLater()
        if self._rows: self._rows[0].set_index(0)
        self._log_msg("전체 목표지점 초기화", "warn")

    # ── Execute ──────────────────────────────────────────
    def _execute(self):
        targets = []
        for r in self._rows:
            try: targets.append(r.values())
            except ValueError as e: QMessageBox.critical(self,"입력 오류",str(e)); return
        vel = self._vel_sp.value() if self._vel_chk.isChecked() else 0.3
        acc = self._acc_sp.value() if self._acc_chk.isChecked() else 0.5
        rel = self._coord_cb.currentIndex() == 1
        self._exec_btn.setEnabled(False); self._stop_btn.setEnabled(True); self._prog.setValue(0)
        self._worker = MotionWorker(self._robot, targets, vel, acc, rel)
        self._worker.log_sig.connect(self._log_msg)
        self._worker.status_sig.connect(lambda m: (
            self._exec_btn.setEnabled(not m), self._stop_btn.setEnabled(m)))
        self._worker.progress_sig.connect(
            lambda d,t: self._prog.setValue(int(d*100/t) if t else 0))
        self._worker.done_sig.connect(
            lambda ok: (self._log_msg("✅ 완료" if ok else "⚠️ 종료", "success" if ok else "warn"),
                        self._prog.setValue(100) if ok else None))
        self._worker.start()

    def _stop(self):
        if self._worker and self._worker.isRunning():
            self._worker.request_stop(); self._log_msg("■ 정지 요청", "warn")

    # ── Robot callback (from non-GUI thread) ─────────────
    def _on_robot_data(self, joints, pose): pass  # data read directly via _refresh

    def _refresh(self):
        # Connection / motion
        if self._robot.is_connected:
            self._conn_lbl.setText("● 연결됨")
            self._conn_lbl.setStyleSheet("color:#a6e3a1;font-weight:bold;")
        else:
            self._conn_lbl.setText("● 연결 끊김")
            self._conn_lbl.setStyleSheet("color:#f38ba8;font-weight:bold;")
        if self._robot.is_moving:
            self._motion_lbl.setText("● 이동 중...")
            self._motion_lbl.setStyleSheet("color:#f9e2af;font-weight:bold;")
        else:
            self._motion_lbl.setText("● 정지")
            self._motion_lbl.setStyleSheet("color:#89dceb;font-weight:bold;")
        # Joints
        for i, v in enumerate(self._robot.joint_states):
            self._joint_lbl[i].setText(f"{v:.2f}°")
        # Pose
        for i, ax in enumerate(['X','Y','Z']):
            pose = self._robot.current_pose
            if i < len(pose): self._pose_lbl[ax].setText(f"{pose[i]:.2f} mm")

    def _log_msg(self, msg, level="info"):
        colors = {"info":"#cdd6f4","success":"#a6e3a1","warn":"#f9e2af","error":"#f38ba8"}
        ts = time.strftime("%H:%M:%S")
        self._log.append(
            f'<span style="color:#585b70">[{ts}]</span> '
            f'<span style="color:{colors.get(level,"#cdd6f4")}">{msg}</span>')
        c = self._log.textCursor(); c.movePosition(QTextCursor.End); self._log.setTextCursor(c)

    def closeEvent(self, event):
        if self._worker and self._worker.isRunning():
            self._worker.request_stop(); self._worker.wait(2000)
        event.accept()


# ════════════════════════════════════════════════════════
#  Entry Point
# ════════════════════════════════════════════════════════
def main(args=None):
    if ROS_AVAILABLE:
        rclpy.init(args=args)

    robot = DoosanRobotNode()

    if ROS_AVAILABLE:
        executor = MultiThreadedExecutor()
        executor.add_node(robot)
        threading.Thread(target=executor.spin, daemon=True).start()

    app = QApplication(sys.argv)
    app.setApplicationName("Doosan E0509 Controller")
    win = MainWindow(robot)
    win.show()
    code = app.exec_()

    if ROS_AVAILABLE:
        robot.destroy_node()
        rclpy.shutdown()

    sys.exit(code)


if __name__ == '__main__':
    main()