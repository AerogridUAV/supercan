#!/usr/bin/env python3
"""
Feetech Instruction GUI
=======================

Simple PyQt GUI to send com.feetech.servo.Instruction messages over DroneCAN
(UAVCAN v0) using the DSDL set contained in this repository.

Features:
 - Select CAN iface (default can0)
 - Configure local node ID
 - Set actuator_id
 - Choose instruction message type (ARM/DISARM, SET_TARGET_ANGLE, SET_SERIAL_FREQ, SET_CAN_FREQ, SET_LOG_LEVEL)
 - Contextual input widgets for each type
 - Send button + convenience quick buttons (Arm, Disarm)
 - Periodic heartbeat (GetNodeInfo auto-handled by dronecan lib)

Requirements:
 pip install dronecan PyQt5 (or PySide2 fallback)

Note: This script expects DSDL root at firmware/lib/DSDL relative to repo root.
"""

import os
import sys
import threading
import time
import traceback
import logging

## Mirror-related imports removed
from typing import Optional

try:
    from PyQt5 import QtWidgets
except ImportError:
    try:
        from PySide2 import QtWidgets  # type: ignore
    except ImportError as e:  # pragma: no cover
        print("PyQt5 or PySide2 required: pip install PyQt5", file=sys.stderr)
        raise

import dronecan

LOG = logging.getLogger("feetech_gui")
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")


# DSDL message info (from 20110.Instruction.uavcan)
MSG_FULL_NAME = "com.feetech.servo.Instruction"
MSG_ID = 20110  # inferred from filename 20110.Instruction.uavcan

# Guard to avoid re-loading DSDL (which causes duplicate DTID errors if standard namespaces overlap)
DSDL_LOADED = False


class DroneCANNodeWrapper:
    def __init__(self, iface: str, node_id: int, dsdl_root: str):
        self.iface = iface
        self.node_id = node_id
        self.dsdl_root = dsdl_root
        self.node = None
        self._spin_thread = None
        self._stop = threading.Event()
        # mirror no longer used

    def _split_dsdl_paths(self) -> list[str]:
        # Accept ':' or ',' separated list
        raw = self.dsdl_root
        raw = raw.replace(",", os.pathsep)
        parts = []
        for seg in raw.split(os.pathsep):
            seg = os.path.expanduser(seg.strip())
            if seg:
                parts.append(seg)
        return parts

    def start(self):
        LOG.info("Loading DSDL from %s", self.dsdl_root)
        paths = self._split_dsdl_paths()
        global DSDL_LOADED
        if not DSDL_LOADED:
            # Strategy: For each user-supplied path, derive namespace root dirs (e.g. /path/com) and load those.
            # DroneCAN forbids nested search dirs; we must pass only the top-level namespace directories, not their parent.
            def find_namespace_roots(base: str):
                roots = []
                if not os.path.isdir(base):
                    return roots

                # Look at immediate children for lowercase namespace dirs.
                for child in sorted(os.listdir(base)):
                    if child.startswith("."):
                        continue
                    child_path = os.path.join(base, child)
                    if not os.path.isdir(child_path):
                        continue
                    if child in {"tests", "test", "__pycache__", "LICENSE"}:
                        continue
                    if not child.islower():
                        continue
                    # Check if contains at least one .uavcan file in subtree.
                    has = False
                    for r, _d, files in os.walk(child_path):
                        if any(f.endswith(".uavcan") for f in files):
                            has = True
                            break
                    if has:
                        roots.append(child_path)

                # If no child namespaces found, check if base itself is a namespace directory
                if not roots:
                    base_name = os.path.basename(os.path.normpath(base))
                    if base_name.islower():
                        # Heuristic: contains .uavcan somewhere beneath.
                        for r, _dirs, files in os.walk(base):
                            if any(f.endswith(".uavcan") for f in files):
                                roots.append(base)
                                break
                return roots

            all_roots = []
            for p in paths:
                roots = find_namespace_roots(p)
                if roots:
                    LOG.info("Derived namespace roots from %s: %s", p, roots)
                    all_roots.extend(roots)
                else:
                    LOG.warning("No namespace roots discovered under %s", p)
            # Filter only those relevant to our message (contain /com/ or end with com)
            relevant = []
            for r in all_roots:
                if os.path.basename(r) == "com" or "/com/" in r.replace("\\", "/"):
                    relevant.append(r)
            if not relevant:
                LOG.warning("No 'com' namespace root found; using all discovered roots")
                relevant = all_roots
            # Deduplicate
            seen = set()
            dedup = []
            for r in relevant:
                if r not in seen:
                    seen.add(r)
                    dedup.append(r)
            if not dedup:
                raise RuntimeError("No DSDL namespace roots found to load.")
            LOG.info("Loading DSDL namespace roots: %s", dedup)
            try:
                # dronecan.load_dsdl expects individual path arguments, not a list
                dronecan.load_dsdl(*dedup)
            except Exception as ex:  # noqa: E722
                raise RuntimeError(f"Failed loading DSDL roots {dedup}: {ex}") from ex
            LOG.info("DSDL loaded successfully (%d roots)", len(dedup))
            DSDL_LOADED = True
        else:
            LOG.info("DSDL already loaded; skipping reload")
        node_info = dronecan.uavcan.protocol.GetNodeInfo.Response()
        node_info.name = "org.tudelft.feetech_gui"
        node_info.software_version.major = 1
        node_info.software_version.minor = 0
        self.node = dronecan.make_node(
            self.iface, node_id=self.node_id, node_info=node_info
        )
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        LOG.info("DroneCAN node started on %s id=%d", self.iface, self.node_id)

    def _spin(self):
        while not self._stop.is_set():
            try:
                self.node.spin(0.01)
            except (
                Exception
            ):  # noqa: E722  # pragma: no cover (library spin errors vary)
                LOG.error("Spin error:\n%s", traceback.format_exc())
                time.sleep(0.2)

    def close(self):
        self._stop.set()
        if self._spin_thread:
            self._spin_thread.join(timeout=1)
        if self.node:
            try:
                self.node.close()
            except Exception:  # noqa: E722
                pass

    def make_instruction(self):
        # Dynamic lookup after dsdl loaded
        parts = MSG_FULL_NAME.split(".")
        mod = dronecan
        for p in parts:
            mod = getattr(mod, p)
        return mod()

    def broadcast(self, msg):
        # Broadcast with default priority
        self.node.broadcast(msg)


class InstructionWidget(QtWidgets.QWidget):
    def __init__(
        self,
        default_dsdl_path: Optional[str] = None,
        default_iface: str = "can0",
        default_node_id: int = 100,
    ):
        super().__init__()
        self.setWindowTitle("Feetech Instruction Sender")
        self.node_wrapper = None
        self.default_dsdl_path = default_dsdl_path
        self.default_iface = default_iface
        self.default_node_id = default_node_id
        self._build_ui()

    # UI Construction -------------------------------------------------
    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        # Connection group
        conn_group = QtWidgets.QGroupBox("Connection")
        conn_form = QtWidgets.QFormLayout(conn_group)
        self.iface_edit = QtWidgets.QLineEdit(self.default_iface)
        self.node_id_spin = QtWidgets.QSpinBox()
        self.node_id_spin.setRange(1, 255)
        self.node_id_spin.setValue(self.default_node_id)
        # DSDL path
        self.dsdl_path_edit = QtWidgets.QLineEdit(
            self.default_dsdl_path or self._auto_detect_dsdl()
        )
        browse_btn = QtWidgets.QPushButton("...")
        browse_btn.setToolTip("Browse for DSDL root directory")

        def browse():
            path = QtWidgets.QFileDialog.getExistingDirectory(
                self, "Select DSDL Root", self.dsdl_path_edit.text()
            )
            if path:
                self.dsdl_path_edit.setText(path)

        browse_btn.clicked.connect(browse)
        dsdl_row = QtWidgets.QHBoxLayout()
        dsdl_row.addWidget(self.dsdl_path_edit)
        dsdl_row.addWidget(browse_btn)
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.on_connect)
        conn_form.addRow("Interface", self.iface_edit)
        conn_form.addRow("Node ID", self.node_id_spin)
        conn_form.addRow("DSDL Path", dsdl_row)
        conn_form.addRow(self.connect_btn)
        layout.addWidget(conn_group)

        # Instruction group - redesigned with individual rows
        instr_group = QtWidgets.QGroupBox("Instructions")
        instr_layout = QtWidgets.QVBoxLayout(instr_group)

        # Actuator ID (shared across all commands)
        actuator_row = QtWidgets.QHBoxLayout()
        actuator_row.addWidget(QtWidgets.QLabel("Actuator ID:"))
        self.actuator_id_spin = QtWidgets.QSpinBox()
        self.actuator_id_spin.setRange(0, 255)
        self.actuator_id_spin.setValue(0)
        actuator_row.addWidget(self.actuator_id_spin)
        actuator_row.addStretch()
        instr_layout.addLayout(actuator_row)

        # Separator
        line1 = QtWidgets.QFrame()
        line1.setFrameShape(QtWidgets.QFrame.HLine)
        line1.setFrameShadow(QtWidgets.QFrame.Sunken)
        instr_layout.addWidget(line1)

        # Row 1: Arm/Disarm buttons
        arm_row = QtWidgets.QHBoxLayout()
        arm_row.addWidget(QtWidgets.QLabel("Servo Control:"))
        self.arm_btn = QtWidgets.QPushButton("Arm")
        self.disarm_btn = QtWidgets.QPushButton("Disarm")
        self.arm_btn.clicked.connect(lambda: self.send_command("ARM", True))
        self.disarm_btn.clicked.connect(lambda: self.send_command("ARM", False))
        arm_row.addWidget(self.arm_btn)
        arm_row.addWidget(self.disarm_btn)
        arm_row.addStretch()
        instr_layout.addLayout(arm_row)

        # Row 2: Log level buttons
        log_row = QtWidgets.QHBoxLayout()
        log_row.addWidget(QtWidgets.QLabel("Log Level:"))
        self.short_log_btn = QtWidgets.QPushButton("Short Log")
        self.long_log_btn = QtWidgets.QPushButton("Long Log")
        self.short_log_btn.clicked.connect(lambda: self.send_command("LOG_LEVEL", 0))
        self.long_log_btn.clicked.connect(lambda: self.send_command("LOG_LEVEL", 1))
        log_row.addWidget(self.short_log_btn)
        log_row.addWidget(self.long_log_btn)
        log_row.addStretch()
        instr_layout.addLayout(log_row)

        # Row 3: Set target angle
        angle_row = QtWidgets.QHBoxLayout()
        angle_row.addWidget(QtWidgets.QLabel("Target Angle:"))
        self.angle_spin = QtWidgets.QSpinBox()
        self.angle_spin.setRange(-32768, 32767)
        self.angle_spin.setValue(0)
        self.angle_spin.setSuffix(" °")
        angle_row.addWidget(self.angle_spin)
        self.angle_send_btn = QtWidgets.QPushButton("Set Angle")
        self.angle_send_btn.clicked.connect(
            lambda: self.send_command("ANGLE", self.angle_spin.value())
        )
        angle_row.addWidget(self.angle_send_btn)
        angle_row.addStretch()
        instr_layout.addLayout(angle_row)

        # Row 4: Serial frequency
        serial_row = QtWidgets.QHBoxLayout()
        serial_row.addWidget(QtWidgets.QLabel("Serial Freq:"))
        self.serial_freq_combo = QtWidgets.QComboBox()
        self.serial_freq_combo.setEditable(True)
        self.serial_freq_combo.addItems(["10", "20", "50", "100", "200"])
        self.serial_freq_combo.setCurrentText("50")
        self.serial_freq_combo.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed
        )
        serial_row.addWidget(self.serial_freq_combo)
        serial_row.addWidget(QtWidgets.QLabel("Hz"))
        self.serial_send_btn = QtWidgets.QPushButton("Set Serial Freq")
        self.serial_send_btn.clicked.connect(
            lambda: self.send_command(
                "SERIAL_FREQ", int(self.serial_freq_combo.currentText())
            )
        )
        serial_row.addWidget(self.serial_send_btn)
        instr_layout.addLayout(serial_row)

        # Row 5: CAN frequency
        can_row = QtWidgets.QHBoxLayout()
        can_row.addWidget(QtWidgets.QLabel("CAN Freq:"))
        self.can_freq_combo = QtWidgets.QComboBox()
        self.can_freq_combo.setEditable(True)
        self.can_freq_combo.addItems(["1", "5", "10", "20", "50"])
        self.can_freq_combo.setCurrentText("10")
        self.can_freq_combo.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed
        )
        can_row.addWidget(self.can_freq_combo)
        can_row.addWidget(QtWidgets.QLabel("Hz"))
        self.can_send_btn = QtWidgets.QPushButton("Set CAN Freq")
        self.can_send_btn.clicked.connect(
            lambda: self.send_command(
                "CAN_FREQ", int(self.can_freq_combo.currentText())
            )
        )
        can_row.addWidget(self.can_send_btn)
        instr_layout.addLayout(can_row)

        # Row 6: Set min angle
        min_angle_row = QtWidgets.QHBoxLayout()
        min_angle_row.addWidget(QtWidgets.QLabel("Min Angle:"))
        self.min_angle_spin = QtWidgets.QSpinBox()
        self.min_angle_spin.setRange(-32768, 32767)
        self.min_angle_spin.setValue(0)
        self.min_angle_spin.setSuffix(" °")
        min_angle_row.addWidget(self.min_angle_spin)
        self.min_angle_send_btn = QtWidgets.QPushButton("Set Min Angle")
        self.min_angle_send_btn.clicked.connect(
            lambda: self.send_command("MIN_ANGLE", self.min_angle_spin.value())
        )
        min_angle_row.addWidget(self.min_angle_send_btn)
        min_angle_row.addStretch()
        instr_layout.addLayout(min_angle_row)

        # Row 7: Set max angle
        max_angle_row = QtWidgets.QHBoxLayout()
        max_angle_row.addWidget(QtWidgets.QLabel("Max Angle:"))
        self.max_angle_spin = QtWidgets.QSpinBox()
        self.max_angle_spin.setRange(-32768, 32767)
        self.max_angle_spin.setValue(180)
        self.max_angle_spin.setSuffix(" °")
        max_angle_row.addWidget(self.max_angle_spin)
        self.max_angle_send_btn = QtWidgets.QPushButton("Set Max Angle")
        self.max_angle_send_btn.clicked.connect(
            lambda: self.send_command("MAX_ANGLE", self.max_angle_spin.value())
        )
        max_angle_row.addWidget(self.max_angle_send_btn)
        max_angle_row.addStretch()
        instr_layout.addLayout(max_angle_row)

        layout.addWidget(instr_group)

        # Log output
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        layout.addWidget(self.log_view)

    # UI Callbacks ----------------------------------------------------
    def append_log(self, text: str):
        self.log_view.appendPlainText(text)
        self.log_view.verticalScrollBar().setValue(
            self.log_view.verticalScrollBar().maximum()
        )

    def on_connect(self):
        if self.node_wrapper:
            self.node_wrapper.close()
            self.node_wrapper = None
            self.connect_btn.setText("Connect")
            self.append_log("Disconnected")
            return
        iface = self.iface_edit.text().strip()
        node_id = self.node_id_spin.value()
        dsdl_root = self.dsdl_path_edit.text().strip()
        dsdl_root = os.path.expanduser(dsdl_root)
        if not dsdl_root:
            dsdl_root = self._auto_detect_dsdl()
        if not os.path.isdir(dsdl_root):
            self.append_log(f"Invalid DSDL path: {dsdl_root}")
            return
        try:
            self.node_wrapper = DroneCANNodeWrapper(iface, node_id, dsdl_root)
            self.node_wrapper.start()
            self.connect_btn.setText("Disconnect")
            self.append_log(f"Connected on {iface} node_id={node_id}")
        except Exception as e:  # noqa: E722
            self.append_log(f"Connect failed: {e}")
            LOG.error("Connect failed", exc_info=True)
            self.node_wrapper = None

    def send_command(self, cmd_type: str, value):
        """Send a specific command with given value"""
        if not self.node_wrapper:
            self.append_log("Not connected")
            return

        try:
            m = self.node_wrapper.make_instruction()
            m.actuator_id = self.actuator_id_spin.value()
            data = [0, 0, 0, 0]

            if cmd_type == "ARM":
                m.message_type = 0  # ARM_DISARM
                data[0] = 1 if value else 0
                cmd_text = "Arm" if value else "Disarm"
            elif cmd_type == "LOG_LEVEL":
                m.message_type = 4  # SET_LOG_LEVEL
                data[0] = value  # 0=short, 1=long
                cmd_text = f"Set Log Level {'Long' if value else 'Short'}"
            elif cmd_type == "ANGLE":
                m.message_type = 1  # SET_TARGET_ANGLE
                angle = int(value) & 0xFFFF
                data[0] = angle & 0xFF
                data[1] = (angle >> 8) & 0xFF
                cmd_text = f"Set Target Angle {value}°"
            elif cmd_type == "SERIAL_FREQ":
                m.message_type = 2  # SET_SERIAL_FREQ
                hz = int(value) & 0xFFFF
                data[0] = hz & 0xFF
                data[1] = (hz >> 8) & 0xFF
                cmd_text = f"Set Serial Freq {value}Hz"
            elif cmd_type == "CAN_FREQ":
                m.message_type = 3  # SET_CAN_FREQ
                hz = int(value) & 0xFFFF
                data[0] = hz & 0xFF
                data[1] = (hz >> 8) & 0xFF
                cmd_text = f"Set CAN Freq {value}Hz"
            elif cmd_type == "MIN_ANGLE":
                m.message_type = 5  # SET_MIN_ANGLE
                angle = int(value) & 0xFFFF
                data[0] = angle & 0xFF
                data[1] = (angle >> 8) & 0xFF
                cmd_text = f"Set Min Angle {value}°"
            elif cmd_type == "MAX_ANGLE":
                m.message_type = 6  # SET_MAX_ANGLE
                angle = int(value) & 0xFFFF
                data[0] = angle & 0xFF
                data[1] = (angle >> 8) & 0xFF
                cmd_text = f"Set Max Angle {value}°"
            else:
                self.append_log(f"Unknown command: {cmd_type}")
                return

            m.data = data
            self.node_wrapper.broadcast(m)
            self.append_log(
                f"Sent: {cmd_text} (data={list(m.data)} actuator={m.actuator_id})"
            )

        except ValueError as e:
            self.append_log(f"Invalid value for {cmd_type}: {e}")
        except Exception as e:  # noqa: E722
            self.append_log(f"Send {cmd_type} failed: {e}")
            LOG.error("Send failed", exc_info=True)

    # Cleanup ---------------------------------------------------------
    def closeEvent(self, event):  # noqa: N802
        if self.node_wrapper:
            self.node_wrapper.close()
        event.accept()

    def _auto_detect_dsdl(self) -> str:
        preferred = os.path.expanduser("~/uavcan_vendor_specific_types/")
        if os.path.isdir(preferred):
            return preferred
        fallback = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "firmware", "lib", "DSDL")
        )
        return fallback


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Feetech DroneCAN Instruction GUI")
    parser.add_argument("--iface", default="can0", help="CAN interface (default can0)")
    parser.add_argument(
        "--node-id", type=int, default=100, help="Local node ID (1-125)"
    )
    parser.add_argument(
        "--dsdl", default=None, help="Explicit DSDL root path (overrides auto-detect)"
    )
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    w = InstructionWidget(
        default_dsdl_path=args.dsdl,
        default_iface=args.iface,
        default_node_id=args.node_id,
    )
    w.resize(720, 540)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
