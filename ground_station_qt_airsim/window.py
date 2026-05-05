from __future__ import annotations

import json
import re
from datetime import datetime

try:
    from PySide6.QtCore import QObject, Qt, QTimer, QUrl, Signal, Slot
    from PySide6.QtGui import QAction, QColor
    from PySide6.QtWidgets import (
        QApplication,
        QComboBox,
        QDoubleSpinBox,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QHeaderView,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QScrollArea,
        QSizePolicy,
        QTableWidget,
        QTableWidgetItem,
        QTextEdit,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )
    try:
        from PySide6.QtWebChannel import QWebChannel
        from PySide6.QtWebEngineWidgets import QWebEngineView
    except Exception:
        QWebChannel = None
        QWebEngineView = None
except Exception as exc:  # pragma: no cover
    raise RuntimeError("PySide6 is required for the AirSim Qt ground station.") from exc

from ground_station_qt_airsim.backend import AirSimBackend
from ground_station_qt_airsim.config import GroundStationAirSimConfig
from ground_station_qt_airsim.models import Waypoint


COMMAND_LABELS = {
    "arm": "\u89e3\u9501",
    "disarm": "\u4e0a\u9501",
    "takeoff": "\u8d77\u98de",
    "land": "\u964d\u843d",
    "rtl": "\u8fd4\u822a",
    "hover": "\u60ac\u505c",
    "expert_goto_map": "\u4e13\u5bb6\u907f\u969c\u6307\u70b9",
    "expert_goto_local": "\u4e13\u5bb6\u907f\u969c\u6307\u70b9",
    "expert_goto": "\u4e13\u5bb6\u907f\u969c",
    "goto_local": "\u76f4\u63a5\u6307\u70b9",
    "stop_motion": "\u505c\u6b62",
}

CRITICAL_COMMANDS = {"arm", "disarm", "takeoff", "land", "rtl"}


class MapBridge(QObject):
    clicked = Signal(float, float)

    @Slot(float, float)
    def mapClicked(self, x: float, y: float) -> None:
        self.clicked.emit(float(x), float(y))


class AirSimGroundStationWindow(QMainWindow):
    def __init__(self, config: GroundStationAirSimConfig) -> None:
        super().__init__()
        self.config = config
        self.backend = AirSimBackend(config=config, log=self.log)
        self.bridge = MapBridge()
        self.web_channel = None
        self.map_view = None
        self.map_ready = False
        self.selected_uav = int(config.ui.default_selected_uav)
        self.map_mode = "goto"
        self.navigation_mode = "expert"
        self.waypoints: list[Waypoint] = []
        self.current_theme = config.ui.theme_mode or "light"
        self.quick_labels: dict[str, QLabel] = {}

        self.setWindowTitle(config.ui.title)
        try:
            width, height = [int(part) for part in config.ui.geometry.lower().split("x", 1)]
            self.resize(width, height)
        except Exception:
            self.resize(1480, 920)
        self.setMinimumSize(config.ui.min_width, config.ui.min_height)

        self._build_ui()
        self._wire_signals()
        self._connect_backend(initial=True)
        self._start_timer()

    @staticmethod
    def create_application(argv: list[str]) -> QApplication:
        app = QApplication.instance() or QApplication(argv)
        app.setApplicationName("\u57fa\u4e8e AirSim \u7684\u65e0\u4eba\u673a\u5730\u9762\u7ad9")
        app.setStyle("Fusion")
        return app

    def _build_ui(self) -> None:
        self.setStyleSheet(APP_STYLE if self.current_theme != "dark" else DARK_STYLE)
        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.setContentsMargins(14, 14, 14, 14)
        main.setSpacing(14)

        header = QFrame()
        header.setObjectName("Hero")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(16, 10, 16, 10)
        header_layout.addStretch(1)
        self.status_label = QLabel("\u672a\u8fde\u63a5")
        self.status_label.setObjectName("StatusPillWarn")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.header_info = QLabel("\u7b49\u5f85 AirSim \u8fde\u63a5")
        self.header_info.setObjectName("HeaderInfo")
        header_layout.addWidget(self.status_label)
        header_layout.addWidget(self.header_info)
        main.addWidget(header)

        content = QHBoxLayout()
        content.setSpacing(14)
        main.addLayout(content, stretch=1)

        sidebar = QWidget()
        sidebar.setObjectName("Sidebar")
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setSpacing(10)
        sidebar_layout.setContentsMargins(0, 0, 0, 0)

        sidebar_scroll = QScrollArea()
        sidebar_scroll.setObjectName("SidebarScroll")
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        sidebar_scroll.setFixedWidth(440)
        sidebar_scroll.setWidget(sidebar)
        content.addWidget(sidebar_scroll)

        self._add_selector_section(sidebar_layout)
        self._add_status_section(sidebar_layout)
        self._add_basic_command_section(sidebar_layout)
        self._add_map_section(sidebar_layout)
        self._add_llm_section(sidebar_layout)
        self._add_backend_section(sidebar_layout)
        sidebar_layout.addStretch(1)

        right = QVBoxLayout()
        right.setSpacing(12)
        content.addLayout(right, stretch=1)

        map_panel = QFrame()
        map_panel.setObjectName("MapPanel")
        map_layout = QVBoxLayout(map_panel)
        map_layout.setContentsMargins(0, 0, 0, 0)
        map_layout.setSpacing(0)
        self._create_map_view(map_layout)
        right.addWidget(map_panel, stretch=2)

        tabs = QTabWidget()
        tabs.setObjectName("ContentTabs")

        self.fleet_table = QTableWidget(0, 10)
        self.fleet_table.setObjectName("InfoTable")
        self.fleet_table.setHorizontalHeaderLabels(
            ["\u7f16\u53f7", "\u540d\u79f0", "\u5728\u7ebf", "\u6a21\u5f0f", "\u89e3\u9501", "\u9ad8\u5ea6(m)", "\u901f\u5ea6", "X", "Y", "Z"]
        )
        self.fleet_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tabs.addTab(self.fleet_table, "\u98de\u673a\u72b6\u6001")

        self.command_table = QTableWidget(0, 5)
        self.command_table.setObjectName("InfoTable")
        self.command_table.setHorizontalHeaderLabels(["\u5e8f\u53f7", "\u76ee\u6807", "\u547d\u4ee4", "\u72b6\u6001", "\u65f6\u95f4"])
        self.command_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tabs.addTab(self.command_table, "\u63a7\u5236\u8bb0\u5f55")

        self.overview_table = QTableWidget(5, 2)
        self.overview_table.setObjectName("InfoTable")
        self.overview_table.setHorizontalHeaderLabels(["\u9879\u76ee", "\u503c"])
        self.overview_table.verticalHeader().setVisible(False)
        self.overview_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tabs.addTab(self.overview_table, "\u8fd0\u884c\u6982\u89c8")
        right.addWidget(tabs, stretch=1)

        self.log_view = QTextEdit()
        self.log_view.setObjectName("LogView")
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(150)
        right.addWidget(self.log_view)

        view_menu = self.menuBar().addMenu("\u89c6\u56fe")
        theme_action = QAction("\u5207\u6362\u4e3b\u9898", self)
        theme_action.triggered.connect(self._toggle_theme)
        view_menu.addAction(theme_action)

        conn_menu = self.menuBar().addMenu("\u8fde\u63a5")
        reconnect_action = QAction("\u91cd\u65b0\u8fde\u63a5 AirSim", self)
        reconnect_action.triggered.connect(self._reconnect_backend)
        conn_menu.addAction(reconnect_action)

    def _wrap_card(self, title: str) -> tuple[QGroupBox, QGridLayout]:
        frame = QGroupBox(title)
        frame.setObjectName("Card")
        layout = QGridLayout(frame)
        layout.setContentsMargins(14, 18, 14, 14)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(9)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 2)
        return frame, layout

    def _add_selector_section(self, parent: QVBoxLayout) -> None:
        frame, layout = self._wrap_card("\u63a7\u5236\u76ee\u6807")
        self.uav_combo = QComboBox()
        self.uav_combo.addItem("UAV1")
        self.uav_combo.setMinimumWidth(200)
        self.alt_spin = QDoubleSpinBox()
        self.alt_spin.setRange(0.5, 120.0)
        self.alt_spin.setDecimals(1)
        self.alt_spin.setValue(abs(float(self.config.ui.default_altitude_m)))
        self.alt_spin.setSuffix(" m")
        self.alt_spin.setMinimumWidth(200)
        layout.addWidget(QLabel("\u65e0\u4eba\u673a"), 0, 0)
        layout.addWidget(self.uav_combo, 0, 1)
        layout.addWidget(QLabel("\u76ee\u6807\u9ad8\u5ea6"), 1, 0)
        layout.addWidget(self.alt_spin, 1, 1)
        parent.addWidget(frame)

    def _add_status_section(self, parent: QVBoxLayout) -> None:
        frame, layout = self._wrap_card("\u72b6\u6001\u56de\u4f20")
        items = [
            ("\u8fde\u63a5", "\u672a\u8fde\u63a5"),
            ("\u6a21\u5f0f", "-"),
            ("\u4f4d\u7f6e", "-"),
            ("\u672c\u5730\u5750\u6807", "-"),
            ("\u9ad8\u5ea6", "0.0 m"),
            ("\u901f\u5ea6", "0.0 m/s"),
        ]
        for row, (name, value) in enumerate(items):
            name_label = QLabel(name)
            name_label.setObjectName("FieldName")
            value_label = QLabel(value)
            value_label.setObjectName("QuickValue")
            value_label.setWordWrap(True)
            layout.addWidget(name_label, row, 0)
            layout.addWidget(value_label, row, 1)
            self.quick_labels[name] = value_label
        parent.addWidget(frame)

    def _add_basic_command_section(self, parent: QVBoxLayout) -> None:
        frame, layout = self._wrap_card("\u57fa\u7840\u63a7\u5236")
        commands = ["arm", "disarm", "takeoff", "land", "rtl", "hover"]
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        for idx, name in enumerate(commands):
            button = QPushButton(COMMAND_LABELS[name])
            button.setMinimumHeight(42)
            button.setMinimumWidth(150)
            button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            button.clicked.connect(lambda _checked=False, cmd=name: self.send_command(cmd))
            layout.addWidget(button, idx // 2, idx % 2)
        parent.addWidget(frame)

    def _add_map_section(self, parent: QVBoxLayout) -> None:
        frame, layout = self._wrap_card("\u5730\u56fe\u63a7\u5236")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["\u5355\u70b9\u6307\u70b9", "\u822a\u70b9\u4efb\u52a1"])
        self.mode_combo.setMinimumWidth(200)
        self.nav_mode_combo = QComboBox()
        self.nav_mode_combo.addItems(["\u4e13\u5bb6\u907f\u969c", "\u76f4\u63a5\u98de\u884c"])
        self.nav_mode_combo.setMinimumWidth(200)
        self.layer_combo = QComboBox()
        self.layer_combo.addItems(["AirSim \u573a\u666f\u5750\u6807"])
        self.layer_combo.setEnabled(False)
        self.layer_combo.setMinimumWidth(200)
        clear_btn = QPushButton("\u6e05\u7a7a\u822a\u70b9")
        clear_btn.setMinimumHeight(40)
        clear_btn.clicked.connect(self._clear_waypoints)
        run_btn = QPushButton("\u6267\u884c\u822a\u70b9")
        run_btn.setMinimumHeight(40)
        run_btn.clicked.connect(self._run_waypoints)
        layout.addWidget(QLabel("\u70b9\u51fb\u6a21\u5f0f"), 0, 0)
        layout.addWidget(self.mode_combo, 0, 1)
        layout.addWidget(QLabel("\u5bfc\u822a\u65b9\u5f0f"), 1, 0)
        layout.addWidget(self.nav_mode_combo, 1, 1)
        layout.addWidget(QLabel("\u5730\u56fe"), 2, 0)
        layout.addWidget(self.layer_combo, 2, 1)
        layout.addWidget(clear_btn, 3, 0)
        layout.addWidget(run_btn, 3, 1)
        parent.addWidget(frame)

    def _add_llm_section(self, parent: QVBoxLayout) -> None:
        frame, layout = self._wrap_card("\u81ea\u7136\u8bed\u8a00\u547d\u4ee4")
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        self.llm_input = QTextEdit()
        self.llm_input.setObjectName("LlmInput")
        self.llm_input.setPlaceholderText("\u793a\u4f8b\uff1a\u98de\u5230 X=20 Y=5 \u9ad8\u5ea6 8m\uff1b\u60ac\u505c\uff1b\u8d77\u98de\uff1b\u964d\u843d")
        self.llm_input.setMinimumHeight(76)
        self.llm_input.setMaximumHeight(100)
        preview_btn = QPushButton("\u9884\u89c8")
        preview_btn.setMinimumHeight(40)
        preview_btn.clicked.connect(self._preview_llm_command)
        run_btn = QPushButton("\u6267\u884c")
        run_btn.setMinimumHeight(40)
        run_btn.clicked.connect(self._execute_llm_command)
        self.llm_preview = QTextEdit()
        self.llm_preview.setObjectName("LlmPreview")
        self.llm_preview.setReadOnly(True)
        self.llm_preview.setMaximumHeight(86)
        layout.addWidget(self.llm_input, 0, 0, 1, 2)
        layout.addWidget(preview_btn, 1, 0)
        layout.addWidget(run_btn, 1, 1)
        layout.addWidget(self.llm_preview, 2, 0, 1, 2)
        parent.addWidget(frame)

    def _add_backend_section(self, parent: QVBoxLayout) -> None:
        frame, layout = self._wrap_card("AirSim \u8fde\u63a5")
        self.backend_host_label = QLabel(self.config.airsim.host)
        self.backend_host_label.setObjectName("QuickValue")
        self.backend_port_label = QLabel(str(self.config.airsim.port))
        self.backend_port_label.setObjectName("QuickValue")
        reconnect_btn = QPushButton("\u91cd\u65b0\u8fde\u63a5")
        reconnect_btn.clicked.connect(self._reconnect_backend)
        layout.addWidget(QLabel("\u4e3b\u673a"), 0, 0)
        layout.addWidget(self.backend_host_label, 0, 1)
        layout.addWidget(QLabel("\u7aef\u53e3"), 1, 0)
        layout.addWidget(self.backend_port_label, 1, 1)
        layout.addWidget(reconnect_btn, 2, 0, 1, 2)
        parent.addWidget(frame)

    def _create_map_view(self, layout: QVBoxLayout) -> None:
        if QWebEngineView is None or QWebChannel is None:
            fallback = QLabel("\u672a\u5b89\u88c5 PySide6 WebEngine\uff0c\u65e0\u6cd5\u663e\u793a AirSim \u5750\u6807\u5730\u56fe\u3002")
            fallback.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(fallback)
            return
        self.map_view = QWebEngineView()
        self.map_view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.web_channel = QWebChannel(self.map_view.page())
        self.web_channel.registerObject("bridge", self.bridge)
        self.map_view.page().setWebChannel(self.web_channel)
        self.map_view.loadFinished.connect(self._on_map_loaded)
        self.map_view.setHtml(self._map_html(), QUrl("http://127.0.0.1/"))
        layout.addWidget(self.map_view)

    def _wire_signals(self) -> None:
        self.uav_combo.currentTextChanged.connect(self._on_uav_changed)
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        self.nav_mode_combo.currentTextChanged.connect(self._on_nav_mode_changed)
        self.bridge.clicked.connect(self._on_map_clicked)

    def _connect_backend(self, *, initial: bool = False) -> None:
        try:
            self.backend.connect()
            self.status_label.setText("\u5df2\u8fde\u63a5")
            self.status_label.setObjectName("StatusPill")
            self.status_label.style().unpolish(self.status_label)
            self.status_label.style().polish(self.status_label)
            self._refresh_uav_combo()
            if not initial:
                self.log("AirSim \u540e\u7aef\u5df2\u91cd\u65b0\u8fde\u63a5\u3002", "INFO")
        except Exception as exc:
            self.status_label.setText("\u8fde\u63a5\u5931\u8d25")
            self.status_label.setObjectName("StatusPillError")
            self.status_label.style().unpolish(self.status_label)
            self.status_label.style().polish(self.status_label)
            self.log(f"AirSim \u8fde\u63a5\u5931\u8d25: {exc}", "ERROR")

    def _reconnect_backend(self) -> None:
        try:
            self.backend.close()
        except Exception:
            pass
        self._connect_backend()

    def _start_timer(self) -> None:
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_dashboard)
        self.timer.start(int(self.config.ui.dashboard_interval_ms))

    def _refresh_uav_combo(self) -> None:
        current = self.selected_uav
        self.uav_combo.blockSignals(True)
        self.uav_combo.clear()
        ids = [binding.sysid for binding in self.backend.vehicle_bindings] or [1]
        for sysid in ids:
            self.uav_combo.addItem(f"UAV{sysid}")
        self.selected_uav = current if current in ids else ids[0]
        self.uav_combo.setCurrentText(f"UAV{self.selected_uav}")
        self.uav_combo.blockSignals(False)

    def _on_uav_changed(self, text: str) -> None:
        try:
            self.selected_uav = int(text.replace("UAV", "").strip())
        except Exception:
            self.selected_uav = 1

    def _on_mode_changed(self, text: str) -> None:
        lowered = text.lower()
        self.map_mode = "waypoint" if "waypoint" in lowered or "\u822a\u70b9" in text else "goto"

    def _on_nav_mode_changed(self, text: str) -> None:
        lowered = text.lower()
        self.navigation_mode = "expert" if "expert" in lowered or "\u4e13\u5bb6" in text else "direct"

    def _toggle_theme(self) -> None:
        self.current_theme = "dark" if self.current_theme != "dark" else "light"
        self.setStyleSheet(APP_STYLE if self.current_theme != "dark" else DARK_STYLE)

    @Slot()
    def _on_map_loaded(self) -> None:
        self.map_ready = True

    def update_dashboard(self) -> None:
        telemetry = self.backend.refresh()
        self.header_info.setText(
            f"\u98de\u673a {len(self.backend.vehicle_bindings)} \u67b6  \u901f\u5ea6 {self.backend.command_speed_mps:.1f} m/s  \u76ee\u6807\u9ad8\u5ea6 {self.alt_spin.value():.1f} m"
        )
        self._update_fleet_table(telemetry)
        self._update_command_table()
        self._update_overview_table(telemetry)
        self._update_quick_status(telemetry)
        self._update_map_state(telemetry)

    def _update_fleet_table(self, telemetry: dict[int, object]) -> None:
        now = datetime.now().timestamp()
        ids = sorted(telemetry.keys())
        self.fleet_table.setRowCount(len(ids))
        for row, sysid in enumerate(ids):
            tele = telemetry[sysid]
            values = [
                f"UAV{sysid}",
                tele.display_name,
                "\u5728\u7ebf" if tele.is_online(now, 3.0) else "\u79bb\u7ebf",
                tele.mode,
                "\u662f" if tele.armed else "\u5426",
                f"{tele.alt:.1f}",
                f"{tele.speed:.1f}",
                f"{tele.local_x:.1f}",
                f"{tele.local_y:.1f}",
                f"{tele.local_z:.1f}",
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                if col == 2 and value == "\u79bb\u7ebf":
                    item.setForeground(QColor("#d64545"))
                self.fleet_table.setItem(row, col, item)

    def _update_command_table(self) -> None:
        rows = self.backend.command_history
        self.command_table.setRowCount(len(rows))
        for row, record in enumerate(rows):
            values = [
                str(record.index),
                f"UAV{record.target_id}",
                COMMAND_LABELS.get(record.command, record.command),
                record.status,
                record.issued_at.replace("T", " "),
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                if col == 3 and value == "FAILED":
                    item.setForeground(QColor("#d64545"))
                self.command_table.setItem(row, col, item)

    def _update_overview_table(self, telemetry: dict[int, object]) -> None:
        now = datetime.now().timestamp()
        online = [tele for tele in telemetry.values() if tele.is_online(now, 3.0)]
        armed = [tele for tele in online if tele.armed]
        avg_speed = sum(tele.speed for tele in online) / len(online) if online else 0.0
        metrics = [
            ("\u5728\u7ebf\u98de\u673a", str(len(online))),
            ("\u5df2\u89e3\u9501\u98de\u673a", str(len(armed))),
            ("\u5e73\u5747\u901f\u5ea6", f"{avg_speed:.1f} m/s"),
            ("\u5730\u56fe\u822a\u70b9", str(len(self.waypoints))),
            ("\u76ee\u6807\u9ad8\u5ea6", f"{self.alt_spin.value():.1f} m"),
        ]
        for row, (name, value) in enumerate(metrics):
            self.overview_table.setItem(row, 0, QTableWidgetItem(name))
            self.overview_table.setItem(row, 1, QTableWidgetItem(value))

    def _update_quick_status(self, telemetry: dict[int, object]) -> None:
        tele = telemetry.get(self.selected_uav)
        if tele is None:
            return
        online = tele.is_online(datetime.now().timestamp(), 3.0)
        self.quick_labels["\u8fde\u63a5"].setText("\u5728\u7ebf" if online else "\u79bb\u7ebf")
        self.quick_labels["\u6a21\u5f0f"].setText(tele.mode)
        self.quick_labels["\u4f4d\u7f6e"].setText(f"X {tele.local_x:.1f}, Y {tele.local_y:.1f}")
        self.quick_labels["\u672c\u5730\u5750\u6807"].setText(f"X {tele.local_x:.1f} / Y {tele.local_y:.1f} / Z {tele.local_z:.1f}")
        self.quick_labels["\u9ad8\u5ea6"].setText(f"\u9ad8\u5ea6 {tele.alt:.1f} m  |  AirSim Z {tele.local_z:.1f}")
        self.quick_labels["\u901f\u5ea6"].setText(f"{tele.speed:.1f} m/s")

    def _update_map_state(self, telemetry: dict[int, object]) -> None:
        if not self.map_ready:
            return
        payload = {
            "uavs": [
                {
                    "sysid": tele.sysid,
                    "x": tele.local_x,
                    "y": tele.local_y,
                    "z": tele.local_z,
                    "alt": tele.alt,
                    "mode": tele.mode,
                    "armed": tele.armed,
                    "trail": [
                        {"x": x, "y": y}
                        for lat, lon, x, y, _ts in list(tele.trail)[-120:]
                    ],
                }
                for tele in telemetry.values()
            ],
            "waypoints": [{"x": wp.x, "y": wp.y, "z": wp.z, "idx": idx + 1} for idx, wp in enumerate(self.waypoints)],
        }
        self._run_js(f"window.updateAirSimState({json.dumps(payload, ensure_ascii=True)});")

    def send_command(self, command: str) -> None:
        if command in CRITICAL_COMMANDS:
            answer = QMessageBox.question(
                self,
                "\u786e\u8ba4\u547d\u4ee4",
                f"\u5411 UAV{self.selected_uav} \u53d1\u9001 {COMMAND_LABELS.get(command, command)} \u547d\u4ee4\uff1f",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                return
        try:
            self.backend.send_basic_command(self.selected_uav, command)
            self.log(f"\u547d\u4ee4\u5df2\u53d1\u9001 -> UAV{self.selected_uav}: {COMMAND_LABELS.get(command, command)}", "COMMAND")
        except Exception as exc:
            self.log(f"\u547d\u4ee4\u5931\u8d25 -> UAV{self.selected_uav}: {exc}", "ERROR")

    def _on_map_clicked(self, x: float, y: float) -> None:
        target_z = self._target_z()
        if self.map_mode == "waypoint":
            self.waypoints.append(Waypoint(x=x, y=y, z=target_z))
            self.log(f"\u822a\u70b9 #{len(self.waypoints)} \u5df2\u6dfb\u52a0: x={x:.1f}, y={y:.1f}, z={target_z:.1f}", "INFO")
            self._update_map_state(self.backend.telemetry)
            return
        answer = QMessageBox.question(
            self,
            "\u786e\u8ba4\u6307\u70b9\u98de\u884c",
            f"\u8ba9 UAV{self.selected_uav} \u98de\u5230\u8fd9\u4e2a AirSim \u5750\u6807\u70b9\uff1f\n\nX={x:.1f}\nY={y:.1f}\nZ={target_z:.1f}\n\u9ad8\u5ea6={-target_z:.1f} m",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        try:
            if self.navigation_mode == "expert":
                self.backend.start_expert_goto_local(self.selected_uav, x, y, target_z)
                self.log(f"\u4e13\u5bb6\u907f\u969c\u5bfc\u822a\u5df2\u542f\u52a8 -> UAV{self.selected_uav}: x={x:.1f}, y={y:.1f}, z={target_z:.1f}", "COMMAND")
            else:
                self.backend.goto_local(self.selected_uav, x, y, target_z)
                self.log(f"\u76f4\u63a5\u6307\u70b9\u5df2\u53d1\u9001 -> UAV{self.selected_uav}: x={x:.1f}, y={y:.1f}, z={target_z:.1f}", "COMMAND")
            self.waypoints = [Waypoint(x=x, y=y, z=target_z)]
        except Exception as exc:
            self.log(f"\u5bfc\u822a\u5931\u8d25: {exc}", "ERROR")

    def _clear_waypoints(self) -> None:
        self.waypoints.clear()
        self.log("\u822a\u70b9\u5df2\u6e05\u7a7a\u3002", "INFO")

    def _run_waypoints(self) -> None:
        if not self.waypoints:
            self.log("\u6ca1\u6709\u53ef\u6267\u884c\u7684\u822a\u70b9\u3002", "WARN")
            return
        try:
            self.backend.run_waypoints(self.selected_uav, list(self.waypoints))
            self.log(f"\u822a\u70b9\u4efb\u52a1\u5df2\u542f\u52a8 -> UAV{self.selected_uav}: {len(self.waypoints)} \u4e2a\u70b9", "COMMAND")
        except Exception as exc:
            self.log(f"\u822a\u70b9\u4efb\u52a1\u5931\u8d25: {exc}", "ERROR")

    def _preview_llm_command(self) -> None:
        try:
            plan = self._parse_llm_command(self.llm_input.toPlainText())
            self.llm_preview.setPlainText(json.dumps(plan, ensure_ascii=False, indent=2))
        except Exception as exc:
            self.llm_preview.setPlainText(f"\u89e3\u6790\u5931\u8d25: {exc}")

    def _execute_llm_command(self) -> None:
        try:
            plan = self._parse_llm_command(self.llm_input.toPlainText())
            self.llm_preview.setPlainText(json.dumps(plan, ensure_ascii=False, indent=2))
            self._run_llm_plan(plan)
        except Exception as exc:
            self.log(f"\u81ea\u7136\u8bed\u8a00\u547d\u4ee4\u5931\u8d25: {exc}", "ERROR")

    def _parse_llm_command(self, text: str) -> dict:
        raw = text.strip()
        if not raw:
            return {"steps": []}
        try:
            parsed = json.loads(raw)
            if isinstance(parsed, dict):
                return parsed
            if isinstance(parsed, list):
                return {"steps": parsed}
        except json.JSONDecodeError:
            pass
        lowered = raw.lower()
        steps: list[dict] = []
        z = self._extract_z(raw)
        x_match = re.search(r"\bX\s*=?\s*(-?\d+(?:\.\d+)?)", raw, re.IGNORECASE)
        y_match = re.search(r"\bY\s*=?\s*(-?\d+(?:\.\d+)?)", raw, re.IGNORECASE)
        if "takeoff" in lowered or "\u8d77\u98de" in raw:
            steps.append({"command": "takeoff", "args": {}})
        if x_match and y_match:
            steps.append({"command": "expert_goto_local", "args": {"x": float(x_match.group(1)), "y": float(y_match.group(1)), "z": z}})
        if "hover" in lowered or "stop" in lowered or "\u60ac\u505c" in raw or "\u505c\u6b62" in raw:
            steps.append({"command": "hover", "args": {}})
        if "rtl" in lowered or "go home" in lowered or "return" in lowered or "\u8fd4\u822a" in raw:
            steps.append({"command": "rtl", "args": {}})
        if "land" in lowered or "\u964d\u843d" in raw:
            steps.append({"command": "land", "args": {}})
        return {"task": raw, "steps": steps}

    def _extract_z(self, text: str) -> float:
        explicit = re.search(r"\bZ\s*=?\s*(-?\d+(?:\.\d+)?)", text, re.IGNORECASE)
        if explicit:
            return float(explicit.group(1))
        height = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|meter|meters)", text, re.IGNORECASE)
        if height:
            return -abs(float(height.group(1)))
        return self._target_z()

    def _target_z(self) -> float:
        return -abs(float(self.alt_spin.value()))

    def _run_llm_plan(self, plan: dict) -> None:
        steps = plan.get("steps", plan.get("plan", []))
        if not isinstance(steps, list) or not steps:
            self.log("\u81ea\u7136\u8bed\u8a00\u547d\u4ee4\u6ca1\u6709\u53ef\u6267\u884c\u6b65\u9aa4\u3002", "WARN")
            return
        for step in steps:
            if not isinstance(step, dict):
                continue
            command = str(step.get("command", step.get("skill", ""))).strip()
            args = step.get("args", step.get("parameters", {}))
            if not isinstance(args, dict):
                args = {}
            if command in {"navigate_with_expert", "expert_goto_local", "goto_local"}:
                target_z = float(args.get("target_z", args.get("z", self._target_z())))
                self.backend.start_expert_goto_local(
                    self.selected_uav,
                    float(args.get("target_x", args.get("x"))),
                    float(args.get("target_y", args.get("y"))),
                    target_z,
                )
                self.log(f"\u81ea\u7136\u8bed\u8a00\u5df2\u542f\u52a8\u4e13\u5bb6\u907f\u969c\u6307\u70b9 -> UAV{self.selected_uav}", "COMMAND")
            elif command in COMMAND_LABELS or command in {"hover", "rtl", "land", "takeoff", "arm", "disarm"}:
                self.backend.send_basic_command(self.selected_uav, command)
                self.log(f"\u81ea\u7136\u8bed\u8a00\u547d\u4ee4\u5df2\u53d1\u9001 -> UAV{self.selected_uav}: {COMMAND_LABELS.get(command, command)}", "COMMAND")
            else:
                raise ValueError(f"\u4e0d\u652f\u6301\u7684\u81ea\u7136\u8bed\u8a00\u547d\u4ee4: {command}")

    def _run_js(self, script: str) -> None:
        if self.map_view is None or not self.map_ready:
            return
        self.map_view.page().runJavaScript(script)

    def log(self, message: str, level: str = "INFO") -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_view.append(f"[{ts}] {level.upper()} | {message}")

    def closeEvent(self, event) -> None:  # noqa: N802
        try:
            self.backend.close()
        finally:
            super().closeEvent(event)

    def _map_html(self) -> str:
        return MAP_HTML


MAP_HTML = """<!doctype html>
<html>
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<script src="qrc:///qtwebchannel/qwebchannel.js"></script>
<style>
html, body, #map { width: 100%; height: 100%; margin: 0; overflow: hidden; background: #f7f4ee; }
#map { position: relative; }
#canvas { width: 100%; height: 100%; display: block; cursor: grab; }
#canvas.dragging { cursor: grabbing; }
.hud { position: absolute; top: 12px; left: 12px; display: flex; gap: 8px; }
.badge { background: rgba(255,253,248,0.9); border: 1px solid rgba(177,168,145,0.45); color: #32433a; padding: 5px 9px; border-radius: 999px; font-size: 11px; }
</style>
</head>
<body>
<div id="map">
  <canvas id="canvas"></canvas>
  <div class="hud">
    <div class="badge" id="statusBadge">&#24050;&#23601;&#32490;</div>
    <div class="badge" id="scaleBadge">&#27604;&#20363; 10m</div>
    <div class="badge" id="cursorBadge">X 0.0 / Y 0.0</div>
  </div>
</div>
<script>
let bridge = null;
if (typeof QWebChannel !== 'undefined') {
  new QWebChannel(qt.webChannelTransport, function(channel) { bridge = channel.objects.bridge; });
}
const state = { uavs: [], waypoints: [], scaleMetersPerGrid: 10, viewX: 0, viewY: 0, dragging: false, moved: false, lastX: 0, lastY: 0 };
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const statusBadge = document.getElementById('statusBadge');
const cursorBadge = document.getElementById('cursorBadge');
function resize() { canvas.width = document.getElementById('map').clientWidth; canvas.height = document.getElementById('map').clientHeight; draw(); }
function emitClick(x, y) { if (bridge) bridge.mapClicked(x, y); }
function allPoints() { const points = []; for (const uav of state.uavs) { points.push([uav.x || 0, uav.y || 0]); for (const p of (uav.trail || [])) points.push([p.x || 0, p.y || 0]); } return points; }
function fitScale() { return state.scaleMetersPerGrid; }
function metersToPixels(m) { return m * (40 / state.scaleMetersPerGrid); }
function pixelsToMeters(px) { return px / (40 / state.scaleMetersPerGrid); }
function localToCanvas(x, y) { return [canvas.width / 2 + metersToPixels(y - state.viewY), canvas.height / 2 - metersToPixels(x - state.viewX)]; }
function canvasToLocal(x, y) { return [state.viewX + pixelsToMeters(canvas.height / 2 - y), state.viewY + pixelsToMeters(x - canvas.width / 2)]; }
function drawGrid() { ctx.clearRect(0,0,canvas.width,canvas.height); ctx.fillStyle='#f7f4ee'; ctx.fillRect(0,0,canvas.width,canvas.height); const s=40; const origin=localToCanvas(0,0); ctx.strokeStyle='rgba(43,71,57,0.12)'; ctx.lineWidth=1; for(let x=origin[0]%s;x<canvas.width;x+=s){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,canvas.height);ctx.stroke();} for(let y=origin[1]%s;y<canvas.height;y+=s){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(canvas.width,y);ctx.stroke();} ctx.strokeStyle='rgba(43,71,57,0.30)'; ctx.lineWidth=2; ctx.beginPath(); ctx.moveTo(origin[0],0);ctx.lineTo(origin[0],canvas.height);ctx.moveTo(0,origin[1]);ctx.lineTo(canvas.width,origin[1]);ctx.stroke(); }
function drawTrail(trail) { if (!trail || trail.length < 2) return; ctx.strokeStyle='rgba(15,157,88,0.55)'; ctx.lineWidth=2.5; ctx.beginPath(); trail.forEach((p,i)=>{const [cx,cy]=localToCanvas(p.x||0,p.y||0); if(i===0)ctx.moveTo(cx,cy); else ctx.lineTo(cx,cy);}); ctx.stroke(); }
function drawWaypoints() { if (state.waypoints.length > 1) { ctx.strokeStyle='#f57c00'; ctx.lineWidth=2; ctx.setLineDash([8,8]); ctx.beginPath(); state.waypoints.forEach((wp,i)=>{const [cx,cy]=localToCanvas(wp.x||0,wp.y||0); if(i===0)ctx.moveTo(cx,cy); else ctx.lineTo(cx,cy);}); ctx.stroke(); ctx.setLineDash([]); } state.waypoints.forEach((wp)=>{const [cx,cy]=localToCanvas(wp.x||0,wp.y||0); ctx.fillStyle='#f57c00'; ctx.beginPath(); ctx.arc(cx,cy,7,0,Math.PI*2); ctx.fill(); ctx.strokeStyle='#fff'; ctx.lineWidth=2; ctx.stroke(); ctx.fillStyle='#8a4700'; ctx.font='600 12px Arial'; ctx.fillText(String(wp.idx),cx+10,cy-8);}); }
function drawUavs() { for (const uav of state.uavs) { drawTrail(uav.trail || []); const [cx,cy]=localToCanvas(uav.x||0,uav.y||0); ctx.fillStyle='#0f9d58'; ctx.beginPath(); ctx.arc(cx,cy,9,0,Math.PI*2); ctx.fill(); ctx.strokeStyle='#fff'; ctx.lineWidth=2.5; ctx.stroke(); ctx.fillStyle='#153b2a'; ctx.font='600 12px Arial'; ctx.fillText(`UAV${uav.sysid}`,cx+12,cy-10); } }
function draw() { state.scaleMetersPerGrid = fitScale(); document.getElementById('scaleBadge').textContent = `\u6bd4\u4f8b ${state.scaleMetersPerGrid.toFixed(0)}m`; drawGrid(); drawWaypoints(); drawUavs(); }
window.updateAirSimState = function(payload) { state.uavs = payload.uavs || []; state.waypoints = payload.waypoints || []; statusBadge.textContent = `\u98de\u673a ${state.uavs.length} \u67b6`; draw(); };
canvas.addEventListener('click', function(event) { if (state.moved) { state.moved=false; return; } const rect=canvas.getBoundingClientRect(); const x=event.clientX-rect.left; const y=event.clientY-rect.top; const [lx,ly]=canvasToLocal(x,y); emitClick(lx,ly); });
canvas.addEventListener('mousemove', function(event) { const rect=canvas.getBoundingClientRect(); const x=event.clientX-rect.left; const y=event.clientY-rect.top; const [lx,ly]=canvasToLocal(x,y); cursorBadge.textContent = `X ${lx.toFixed(1)} / Y ${ly.toFixed(1)}`; });
canvas.addEventListener('mousedown', function(event) { if (event.button !== 0) return; state.dragging=true; state.moved=false; state.lastX=event.clientX; state.lastY=event.clientY; canvas.classList.add('dragging'); });
window.addEventListener('mouseup', function() { state.dragging=false; canvas.classList.remove('dragging'); });
window.addEventListener('mousemove', function(event) { if (!state.dragging) return; const dx=event.clientX-state.lastX; const dy=event.clientY-state.lastY; if (Math.abs(dx)+Math.abs(dy)>2) state.moved=true; state.viewY -= pixelsToMeters(dx); state.viewX += pixelsToMeters(dy); state.lastX=event.clientX; state.lastY=event.clientY; draw(); });
canvas.addEventListener('wheel', function(event) { event.preventDefault(); const rect=canvas.getBoundingClientRect(); const cx=event.clientX-rect.left; const cy=event.clientY-rect.top; const before=canvasToLocal(cx,cy); const factor=event.deltaY<0 ? 0.85 : 1.18; state.scaleMetersPerGrid=Math.max(1, Math.min(80, state.scaleMetersPerGrid*factor)); const after=canvasToLocal(cx,cy); state.viewX += before[0]-after[0]; state.viewY += before[1]-after[1]; draw(); }, { passive: false });
window.addEventListener('resize', resize); resize();
</script>
</body>
</html>
"""

APP_STYLE = """
QMainWindow { background: #f5f7f8; font-family: "Microsoft YaHei", "Segoe UI", Arial, sans-serif; }
QScrollArea#SidebarScroll { background: transparent; border: none; }
QFrame#Hero, QFrame#MapPanel, QGroupBox#Card, QTabWidget::pane, QTableWidget#InfoTable, QTextEdit#LogView, QTextEdit#LlmInput, QTextEdit#LlmPreview { background: #ffffff; border: 1px solid #d8e0e6; border-radius: 8px; }
QGroupBox#Card { margin-top: 10px; color: #25313a; }
QGroupBox#Card::title { subcontrol-origin: margin; left: 10px; padding: 0 6px; color: #1f5f7a; font-size: 13px; font-weight: 700; }
QLabel#HeaderInfo, QLabel#FieldName { color: #667783; font-size: 12px; }
QLabel#QuickValue { color: #1f3948; font-size: 13px; font-weight: 700; }
QLabel#StatusPill, QLabel#StatusPillWarn, QLabel#StatusPillError { color: #ffffff; border-radius: 999px; padding: 4px 10px; font-weight: 700; min-width: 72px; }
QLabel#StatusPill { background: #22845d; } QLabel#StatusPillWarn { background: #b7791f; } QLabel#StatusPillError { background: #bf3f3f; }
QPushButton { background: #216b84; color: #ffffff; border: none; border-radius: 8px; padding: 6px 8px; font-size: 14px; font-weight: 700; min-height: 34px; }
QPushButton:hover { background: #18576d; }
QComboBox, QDoubleSpinBox { background: #ffffff; color: #25313a; border: 1px solid #cfd9df; border-radius: 8px; padding: 6px 8px; font-size: 13px; min-height: 28px; }
QHeaderView::section { background: #edf2f5; color: #3f515d; border: none; border-bottom: 1px solid #d8e0e6; padding: 8px; font-size: 12px; font-weight: 700; }
QTableWidget#InfoTable { gridline-color: #e4eaee; color: #25313a; }
QTextEdit#LogView, QTextEdit#LlmInput, QTextEdit#LlmPreview { color: #25313a; padding: 8px; font-size: 13px; }
QTextEdit#LogView, QTextEdit#LlmPreview { font-family: Consolas, "Courier New", monospace; font-size: 12px; }
"""

DARK_STYLE = """
QMainWindow { background: #172026; font-family: "Microsoft YaHei", "Segoe UI", Arial, sans-serif; }
QScrollArea#SidebarScroll { background: transparent; border: none; }
QFrame#Hero, QFrame#MapPanel, QGroupBox#Card, QTabWidget::pane, QTableWidget#InfoTable, QTextEdit#LogView, QTextEdit#LlmInput, QTextEdit#LlmPreview { background: #202c33; border: 1px solid #33444e; border-radius: 8px; }
QGroupBox#Card { margin-top: 10px; color: #e8eef2; }
QGroupBox#Card::title { subcontrol-origin: margin; left: 10px; padding: 0 6px; color: #7cc4df; font-size: 13px; font-weight: 700; }
QLabel#HeaderInfo, QLabel#FieldName { color: #9eb0bb; font-size: 12px; }
QLabel#QuickValue { color: #b5e8f7; font-size: 13px; font-weight: 700; }
QLabel#StatusPill, QLabel#StatusPillWarn, QLabel#StatusPillError { color: #ffffff; border-radius: 999px; padding: 4px 10px; font-weight: 700; min-width: 72px; }
QLabel#StatusPill { background: #2ea043; } QLabel#StatusPillWarn { background: #d29922; } QLabel#StatusPillError { background: #f85149; }
QPushButton { background: #26738c; color: #ffffff; border: none; border-radius: 8px; padding: 6px 8px; font-size: 14px; font-weight: 700; min-height: 34px; }
QPushButton:hover { background: #2f88a6; }
QComboBox, QDoubleSpinBox { background: #172026; color: #e8eef2; border: 1px solid #33444e; border-radius: 8px; padding: 6px 8px; font-size: 13px; min-height: 28px; }
QHeaderView::section { background: #172026; color: #c8d5dc; border: none; border-bottom: 1px solid #33444e; padding: 8px; font-size: 12px; font-weight: 700; }
QTableWidget#InfoTable { gridline-color: #2d3e47; color: #e8eef2; }
QTextEdit#LogView, QTextEdit#LlmInput, QTextEdit#LlmPreview { color: #e8eef2; padding: 8px; font-size: 13px; }
QTextEdit#LogView, QTextEdit#LlmPreview { font-family: Consolas, "Courier New", monospace; font-size: 12px; }
"""
