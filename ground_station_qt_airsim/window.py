from __future__ import annotations

import json
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
    "arm": "解锁",
    "disarm": "上锁",
    "takeoff": "起飞",
    "land": "降落",
    "rtl": "返航",
    "hover": "悬停",
}

CRITICAL_COMMANDS = {"arm", "disarm", "takeoff", "land", "rtl"}


class MapBridge(QObject):
    clicked = Signal(float, float)

    @Slot(float, float)
    def mapClicked(self, lat: float, lon: float) -> None:
        self.clicked.emit(float(lat), float(lon))


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
        app.setApplicationName("AirSim 基础地面站")
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
        self.status_label = QLabel("未连接")
        self.status_label.setObjectName("StatusPillWarn")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.header_info = QLabel("等待连接 AirSim")
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
            ["编号", "名称", "在线", "模式", "解锁", "高度(m)", "速度", "纬度", "经度", "本地坐标"]
        )
        self.fleet_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tabs.addTab(self.fleet_table, "飞机状态")

        self.command_table = QTableWidget(0, 5)
        self.command_table.setObjectName("InfoTable")
        self.command_table.setHorizontalHeaderLabels(["序号", "目标", "命令", "状态", "时间"])
        self.command_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tabs.addTab(self.command_table, "控制记录")

        self.overview_table = QTableWidget(5, 2)
        self.overview_table.setObjectName("InfoTable")
        self.overview_table.setHorizontalHeaderLabels(["项目", "值"])
        self.overview_table.verticalHeader().setVisible(False)
        self.overview_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tabs.addTab(self.overview_table, "运行概览")

        right.addWidget(tabs, stretch=1)

        self.log_view = QTextEdit()
        self.log_view.setObjectName("LogView")
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(150)
        right.addWidget(self.log_view)

        menubar = self.menuBar()
        view_menu = menubar.addMenu("视图")
        theme_action = QAction("切换主题", self)
        theme_action.triggered.connect(self._toggle_theme)
        view_menu.addAction(theme_action)

        conn_menu = menubar.addMenu("连接")
        reconnect_action = QAction("重新连接 AirSim", self)
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
        frame, layout = self._wrap_card("当前目标")
        self.uav_combo = QComboBox()
        self.uav_combo.addItem("UAV1")
        self.uav_combo.setMinimumWidth(200)
        self.alt_spin = QDoubleSpinBox()
        self.alt_spin.setRange(-120.0, -0.5)
        self.alt_spin.setDecimals(1)
        self.alt_spin.setValue(float(self.config.ui.default_altitude_m))
        self.alt_spin.setSuffix(" m")
        self.alt_spin.setMinimumWidth(200)
        layout.addWidget(QLabel("控制对象"), 0, 0)
        layout.addWidget(self.uav_combo, 0, 1)
        layout.addWidget(QLabel("目标Z坐标"), 1, 0)
        layout.addWidget(self.alt_spin, 1, 1)
        parent.addWidget(frame)

    def _add_status_section(self, parent: QVBoxLayout) -> None:
        frame, layout = self._wrap_card("状态回传")
        items = [
            ("连接状态", "未连接"),
            ("飞行模式", "-"),
            ("当前位置", "-"),
            ("本地坐标", "-"),
            ("当前高度", "0.0 m"),
            ("当前速度", "0.0 m/s"),
        ]
        for row, (name, value) in enumerate(items):
            name_label = QLabel(name)
            name_label.setObjectName("FieldName")
            name_label.setMinimumHeight(26)
            name_label.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)
            value_label = QLabel(value)
            value_label.setObjectName("QuickValue")
            value_label.setWordWrap(True)
            value_label.setMinimumHeight(26)
            value_label.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)
            layout.addWidget(name_label, row, 0)
            layout.addWidget(value_label, row, 1)
            self.quick_labels[name] = value_label
        parent.addWidget(frame)

    def _add_basic_command_section(self, parent: QVBoxLayout) -> None:
        frame, layout = self._wrap_card("基础控制")
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
        frame, layout = self._wrap_card("离线地图控制")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["单点导航", "航点模式"])
        self.mode_combo.setMinimumWidth(200)
        self.layer_combo = QComboBox()
        self.layer_combo.addItems(["离线地图"])
        self.layer_combo.setEnabled(False)
        self.layer_combo.setMinimumWidth(200)
        clear_btn = QPushButton("清空航点")
        clear_btn.setMinimumHeight(40)
        clear_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        clear_btn.clicked.connect(self._clear_waypoints)
        run_btn = QPushButton("执行航点")
        run_btn.setMinimumHeight(40)
        run_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        run_btn.clicked.connect(self._run_waypoints)
        layout.addWidget(QLabel("点击模式"), 0, 0)
        layout.addWidget(self.mode_combo, 0, 1)
        layout.addWidget(QLabel("地图类型"), 1, 0)
        layout.addWidget(self.layer_combo, 1, 1)
        layout.addWidget(clear_btn, 2, 0)
        layout.addWidget(run_btn, 2, 1)
        parent.addWidget(frame)

    def _add_backend_section(self, parent: QVBoxLayout) -> None:
        frame, layout = self._wrap_card("AirSim 连接")
        self.backend_host_label = QLabel(self.config.airsim.host)
        self.backend_host_label.setObjectName("QuickValue")
        self.backend_port_label = QLabel(str(self.config.airsim.port))
        self.backend_port_label.setObjectName("QuickValue")
        reconnect_btn = QPushButton("重新连接")
        reconnect_btn.clicked.connect(self._reconnect_backend)
        layout.addWidget(QLabel("主机地址"), 0, 0)
        layout.addWidget(self.backend_host_label, 0, 1)
        layout.addWidget(QLabel("端口"), 1, 0)
        layout.addWidget(self.backend_port_label, 1, 1)
        layout.addWidget(reconnect_btn, 2, 0, 1, 2)
        parent.addWidget(frame)

    def _create_map_view(self, layout: QVBoxLayout) -> None:
        if QWebEngineView is None or QWebChannel is None:
            fallback = QLabel("未安装 PySide6 WebEngine，无法显示离线地图。")
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
        self.bridge.clicked.connect(self._on_map_clicked)

    def _connect_backend(self, *, initial: bool = False) -> None:
        try:
            self.backend.connect()
            self.status_label.setText("已连接")
            self.status_label.setObjectName("StatusPill")
            self.status_label.style().unpolish(self.status_label)
            self.status_label.style().polish(self.status_label)
            self._refresh_uav_combo()
            if not initial:
                self.log("AirSim 后端已重新连接。", "INFO")
        except Exception as exc:
            self.status_label.setText("失败")
            self.status_label.setObjectName("StatusPillError")
            self.status_label.style().unpolish(self.status_label)
            self.status_label.style().polish(self.status_label)
            self.log(f"AirSim 连接失败: {exc}", "ERROR")

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
        self.map_mode = "waypoint" if "航点" in text else "goto"

    def _toggle_theme(self) -> None:
        self.current_theme = "dark" if self.current_theme != "dark" else "light"
        self.setStyleSheet(APP_STYLE if self.current_theme != "dark" else DARK_STYLE)

    @Slot()
    def _on_map_loaded(self) -> None:
        self.map_ready = True

    def update_dashboard(self) -> None:
        telemetry = self.backend.refresh()
        self.header_info.setText(
            f"飞机数 {len(self.backend.vehicle_bindings)}  默认速度 {self.backend.command_speed_mps:.1f} m/s  目标Z {self.alt_spin.value():.1f}"
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
                "在线" if tele.is_online(now, 3.0) else "离线",
                tele.mode,
                "是" if tele.armed else "否",
                f"{tele.alt:.1f}",
                f"{tele.speed:.1f}",
                f"{tele.lat:.6f}",
                f"{tele.lon:.6f}",
                f"X {tele.local_x:.1f}, Y {tele.local_y:.1f}",
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                if col == 2 and value == "离线":
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
            ("在线飞机", str(len(online))),
            ("已解锁", str(len(armed))),
            ("平均速度", f"{avg_speed:.1f} m/s"),
            ("地图航点", str(len(self.waypoints))),
            ("目标Z坐标", f"{self.alt_spin.value():.1f} m"),
        ]
        for row, (name, value) in enumerate(metrics):
            self.overview_table.setItem(row, 0, QTableWidgetItem(name))
            self.overview_table.setItem(row, 1, QTableWidgetItem(value))

    def _update_quick_status(self, telemetry: dict[int, object]) -> None:
        tele = telemetry.get(self.selected_uav)
        if tele is None:
            return
        online = tele.is_online(datetime.now().timestamp(), 3.0)
        self.quick_labels["连接状态"].setText("在线" if online else "离线")
        self.quick_labels["飞行模式"].setText(tele.mode)
        self.quick_labels["当前位置"].setText(f"{tele.lat:.6f}, {tele.lon:.6f}")
        self.quick_labels["本地坐标"].setText(f"X {tele.local_x:.1f} / Y {tele.local_y:.1f} / Z {tele.local_z:.1f}")
        self.quick_labels["当前高度"].setText(f"Z {tele.local_z:.1f}  |  H {tele.alt:.1f} m")
        self.quick_labels["当前速度"].setText(f"{tele.speed:.1f} m/s")

    def _update_map_state(self, telemetry: dict[int, object]) -> None:
        if not self.map_ready:
            return

        center_lat = self.config.ui.default_map_center_lat
        center_lon = self.config.ui.default_map_center_lon
        for tele in telemetry.values():
            if tele.gps_valid:
                center_lat = tele.lat
                center_lon = tele.lon
                break

        payload = {
            "center": {"lat": center_lat, "lon": center_lon},
            "uavs": [
                {
                    "sysid": tele.sysid,
                    "lat": tele.lat,
                    "lon": tele.lon,
                    "x": tele.local_x,
                    "y": tele.local_y,
                    "z": tele.local_z,
                    "alt": tele.alt,
                    "mode": tele.mode,
                    "armed": tele.armed,
                    "trail": [
                        {"lat": lat, "lon": lon, "x": x, "y": y}
                        for lat, lon, x, y, _ts in list(tele.trail)[-120:]
                    ],
                }
                for tele in telemetry.values()
            ],
            "waypoints": [
                {
                    "lat": wp.lat,
                    "lon": wp.lon,
                    "idx": idx + 1,
                }
                for idx, wp in enumerate(self.waypoints)
            ],
        }
        self._run_js(f"window.updateAirSimState({json.dumps(payload, ensure_ascii=True)});")

    def send_command(self, command: str) -> None:
        if command in CRITICAL_COMMANDS:
            answer = QMessageBox.question(
                self,
                "确认命令",
                f"确认向 UAV{self.selected_uav} 发送“{COMMAND_LABELS.get(command, command)}”吗？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                return
        try:
            self.backend.send_basic_command(self.selected_uav, command)
            self.log(f"命令已发送 -> UAV{self.selected_uav}: {COMMAND_LABELS.get(command, command)}", "COMMAND")
        except Exception as exc:
            self.log(f"命令发送失败 -> UAV{self.selected_uav}: {exc}", "ERROR")

    def _on_map_clicked(self, lat: float, lon: float) -> None:
        target_z = float(self.alt_spin.value())
        if self.map_mode == "waypoint":
            self.waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=target_z))
            self.log(
                f"已添加航点 #{len(self.waypoints)}: lat={lat:.6f}, lon={lon:.6f}, z={target_z:.1f}",
                "INFO",
            )
            self._update_map_state(self.backend.telemetry)
            return

        answer = QMessageBox.question(
            self,
            "确认导航",
            f"确认让 UAV{self.selected_uav} 飞往该点吗？\n\n纬度={lat:.6f}\n经度={lon:.6f}\nZ={target_z:.1f}",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        try:
            self.backend.goto_gps(self.selected_uav, lat, lon, target_z)
            self.log(
                f"地图导航命令已发送 -> UAV{self.selected_uav}: lat={lat:.6f}, lon={lon:.6f}, z={target_z:.1f}",
                "COMMAND",
            )
            self.waypoints = [Waypoint(lat=lat, lon=lon, alt_m=target_z)]
        except Exception as exc:
            self.log(f"导航失败: {exc}", "ERROR")

    def _clear_waypoints(self) -> None:
        self.waypoints.clear()
        self.log("航点已清空。", "INFO")

    def _run_waypoints(self) -> None:
        if not self.waypoints:
            self.log("当前没有可执行的航点。", "WARN")
            return
        try:
            self.backend.run_waypoints(self.selected_uav, list(self.waypoints))
            self.log(f"航点任务已启动 -> UAV{self.selected_uav}: 共 {len(self.waypoints)} 个点", "COMMAND")
        except Exception as exc:
            self.log(f"航点任务执行失败: {exc}", "ERROR")

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
        return MAP_HTML.replace("__LAT__", str(self.config.ui.default_map_center_lat)).replace(
            "__LON__", str(self.config.ui.default_map_center_lon)
        )


MAP_HTML = """<!doctype html>
<html>
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<script src="qrc:///qtwebchannel/qwebchannel.js"></script>
<style>
html, body, #map { width: 100%; height: 100%; margin: 0; overflow: hidden; background: #f7f4ee; }
body { font-family: "Microsoft YaHei", "Segoe UI", sans-serif; }
#map { position: relative; }
#canvas { width: 100%; height: 100%; display: block; }
.hud {
  position: absolute;
  top: 12px;
  left: 12px;
  z-index: 10;
  display: flex;
  gap: 8px;
}
.badge {
  background: rgba(255,253,248,0.82);
  border: 1px solid rgba(177,168,145,0.45);
  color: #32433a;
  padding: 5px 9px;
  border-radius: 999px;
  font-size: 11px;
  box-shadow: 0 6px 18px rgba(41,56,48,0.08);
}
</style>
</head>
<body>
<div id="map">
  <canvas id="canvas"></canvas>
  <div class="hud">
    <div class="badge" id="statusBadge">已就绪</div>
    <div class="badge" id="scaleBadge">比例 1格 ≈ 10m</div>
  </div>
</div>
<script>
let bridge = null;
if (typeof QWebChannel !== 'undefined') {
  new QWebChannel(qt.webChannelTransport, function(channel) { bridge = channel.objects.bridge; });
}

const state = {
  center: { lat: __LAT__, lon: __LON__ },
  uavs: [],
  waypoints: [],
  scaleMetersPerGrid: 10,
};

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const statusBadge = document.getElementById('statusBadge');

function resize() {
  canvas.width = canvas.clientWidth = document.getElementById('map').clientWidth;
  canvas.height = canvas.clientHeight = document.getElementById('map').clientHeight;
  draw();
}

function emitClick(lat, lon) {
  if (bridge) bridge.mapClicked(lat, lon);
}

function allPoints() {
  const points = [];
  for (const uav of state.uavs) {
    points.push([uav.x || 0, uav.y || 0]);
    for (const p of (uav.trail || [])) points.push([p.x || 0, p.y || 0]);
  }
  return points;
}

function fitScale() {
  const pts = allPoints();
  if (!pts.length) return 10;
  let maxAbs = 20;
  for (const [x, y] of pts) {
    maxAbs = Math.max(maxAbs, Math.abs(x), Math.abs(y));
  }
  const smallerSide = Math.max(300, Math.min(canvas.width, canvas.height));
  const metersVisibleHalf = maxAbs + 20;
  const pxPerMeter = (smallerSide * 0.42) / metersVisibleHalf;
  return Math.max(4, Math.min(30, 1 / Math.max(pxPerMeter / 40, 0.01)));
}

function metersToPixels(meters) {
  return meters * (40 / state.scaleMetersPerGrid);
}

function localToCanvas(x, y) {
  return [
    canvas.width / 2 + metersToPixels(y),
    canvas.height / 2 - metersToPixels(x),
  ];
}

function canvasToLocal(x, y) {
  return [
    (canvas.height / 2 - y) / (40 / state.scaleMetersPerGrid),
    (x - canvas.width / 2) / (40 / state.scaleMetersPerGrid),
  ];
}

function llToLocal(lat, lon) {
  const lat0 = state.center.lat;
  const lon0 = state.center.lon;
  const earthRadius = 6378137.0;
  const x = (lat - lat0) * Math.PI / 180 * earthRadius;
  const y = (lon - lon0) * Math.PI / 180 * earthRadius * Math.cos(lat0 * Math.PI / 180);
  return [x, y];
}

function localToLL(x, y) {
  const lat0 = state.center.lat;
  const lon0 = state.center.lon;
  const earthRadius = 6378137.0;
  const lat = lat0 + (x / earthRadius) * 180 / Math.PI;
  const lon = lon0 + (y / (earthRadius * Math.cos(lat0 * Math.PI / 180))) * 180 / Math.PI;
  return [lat, lon];
}

function drawGrid() {
  const step = 40;
  const cols = Math.ceil(canvas.width / step);
  const rows = Math.ceil(canvas.height / step);
  const bg = ctx.createLinearGradient(0, 0, canvas.width, canvas.height);
  bg.addColorStop(0, '#f7f3ea');
  bg.addColorStop(1, '#efe7d8');
  ctx.fillStyle = bg;
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = 'rgba(115,120,102,0.14)';
  ctx.lineWidth = 1;
  for (let i = 0; i <= cols; i++) {
    const x = i * step;
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.height);
    ctx.stroke();
  }
  for (let i = 0; i <= rows; i++) {
    const y = i * step;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
  }

  ctx.strokeStyle = 'rgba(43,71,57,0.30)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(canvas.width / 2, 0);
  ctx.lineTo(canvas.width / 2, canvas.height);
  ctx.moveTo(0, canvas.height / 2);
  ctx.lineTo(canvas.width, canvas.height / 2);
  ctx.stroke();
}

function drawTrail(trail) {
  if (!trail || trail.length < 2) return;
  ctx.strokeStyle = 'rgba(15,157,88,0.55)';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  trail.forEach((p, index) => {
    const [cx, cy] = localToCanvas(p.x || 0, p.y || 0);
    if (index === 0) ctx.moveTo(cx, cy);
    else ctx.lineTo(cx, cy);
  });
  ctx.stroke();
}

function drawWaypoints() {
  if (state.waypoints.length > 1) {
    ctx.strokeStyle = '#f57c00';
    ctx.lineWidth = 2;
    ctx.setLineDash([8, 8]);
    ctx.beginPath();
    state.waypoints.forEach((wp, index) => {
      const [lx, ly] = llToLocal(wp.lat, wp.lon);
      const [cx, cy] = localToCanvas(lx, ly);
      if (index === 0) ctx.moveTo(cx, cy);
      else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
    ctx.setLineDash([]);
  }

  state.waypoints.forEach((wp) => {
    const [lx, ly] = llToLocal(wp.lat, wp.lon);
    const [cx, cy] = localToCanvas(lx, ly);
    ctx.fillStyle = '#f57c00';
    ctx.beginPath();
    ctx.arc(cx, cy, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#8a4700';
    ctx.font = '600 12px Microsoft YaHei';
    ctx.fillText(String(wp.idx), cx + 10, cy - 8);
  });
}

function drawUavs() {
  for (const uav of state.uavs) {
    drawTrail(uav.trail || []);
    const [cx, cy] = localToCanvas(uav.x || 0, uav.y || 0);
    ctx.fillStyle = '#0f9d58';
    ctx.beginPath();
    ctx.arc(cx, cy, 9, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2.5;
    ctx.stroke();
    ctx.fillStyle = '#153b2a';
    ctx.font = '600 12px Microsoft YaHei';
    ctx.fillText(`UAV${uav.sysid} ${uav.armed ? '已解锁' : '已上锁'}`, cx + 12, cy - 10);
  }
}

function draw() {
  state.scaleMetersPerGrid = fitScale();
  document.getElementById('scaleBadge').textContent = `比例 1格 ≈ ${state.scaleMetersPerGrid.toFixed(0)}m`;
  drawGrid();
  drawWaypoints();
  drawUavs();
}

window.updateAirSimState = function(payload) {
  state.center = payload.center || state.center;
  state.uavs = payload.uavs || [];
  state.waypoints = payload.waypoints || [];
  statusBadge.textContent = `飞机 ${state.uavs.length} 架`;
  draw();
};

canvas.addEventListener('click', function(event) {
  const rect = canvas.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  const [lx, ly] = canvasToLocal(x, y);
  const [lat, lon] = localToLL(lx, ly);
  emitClick(lat, lon);
});

window.addEventListener('resize', resize);
resize();
</script>
</body>
</html>
"""


APP_STYLE = """
QMainWindow { background: #f5f7f8; font-family: "Microsoft YaHei", "Segoe UI", sans-serif; }
QScrollArea#SidebarScroll {
    background: transparent;
    border: none;
}
QWidget#Sidebar { background: transparent; }
QMenuBar {
    background: #ffffff;
    color: #25313a;
    border: 1px solid #d8e0e6;
    border-radius: 8px;
    padding: 3px 6px;
}
QMenuBar::item {
    padding: 6px 10px;
    border-radius: 6px;
}
QMenuBar::item:selected { background: #e8eef2; }
QFrame#Hero, QFrame#MapPanel, QGroupBox#Card, QTabWidget::pane, QTableWidget#InfoTable, QTextEdit#LogView {
    background: #ffffff;
    border: 1px solid #d8e0e6;
    border-radius: 8px;
}
QGroupBox#Card {
    margin-top: 10px;
    color: #25313a;
}
QGroupBox#Card::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 6px;
    color: #1f5f7a;
    font-size: 13px;
    font-weight: 700;
}
QLabel#HeaderInfo {
    color: #62717c;
    font-size: 12px;
}
QLabel#StatusPill, QLabel#StatusPillWarn, QLabel#StatusPillError {
    color: #ffffff;
    border-radius: 999px;
    padding: 4px 10px;
    font-weight: 700;
    min-width: 72px;
}
QLabel#StatusPill { background: #22845d; }
QLabel#StatusPillWarn { background: #b7791f; }
QLabel#StatusPillError { background: #bf3f3f; }
QLabel#FieldName {
    color: #667783;
    font-size: 12px;
}
QLabel#QuickValue {
    color: #1f3948;
    font-size: 13px;
    font-weight: 700;
}
QPushButton {
    background: #216b84;
    color: #ffffff;
    border: none;
    border-radius: 8px;
    padding: 6px 8px;
    font-size: 14px;
    font-weight: 700;
    min-height: 34px;
}
QPushButton:hover { background: #18576d; }
QPushButton:pressed { background: #123f50; }
QComboBox, QDoubleSpinBox {
    background: #ffffff;
    color: #25313a;
    border: 1px solid #cfd9df;
    border-radius: 8px;
    padding: 6px 8px;
    font-size: 13px;
    min-height: 28px;
}
QComboBox:disabled {
    background: #eef2f4;
    color: #84919a;
}
QTabBar::tab {
    background: #e8eef2;
    color: #53636f;
    padding: 8px 14px;
    border-top-left-radius: 8px;
    border-top-right-radius: 8px;
    margin-right: 4px;
}
QTabBar::tab:selected {
    background: #ffffff;
    color: #1f5f7a;
    font-weight: 700;
}
QHeaderView::section {
    background: #edf2f5;
    color: #3f515d;
    border: none;
    border-bottom: 1px solid #d8e0e6;
    padding: 8px;
    font-size: 12px;
    font-weight: 700;
}
QTableWidget#InfoTable {
    gridline-color: #e4eaee;
    color: #25313a;
}
QTableWidget#InfoTable::item { padding: 7px; }
QTextEdit#LogView {
    color: #25313a;
    padding: 8px;
    font-size: 13px;
}
QTextEdit#LogView {
    font-family: Consolas, "Courier New", monospace;
    font-size: 12px;
}
"""


DARK_STYLE = """
QMainWindow { background: #172026; font-family: "Microsoft YaHei", "Segoe UI", sans-serif; }
QScrollArea#SidebarScroll {
    background: transparent;
    border: none;
}
QWidget#Sidebar { background: transparent; }
QMenuBar {
    background: #202c33;
    color: #e8eef2;
    border: 1px solid #33444e;
    border-radius: 8px;
    padding: 3px 6px;
}
QMenuBar::item {
    padding: 6px 10px;
    border-radius: 6px;
}
QMenuBar::item:selected { background: #2b3a43; }
QFrame#Hero, QFrame#MapPanel, QGroupBox#Card, QTabWidget::pane, QTableWidget#InfoTable, QTextEdit#LogView,
QTextEdit#AgentInput, QTextEdit#AgentPlanView {
    background: #202c33;
    border: 1px solid #33444e;
    border-radius: 8px;
}
QGroupBox#Card {
    margin-top: 10px;
    color: #e8eef2;
}
QGroupBox#Card::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 6px;
    color: #7cc4df;
    font-size: 13px;
    font-weight: 700;
}
QLabel#HeaderInfo {
    color: #9eb0bb;
    font-size: 12px;
}
QLabel#StatusPill, QLabel#StatusPillWarn, QLabel#StatusPillError {
    color: #ffffff;
    border-radius: 999px;
    padding: 4px 10px;
    font-weight: 700;
    min-width: 72px;
}
QLabel#StatusPill { background: #2ea043; }
QLabel#StatusPillWarn { background: #d29922; }
QLabel#StatusPillError { background: #f85149; }
QLabel#FieldName {
    color: #9eb0bb;
    font-size: 12px;
}
QLabel#QuickValue {
    color: #b5e8f7;
    font-size: 13px;
    font-weight: 700;
}
QPushButton {
    background: #26738c;
    color: #ffffff;
    border: none;
    border-radius: 8px;
    padding: 6px 8px;
    font-size: 14px;
    font-weight: 700;
    min-height: 34px;
}
QPushButton:hover { background: #2f88a6; }
QPushButton:pressed { background: #1e5d72; }
QComboBox, QDoubleSpinBox {
    background: #172026;
    color: #e8eef2;
    border: 1px solid #33444e;
    border-radius: 8px;
    padding: 6px 8px;
    font-size: 13px;
    min-height: 28px;
}
QComboBox:disabled {
    background: #1b262d;
    color: #7f929e;
}
QTabBar::tab {
    background: #172026;
    color: #9eb0bb;
    padding: 8px 14px;
    border-top-left-radius: 8px;
    border-top-right-radius: 8px;
    margin-right: 4px;
}
QTabBar::tab:selected {
    background: #202c33;
    color: #7cc4df;
    font-weight: 700;
}
QHeaderView::section {
    background: #172026;
    color: #c8d5dc;
    border: none;
    border-bottom: 1px solid #33444e;
    padding: 8px;
    font-size: 12px;
    font-weight: 700;
}
QTableWidget#InfoTable {
    gridline-color: #2d3e47;
    color: #e8eef2;
}
QTableWidget#InfoTable::item { padding: 7px; }
QTextEdit#LogView, QTextEdit#AgentInput, QTextEdit#AgentPlanView {
    color: #e8eef2;
    padding: 8px;
    font-size: 13px;
}
QTextEdit#LogView, QTextEdit#AgentPlanView {
    font-family: Consolas, "Courier New", monospace;
    font-size: 12px;
}
"""
