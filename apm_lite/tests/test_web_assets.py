from html.parser import HTMLParser
from pathlib import Path


WEB_ROOT = Path(__file__).resolve().parents[1] / "web"


class _GroundStationHtml(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []
        self.stylesheets = []
        self.scripts = []
        self.commands = []

    def handle_starttag(self, tag, attrs):
        values = dict(attrs)
        if "id" in values:
            self.ids.append(values["id"])
        if tag == "link" and values.get("rel") == "stylesheet":
            self.stylesheets.append(values.get("href"))
        if tag == "script" and values.get("src"):
            self.scripts.append(values["src"])
        if values.get("data-command"):
            self.commands.append(values["data-command"])


def _document():
    parser = _GroundStationHtml()
    parser.feed((WEB_ROOT / "index.html").read_text(encoding="utf-8"))
    return parser


def test_ground_station_assets_and_ids_are_self_consistent():
    document = _document()

    assert document.stylesheets == ["/styles.css?v=20260731.2"]
    assert document.scripts == ["/app.js?v=20260731.2"]
    assert len(document.ids) == len(set(document.ids))
    assert {"vehicleSelect", "nedCanvas", "evidenceList", "cameraFrame",
            "p9Badge", "agentBadge", "linkDiagnostics"} <= set(
        document.ids
    )
    assert (WEB_ROOT / "styles.css").stat().st_size > 10_000
    assert (WEB_ROOT / "app.js").stat().st_size > 10_000


def test_ground_station_exposes_only_supported_m1_flight_commands():
    document = _document()

    assert set(document.commands) == {
        "arm",
        "disarm",
        "takeoff",
        "hold",
        "land",
        "rtl",
    }
    assert "cmd_vel" not in (WEB_ROOT / "app.js").read_text(encoding="utf-8")


def test_real_controls_require_onboard_command_output_gate():
    javascript = (WEB_ROOT / "app.js").read_text(encoding="utf-8")

    assert "state.status?.command_output_enabled === true" in javascript
    assert 'deploymentMode !== "real" || onboardOutputEnabled' in javascript
    assert "state.status?.allowed_commands" in javascript
    assert "allowedCommands.has" in javascript
    assert "button.dataset.command" in javascript
    assert "只读验收模式：机载飞控命令输出已禁用" in javascript


def test_ground_station_does_not_claim_ros_depth_or_unconfigured_vlm():
    html = (WEB_ROOT / "index.html").read_text(encoding="utf-8")

    assert "ROS" not in html
    assert "D435" not in html
    assert 'id="vlmBadge" class="camera-badge unavailable"' in html
    assert 'id="cameraBadge" class="camera-badge offline"' in html


def test_ground_station_exposes_visible_semantic_and_agent_controls():
    document = _document()
    required = {
        "visionAnalysisForm",
        "visionPrompt",
        "analyzeVision",
        "missionParseForm",
        "missionInstruction",
        "parseMission",
        "semanticResult",
        "visionWorkspace",
        "missionWorkspace",
        "semanticResultTitle",
        "agentConversation",
        "agentVehicleState",
        "agentLinkState",
        "agentGpsState",
        "agentPermissionState",
        "agentDraft",
        "confirmAgentDraft",
        "cancelAgentDraft",
        "resetAgentSession",
    }
    assert required <= set(document.ids)
    javascript = (WEB_ROOT / "app.js").read_text(encoding="utf-8")
    assert "/api/semantic/vision/analyze" in javascript
    assert "/api/agent/sessions" in javascript
    assert "/api/agent/drafts/" in javascript
    assert "/api/semantic/mission/parse" not in javascript
    assert "raw_response" in javascript
    assert "flight_command_generated" not in javascript


def test_camera_and_semantic_workspace_occupy_the_center_column():
    html = (WEB_ROOT / "index.html").read_text(encoding="utf-8")
    right_start = html.index('<section class="right-column">')
    center_start = html.index('<section class="center-column">')
    main_end = html.index("</main>")
    right = html[right_start:center_start]
    center = html[center_start:main_end]

    assert all(item in center for item in (
        'id="cameraFrame"',
        'id="visionWorkspace"',
        'id="missionWorkspace"',
    ))
    assert all(item in right for item in (
        'id="controlState"',
        'id="nedCanvas"',
        'id="evidenceList"',
    ))


def test_ground_station_has_desktop_and_mobile_layout_breakpoints():
    css = (WEB_ROOT / "styles.css").read_text(encoding="utf-8")

    assert 'grid-template-areas: "left center right"' in css
    assert ".center-column #visionWorkspace" in css
    assert "aspect-ratio: 16 / 9" in css
    assert "@media (max-width: 1180px)" in css
    assert "@media (max-width: 780px)" in css
    assert "@media (max-width: 420px)" in css
    assert "letter-spacing: 0" in css


def test_ground_station_exposes_georeference_editor_and_round_trip_result():
    document = _document()
    required = {
        "geoSettingsButton",
        "geoDialog",
        "geoForm",
        "geoCalibrationId",
        "geoStatus",
        "geoHeading",
        "geoMapLatitude",
        "geoAirSimLatitude",
        "geoHomes",
        "geoCurrentHash",
        "geoRoundTrip",
    }
    assert required <= set(document.ids)

    javascript = (WEB_ROOT / "app.js").read_text(encoding="utf-8")
    assert 'requestJson("/api/georeference")' in javascript
    assert 'method: "PUT"' in javascript
    assert "map_x_heading_from_true_north_deg" in javascript
    assert "round_trip_report" in javascript
