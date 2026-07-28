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

    assert document.stylesheets == ["/styles.css"]
    assert document.scripts == ["/app.js"]
    assert len(document.ids) == len(set(document.ids))
    assert {"vehicleSelect", "nedCanvas", "evidenceList", "cameraFrame"} <= set(
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


def test_ground_station_does_not_claim_ros_depth_or_vlm_availability():
    html = (WEB_ROOT / "index.html").read_text(encoding="utf-8")

    assert "ROS" not in html
    assert "D435" not in html
    assert 'id="vlmBadge" class="camera-badge unavailable"' in html
    assert 'id="cameraBadge" class="camera-badge offline"' in html


def test_ground_station_has_desktop_and_mobile_layout_breakpoints():
    css = (WEB_ROOT / "styles.css").read_text(encoding="utf-8")

    assert "@media (max-width: 1180px)" in css
    assert "@media (max-width: 780px)" in css
    assert "@media (max-width: 420px)" in css
    assert "letter-spacing: 0" in css
