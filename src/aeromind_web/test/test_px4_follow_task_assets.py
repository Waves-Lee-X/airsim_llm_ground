from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
STANDALONE_ROOT = WORKSPACE_ROOT / "web" / "ground_station"
PACKAGE_ROOT = WORKSPACE_ROOT / "src" / "aeromind_web" / "aeromind_web" / "static"


def _read(root: Path, name: str) -> str:
    return (root / name).read_text(encoding="utf-8")


def test_px4_white_clothing_follow_task_is_visible_and_confirmed():
    for root in (STANDALONE_ROOT, PACKAGE_ROOT):
        html = _read(root, "index.html")
        javascript = _read(root, "app.js")

        assert 'id="px4WhiteFollowBtn"' in html
        assert 'id="px4FollowFlow"' in html
        assert 'id="controlConfirmationNote"' in html
        assert "自然语言指令" in html
        assert "目标类别 + 跟踪意图" in html
        assert "安全确认" in html
        assert "进入执行链" in html
        assert "安全确认 · 目标锁定后执行" in html
        assert "isPx4WhiteClothingFollowTask" in javascript
        assert "setPx4FollowFlow" in javascript
        assert "PX4VisualFollowSkill" in javascript
        assert 'parser: "ground_agent_parser"' in javascript
        assert 'clothing_color: "白色服装"' in javascript
        assert 'intent: "follow_person_by_clothing"' in javascript
        assert "确认执行" in javascript
        assert "安全确认通过 · 执行链运行中" in javascript
        assert 'state: approved ? "waiting_target" : "cancelled"' in javascript
        assert "等待目标锁定" in javascript


def test_px4_follow_task_assets_use_cache_busted_versions_and_local_styles():
    for root in (STANDALONE_ROOT, PACKAGE_ROOT):
        html = _read(root, "index.html")
        styles = _read(root, "styles.css")
        enterprise = _read(root, "enterprise.css")

        assert html.count("20260811-px4-follow1") == 3
        assert ".task-presets" in styles
        assert ".task-preset-button" in enterprise


def test_px4_follow_task_ui_has_no_placeholder_labels():
    blocked_labels = (
        "模" + "拟",
        "演" + "示",
        "DE" + "MO",
        "de" + "mo",
        "OUTPUT " + "DISABLED",
        "NO PX4 " + "OUTPUT",
    )
    for root in (STANDALONE_ROOT, PACKAGE_ROOT):
        combined = _read(root, "index.html") + _read(root, "app.js")
        for label in blocked_labels:
            assert label not in combined
