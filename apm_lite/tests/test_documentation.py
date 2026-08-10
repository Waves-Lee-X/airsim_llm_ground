import re
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DOCS_ROOT = PROJECT_ROOT / "docs"
MARKDOWN_LINK = re.compile(r"\[[^\]]*\]\(([^)]+\.md(?:#[^)]*)?)\)")

EXPECTED_DOCUMENTS = {
    "01-实机硬件与接线基线.md",
    "02-总体架构与数据流.md",
    "03-Lite-v1通信协议详解.md",
    "04-配置文件与运行模式.md",
    "05-Web地面站设计与接口.md",
    "06-树莓派机载端设计.md",
    "07-仿真环境启动与调试.md",
    "08-实机部署与P9串口调试.md",
    "09-相机视频与视觉语义.md",
    "10-Mission-Agent设计与调试.md",
    "11-场地标定与坐标转换.md",
    "12-导航状态机与故障注入.md",
    "13-轨迹证据与虚实误差.md",
    "14-常见故障定位与恢复.md",
    "15-测试验收与后续开发.md",
    "16-编队设计与验收.md",
    "17-视觉目标导航与深度停靠.md",
    "18-M1多保真平行推演与证据包.md",
    "19-无人机身份文档.md",
    "90-M0协议与软件边界验收记录.md",
    "91-M1单机自动仿真验收记录.md",
    "92-M1.5手动仿真与Web联调记录.md",
    "93-UAV3实机链路与视频联调记录.md",
    "94-M5四机编队仿真验收记录.md",
    "95-视觉目标导航与Agent闭环仿真验收记录.md",
    "README.md",
}

OLD_DOCUMENT_NAMES = {
    "GEOREFERENCE.md",
    "HARDWARE_BASELINE.md",
    "M0_ACCEPTANCE.md",
    "M1_5_ACCEPTANCE.md",
    "M1_PROGRESS.md",
    "M3_NAVIGATION_ACCEPTANCE.md",
    "MANUAL_SIMULATION.md",
    "MISSION_AGENT.md",
    "REAL_SERIAL_DEPLOYMENT.md",
    "TRAJECTORY_EVIDENCE.md",
    "UAV3_REAL_BENCH_AND_VIDEO.md",
}


def test_chinese_document_set_and_relative_links_are_complete():
    actual_documents = {path.name for path in DOCS_ROOT.glob("*.md")}
    assert actual_documents == EXPECTED_DOCUMENTS

    markdown_files = [PROJECT_ROOT / "README.md", *sorted(DOCS_ROOT.glob("*.md"))]
    broken_links = []
    stale_names = []
    for markdown_file in markdown_files:
        content = markdown_file.read_text(encoding="utf-8")
        stale_names.extend(
            (markdown_file, name)
            for name in OLD_DOCUMENT_NAMES
            if name in content
        )
        for raw_target in MARKDOWN_LINK.findall(content):
            target = raw_target.split("#", 1)[0].strip("<>")
            if "://" in target:
                continue
            resolved = (markdown_file.parent / target).resolve()
            if not resolved.is_file():
                broken_links.append((markdown_file, raw_target))

    assert stale_names == []
    assert broken_links == []
