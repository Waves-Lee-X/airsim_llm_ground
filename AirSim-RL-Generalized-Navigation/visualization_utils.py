import os
from pathlib import Path

import matplotlib.font_manager as font_manager
import matplotlib.pyplot as plt
import pandas as pd


def zh(text):
    return text.encode("ascii").decode("unicode_escape")


def configure_chinese_font():
    """Configure matplotlib to render Chinese labels on common Windows setups."""
    font_candidates = [
        Path(r"C:\Windows\Fonts\NotoSansSC-VF.ttf"),
        Path(r"C:\Windows\Fonts\simhei.ttf"),
        Path(r"C:\Windows\Fonts\msyh.ttc"),
        Path(r"C:\Windows\Fonts\simsun.ttc"),
    ]

    for font_path in font_candidates:
        if font_path.exists():
            font_manager.fontManager.addfont(str(font_path))
            font_name = font_manager.FontProperties(fname=str(font_path)).get_name()
            plt.rcParams["font.sans-serif"] = [font_name]
            plt.rcParams["axes.unicode_minus"] = False
            return font_name

    plt.rcParams["font.sans-serif"] = [
        "Noto Sans CJK SC",
        "Noto Sans SC",
        "SimHei",
        "Microsoft YaHei",
        "SimSun",
        "Arial Unicode MS",
    ]
    plt.rcParams["axes.unicode_minus"] = False
    return None


def _save_line_chart(df, y_column, title, ylabel, output_path):
    plt.figure(figsize=(12, 6))
    plt.plot(df["Episode"], df[y_column])
    plt.title(zh(title))
    plt.xlabel(zh(r"\u56de\u5408"))
    plt.ylabel(zh(ylabel))
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


def generate_training_visualizations(csv_filename="training_data.csv", output_dir="visualization"):
    configure_chinese_font()
    df = pd.read_csv(csv_filename)
    os.makedirs(output_dir, exist_ok=True)

    charts = [
        ("Total_Reward", r"\u8bad\u7ec3\u5956\u52b1\u66f2\u7ebf", r"\u603b\u5956\u52b1", "reward_curve.png"),
        ("Collisions", r"\u78b0\u649e\u6b21\u6570", r"\u78b0\u649e\u6b21\u6570", "collisions.png"),
        ("Navigation_Time", r"\u5bfc\u822a\u65f6\u95f4", r"\u65f6\u95f4 (\u79d2)", "navigation_time.png"),
        ("Path_Length", r"\u8def\u5f84\u957f\u5ea6", r"\u8def\u5f84\u957f\u5ea6 (\u7c73)", "path_length.png"),
        ("Obstacle_Avoidance_Count", r"\u907f\u969c\u6b21\u6570", r"\u907f\u969c\u6b21\u6570", "obstacle_avoidance.png"),
        ("Success", r"\u6210\u529f\u7387", r"\u6210\u529f (1=\u6210\u529f, 0=\u5931\u8d25)", "success_rate.png"),
    ]

    for y_column, title, ylabel, filename in charts:
        _save_line_chart(
            df,
            y_column,
            title,
            ylabel,
            os.path.join(output_dir, filename),
        )

    plt.figure(figsize=(12, 6))
    plt.plot(df["Episode"], df["Total_Reward"], label=zh(r"\u603b\u5956\u52b1"))
    plt.plot(df["Episode"], df["Path_Length"] / 10, label=zh(r"\u8def\u5f84\u957f\u5ea6 (\u7f29\u653e)"))
    plt.plot(df["Episode"], df["Navigation_Time"] * 10, label=zh(r"\u5bfc\u822a\u65f6\u95f4 (\u7f29\u653e)"))
    plt.title(zh(r"\u7efc\u5408\u6307\u6807"))
    plt.xlabel(zh(r"\u56de\u5408"))
    plt.ylabel(zh(r"\u503c"))
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "comprehensive.png"), dpi=150)
    plt.close()
