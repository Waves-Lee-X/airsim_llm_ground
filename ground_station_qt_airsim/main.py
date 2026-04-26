from __future__ import annotations

import argparse
import sys

from ground_station_qt_airsim.config import load_config
from ground_station_qt_airsim.window import AirSimGroundStationWindow


def run() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config", default="config_airsim.yaml")
    args, qt_args = parser.parse_known_args(sys.argv[1:])
    app = AirSimGroundStationWindow.create_application([sys.argv[0], *qt_args])
    window = AirSimGroundStationWindow(config=load_config(args.config))
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    run()
