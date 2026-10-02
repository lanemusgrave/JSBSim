"""Module 03 - run any script that writes a CSV and plot every logged column.

    python modules/03_scripts_events_output/solutions/run_card.py PATH/TO/card.xml [--show]
"""

import gc
import xml.etree.ElementTree as ET
from pathlib import Path

from gnclab.cli import parse_args

args = parse_args(__doc__, extra=lambda p: p.add_argument(
    "card", nargs="?", default=str(Path(__file__).with_name("c172_ap_card.xml"))))

import pandas as pd  # noqa: E402

from gnclab import OUTPUT_DIR, load_script  # noqa: E402
from gnclab.plotting import save, timehistory  # noqa: E402

card = Path(args.card)
csv_name = ET.parse(card).getroot().find("output").get("name")

fdm = load_script(card, output=True)
while fdm.run():
    pass
del fdm
gc.collect()

log = pd.read_csv(OUTPUT_DIR / csv_name).set_index("Time")
print(log.iloc[:: max(1, len(log) // 10)].round(2).to_string())
fig, _ = timehistory(log, list(log.columns), title=card.name)
save(fig, "03_scripts_events_output", card.stem, show=args.show)
