import subprocess
import time

TERM = "gnome-terminal"

subprocess.Popen([
    TERM,
    "--geometry=90x24+900+0",
    "--", "bash", "-c",
    "python3 /home/michelle/manas-planner/mission_planner/cleo_connect.py; exec bash",
])

subprocess.Popen([
    TERM,
    "--geometry=90x24+900+0",
    "--", "bash", "-c",
    "python3 /home/michelle/manas-planner/mission_planner/freyja_connect.py; exec bash",
])
