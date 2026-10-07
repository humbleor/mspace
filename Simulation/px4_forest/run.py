#!/usr/bin/env python3
"""PX4 flight-stack-in-loop experiments; run from the project root after ROS setup."""
import subprocess,sys
from pathlib import Path
base=Path(__file__).resolve().parent
commands={"build":base/"scripts/build_runtime.py","run":base/"scripts/run_sitl.py","test":base/"tests/test_motion.py","report":base/"scripts/summarize_sitl.py"}
if len(sys.argv)<2 or sys.argv[1] not in commands:
    print("Use: python3 Simulation/px4_forest/run.py {build,run,test,report} [arguments]")
    raise SystemExit(0 if len(sys.argv)<2 or sys.argv[1] in ("-h","--help") else 2)
if sys.argv[1]=="test":
    raise SystemExit(subprocess.call([sys.executable,"-m","unittest","discover","-s",str(base/"tests")]+sys.argv[2:]))
raise SystemExit(subprocess.call([sys.executable,str(commands[sys.argv[1]])]+sys.argv[2:]))
