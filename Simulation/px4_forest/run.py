#!/usr/bin/env python3
"""PX4 flight-stack-in-loop experiments; run from the project root after ROS setup."""
import os,sys,subprocess
from pathlib import Path
base=Path(__file__).resolve().parent
commands={"build":base/"scripts/build_runtime.py","run":base/"scripts/run_sitl.py","swarm":base/"scripts/run_swarm.py","test":base/"tests/test_motion.py","report":base/"scripts/summarize_sitl.py","stop":base/"scripts/process_lifecycle.py"}
if len(sys.argv)<2 or sys.argv[1] not in commands:
    print("Use: python3 Simulation/px4_forest/run.py {build,run,swarm,test,report,stop} [arguments]")
    raise SystemExit(0 if len(sys.argv)<2 or sys.argv[1] in ("-h","--help") else 2)
if sys.argv[1]=="test":
    suites=[base/"checks"]+([base/"tests"] if (base/"tests").is_dir() else [])
    codes=[subprocess.call([sys.executable,"-m","unittest","discover","-s",str(suite)]+sys.argv[2:]) for suite in suites]
    raise SystemExit(int(any(codes)))
os.execv(sys.executable,[sys.executable,str(commands[sys.argv[1]])]+sys.argv[2:])
