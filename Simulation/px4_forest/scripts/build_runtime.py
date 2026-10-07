#!/usr/bin/env python3
"""Build pinned PX4/Gazebo dependencies in the project's ignored artifact directory."""
import argparse,json,subprocess,os
from pathlib import Path
import shutil

def run(args,cwd=None):subprocess.run([str(a) for a in args],cwd=cwd,check=True)
def main():
    os.environ["PATH"]=":".join(p for p in os.environ["PATH"].split(":") if not p.startswith("/mnt/"))
    os.environ["CCACHE_DIR"]=str(Path("artifacts/px4_forest/ccache").resolve())
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source",type=Path,default=Path("/home/mspace/workspace/PX4_Firmware"))
    ap.add_argument("--jobs",type=int,default=2)
    ap.add_argument("--output",type=Path,default=Path("artifacts/px4_forest/PX4-Autopilot"))
    a=ap.parse_args()
    if a.jobs<1:ap.error("jobs must be positive")
    dst=a.output.resolve();src=a.source.resolve();dst.parent.mkdir(parents=True,exist_ok=True)
    if dst==src:raise ValueError("Build must be isolated from source checkout")
    pinned="46a12a09bf11c8cbafc5ad905996645b4fe1a9df"
    if not dst.exists():
        run(["git","clone","--local","--no-hardlinks",src,dst])
        run(["git","-C",dst,"checkout","--detach",pinned])
    if subprocess.check_output(["git","-C",str(dst),"rev-parse","HEAD"],text=True).strip()!=pinned:
        raise ValueError("PX4 clone must use the validated v1.13.2 commit "+pinned)
    # Prefer cached exact submodule revisions; source checkout is never modified.
    needed=["Tools/sitl_gazebo","src/lib/events/libevents","src/modules/mavlink/mavlink","src/drivers/gps/devices"]
    source_git=Path(subprocess.check_output(["git","-C",str(src),"rev-parse","--absolute-git-dir"],text=True).strip())
    for rel in needed:
        target=dst/rel
        commit=subprocess.check_output(["git","-C",str(dst),"ls-tree","HEAD",rel],text=True).split()[2]
        if not (target/".git").exists():
            cache=source_git/"modules"/rel
            if cache.exists():
                run(["git","clone","--local","--no-hardlinks",cache,target])
                run(["git","-C",target,"checkout","--detach",commit])
            else:
                run(["git","submodule","update","--init",rel],dst)
        run(["git","submodule","init",rel],dst)
    run(["git","submodule","update","--init","--recursive","src/modules/mavlink/mavlink","Tools/sitl_gazebo"],dst)
    run(["make","px4_sitl_default","-j"+str(a.jobs)],dst)
    run(["cmake","--build",dst/"build/px4_sitl_default","--target","sitl_gazebo","--","-j"+str(a.jobs)])
    root=Path(__file__).resolve().parents[3]
    run(["cmake","-S",root/"Simulation/px4_forest/native","-B",root/"artifacts/px4_forest/plant_build","-DCMAKE_BUILD_TYPE=Release"])
    run(["cmake","--build",root/"artifacts/px4_forest/plant_build","-j1"])
    print(dst/"build/px4_sitl_default/bin/px4",flush=True)
if __name__=="__main__":main()
