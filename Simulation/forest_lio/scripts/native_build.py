"""Build source-hashed local native helpers; no changes to catkin package builds."""
import ctypes
import hashlib
import os
from pathlib import Path
import subprocess

def load_shared(filename):
    source=Path(__file__).resolve().parents[1]/"native"/filename
    compiler=os.environ.get("CXX","g++")
    flags=["-O3","-std=c++14","-shared","-fPIC","-fopenmp"]
    key=hashlib.sha256(source.read_bytes()+repr([compiler,flags]).encode()).hexdigest()[:16]
    build=source.parents[3]/"artifacts/forest_lio/native";build.mkdir(parents=True,exist_ok=True)
    library=build/(source.stem+"_"+key+".so")
    if not library.exists():
        temporary=build/(source.stem+"_"+key+"."+str(os.getpid())+".so")
        subprocess.run([compiler,*flags,str(source),"-o",str(temporary)],check=True)
        os.replace(temporary,library)
    return ctypes.CDLL(str(library)),str(library)
