#!/usr/bin/env python3
"""Discover sensor-model and native renderer verification tests."""
from pathlib import Path
import sys
import unittest
if __name__=="__main__":
    unittest.main(module=None,argv=[sys.argv[0],"discover","-s",str(Path(__file__).resolve().parent)]+sys.argv[1:])
