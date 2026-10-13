import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from single_trunk import assess,trunk_model


class SingleTrunkEvidence(unittest.TestCase):
    config={"duration":3,"launch_speed":1.5,"trunk":{"x":2,"y":0,"radius":.2,"height":3}}

    def test_collision_requires_contacts_and_motion_response(self):
        rows=np.zeros((2,10)); rows[:,0]=[0,3]; rows[-1,1]=1.6
        self.assertFalse(assess(rows,[],self.config,0)["passed"])
        self.assertTrue(assess(rows,[{"force_n":"1000"}],self.config,0)["passed"])
        rows[-1,1]=4.5; rows[-1,4]=1.5
        self.assertFalse(assess(rows,[{"force_n":"1000"}],self.config,0)["passed"])

    def test_bypass_requires_free_motion_and_no_contact(self):
        rows=np.zeros((2,10)); rows[:,0]=[0,3]; rows[:,2]=1; rows[-1,1]=4.5;rows[:,4]=1.5
        self.assertTrue(assess(rows,[],self.config,1)["passed"])
        self.assertFalse(assess(rows,[{"force_n":"1000"}],self.config,1)["passed"])

    def test_incomplete_evidence_rejected(self):
        with self.assertRaises(ValueError):assess(np.zeros((2,10)),[],self.config,0)

    def test_glancing_requires_contact_and_physical_deflection(self):
        rows=np.zeros((2,10));rows[:,0]=[0,3];rows[:,2]=.4;rows[:,4]=1.5;rows[-1,1]=4.5
        self.assertFalse(assess(rows,[{"force_n":"2"}],self.config,.4,True)["passed"])
        rows[-1,2]=.5
        self.assertTrue(assess(rows,[{"force_n":"2"}],self.config,.4,True)["passed"])

    def test_trunk_has_physical_collision_and_fixed_geometry(self):
        m=trunk_model(self.config["trunk"])
        self.assertEqual(m.findtext("static"),"true")
        self.assertEqual(m.findtext("link/collision/geometry/cylinder/radius"),"0.2")
        self.assertEqual(m.findtext("link/collision/geometry/cylinder/length"),"3")


if __name__=="__main__":unittest.main()
