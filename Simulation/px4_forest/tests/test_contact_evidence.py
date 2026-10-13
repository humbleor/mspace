import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from contact_evidence import contact_acceptance


class ContactEvidence(unittest.TestCase):
    def test_silence_and_timeline_gaps_never_prove_no_contact(self):
        self.assertFalse(contact_acceptance([],0)["passed"])
        self.assertFalse(contact_acceptance([],2,True,[0,.2],(0,.2))["passed"])
        self.assertFalse(contact_acceptance([],2,True,[.1,.11],(0,.2))["passed"])
        self.assertTrue(contact_acceptance([],3,True,[0,.004,.008],(0,.008))["passed"])

    def test_real_contact_fails_even_with_valid_monitor(self):
        rows=[{"sim_time_s":2,"force_n":[3,4,0]},{"sim_time_s":1,"force_n":[0,1,0]}]
        r=contact_acceptance(rows,100)
        self.assertFalse(r["passed"]);self.assertEqual(r["status"],"contact")
        self.assertEqual(r["first_contact"]["sim_time_s"],1);self.assertEqual(r["max_force_n"],5)

    def test_disabled_cannot_be_claimed_as_clear(self):
        r=contact_acceptance([],100,False)
        self.assertEqual(r["status"],"disabled");self.assertFalse(r["passed"])
