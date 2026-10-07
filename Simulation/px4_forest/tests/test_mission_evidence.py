import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from mission_evidence import ordered_arrivals

class MissionEvidence(unittest.TestCase):
    def test_unordered_visits_cannot_prove_completion(self):
        r=ordered_arrivals([[1,2,0,0],[2,1,0,0]],[[1,0,0],[2,0,0]],0)
        self.assertFalse(r["complete"])
        self.assertEqual(len(r["arrivals"]),1)
    def test_pretrigger_and_radius_boundary_do_not_count(self):
        r=ordered_arrivals([[1,0,0,0],[3,.5,0,0]],[[0,0,0]],2)
        self.assertFalse(r["complete"])
    def test_relocated_goal_does_not_prove_original_goal(self):
        samples=[[3,1,0,0]]
        self.assertTrue(ordered_arrivals(samples,[[1,0,0]],2)["complete"])
        self.assertFalse(ordered_arrivals(samples,[[0,0,0]],2)["complete"])
if __name__=="__main__":unittest.main()
