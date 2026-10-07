import os,shutil,subprocess,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from runtime_model import precise_timeout_scaling

class SlowClockTimeouts(unittest.TestCase):
    @unittest.skipUnless(shutil.which("bc"),"PX4 timeout calculation requires bc")
    def test_offboard_timeout_retains_fraction_at_slow_speed(self):
        expression='echo "$PX4_SIM_SPEED_FACTOR * 0.5" | bc'
        def evaluate(script,speed):
            env=dict(os.environ,PX4_SIM_SPEED_FACTOR=str(speed))
            return float(subprocess.check_output(["bash","-c",script],env=env,text=True))
        self.assertEqual(evaluate(expression,.1),0)
        self.assertAlmostEqual(evaluate(precise_timeout_scaling(expression),.1),.05)
        self.assertAlmostEqual(evaluate(precise_timeout_scaling(expression),.25),.125)
        self.assertAlmostEqual(evaluate(precise_timeout_scaling(expression),1),.5)

if __name__=="__main__":unittest.main()
