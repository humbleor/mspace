import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from process_lifecycle import owned_processes,stop_owned,process_identity,send_signal


class ProcessLifecycle(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.target=self.root/'run';self.target.mkdir()
        self.procs=[]

    def tearDown(self):
        for p in self.procs:
            if p.poll() is None:p.kill()
            p.wait(timeout=3)
        for item in owned_processes(self.target):
            send_signal(item,signal.SIGKILL)
        self.temp.cleanup()

    def spawn(self,code,target=None):
        env=os.environ.copy();env['ROS_HOME']=str((target or self.target)/'ros_home')
        p=subprocess.Popen([sys.executable,'-c',code],env=env,start_new_session=True,
                           stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        self.procs.append(p);return p

    def ready(self,path):
        end=time.monotonic()+4
        while time.monotonic()<end:
            if path.exists():return
            time.sleep(.02)
        self.fail('Fixture did not become ready')

    def test_detached_grandchild_survives_leader_then_is_killed_without_other_run(self):
        marker=self.root/'grandchild'
        child="import os,signal,time;from pathlib import Path;signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN);Path("+repr(str(marker))+").write_text(str(os.getpid()));time.sleep(60)"
        leader=self.spawn("import subprocess,sys;subprocess.Popen([sys.executable,'-c',"+repr(child)+"],start_new_session=True)")
        leader.wait(timeout=4);self.ready(marker);pid=int(marker.read_text())
        other=self.root/'run_extra';other.mkdir()
        sentinel=self.spawn('import time;time.sleep(60)',other)
        time.sleep(.05)
        report=stop_owned(self.target,grace=.12,term=.12,kill=.6)
        self.assertEqual(report['status'],'complete')
        self.assertEqual(owned_processes(self.target),[])
        self.assertIsNone(sentinel.poll())
        self.assertTrue(any(e['pid']==pid and e['signal']=='SIGKILL' for e in report['signals']))
        self.assertEqual(json.loads((self.target/'cleanup.json').read_text())['remaining'],[])

    def test_clock_protected_until_navigation_exits(self):
        ready=self.root/'ready';clock_ready=self.root/'clock_ready'
        nav_end=self.root/'nav_end';clock_end=self.root/'clock_end'
        nav=self.spawn("import signal,time,sys;from pathlib import Path;\ndef stop(*a):\n time.sleep(.2);Path("+repr(str(nav_end))+").write_text(str(time.monotonic()));sys.exit(0)\nsignal.signal(signal.SIGINT,stop);Path("+repr(str(ready))+").touch();time.sleep(60)")
        clock=self.spawn("import signal,time,sys;from pathlib import Path;\ndef stop(*a):\n Path("+repr(str(clock_end))+").write_text(str(time.monotonic()));sys.exit(0)\nsignal.signal(signal.SIGINT,stop);Path("+repr(str(clock_ready))+").touch();time.sleep(60)")
        self.ready(ready);self.ready(clock_ready)
        report=stop_owned(self.target,keep_alive=[clock.pid],grace=1,term=.1,kill=.1)
        self.assertEqual(report['status'],'complete')
        self.assertLess(float(nav_end.read_text()),float(clock_end.read_text()))

    def test_reused_pid_token_never_signalled(self):
        marker=self.root/'ready';p=self.spawn("import time;from pathlib import Path;Path("+repr(str(marker))+").touch();time.sleep(60)")
        self.ready(marker);info=process_identity(p.pid);info['start_ticks']+=1
        self.assertFalse(send_signal(info,signal.SIGKILL));self.assertIsNone(p.poll())

    def test_repeated_interrupt_during_cleanup_does_not_abort_controller(self):
        marker=self.root/'cleaning';finished=self.root/'finished'
        code="import sys,time;from pathlib import Path;sys.path.insert(0,"+repr(str(Path(__file__).resolve().parents[1]/'scripts'))+");from process_lifecycle import shutdown_signals_ignored\nwith shutdown_signals_ignored():\n Path("+repr(str(marker))+").touch();time.sleep(.4);Path("+repr(str(finished))+").touch()"
        p=self.spawn(code);self.ready(marker)
        os.kill(p.pid,signal.SIGINT);os.kill(p.pid,signal.SIGTERM)
        self.assertEqual(p.wait(timeout=3),0);self.assertTrue(finished.exists())

