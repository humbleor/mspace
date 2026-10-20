"""Linux process ownership and bounded shutdown for one isolated SITL run.

Ownership uses the exact ROS_HOME inherited at spawn, not executable names.
PID start ticks are rechecked before every signal. Other runs are never selected.
"""
import argparse
import contextlib
import json
import os
from pathlib import Path
import signal
import time


def process_identity(pid):
    try:
        base=Path('/proc')/str(pid)
        if base.stat().st_uid!=os.getuid():return None
        fields=(base/'stat').read_text().rsplit(')',1)[1].split()
        if fields[0]=='Z':return None
        return {'pid':pid,'start_ticks':int(fields[19]),'ppid':int(fields[1]),
                'pgid':int(fields[2]),'session':int(fields[3])}
    except (OSError,ValueError,IndexError):return None


def ancestors():
    result={os.getpid()};pid=os.getpid()
    while True:
        info=process_identity(pid)
        if not info or info['ppid'] in result or info['ppid']<=1:break
        pid=info['ppid'];result.add(pid)
    return result


def owned_processes(output,exclude=()):
    home=str((Path(output).resolve()/'ros_home')).encode()
    excluded=set(exclude)|ancestors();result=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        pid=int(entry.name)
        if pid<=1 or pid in excluded:continue
        info=process_identity(pid)
        if info is None:continue
        try:
            entries=(entry/'environ').read_bytes().split(b'\0')
            if b'ROS_HOME='+home not in entries:continue
            info['command']=(entry/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace').strip()
        except OSError:continue
        result.append(info)
    return result


def send_signal(info,signum):
    current=process_identity(info['pid'])
    if current is None or current['start_ticks']!=info['start_ticks']:return False
    try:os.kill(info['pid'],signum);return True
    except ProcessLookupError:return False


@contextlib.contextmanager
def shutdown_signals_ignored():
    old={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        for sig in old:signal.signal(sig,signal.SIG_IGN)
        yield
    finally:
        for sig,handler in old.items():signal.signal(sig,handler)


def stop_owned(output,keep_alive=(),grace=8.,term=3.,kill=2.,poll=.05):
    """Signal all owned descendants concurrently; keep clock/master until last."""
    output=Path(output).resolve();started=time.monotonic();events=[]
    protected={info['pid']:info['start_ticks'] for pid in keep_alive
               for info in [process_identity(pid)] if info is not None}
    def protected_pid(info):
        return protected.get(info['pid'])==info['start_ticks']
    def phase(include_protected):
        for sig,timeout in ((signal.SIGINT,grace),(signal.SIGTERM,term),(signal.SIGKILL,kill)):
            sent=set();deadline=time.monotonic()+timeout
            while True:
                pending=[p for p in owned_processes(output) if include_protected or not protected_pid(p)]
                if not pending:return
                for info in pending:
                    key=(info['pid'],info['start_ticks'])
                    if key not in sent and send_signal(info,sig):
                        sent.add(key);events.append(dict(info,signal=signal.Signals(sig).name))
                if time.monotonic()>=deadline:break
                time.sleep(poll)
    phase(False)
    phase(True)
    remaining=owned_processes(output)
    report={'output':str(output),'status':'complete' if not remaining else 'incomplete',
            'wall_s':time.monotonic()-started,'signals':events,'remaining':remaining}
    (output/'cleanup.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    ap=argparse.ArgumentParser(description='Stop only processes owned by one SITL output directory.')
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--dry-run',action='store_true')
    a=ap.parse_args();output=a.output.resolve()
    manifest=output/'manifest.json'
    if not manifest.is_file():ap.error('Output must contain the SITL manifest.json')
    data=json.loads(manifest.read_text())
    if 'parameters' not in data or 'ports' not in data:ap.error('Not a SITL run manifest')
    if Path(data['parameters']['output']).resolve()!=output:
        # Relative --output is resolved against the project, not the caller cwd.
        root=Path(__file__).resolve().parents[3]
        if (root/data['parameters']['output']).resolve()!=output:ap.error('Manifest output does not match target')
    if a.dry_run:
        print(json.dumps({'output':str(output),'owned_processes':owned_processes(output)},indent=2))
        return
    with shutdown_signals_ignored():report=stop_owned(output)
    print(json.dumps(report,indent=2))
    if report['remaining']:raise SystemExit(1)


if __name__=='__main__':main()

