"""One-command entry after the four preflight stages have passed."""
from pathlib import Path
import argparse, json, os, subprocess, sys,hashlib,time,ctypes
from cohort_integrity import ROOT,verify_freeze
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=Path('outputs/paper_unified_810_20261116'))
    ap.add_argument('--resume',action='store_true');ap.add_argument('--check-only',action='store_true')
    a=ap.parse_args();os.chdir(ROOT);freeze=verify_freeze()
    print('Verified frozen experiment: '+freeze['identity']['signature'],flush=True)
    subprocess.run([sys.executable,'tools/check_environment.py','--historical-hashes'],check=True)
    subprocess.run([sys.executable,'tools/check_repository.py'],check=True)
    subprocess.run([sys.executable,'tools/check_model.py'],check=True)
    if a.check_only:return 0
    cmd=[sys.executable,'tools/run_experiments.py','--suite','all','--output',str(a.output),'--execute']
    if a.resume:cmd.append('--resume')
    # Keep the system awake while the batch and analysis run; the display may sleep.
    keep_awake=ctypes.windll.kernel32.SetThreadExecutionState
    if not keep_awake(0x80000001):raise OSError('Cannot request execution state')
    try:
        subprocess.run(cmd,check=True)
        analyse(a.output.resolve())
    finally:keep_awake(0x80000000)
    print('All 810 runs and analysis completed: '+str(a.output.resolve()),flush=True)
    return 0

def analyse(out):
    from cohort_recovery import write_json
    source=out/'outcomes.json';target=out/'analysis';receipt=target/'complete.json'
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    if receipt.exists():
        saved=json.loads(receipt.read_text())
        if saved['input_sha256']==digest(source) and all(digest(target/p)==h for p,h in saved['files'].items()):return
        raise RuntimeError('Completed analysis changed; inspect before replacing')
    if target.exists():
        archive=out/'interrupted'/('analysis_'+str(time.time_ns()))
        if not archive.resolve().is_relative_to(out):raise ValueError('Archive escaped output')
        archive.parent.mkdir(parents=True,exist_ok=True);target.rename(archive)
    subprocess.run([sys.executable,str(ROOT/'analysis/complete_set_analysis.py'),'--input',str(source),'--output',str(target)],check=True)
    write_json(receipt,dict(input_sha256=digest(source),files={p.name:digest(p) for p in target.glob('*.csv')}))
if __name__=='__main__':raise SystemExit(main())
