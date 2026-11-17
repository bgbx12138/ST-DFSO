"""Recover an interrupted run without warming its retry with discarded work."""
from pathlib import Path
import contextlib,hashlib,json,sqlite3,time,os
import psutil

def write_json(path,value):
    path=Path(path);tmp=path.with_name(path.name+'.partial')
    with tmp.open('w',encoding='utf8',newline='\n') as stream:
        json.dump(value,stream,indent=2,allow_nan=False);stream.flush();os.fsync(stream.fileno())
    tmp.replace(path)

@contextlib.contextmanager
def cohort_lock(out):
    import msvcrt
    with (Path(out)/'.running.lock').open('a+b') as stream:
        try:msvcrt.locking(stream.fileno(),msvcrt.LK_NBLCK,1)
        except OSError as e:raise RuntimeError('This cohort is already running; do not start a second copy') from e
        try:yield
        finally:stream.seek(0);msvcrt.locking(stream.fileno(),msvcrt.LK_UNLCK,1)

def checkpoint(cache,upper=None):
    with sqlite3.connect(Path(cache).as_uri()+'?mode=ro',uri=True) as db:
        maximum=db.execute('SELECT COALESCE(MAX(rowid),0) FROM unified_responses').fetchone()[0] if upper is None else upper
        digest=hashlib.sha256();count=0
        for row in db.execute('SELECT rowid,input_sha256,payload FROM unified_responses WHERE rowid<=? ORDER BY rowid',(maximum,)):
            digest.update(json.dumps(row,separators=(',',':')).encode());count+=1
        revision=dict(db.execute('SELECT name,value FROM unified_metadata'))['revision']
    return dict(max_rowid=maximum,records=count,sha256=digest.hexdigest(),revision=revision)

def recover(dest,cache,out):
    dest,cache,out=Path(dest).resolve(),Path(cache).resolve(),Path(out).resolve()
    if not dest.is_relative_to(out/'runs') or not cache.is_relative_to(out):
        raise ValueError('Recovery must stay inside this cohort')
    cost_path=dest/'cost.json';cost=json.loads(cost_path.read_text()) if cost_path.exists() else None
    if cost and cost['returncode']==0 and cost['complete_job_accounting'] and not cost['observer_error']:
        from collect_run import collect
        value=collect(dest);write_json(dest/'outcome.json',value)
        return 'recovered_completed_output'
    if cost and 'KeyboardInterrupt' not in str(cost.get('observer_error')):
        raise RuntimeError('Run failed for a non-interruption reason; inspect '+str(dest/'console.log'))
    trace=dest/'cpu_trace.jsonl'
    if trace.exists():
        with trace.open() as f:first=json.loads(next(f))
        try:
            p=psutil.Process(first['root_pid'])
            if p.create_time()<=first['utc_epoch_ns']/1e9+2:
                raise RuntimeError('Original run is still active; refusing recovery')
        except psutil.NoSuchProcess:pass
    launch=dest/'launch.json'
    removed=0
    if launch.exists():
        mark=json.loads(launch.read_text())['cache_checkpoint']
        if checkpoint(cache,mark['max_rowid'])!=mark:
            raise RuntimeError('Cache prefix changed; cannot restore interrupted start condition')
        with sqlite3.connect(cache) as db:
            removed=db.execute('DELETE FROM unified_responses WHERE rowid>?',(mark['max_rowid'],)).rowcount
            db.commit()
        assert checkpoint(cache)==mark
    archive=out/'interrupted'/dest.name/str(time.time_ns())
    if not archive.resolve().is_relative_to(out):raise ValueError('Archive escaped cohort')
    archive.parent.mkdir(parents=True,exist_ok=True)
    write_json(dest/'recovery.json',dict(reason='Interrupted run retained; only its new cache rows removed',removed_cache_rows=removed))
    dest.rename(archive)
    return 'archived_interrupted_run'
