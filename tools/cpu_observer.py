# -*- coding: utf-8 -*-
"""Independent Windows Job CPU samples enclosed by raw, same-machine QPC."""
import ctypes as C
import json, math, subprocess, time
from process_handle import retained_popen, HOLDER
from pathlib import Path

from winjob import K, W, Accounting, ThreadEntry, checked,kill_descendants_on_owner_exit
K.QueryPerformanceCounter.argtypes = [C.POINTER(C.c_longlong)]
K.QueryPerformanceCounter.restype = W.BOOL
K.QueryPerformanceFrequency.argtypes = [C.POINTER(C.c_longlong)]
K.QueryPerformanceFrequency.restype = W.BOOL

def qpc():
    value = C.c_longlong(); checked(K.QueryPerformanceCounter(C.byref(value)))
    return value.value

def frequency():
    value = C.c_longlong(); checked(K.QueryPerformanceFrequency(C.byref(value)))
    assert value.value > 0
    return value.value

def run_with_qpc(cmd, cwd, log, env, timeout, trace, interval=.25):
    assert math.isfinite(interval) and .02 <= interval <= 60
    assert math.isfinite(timeout) and timeout > 0
    assert not Path(log).exists() and not Path(trace).exists(), 'PRESERVE_PRIOR_OUTPUT180'
    started = time.perf_counter(); parent_started = time.process_time()
    freq = frequency(); job = None; proc = None; rows = []; error = None; code = None
    complete = False; timed_out = False; stream = None; tr = None; assigned = False
    def query():
        info = Accounting()
        low = qpc()
        checked(K.QueryInformationJobObject(job, 1, C.byref(info), C.sizeof(info), None))
        high = qpc()
        assert low <= high
        return info, low, high
    def sample(phase):
        info, low, high = query()
        row = dict(index=len(rows), phase=phase, root_pid=proc.pid, qpc_before_ticks=low,
                   qpc_after_ticks=high, qpc_frequency=freq,
                   process_tree_cpu_seconds=(info.user + info.kernel) * 1e-7,
                   processes=int(info.processes), active=int(info.active),
                   wall_seconds=time.perf_counter() - started, utc_epoch_ns=time.time_ns(),
                   observer_cpu_seconds=time.process_time() - parent_started)
        tr.write(json.dumps(row, separators=(',', ':')) + '\n'); tr.flush(); rows.append(row)
        return info
    try:
        job = checked(K.CreateJobObjectW(None, None))
        kill_descendants_on_owner_exit(job)
        stream = open(log, 'xb'); tr = open(trace, 'x', encoding='utf-8', newline='\n')
        proc = retained_popen(cmd, cwd=cwd, stdout=stream, stderr=subprocess.STDOUT, env=env,
                                creationflags=subprocess.CREATE_NO_WINDOW | 0x4)
        checked(K.AssignProcessToJobObject(job, W.HANDLE(int(proc._handle))))
        assigned = True
        sample('suspended_before_start')
        snapshot = K.CreateToolhelp32Snapshot(4, 0); entry = ThreadEntry(); entry.size = C.sizeof(entry)
        resumed = 0
        try:
            yes = K.Thread32First(snapshot, C.byref(entry))
            while yes:
                if entry.owner == proc.pid:
                    thread = checked(K.OpenThread(2, False, entry.thread))
                    try:
                        if K.ResumeThread(thread) == 0xffffffff:
                            raise C.WinError(C.get_last_error())
                        resumed += 1
                    finally:
                        K.CloseHandle(thread)
                yes = K.Thread32Next(snapshot, C.byref(entry))
        finally:
            K.CloseHandle(snapshot)
        assert resumed, 'NO_INITIAL_THREAD_RESUMED180'
        while proc.poll() is None:
            sample('running')
            left = timeout - (time.perf_counter() - started)
            if left <= 0:
                checked(K.TerminateJobObject(job, 124)); proc.wait(); timed_out = True; break
            time.sleep(min(interval, left))
        code = 124 if timed_out else proc.returncode
        info = sample('root_exited'); deadline = time.perf_counter() + 5
        while info.active and time.perf_counter() < deadline:
            time.sleep(.02); info = sample('descendant_drain')
        assert info.active == 0, 'DESCENDANT_STILL_ACTIVE180'
        sample('final'); complete = True
    except BaseException as exc:
        error = repr(exc)
    finally:
        # Terminate/drain the full Job on every infrastructure failure; retain its final cost.
        if job:
            if proc is not None:
                try:
                    if assigned:
                        checked(K.TerminateJobObject(job, 125))
                    elif proc.poll() is None:
                        # An unassigned CREATE_SUSPENDED process cannot be killed through the Job.
                        proc.kill()
                    if proc.poll() is None:
                        try:
                            proc.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            proc.kill(); proc.wait(timeout=10)
                    if code is None:
                        code = proc.returncode
                    if error is not None and assigned and tr is not None:
                        info = sample('failure_final')
                        complete = info.active == 0
                except BaseException as final_error:
                    error = (error or '') + '; FINAL_ACCOUNTING_ERROR=' + repr(final_error)
                    complete = False
            K.CloseHandle(job)
        if tr is not None: tr.close()
        if stream is not None: stream.close()
    if HOLDER.proc is not None:
        HOLDER.release()
    last = rows[-1] if rows else {}
    child_cpu = last.get('process_tree_cpu_seconds', 0.)
    observer_cpu = time.process_time() - parent_started
    return dict(schema='ENCLOSED_JOB_QPC180_V1', returncode=code, observer_error=error,
                complete_job_accounting=complete, timed_out=timed_out,
                pid=proc.pid if proc is not None else None, qpc_frequency=freq, samples=len(rows),
                interval_seconds=interval, process_tree_cpu_seconds=child_cpu,
                observer_cpu_seconds=observer_cpu, all_process_cpu_seconds=child_cpu + observer_cpu,
                wall_seconds=time.perf_counter() - started, processes=last.get('processes', 0),
                trace=str(Path(trace).resolve()),
                scope='Full suspended-start Windows Job cumulative user+kernel CPU, including exited descendants. Observer CPU belongs to parent. Raw QPC before/after encloses every CPU query; UTC is diagnostic only.')
