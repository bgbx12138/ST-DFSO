"""Retain the exact Popen handle. Read exit FILETIME only after a signalled wait."""
import ctypes as C,ctypes.wintypes as W,subprocess
K=C.WinDLL('kernel32',use_last_error=True)
K.GetProcessTimes.argtypes=[W.HANDLE]+[C.POINTER(W.FILETIME)]*4;K.GetProcessTimes.restype=W.BOOL
K.GetExitCodeProcess.argtypes=[W.HANDLE,C.POINTER(W.DWORD)];K.GetExitCodeProcess.restype=W.BOOL
K.WaitForSingleObject.argtypes=[W.HANDLE,W.DWORD];K.WaitForSingleObject.restype=W.DWORD
K.OpenProcess.argtypes=[W.DWORD,W.BOOL,W.DWORD];K.OpenProcess.restype=W.HANDLE
K.CloseHandle.argtypes=[W.HANDLE];K.CloseHandle.restype=W.BOOL
def stamp(t):return (t.dwHighDateTime<<32)|t.dwLowDateTime
def snapshot(handle,pid):
    wait=int(K.WaitForSingleObject(handle,0))
    if wait not in (0,258):raise C.WinError(C.get_last_error())
    ts=[W.FILETIME() for _ in range(4)]
    if not K.GetProcessTimes(handle,*[C.byref(v) for v in ts]):raise C.WinError(C.get_last_error())
    code=W.DWORD()
    if not K.GetExitCodeProcess(handle,C.byref(code)):raise C.WinError(C.get_last_error())
    return dict(pid=pid,open=True,times_ok=True,creation_filetime100ns=stamp(ts[0]),exit_filetime100ns=stamp(ts[1]) if wait==0 else None,
      kernel_cpu_s=stamp(ts[2])*1e-7,user_cpu_s=stamp(ts[3])*1e-7,exit_code=int(code.value),wait_signalled=wait==0,wait_result=wait,
      exit_field_not_interpreted_while_live=wait!=0)
def by_pid(pid):
    h=K.OpenProcess(0x1000|0x100000,False,pid)
    if not h:return dict(pid=pid,open=False,last_error=C.get_last_error())
    try:return snapshot(h,pid)
    finally:K.CloseHandle(h)
class Holder:
    def __init__(self):self.proc=None
    def spawn(self,*args,**kwargs):
        assert self.proc is None,'ONE_CHILD_PER_SUPERVISION255'
        p=subprocess.Popen(*args,**kwargs)
        # No extra native query can throw between Popen and the observer taking ownership.
        self.proc=p;return p
    def proof(self):
        assert self.proc is not None,'NO_HELD_PROCESS255'
        return snapshot(W.HANDLE(int(self.proc._handle)),self.proc.pid)
    def release(self):
        if self.proc is None:return
        assert self.proof()['wait_signalled'],'CANNOT_RELEASE_LIVE_PROOF_HANDLE255'
        self.proc._handle.Close();self.proc=None
HOLDER=Holder()
def retained_popen(*args,**kwargs):return HOLDER.spawn(*args,**kwargs)
