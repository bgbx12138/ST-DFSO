# -*- coding: utf-8 -*-
"""Windows Job accounting includes descendants after exit. No process-name guessing."""
import ctypes as C
from ctypes import wintypes as W
import subprocess,time,os
K=C.WinDLL('kernel32',use_last_error=True)
class Accounting(C.Structure):
    _fields_=[('user',C.c_longlong),('kernel',C.c_longlong),('period_user',C.c_longlong),('period_kernel',C.c_longlong),('page_faults',W.DWORD),('processes',W.DWORD),('active',W.DWORD),('terminated',W.DWORD)]
class ThreadEntry(C.Structure):
    _fields_=[('size',W.DWORD),('usage',W.DWORD),('thread',W.DWORD),('owner',W.DWORD),('priority',W.LONG),('delta',W.LONG),('flags',W.DWORD)]
K.CreateJobObjectW.restype=W.HANDLE;K.CreateJobObjectW.argtypes=[C.c_void_p,W.LPCWSTR]
K.AssignProcessToJobObject.argtypes=[W.HANDLE,W.HANDLE];K.AssignProcessToJobObject.restype=W.BOOL
K.QueryInformationJobObject.argtypes=[W.HANDLE,C.c_int,C.c_void_p,W.DWORD,C.c_void_p];K.QueryInformationJobObject.restype=W.BOOL
K.TerminateJobObject.argtypes=[W.HANDLE,W.UINT];K.TerminateJobObject.restype=W.BOOL
K.CloseHandle.argtypes=[W.HANDLE];K.OpenThread.argtypes=[W.DWORD,W.BOOL,W.DWORD];K.OpenThread.restype=W.HANDLE
K.ResumeThread.argtypes=[W.HANDLE];K.ResumeThread.restype=W.DWORD
K.CreateToolhelp32Snapshot.argtypes=[W.DWORD,W.DWORD];K.CreateToolhelp32Snapshot.restype=W.HANDLE
K.Thread32First.argtypes=[W.HANDLE,C.POINTER(ThreadEntry)];K.Thread32Next.argtypes=[W.HANDLE,C.POINTER(ThreadEntry)]
def checked(value):
    if not value:raise C.WinError(C.get_last_error())
    return value

class BasicLimits(C.Structure):
    _fields_=[('process_time',C.c_longlong),('job_time',C.c_longlong),('flags',W.DWORD),
              ('minimum_working_set',C.c_size_t),('maximum_working_set',C.c_size_t),
              ('active_limit',W.DWORD),('affinity',C.c_size_t),('priority',W.DWORD),('scheduling',W.DWORD)]
class ExtendedLimits(C.Structure):
    _fields_=[('basic',BasicLimits),('io',C.c_ulonglong*6),('process_memory',C.c_size_t),
              ('job_memory',C.c_size_t),('peak_process_memory',C.c_size_t),('peak_job_memory',C.c_size_t)]
def kill_descendants_on_owner_exit(job):
    limits=ExtendedLimits();limits.basic.flags=0x2000
    K.SetInformationJobObject.argtypes=[W.HANDLE,C.c_int,C.c_void_p,W.DWORD]
    K.SetInformationJobObject.restype=W.BOOL
    checked(K.SetInformationJobObject(job,9,C.byref(limits),C.sizeof(limits)))
