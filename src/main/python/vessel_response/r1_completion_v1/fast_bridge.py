# -*- coding: utf-8 -*-
import subprocess,struct,atexit
from pathlib import Path
import numpy as np
P=Path(__file__).resolve().parent;_process=None;_log=None
def close():
 global _process,_log
 if _process is not None:
  try:_process.stdin.write(struct.pack('>i',-1));_process.stdin.flush();_process.wait(timeout=10)
  except Exception:_process.terminate()
  _process=None
 if _log is not None:_log.close();_log=None
def trace(force,dt,A):
 global _process,_log
 if _process is None:
  _log=(P/'roll_bridge_runtime.log').open('a',encoding='utf-8')
  _process=subprocess.Popen(['java','-cp',str(next(q for q in P.parents if (q/'pom.xml').is_file())/'target/classes'),'FrozenRollBridge'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=_log,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
 n=len(force);_process.stdin.write(struct.pack('>i5d',n,dt,A['I'],A['C'],A['alpha'],A['beta']))
 _process.stdin.write(np.asarray(force,dtype='>f8').tobytes());_process.stdin.flush()
 remaining=n*8;chunks=[]
 while remaining:
  b=_process.stdout.read(remaining)
  if not b:raise RuntimeError('Frozen RK4 bridge ended unexpectedly; see runtime log')
  chunks.append(b);remaining-=len(b)
 return np.frombuffer(b''.join(chunks),dtype='>f8').astype(float)
atexit.register(close)
