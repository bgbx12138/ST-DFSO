"""Process-safe exact-input reuse. No rounding, interpolation or algorithm key."""
import contextlib
import os
import time
from pathlib import Path


@contextlib.contextmanager
def input_lock(cache, key):
    folder = Path(str(cache) + '.input_locks')
    folder.mkdir(parents=True, exist_ok=True)
    with (folder / (key + '.lock')).open('a+b') as stream:
        # Lock the byte range without writing it. A competing initial write
        # can fail on Windows after another process has locked that range.
        if os.name == 'nt':
            import msvcrt
            started = time.monotonic()
            while True:
                try:
                    stream.seek(0); msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
                    break
                except OSError:
                    if time.monotonic() - started > 360:
                        raise TimeoutError('SHARED_INPUT_OWNER_NOT_CLOSED')
                    time.sleep(.025)
            try:
                yield
            finally:
                stream.seek(0); msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
        else:
            import fcntl
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
