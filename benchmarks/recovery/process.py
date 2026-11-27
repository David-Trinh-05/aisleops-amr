"""Manage and log a child process and its process group."""

from collections import deque
from contextlib import contextmanager, suppress
import os
import signal
import subprocess
import sys


@contextmanager
def start(command, log, cwd):
    with log.open("w") as output:
        process = subprocess.Popen(
            command, cwd=cwd, stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
        )
    try:
        yield process
    except RuntimeError:
        with log.open(errors="replace") as output:
            print("".join(deque(output, maxlen=65)), file=sys.stderr, end="")
        raise
    finally:
        if process.poll() is None:
            with suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                with suppress(ProcessLookupError):
                    os.killpg(process.pid, signal.SIGKILL)
                process.wait()
