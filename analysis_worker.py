"""Bounded automatic analysis queue; isolated subprocesses never control capture."""

from dataclasses import dataclass, field
import json
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
from recording import save_json


@dataclass
class AnalysisJob:
    summary: str
    done: threading.Event = field(default_factory=threading.Event)
    result: object = None


def failed_job(summary, status, error):
    job = AnalysisJob(summary)
    publish_failure(job, status, error)
    return job


def publish_failure(job, status, error):
    job.result = dict(analysis_schema=1, status=status, error=error, peaks=None)
    try:
        summary = Path(job.summary)
        manifest = json.loads(summary.read_text())
        save_json(summary.parent / manifest["analysis_result"], job.result)
    except Exception as exc:
        # Unavailable storage cannot prevent capture finalization or shutdown.
        job.result["publication_error"] = f"{type(exc).__name__}: {exc}"
    job.done.set()


class AnalysisWorker:
    def __init__(self):
        self.queue = queue.Queue(maxsize=4)
        self.closing = threading.Event()
        self.process = None
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, name="analysis", daemon=True)
        self.thread.start()

    def submit(self, summary):
        job = AnalysisJob(summary)
        try:
            self.queue.put_nowait(job)
        except queue.Full:
            publish_failure(job, "deferred", "analysis queue full; retry the saved run")
        return job

    def _run(self):
        while not self.closing.is_set() or not self.queue.empty():
            try:
                job = self.queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                with self.lock:
                    if self.closing.is_set():
                        publish_failure(job, "deferred", "logger closed; retry the saved run")
                    else:
                        self.process = subprocess.Popen(
                            [
                                sys.executable,
                                str(Path(__file__).with_name("analysis_runner.py")),
                                job.summary,
                            ]
                        )
                if self.process is not None:
                    self.process.wait()
                    manifest = json.loads(Path(job.summary).read_text())
                    result_path = Path(job.summary).parent / manifest["analysis_result"]
                    job.result = json.loads(result_path.read_text())
                    with self.lock:
                        self.process = None
            except Exception as exc:
                publish_failure(job, "failed", f"{type(exc).__name__}: {exc}")
            finally:
                with self.lock:
                    self.process = None
                job.done.set()
                self.queue.task_done()

    def close(self, timeout):
        # Finish queued analysis when possible; it cannot extend capture shutdown.
        self.thread.join(0)
        deadline = time.monotonic() + timeout
        while self.queue.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(0.01)
        self.closing.set()
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
        self.thread.join(max(0, deadline - time.monotonic()) + 0.1)
