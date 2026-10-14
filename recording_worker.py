"""Bounded FIFO storage owner with a reserved, fault-preemptible end barrier."""
from collections import deque
import threading
import time
from recording import RecordingError, CompletionResult


class RecordingWorker:
    def __init__(self, recording, on_sample, on_error, on_finished, *,
                 capacity=256, max_latency=1.5, background=True, completion_lock=None):
        self.recording = recording
        self.on_sample, self.on_error, self.on_finished = on_sample, on_error, on_finished
        self.capacity, self.max_latency = capacity, max_latency
        self.background = background
        self._condition = threading.Condition()
        self._completion_lock = completion_lock or threading.RLock()
        self._queue = deque()
        self._busy_since = self._terminal = self._health_fault = None
        self._accepting = True
        self._draining = False
        self.done = threading.Event()
        self.result = None
        self._thread = None
        if background:
            self._thread = threading.Thread(target=self._run, name='recording')
            self._thread.start()

    def submit(self, operation, *args):
        with self._condition:
            if not self._accepting:
                return False
            if len(self._queue) >= self.capacity:
                self._health_fault = 'recording_overflow'
                return False
            self._queue.append((time.monotonic(), operation, args))
            self._condition.notify()
        if not self.background:
            self._drain()
        return True

    def finish(self, reason, tail_truncated=False, end_ack=None):
        with self._condition:
            if self.done.is_set():
                return
            self._accepting = False
            if self._terminal is None or tail_truncated:
                self._terminal = (reason, tail_truncated, end_ack)
            self._condition.notify()
        if not self.background:
            self._drain()

    def health_error(self):
        with self._condition:
            oldest = self._busy_since
            if oldest is None and self._queue:
                oldest = self._queue[0][0]
            if oldest is not None and time.monotonic()-oldest > self.max_latency:
                self._health_fault = self._health_fault or 'recording_stalled'
            return self._health_fault

    def join(self, timeout):
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout)
            return not self._thread.is_alive()
        return self.done.is_set()

    def _drain(self):
        # Synchronous test mode can reenter through on_error -> finish.
        if self._draining:
            return
        self._draining = True
        try:
            while not self.done.is_set():
                with self._condition:
                    if self._queue:
                        stamp, operation, args = self._queue.popleft()
                        terminal = False
                    elif self._terminal is not None:
                        args, self._terminal = self._terminal, None
                        stamp, operation, terminal = time.monotonic(), self.recording.finish, True
                    else:
                        return
                    self._busy_since = stamp
                try:
                    result = operation(*args)
                    if result is not None and not terminal:
                        self.on_sample(result)
                except Exception as exc:
                    kind = exc.kind if isinstance(exc, RecordingError) else (
                        'storage_fault' if isinstance(exc, OSError) else 'analysis_fault')
                    self.recording.mark_failure(kind)
                    with self._condition:
                        self._queue.clear()
                        self._health_fault = kind
                    if terminal:
                        result = CompletionResult(bool(self.recording.manifest.get('end_ack')), False, kind)
                    else:
                        self.on_error(kind, exc)
                        self.finish(kind, True)
                with self._condition:
                    self._busy_since = None
                # Commit completion and notify control under the same lock as
                # fault submission. A fault queued first must reach storage.
                with self._completion_lock:
                    with self._condition:
                        completed = terminal and self._terminal is None
                        if completed:
                            self.result = result
                            self.done.set()
                    if completed:
                        self.on_finished(result)
        finally:
            self._draining = False

    def _run(self):
        while not self.done.is_set():
            with self._condition:
                self._condition.wait_for(lambda: self._queue or self._terminal is not None or self.done.is_set())
            self._drain()
