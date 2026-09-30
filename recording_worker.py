"""Bounded ordered recording work; transport never waits for filesystem I/O."""
from collections import deque
import threading
import time
import traceback
from recording import RecordingError


class RecordingWorker:
    def __init__(self, recording, on_sample, on_error, on_finished, *,
                 capacity=256, max_latency=1.5, background=True):
        self.recording = recording
        self.on_sample = on_sample
        self.on_error = on_error
        self.on_finished = on_finished
        self.capacity = capacity
        self.max_latency = max_latency
        self.background = background
        self._condition = threading.Condition()
        self._queue = deque()
        self._busy_since = None
        self._terminal = None
        self._accepting = True
        self._health_fault = None
        self.done = threading.Event()
        self._thread = None
        if background:
            self._thread = threading.Thread(target=self._run, daemon=True, name='recording')
            self._thread.start()

    def submit(self, operation, *args):
        """FIFO operation. Capacity excludes the separately reserved end barrier."""
        with self._condition:
            if not self._accepting:
                return False
            if len(self._queue) >= self.capacity:
                self._health_fault = 'recording_overflow'
                return False
            self._queue.append((time.monotonic(), operation, args))
            self._condition.notify()
        if not self.background:
            self._drain_one()
        return True

    def finish(self, reason, tail_truncated=False, end_ack=None):
        with self._condition:
            if self.done.is_set():
                return
            self._accepting = False
            # Faults supersede normal completion, including an ENDED already queued.
            if self._terminal is None or tail_truncated:
                self._terminal = (reason, tail_truncated, end_ack)
            self._condition.notify()
        if not self.background:
            self._drain_one()

    def health_error(self):
        with self._condition:
            oldest = self._busy_since
            if oldest is None and self._queue:
                oldest = self._queue[0][0]
            if oldest is not None and time.monotonic() - oldest > self.max_latency:
                self._health_fault = self._health_fault or 'recording_stalled'
            return self._health_fault

    def _drain_one(self):
        with self._condition:
            if self._queue:
                stamp, operation, args = self._queue.popleft()
                self._busy_since = stamp
                terminal = False
            elif self._terminal is not None:
                args = self._terminal
                self._terminal = None
                operation = self.recording.finish
                self._busy_since = time.monotonic()
                terminal = True
            else:
                return
        try:
            result = operation(*args)
            if result is not None and not terminal:
                self.on_sample(result)
        except Exception as exc:
            if isinstance(exc, RecordingError):
                kind = exc.kind
            elif isinstance(exc, OSError):
                kind = 'storage_fault'
            else:
                kind = 'analysis_fault'
                traceback.print_exc()
            # Protocol/clock faults leave the detector usable: retain its
            # final partial excursion. Failed computation or delivery is not retried.
            self.recording.failed |= kind in ('analysis_fault', 'storage_fault', 'format_fault')
            with self._condition:
                self._queue.clear()
                self._health_fault = kind
            self.on_error(kind, exc)
            if terminal:
                # Teardown itself failed. Report it once; retrying the same
                # broken finalizer can spin forever and never release ownership.
                with self._condition:
                    self._terminal = None
        finally:
            with self._condition:
                self._busy_since = None
            if terminal:
                with self._condition:
                    completed = self._terminal is None
                    if completed:
                        self.done.set()
                if completed:
                    self.on_finished()

    def _run(self):
        while not self.done.is_set():
            with self._condition:
                self._condition.wait_for(lambda: self._queue or self._terminal is not None)
            self._drain_one()
