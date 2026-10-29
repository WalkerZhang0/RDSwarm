"""Append-only JSONL diagnostics, disk spool and bounded CPU worker."""
import hashlib
import io
import json
import os
import queue
import threading
import time
from pathlib import Path


def key_tuple(key):
    return int(key.agent_id), key.session_id, int(key.frame_id)


def key_text(key):
    return '{}:{}:{}'.format(*key_tuple(key))


def write_message(path, message):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    buffer = io.BytesIO()
    message.serialize(buffer)
    temporary.write_bytes(buffer.getvalue())
    os.replace(str(temporary), str(path))


def frame_path(directory, key, suffix='.bin'):
    identity = json.dumps(key_tuple(key), ensure_ascii=False).encode()
    return Path(directory) / (hashlib.sha256(identity).hexdigest() + suffix)


class EventLog:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()

    def __call__(self, stage, **fields):
        fields.update(stage=stage, wall_ns=time.time_ns())
        with self.lock, self.path.open('a') as output:
            output.write(json.dumps(fields, ensure_ascii=False, allow_nan=False) + '\n')


class Worker:
    def __init__(self, handler, log, capacity):
        if capacity < 1:
            raise ValueError('invalid_queue_capacity')
        self.jobs = queue.Queue(capacity)
        self.handler, self.log = handler, log
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def submit(self, job):
        try:
            self.jobs.put_nowait((time.monotonic(), job))
            return True
        except queue.Full:
            self.log('queue_drop', pending=self.jobs.qsize())
            return False

    def run(self):
        while True:
            enqueued, job = self.jobs.get()
            try:
                self.handler(job, (time.monotonic() - enqueued) * 1000)
            except Exception as error:
                self.log('worker_error', error=str(error))
            finally:
                self.jobs.task_done()
