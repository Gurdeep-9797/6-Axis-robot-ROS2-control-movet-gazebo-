#!/usr/bin/env python3
import time
import threading

class SafetyWatchdog:
    """
    Monitors connection health and data freshness.
    """
    def __init__(self, timeout=0.1, callback=None):
        self.timeout = timeout
        self.last_heartbeat = time.time()
        self.running = False
        self.callback = callback
        self.thread = threading.Thread(target=self._monitor)

    def feed(self):
        self.last_heartbeat = time.time()

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def _monitor(self):
        while self.running:
            dt = time.time() - self.last_heartbeat
            if dt > self.timeout:
                # TIMEOUT DETECTED
                if self.callback:
                    self.callback("TIMEOUT")
            time.sleep(0.01)
