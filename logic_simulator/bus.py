"""
Simple Pub/Sub Message Bus (Stub for ROS 2)

This module provides a lightweight, synchronous publish/subscribe mechanism
to simulate ROS 2 middleware for logic validation.
"""

from collections import defaultdict
import threading
import time

class MessageBus:
    def __init__(self):
        self._subscribers = defaultdict(list)
        self._lock = threading.Lock()
        
    def subscribe(self, topic, callback):
        """Subscribe to a topic."""
        with self._lock:
            self._subscribers[topic].append(callback)
            
    def publish(self, topic, message):
        """Publish a message to a topic (Synchronous delivery)."""
        with self._lock:
            # Copy list to avoid modification during iteration
            callbacks = list(self._subscribers[topic])
            
        for callback in callbacks:
            try:
                callback(message)
            except Exception as e:
                print(f"[BUS ERROR] Exception in callback for {topic}: {e}")

# Global instance for simplicity in this harness
default_bus = MessageBus()
