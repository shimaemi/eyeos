import subprocess
from collections import deque, OrderedDict
import time
import threading
from queue import Queue

class SpeechAnnouncer:
    def __init__(self):
        """
        Enhanced TTS announcer designed to prevent sensory overload.
        Features:
        - Single announcement at a time
        - Object priority system
        - Rotation through detections
        - Natural pacing
        """
        # TTS Configuration (keeping your optimized settings)
        self.voice = "en-us+m3"
        self.speed = 135
        self.pitch = 45
        self.gap = 10
        self.capital_emphasis = 20
        
        # Announcement control
        self.cooldown = 2.0  # Increased delay between announcements
        self.min_utterance_time = 1.0  # Minimum time per announcement
        self.current_process = None
        self.lock = threading.Lock()
        
        # Object tracking
        self.object_priority = OrderedDict([
            ('person', 1),
            ('door', 2),
            ('stairs', 3),
            ('chair', 4),
            ('table', 5)
        ])
        self.current_objects = {}
        self.last_announced = None
        self.rotation_index = 0
        
        # Queue system
        self.queue = Queue()
        self.worker_thread = threading.Thread(target=self._process_queue, daemon=True)
        self.worker_thread.start()

    def _speak(self, text):
        """Internal method to execute espeak command"""
        cmd = [
            'espeak',
            '-v', self.voice,
            '-s', str(self.speed),
            '-p', str(self.pitch),
            '-g', str(self.gap),
            '-k', str(self.capital_emphasis),
            '--punct',
            text
        ]
        with self.lock:
            if self.current_process:
                self.current_process.terminate()
            self.current_process = subprocess.Popen(cmd, stderr=subprocess.DEVNULL)
            time.sleep(self.min_utterance_time)  # Ensure full words are spoken

    def _process_queue(self):
        """Worker thread to process announcements"""
        while True:
            text = self.queue.get()
            self._speak(text)
            self.queue.task_done()
            time.sleep(0.5)  # Brief pause between announcements

    def update_objects(self, detections):
        """Update current object tracking and manage announcements"""
        current_time = time.time()
        
        # Clear old detections
        self.current_objects = {}
        
        # Process new detections
        for detection in detections:
            label = detection['label'].lower()
            position = detection['position']
            
            # Only track if not in cooldown
            if label not in self.current_objects:
                self.current_objects[label] = {
                    'position': position,
                    'count': 1,
                    'priority': self.object_priority.get(label, 99)
                }
            else:
                self.current_objects[label]['count'] += 1
        
        # Sort by priority then count
        sorted_objects = sorted(self.current_objects.items(),
                              key=lambda x: (x[1]['priority'], x[1]['count']))
        
        # Announce highest priority object
        if sorted_objects:
            top_object = sorted_objects[0]
            label, info = top_object
            text = f"{label} on your {info['position']}" if info['count'] == 1 else \
                   f"{info['count']} {label}s on your {info['position']}"
            
            if text != self.last_announced or \
               (current_time - self.last_announce_time) > self.cooldown:
                self.queue.put(text)
                self.last_announced = text
                self.last_announce_time = current_time

    def cleanup(self):
        """Clean up resources"""
        with self.lock:
            if self.current_process:
                self.current_process.terminate()
        self.queue.join()