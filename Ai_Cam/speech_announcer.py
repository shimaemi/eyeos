import subprocess
import time
import threading
from queue import Queue

class SpeechAnnouncer:
    def __init__(self):
        """
        Single-object TTS announcer that:
        - Only announces one object at a time
        - Blocks new detections during announcement
        - Maintains cooldown periods
        """
        # TTS Configuration
        self.voice = "en-us+m3"
        self.speed = 135
        self.pitch = 45
        self.gap = 10
        self.capital_emphasis = 20
        
        # Announcement control
        self.cooldown = 2.0  # Minimum time between announcements
        self.last_announce_time = 0
        self.current_object = None
        self.lock = threading.Lock()
        self.speaking = False
        self.new_object_event = threading.Event()
        self.stop_event = threading.Event()
        
        # Start processing thread
        self.process_thread = threading.Thread(
            target=self._process_announcements,
            daemon=True
        )
        self.process_thread.start()

    def announce(self, text):
        """
        Request an announcement for a single object.
        Returns True if announcement was queued, False if rejected.
        """
        with self.lock:
            current_time = time.time()
            
            # Reject if:
            # 1. Already speaking, OR
            # 2. On cooldown, OR
            # 3. Same object as last time
            if (self.speaking or 
                current_time - self.last_announce_time < self.cooldown or
                text == self.current_object):
                return False
            
            self.current_object = text
            self.new_object_event.set()
            return True

    def _process_announcements(self):
        """Process announcements one at a time"""
        while not self.stop_event.is_set():
            # Wait for new object detection
            self.new_object_event.wait()
            
            if self.stop_event.is_set():
                break
                
            self.speaking = True
            try:
                cmd = [
                    'espeak',
                    '-v', self.voice,
                    '-s', str(self.speed),
                    '-p', str(self.pitch),
                    '-g', str(self.gap),
                    '-k', str(self.capital_emphasis),
                    '--punct',
                    self.current_object
                ]
                subprocess.run(cmd, stderr=subprocess.DEVNULL)
                self.last_announce_time = time.time()
            finally:
                self.speaking = False
                self.new_object_event.clear()

    def cleanup(self):
        """Stop the announcer and clean up resources"""
        self.stop_event.set()
        self.new_object_event.set()  # Unblock thread if waiting
        self.process_thread.join()