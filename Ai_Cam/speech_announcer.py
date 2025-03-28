import subprocess
from collections import deque
import time
from threading import Lock

class SpeechAnnouncer:
    def __init__(self):
        """
        Enhanced TTS announcer with intelligent queue management.
        Features:
        - Natural-sounding espeak voice
        - Anti-spam cooldown timer
        - Object tracking to prevent repeats
        - Thread-safe operation
        """
        # Voice configuration
        self.voice = "en-us+m3"
        self.speed = 135
        self.pitch = 45
        self.gap = 10
        self.capital_emphasis = 20
        
        # Announcement control
        self.cooldown = 1.5  # Minimum seconds between announcements
        self.max_history = 5  # Remember last 5 announcements
        self.last_announce_time = 0
        self.recent_objects = deque(maxlen=self.max_history)
        self.lock = Lock()
        
        # Position tracking
        self.last_position = None
        self.position_cooldown = 3.0  # Longer cooldown for same position

    def _should_announce(self, text, position=None):
        """Decision logic for whether to announce"""
        current_time = time.time()
        time_ok = current_time - self.last_announce_time >= self.cooldown
        position_ok = (position is None or 
                      position != self.last_position or
                      current_time - self.last_announce_time >= self.position_cooldown)
        text_ok = text not in self.recent_objects
        
        return time_ok and position_ok and text_ok

    def announce(self, text, position=None):
        """
        Smart announcement with context awareness
        Args:
            text: The message to speak
            position: Optional position info (for spatial cooldown)
        """
        with self.lock:
            if self._should_announce(text, position):
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
                subprocess.Popen(cmd, stderr=subprocess.DEVNULL)
                
                # Update tracking state
                self.recent_objects.append(text)
                self.last_announce_time = time.time()
                if position:
                    self.last_position = position

    def announce_detection(self, label, position):
        """
        Specialized method for object detection announcements
        Formats the message consistently and handles position logic
        """
        message = f"{label} detected on the {position}"
        self.announce(message, position)