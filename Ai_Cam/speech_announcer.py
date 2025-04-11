import subprocess
import time
from threading import Lock

class SpeechAnnouncer:
    def __init__(self):
        """
        TTS announcer optimized for natural speech using espeak.
        Settings tuned through extensive testing on Raspberry Pi.
        """
        self.voice = "en-us+m3"
        self.speed = 135
        self.pitch = 45
        self.gap = 10
        self.capital_emphasis = 20
        self.default_cooldown = 5.0  # Default cooldown for non-ahead announcements
        self.ahead_cooldown = 3.0  # lowered as you wanted
        self.recent_objects = {}  # Changed from deque to dict
        self.lock = Lock()

    def announce(self, text, position="ahead"):
        """Natural-sounding announcement with anti-spam controls"""
        with self.lock:
            current_time = time.time()
            cooldown = self.ahead_cooldown if position == "ahead" else self.default_cooldown

            last_time = self.recent_objects.get(text, 0)
            if current_time - last_time >= cooldown:
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
                
                self.recent_objects[text] = current_time
