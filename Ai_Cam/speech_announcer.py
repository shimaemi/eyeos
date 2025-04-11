import subprocess
from collections import deque
import time
from threading import Lock

class SpeechAnnouncer:
    def __init__(self):
        """
        TTS announcer optimized for natural speech using espeak.
        Settings tuned through extensive testing on Raspberry Pi.
        """
        self.voice = "en-us+m3"        # Male voice 3 (deeper and clearer)
        self.speed = 135                # Optimal balance (135 WPM)
        self.pitch = 45                 # Slightly lower pitch (45/99)
        self.gap = 10                   # Word gap (10ms pauses)
        self.capital_emphasis = 20      # Stress capitalized words
        self.default_cooldown = 1.5     # Default cooldown between announcements
        self.ahead_cooldown = 2.0      # Higher cooldown for "ahead" announcements
        self.last_announce_time = 0
        self.recent_objects = deque(maxlen=5)  # Track recent objects to avoid duplicate announcements
        self.lock = Lock()

    def announce(self, text, position="ahead"):
        """Natural-sounding announcement with anti-spam controls"""
        with self.lock:
            current_time = time.time()

            # Set cooldown based on the position (ahead has a longer cooldown)
            cooldown = self.ahead_cooldown if position == "ahead" else self.default_cooldown

            if (current_time - self.last_announce_time >= cooldown and 
                text not in self.recent_objects):

                cmd = [
                    'espeak',
                    '-v', self.voice,
                    '-s', str(self.speed),
                    '-p', str(self.pitch),
                    '-g', str(self.gap),
                    '-k', str(self.capital_emphasis),
                    '--punct',          # Pause for punctuation
                    text
                ]
                subprocess.Popen(cmd, stderr=subprocess.DEVNULL)  # Suppress errors
                
                self.recent_objects.append(text)
                self.last_announce_time = current_time
