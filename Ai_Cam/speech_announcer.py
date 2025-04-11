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
        self.cooldown = 1.5             # Min. delay between announcements
        self.last_announce_time = 0
        self.last_center_announce_time = 0
        self.recent_objects = deque(maxlen=5)
        self.lock = Lock()

    def announce(self, text, position="center"):
        """Natural-sounding announcement with anti-spam controls."""
        with self.lock:
            current_time = time.time()
            
            # Center position gets its own cooldown check
            if position == "center":
                if current_time - self.last_center_announce_time >= self.cooldown:
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
                    self.last_center_announce_time = current_time

            # Non-center positions follow the regular cooldown and recent object check
            else:
                if (current_time - self.last_announce_time >= self.cooldown and 
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
