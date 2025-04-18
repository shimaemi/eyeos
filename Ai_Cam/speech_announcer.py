import subprocess
import time
import threading
from queue import Queue

class SpeechAnnouncer:
    def __init__(self):
        """
        Thread-safe TTS announcer with anti-spam and no overlapping speech.
        """
        self.voice = "en-us+m3"
        self.speed = 135
        self.pitch = 45
        self.gap = 10
        self.capital_emphasis = 20
        self.default_cooldown = 5.0
        self.ahead_cooldown = 3.0
        self.recent_objects = {}  # Tracks last announcement time per phrase
        self.announcement_queue = Queue()  # Thread-safe queue
        self.speaking = False  # Flag to check if currently speaking
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        
        # Start the announcement processing thread
        self.process_thread = threading.Thread(
            target=self._process_queue,
            daemon=True
        )
        self.process_thread.start()

    def announce(self, text, position="ahead"):
        """Add an announcement to the queue (non-blocking)."""
        with self.lock:
            current_time = time.time()
            cooldown = self.ahead_cooldown if position == "ahead" else self.default_cooldown
            
            # Check cooldown
            last_time = self.recent_objects.get(text, 0)
            if current_time - last_time < cooldown:
                return  # Skip if cooldown is active
            
            # Add to queue
            self.announcement_queue.put((text, position))
            self.recent_objects[text] = current_time

    def _process_queue(self):
        """Process announcements sequentially in a background thread."""
        while not self.stop_event.is_set():
            if not self.announcement_queue.empty() and not self.speaking:
                text, position = self.announcement_queue.get()
                self.speaking = True
                
                # Generate the espeak command
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
                
                # Blocking call to ensure no overlap
                subprocess.run(cmd, stderr=subprocess.DEVNULL)
                self.speaking = False
            time.sleep(0.1)  # Reduce CPU usage

    def cleanup(self):
        """Stop the processing thread and clear resources."""
        self.stop_event.set()
        self.process_thread.join()