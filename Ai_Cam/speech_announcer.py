import subprocess
from collections import deque
import time
import threading
from queue import Queue

class SpeechAnnouncer:
    def __init__(self):
        """
        Enhanced TTS announcer with:
        - Audio clipping prevention
        - Position change debouncing
        - Natural speech pacing
        """
        # TTS Configuration (keeping your optimized settings)
        self.voice = "en-us+m3"
        self.speed = 135
        self.pitch = 45
        self.gap = 10
        self.capital_emphasis = 20
        
        # Announcement control
        self.cooldown = 1.5  # Minimum delay between announcements
        self.min_utterance_duration = 0.8  # Minimum time to say complete words
        self.last_announce_time = 0
        self.current_process = None
        self.lock = threading.Lock()
        
        # Position tracking
        self.last_position = None
        self.last_label = None
        self.position_change_time = 0
        self.position_debounce = 1.2  # Seconds to wait before announcing position changes
        
        # Queue system for clean audio
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

    def _process_queue(self):
        """Worker thread to process announcements sequentially"""
        while True:
            text, is_position_update = self.queue.get()
            current_time = time.time()
            
            # Calculate required delay based on content
            min_delay = self.min_utterance_duration if not is_position_update else self.position_debounce
            elapsed = current_time - self.last_announce_time
            
            if elapsed < min_delay:
                time.sleep(min_delay - elapsed)
            
            self._speak(text)
            self.last_announce_time = time.time()
            self.queue.task_done()

    def announce(self, label, position=None):
        """
        Smart announcement with:
        - Anti-clipping protection
        - Position change debouncing
        - Natural pacing
        """
        current_time = time.time()
        
        # Handle position changes
        if position:
            if (position == self.last_position and 
                label == self.last_label and
                current_time - self.position_change_time < self.position_debounce):
                return
            
            self.last_position = position
            self.last_label = label
            self.position_change_time = current_time
            full_text = f"{label} on your {position}"
            self.queue.put((full_text, True))
        else:
            # Standard object announcement
            if label != self.last_label or current_time - self.last_announce_time >= self.cooldown:
                self.queue.put((label, False))
                self.last_label = label

    def cleanup(self):
        """Clean up resources"""
        with self.lock:
            if self.current_process:
                self.current_process.terminate()
        self.queue.join()