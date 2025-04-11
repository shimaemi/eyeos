import subprocess
from collections import deque
import time
from threading import Lock

class SpeechAnnouncer:
    def __init__(self):
        """
        TTS announcer optimized for natural speech using espeak.
        Designed for vision assistance with clarity, context, and safety.
        """
        self.voice = "en-us+m3"
        self.speed = 135
        self.pitch = 45
        self.gap = 10
        self.capital_emphasis = 20
        self.cooldown = 1.5

        self.last_spoken = {}  # Tracks last time each phrase was spoken
        self.lock = Lock()
        self.queue = deque()
        self.last_queue_time = 0
        self.queue_interval = 0.5  # Seconds between queued messages

        self.last_announcement = None  # Tracks the last spoken announcement

    def set_voice(self, voice):
        self.voice = voice

    def set_speed(self, wpm):
        self.speed = wpm

    def set_pitch(self, pitch):
        self.pitch = pitch

    def set_cooldown(self, seconds):
        self.cooldown = max(0.1, seconds)

    def can_speak(self, phrase):
        """Cooldown check per unique phrase"""
        now = time.time()
        last = self.last_spoken.get(phrase, 0)
        if now - last >= self.cooldown:
            self.last_spoken[phrase] = now
            return True
        return False

    def urgency_from_box(self, box):
        """Estimate how close an object is based on its bounding box size"""
        _, _, w, h = box
        area = w * h
        if area > 60000:
            return "very close"
        elif area > 25000:
            return "nearby"
        return "ahead"

    def format_announcement(self, label, position, box=None):
        """Generate helpful, context-aware speech"""
        urgency = self.urgency_from_box(box) if box else "ahead"

        if label in ["wall", "obstacle", "fence"]:
            return f"Obstacle {urgency} to your {position}"
        elif label == "door":
            return f"Door on your {position}"
        elif label == "person":
            return f"Person {urgency} on your {position}"
        elif label == "car":
            return f"Vehicle {urgency} on your {position}"
        elif label == "stairs":
            return f"Stairs {urgency} on your {position}"
        else:
            return f"{label} {urgency} on your {position}"

    def announce(self, text):
        """Adds a phrase to the queue if not on cooldown and if it's not a duplicate"""
        with self.lock:
            if self.last_announcement != text:  # Check if the announcement is different
                if self.can_speak(text):
                    self.queue.append(text)
                    self.last_announcement = text  # Update the last announcement

    def speak_pending(self):
        """Should be called periodically to play queued announcements"""
        with self.lock:
            now = time.time()
            if self.queue and now - self.last_queue_time >= self.queue_interval:
                text = self.queue.popleft()
                cmd = [
                    "espeak",
                    "-v", self.voice,
                    "-s", str(self.speed),
                    "-p", str(self.pitch),
                    "-g", str(self.gap),
                    "-k", str(self.capital_emphasis),
                    "--punct",
                    text
                ]
                subprocess.Popen(cmd, stderr=subprocess.DEVNULL)
                self.last_queue_time = now
