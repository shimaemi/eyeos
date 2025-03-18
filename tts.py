# pip install pyttsx3
#  sudo apt update && sudo apt install espeak-ng libespeak1

import bluetooth
import pyttsx3
import time

if __name__ == "__main__":
    engine = pyttsx3.init() # object creation
    engine.setProperty('rate', 125)     # setting up new voice rate
    engine.setProperty('volume',1.0)    # setting up volume level  between 0 and 1
    engine.setProperty('voice', voices[1].id)   #changing index, changes voices. 1 for female

    engine.say("warning")
    time.sleep(1)
    engine.say("WARNING")
    engine.runAndWait()
    engine.stop()