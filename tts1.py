from gtts import gTTS
from tempfile import NamedTemporaryFile
from playsound import playsound

def speak(txt, lang='en'):
    gTTS(text=txt, lang=lang).write_to_fp(voice := NamedTemporaryFile())
    playsound(voice.name)
    voice.close()

if __name__ == "__main__":
    txt = "person"
    speak(txt)