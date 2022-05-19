from gtts import gTTS
import os
import playsound
import pyttsx3


class TextToSpeech:
    def __init__(self):
        self.engine = pyttsx3.init()
        self.voices = self.engine.getProperty('voices')

    def setMF(self, gender):
        self.engine.setProperty('voice', self.voices[int(gender.lower() == "female")].id)

    def setRate(self, rate):
        self.engine.setProperty("rate", rate)

    def speak(self, text):
        filename = 'voice.mp3'
        try:
            tts = gTTS(text=text, lang='en')
            tts.save(filename)
            playsound.playsound(filename)
            os.remove(filename)
        except:
            print("Error: No Network Connection")
            if(len(self.voices) > 1):
                self.engine.say(text)
                self.engine.runAndWait()
                self.engine.save_to_file(text, filename)




#tts = TextToSpeech()
#tts.setRate(210)
#tts.setMF("female")

#while True:
#    inp = input()
#    tts.speak(inp)