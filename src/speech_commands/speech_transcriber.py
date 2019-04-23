import speech_recognition as sr

class SpeechTranscriber():
    def __init__(self):
        self.r = sr.Recognizer()
        self.m = sr.Microphone()

    def start_listening(self, cb):
        try:
            print("A moment of silence, please...")
            with self.m as source:
                self.r.adjust_for_ambient_noise(source)
            print("Set minimum energy threshold to {}".format(self.r.energy_threshold))
            while True:
                print("Say something!")
                with self.m as source:
                    audio = self.r.listen(source, timeout=3.0, phrase_time_limit=1.0)
                print("Got it! Now to recognize it...")
                try:
                    # recognize speech using Google Speech Recognition
                    value = self.r.recognize_sphinx(
                        audio_data=audio, grammar='src\speech_commands\commands.gram')

                    # we need some special handling here to correctly print unicode characters to standard output
                    # this version of Python uses bytes for strings (Python 2)
                    if str is bytes:
                        print(u"You said {}".format(value).encode("utf-8"))
                    # this version of Python uses unicode for strings (Python 3+)
                    else:
                        print("You said {}".format(value))
                except sr.UnknownValueError:
                    print("Oops! Didn't catch that")
                except sr.RequestError as e:
                    print(
                        "Uh oh! Couldn't request results from Google Speech Recognition service; {0}".format(e))
        except KeyboardInterrupt:
            pass

sc = SpeechTranscriber()
sc.start_listening('123')
