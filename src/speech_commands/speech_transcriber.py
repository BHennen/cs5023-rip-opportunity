import speech_recognition as sr


class SpeechTranscriber():
    def __init__(self):
        self.r = sr.Recognizer()
        self.m = sr.Microphone()
        print("A moment of silence, please...")
        with self.m as source:
            self.r.adjust_for_ambient_noise(source)
        print("Set minimum energy threshold to {}".format(self.r.energy_threshold))

    def start_listening(self, cb):
        #TODO: clean method of listening for hotword then for grammar in loop
        pass

    def listen(self, grammar=None, keyword_entries=None):
        if grammar is None and keyword_entries is None:
            raise Exception(
                "Must specify grammar file path or keyword_entries iterable of tuples.")
        try:
            while True:
                print("Say something!")
                with self.m as source:
                    audio = self.r.listen(source, phrase_time_limit=3.0)
                print("Got it! Now to recognize it...")
                try:
                    if grammar:
                        value = self.r.recognize_sphinx(
                            audio_data=audio, grammar=grammar)
                    elif keyword_entries:
                        value = self.r.recognize_sphinx(
                            audio_data=audio, keyword_entries=keyword_entries)

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

if __name__ == "__main__":
    import sys
    import os

    if len(sys.argv) > 1:
        test = sys.argv[1]
    else:
        test = 'keywords'

    data_path = os.path.join(os.path.dirname(
        os.path.realpath(__file__)), "data")
    st = SpeechTranscriber()
    print(test)

    if test == 'keywords':
        # run keywords/hotwords test
        keywords = []
        # sensitivity = 1  # best you can do without editing line 769 of __init__.py of speech_recognition library
        # or you can hack some stuff; this will make sensitivity 1e[hack_value]
        # doing this to avoid editing their module
        class Hack:
            def __init__(self, val):
                self.val = val

            def __sub__(self, o):
                return self

            __rsub__ = __sub__
            __mul__ = __sub__
            __rmul__ = __sub__

            def __str__(self):
                return str(self.val)

            def __repr__(self):
                return '1e{}'.format(self)

            def __le__(self, o):
                return True
                
        sensitivity = Hack(-1) 
        with open(os.path.join(data_path, "keywords.txt"), 'r') as f:
            keywords = [(keyword.strip(), sensitivity)
                        for keyword in f.readlines()]
            print(keywords)

        st.listen(keyword_entries=keywords)

    elif test == 'grammar':
        st.listen(os.path.join(data_path, "commands.gram"))
