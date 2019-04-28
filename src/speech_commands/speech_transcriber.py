import speech_recognition as sr
import os


class SpeechTranscriber():
    def __init__(self):
        self.r = sr.Recognizer()
        self.m = sr.Microphone()

        # Get language data
        language_directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "language")
        if not os.path.isdir(language_directory):
            raise Exception("missing language data directory: \"{}\"".format(language_directory))
        acoustic_parameters_directory = os.path.join(language_directory, "acoustic-model")
        language_model_file = os.path.join(language_directory, "language-model.lm.bin")
        phoneme_dictionary_file = os.path.join(language_directory, "pronounciation-dictionary.dict")
        self.language = (acoustic_parameters_directory, language_model_file, phoneme_dictionary_file)

        # Adjust for ambient noise
        print("A moment of silence, please...")
        with self.m as source:
            self.r.adjust_for_ambient_noise(source)
        print("Set minimum energy threshold to {}".format(self.r.energy_threshold))

    def start_listening(self, keyword_cb, command_cb, grammar, keywords):
        ''' Listens for keyword, then listens for command in grammar.
        If valid keyword was found, calls keyword_cb with keyword found, then listens for command.
        If command was found, calls the command_cb with the command string that was said.
        '''

        if grammar is None or keywords is None:
            raise Exception(
                "Must specify grammar file path and keywords file.")

        # Listen for keyword
        keyword = self.listen(keywords=keywords, phrase_time_limit=3.0)
        if callable(keyword_cb):
            keyword_cb(keyword)
        if keyword is not None:            
            # listen for command phrase using grammar, then pass it to the callback
            command = self.listen(grammar=grammar, phrase_time_limit=5.0)
            if callable(command_cb):
                command_cb(command)

    def listen(self, grammar=None, keywords=None, timeout=None, phrase_time_limit=None):
        '''
        Listen to a user using either a grammar or keyword entries
        Returns the first keyword or command found as a string.
        Returns None if didn't understand.
        '''
        if grammar is None and keywords is None:
            raise Exception(
                "Must specify grammar file path or keywords file.")

        print("Say something!")
        with self.m as source:
            audio = self.r.listen(
                source, timeout=timeout, phrase_time_limit=phrase_time_limit)
            print("Got it! Now to recognize it...")
        try:
            if grammar:
                value = self.r.recognize_sphinx(
                    audio_data=audio, grammar=grammar, language=self.language)
            elif keywords:
                value = self.r.recognize_sphinx(
                    audio_data=audio, keyword_entries=self.__generate_keywords(keywords), language=self.language)

            # we need some special handling here to correctly print unicode characters to standard output
            # this version of Python uses bytes for strings (Python 2)
            result = ''
            if str is bytes:
                result = u"{}".format(value).encode("utf-8")
            # this version of Python uses unicode for strings (Python 3+)
            else:
                result = "{}".format(value)
            
            if grammar:
                result = self.__remove_garbage(result)

            return result
        except sr.UnknownValueError:
            return None

    def __generate_keywords(self, path):
        class Hack:
            # sensitivity = 1  --> best you can do without editing line 769 of __init__.py of speech_recognition library
            # or you can hack some stuff; this will make sensitivity 1e[Hack.val]
            # doing this to avoid editing their module
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

        sensitivity = Hack(-3) #TODO: make keywords have individual sensitivities (if desired)
        keywords = []
        with open(path, 'r') as f:
            keywords = [(keyword.strip(), sensitivity)
                        for keyword in f.readlines()]
            return keywords
    
    def __remove_garbage(self, command_str):
        import re
        """ Removes all garbage phonemes from a grammar search.
        
        Returns the string command or None is no command recognized
        """
        regex = r'\s*?zz\d{1,2}\s*'
        result = re.sub(regex, '', command_str)
        if not result: result = None
        return result


if __name__ == "__main__":
    # Use by calling: python speech_transcriber.py [keywords|grammar]
    import sys

    if len(sys.argv) > 1:
        test = sys.argv[1]
    else:
        test = 'keywords'

    data_path = os.path.join(os.path.dirname(
        os.path.realpath(__file__)), "data")
    st = SpeechTranscriber()

    if test == 'keywords':
        # run keywords/hotwords test
        try:
            while True:
                print(st.listen(
                    keywords=os.path.join(data_path, "keywords.txt"), 
                    phrase_time_limit=3.0))
        except KeyboardInterrupt:
            pass

    elif test == 'grammar':
        try:
            while True:
                print(st.listen(
                    grammar=os.path.join(data_path, "commands.gram"),
                    phrase_time_limit=5.0))
        except KeyboardInterrupt:
            pass

    elif test == 'both':
        def test1(speech):
            print("keyword is: {}".format(speech))
        def test2(speech):
            print("command is: {}".format(speech))

        try:
            while True:
                st.start_listening(grammar=os.path.join(data_path, "commands.gram"),
                                keywords=os.path.join(data_path, "keywords.txt"),
                                   keyword_cb=test1, command_cb=test2)

        except KeyboardInterrupt:
            pass
