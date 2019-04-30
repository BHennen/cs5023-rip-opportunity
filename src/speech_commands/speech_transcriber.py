import speech_recognition as sr
import os
import threading


class SpeechTranscriber():
    def __init__(self, grammar=None, keywords=None):
        self.r = sr.Recognizer()
        self.m = sr.Microphone()
        self.st_thread = None
        self.run_thread = False

        # Get data path
        data_directory = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "data")
        if not os.path.isdir(data_directory):
            raise Exception(
                "Data directory not found at: \"{}\"".format(data_directory))

        # Get keywords and grammar
        keywords_path = os.path.join(data_directory, "keywords.txt")
        keywords_path = keywords if keywords is not None else keywords_path
        self.update_keywords_path(keywords_path)

        grammar_path = os.path.join(data_directory, "commands.gram")
        grammar_path = grammar if grammar is not None else grammar_path
        self.update_grammar_path(grammar_path)

        # Get language data
        language_directory = os.path.join(data_directory, "language")
        if not os.path.isdir(language_directory):
            raise Exception(
                "Language data directory not found at: \"{}\"".format(language_directory))
        acoustic_parameters_directory = os.path.join(
            language_directory, "acoustic-model")
        language_model_file = os.path.join(
            language_directory, "language-model.lm.bin")
        phoneme_dictionary_file = os.path.join(
            language_directory, "pronounciation-dictionary.dict")
        self.language = (acoustic_parameters_directory,
                         language_model_file, phoneme_dictionary_file)

        # Adjust for ambient noise
        print("A moment of silence, please...")
        with self.m as source:
            self.r.adjust_for_ambient_noise(source)
        print("Set minimum energy threshold to {}".format(self.r.energy_threshold))

    def update_grammar_path(self, grammar_path):
        """ Update the path used for grammar recognition """
        if not os.path.isfile(grammar_path):
            raise Exception(
                "Grammar file not found at: \"{}\"".format(grammar_path))
        else:
            self.grammar = grammar_path

    def update_keywords_path(self, keywords_path):
        """ Update the path used for keyword recognition """
        if not os.path.isfile(keywords_path):
            raise Exception(
                "Keywords file not found at: \"{}\"".format(keywords_path))
        else:
            self.keyword_entries = self.__generate_keywords(keywords_path)

    def listen(self, use_grammar=False, use_keywords=False, timeout=None, phrase_time_limit=None):
        '''
        Listen to a user using either a grammar or keyword entries
        Returns the first keyword or command found as a string.
        Returns None if didn't understand.
        '''
        if not use_grammar and not use_keywords:
            raise Exception(
                "Must specify grammar file path or keywords file.")

        speech_type = "a command" if use_grammar else "a keyword"
        time_limit = " in {} seconds".format(phrase_time_limit) if phrase_time_limit else ''
        print("SpeechTranscriber: Say {}{}!".format(speech_type, time_limit))
        with self.m as source:
            audio = self.r.listen(
                source, timeout=timeout, phrase_time_limit=phrase_time_limit)
            print("SpeechTranscriber: Got it! Now to recognize it...")
        try:
            if use_grammar:
                value = self.r.recognize_sphinx(
                    audio_data=audio, grammar=self.grammar, language=self.language)
            elif use_keywords:
                value = self.r.recognize_sphinx(
                    audio_data=audio, keyword_entries=self.keyword_entries, language=self.language)

            # we need some special handling here to correctly print unicode characters to standard output
            # this version of Python uses bytes for strings (Python 2)
            result = ''
            if str is bytes:
                result = u"{}".format(value).encode("utf-8")
            # this version of Python uses unicode for strings (Python 3+)
            else:
                result = "{}".format(value)

            if use_grammar:
                result = self.__remove_garbage(result)

            return result
        except sr.UnknownValueError:
            return None

    def start_listen_thread(self, func):
        """ Start a new thread that calls the given function.
            Function passed as parameter which should be called by function to determine
            if thread should continue to be run
         """
        if not self.st_thread or not self.st_thread.is_alive():
            self.st_thread = threading.Thread(
                target=self._start_listen, args=(func,))
            self.run_thread = True
            self.st_thread.start()

    def stop_listen_thread(self):
        self.run_thread = False

    def _start_listen(self, func):
        func(self._check_run_thread)

    def _check_run_thread(self):
        return self.run_thread

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

            __lt__ = __le__
            __eq__ = __le__
            __ne__ = __le__
            __ge__ = __le__
            __gt__ = __le__

        # TODO: make keywords have individual sensitivities (if desired)
        sensitivity = Hack(-3)
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
        regex = r'zz\d{1,2}\s*'
        result = re.sub(regex, '', command_str)
        result = result.strip()
        if not result:
            result = None
        return result


if __name__ == "__main__":
    # Use by calling: python speech_transcriber.py [keywords|grammar|both|both-async]
    import sys
    import time

    if len(sys.argv) > 1:
        test = sys.argv[1]
    else:
        test = 'keywords'

    last_kwd = None
    last_cmd = None

    def print_with_thread(output):
        print("{} - {}".format(threading.current_thread().name, output))

    def keyword_cb(speech):
        global last_kwd
        last_kwd = speech
        print_with_thread("Received keyword: {}".format(speech))

    def command_cb(speech):
        global last_cmd
        last_cmd = speech
        print_with_thread("Received command: {}".format(speech))
        # Loop until we receive valid command
        if speech is None:
            return True
        else:
            return False

    st = SpeechTranscriber()

    if test == 'keywords':
        # run keywords/hotwords test
        try:
            while True:
                print(st.listen(use_keywords=True, phrase_time_limit=3.0))
        except KeyboardInterrupt:
            pass

    elif test == 'grammar':
        try:
            while True:
                print(st.listen(use_grammar=True, phrase_time_limit=5.0))
        except KeyboardInterrupt:
            pass

    elif test == 'both':
        try:
            while True:
                # Listen for keyword, then pass it to callback
                keyword = st.listen(use_keywords=True, phrase_time_limit=3.0)
                keyword_cb(keyword)

                if keyword is not None:
                    # Loop until grammar is heard (if desired)
                    while True:
                        # listen for command phrase using grammar, then pass it to the callback
                        command = st.listen(
                            use_grammar=True, phrase_time_limit=5.0)
                        continue_cmd_chain = command_cb(command)
                        if not continue_cmd_chain:
                            # Stop looping if:
                            # Client callback signals us not to continue the command chain
                            break

        except KeyboardInterrupt:
            pass

    elif test == 'both-async':
        def async_func(continue_thread):
            # Thread will continuously listen (until continue_thread function returns false)
            while continue_thread():
                # Listen for keyword, then pass it to callback
                keyword = st.listen(use_keywords=True, phrase_time_limit=3.0)
                keyword_cb(keyword)

                if keyword is not None:
                    # Loop until grammar is heard (if desired)
                    while True:
                        # listen for command phrase using grammar, then pass it to the callback
                        command = st.listen(
                            use_grammar=True, phrase_time_limit=5.0)
                        continue_cmd_chain = command_cb(command)
                        if not continue_thread() or not continue_cmd_chain:
                            # Stop looping if:
                            # We receive signal to stop the thread
                            # Client callback signals us not to continue the command chain
                            break

        st.start_listen_thread(async_func)
        try:
            while True:
                print_with_thread("last kwd: '{}'\t last cmd: '{}'".format(last_kwd, last_cmd))
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            st.stop_listen_thread()
