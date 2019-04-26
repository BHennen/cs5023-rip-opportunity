from speech_transcriber import SpeechTranscriber
import math
import threading

FEET_2_METERS = 0.3048


class Distance:
    def __init__(self, magnitude, unit):
        """ Distance constructor
        Args:
            magnitude: float - amount of units to move
            unit: string - name of unit type (e.g. "meter(s)", "foot", etc.)
        """
        self.magnitude = magnitude
        self.unit = unit


class Command:
    """ Command constructor
    Attributes:
        f: bool - indicates if forward movement is enabled
        b: bool - indicates if backward movement is enabled
        l: bool - indicates if left rotation is enabled
        r: bool - indicates if right rotation is enabled
        magnitude: float - indicates amount of units to move (meters/radians); None if not specified
    """

    def __init__(self, f, b, l, r, dist):
        """ Command constructor
        Args:
            f: bool - indicates if forward movement is enabled
            b: bool - indicates if backward movement is enabled
            l: bool - indicates if left rotation is enabled
            r: bool - indicates if right rotation is enabled
            dist: Distance - struct used to contain the unit & amount of movement
        """
        self.f = f
        self.b = b
        self.l = l
        self.r = r
        self.magnitude = self.parse_distance(
            dist) if dist is not None else None
        """
        Note: translation/rotational_multiplier are Cam's interpretation, not required/expected
        """
        # Go forward if f, go backward if b, else don't move
        self.translational_multiplier = 1 if f else (-1 if b else 0)
        # Rotate right if r, rotate left if l, else don't rotate
        self.rotational_multiplier = 1 if r else (-1 if l else 0)

    def parse_distance(self, dist):
        # Determine unit of dist, convert current magnitude to correct unit, return converted value
        # Default units: radian(s) (rotational), meter(s) (translational)
        # Supported alternate units: degree(s), foot/feet
        if 'meter' in dist.unit:
            return dist.magnitude
        elif 'feet' in dist.unit or 'foot' in dist.unit:
            return dist.magnitude * FEET_2_METERS
        elif 'degree' in dist.unit:
            return math.radians(dist.magnitude)
        elif 'radian' in dist.unit:
            return dist.magnitude
        else:
            return 0

    def __repr__(self):
        return '(f, b, l, r) = ({0}, {1}, {2}, {3}); magnitude = {4}'.format(self.f, self.b, self.l, self.r, self.magnitude)

    __str__ = __repr__


class CmdParser:
    """ CmdParser is used to listen for commands and calls two callback when a keyword is heard
    and when a command is heard.

    If a keyword is detected, the keyword callback is executed with the keyword string detected sent to the callback function.
    Then, if valid keyword detected, the command callback is executed with a new command object sent to the callback function.

    Both can call the callbacks with value of None if no keyword or command was detected in time.
    """

    def __init__(self, grammar, keywords, command_callback, keyword_callback=None, loop_until_grammar=False):
        """ CmdParser constructor
        Args:
            grammar: string - path to grammar file
            keywords: string - path to keywords file
            command_callback: function - callback function, called when a command is received from the ST model
            keyword_callback: function - callback function, called when a keyword is detected by the ST model
            loop_until_grammar: bool - If set to true, loops until valid grammar command is found after detecting a valid keyword.
        """
        self.st = SpeechTranscriber()
        self.keyword_callback = keyword_callback
        self.command_callback = command_callback
        self.grammar = grammar
        self.keywords = keywords
        self.loop_until_grammar = loop_until_grammar
        """
        Fields:
            sr_thread: Thread - asynchronous, calls ST.start_listening
            run_thread: bool - flag indicating if sr_thread should still be alive
        """
        self.sr_thread = threading.Thread(target=self.__start_listen, args=())
        self.run_thread = True

    def __del__(self):
        # Stop listener thread
        self.stop()

    def make_command(self, command_str):
        """Returns command object given a command string"""
        f, b, l, r, dist = (False, False, False, False, None)
        # 1. Separate into words
        words = command_str.split(" ")
        # 2. Check if stop, do nothing
        if words[0] == "stop":
            return Command(f, b, l, r, dist)  # use default values set above

        # check & set directions independently
        # Also record index of rightmost direction
        end_dir_index = 0
        if 'forward' in words:
            f = True
            end_dir_index = max(end_dir_index, words.index('forward'))
        elif 'backward' in words:
            b = True
            end_dir_index = max(end_dir_index, words.index('backward'))
        if 'left' in words:
            l = True
            end_dir_index = max(end_dir_index, words.index('left'))
        elif 'right' in words:
            r = True
            end_dir_index = max(end_dir_index, words.index('right'))

        # Check if index is end of array; if it is then we have (turn | go) <direction> and no distance
        if end_dir_index + 1 == len(words):
            return Command(f, b, l, r, dist)

        # Combine strings from end_dir_index + 1 until the end of array - 1 to get magnitude of direction
        # ex : turn right | one hundred and eighty | degrees
        magnitude_str = ' '.join(words[end_dir_index+1:-1])
        # create distance object
        dist = Distance(self.__word_to_num(magnitude_str), words[-1])

        return Command(f, b, l, r, dist)

    def __command_callback_st(self, command_str):
        cmd = None
        if command_str is not None:
            # Construct command object
            cmd = self.make_command(command_str)
        # Call client's callback with it.
        self.command_callback(cmd)

    def __keyword_callback_st(self, keyword_str):
        # Call client's keyword callback
        self.keyword_callback(keyword_str)

    def __start_listen(self):
        # Thread definition: run SR listen method until parser object is destroyed
        while self.run_thread:
            # Listen for keyword, then pass it to callback
            keyword = self.st.listen(
                keywords=self.keywords, phrase_time_limit=3.0)
            if callable(self.__keyword_callback_st):
                self.__keyword_callback_st(keyword)

            if keyword is not None:
                # Loop until grammar is heard (if desired)
                while True:
                    # listen for command phrase using grammar, then pass it to the callback
                    command = self.st.listen(
                        grammar=self.grammar, phrase_time_limit=5.0)
                    if callable(self.__command_callback_st):
                        self.__command_callback_st(command)
                    if self.run_thread and not self.loop_until_grammar :
                        break

    def start(self):
        # Start the listener thread to receive commands from the speech recog model
        self.run_thread = True
        self.sr_thread.start()

    def stop(self):
        # Stop SR listener thread
        self.run_thread = False
        # self.sr_thread.join() #No need to wait until thread is done

    def __word_to_num(self, textnum, numwords={}):
        # Code borrowed from original at https://stackoverflow.com/a/493788 (@recursive)
        if not numwords:
            units = [
                "zero", "one", "two", "three", "four", "five", "six", "seven", "eight",
                "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen",
                "sixteen", "seventeen", "eighteen", "nineteen",
            ]

            tens = ["", "", "twenty", "thirty", "forty",
                    "fifty", "sixty", "seventy", "eighty", "ninety"]

            scales = ["hundred", "thousand", "million", "billion", "trillion"]

            numwords["and"] = (1, 0)
            for idx, word in enumerate(units):
                numwords[word] = (1, idx)
            for idx, word in enumerate(tens):
                numwords[word] = (1, idx * 10)
            for idx, word in enumerate(scales):
                numwords[word] = (10 ** (idx * 3 or 2), 0)

        current = result = 0
        for word in textnum.split():
            if word not in numwords:
                raise Exception("Illegal word: " + word)

            scale, increment = numwords[word]
            current = current * scale + increment
            if scale > 100:
                result += current
                current = 0

        return result + current


if __name__ == "__main__":
    # Use by calling: python command_parser.py [text|speech|speech-loop]
    import sys

    if len(sys.argv) > 1:
        test = sys.argv[1]
    else:
        test = 'speech'

    if test == 'text':
        print("*"*25 + "Testing command parser" + "*"*25)
        cp = CmdParser(None, None, None, None)
        print("Created instance.")
        test_str = 'go forward and right eight meters'
        print("Test string: "+test_str)
        print(str(cp.make_command(test_str)))
        test_str = 'go backward and right six feet'
        print("Test string: "+test_str)
        print(str(cp.make_command(test_str)))
        test_str = 'go backward and left one meter'
        print("Test string: "+test_str)
        print(str(cp.make_command(test_str)))
        test_str = 'go left three foot'
        print("Test string: "+test_str)
        print(str(cp.make_command(test_str)))
        test_str = 'go forward'
        print("Test string: "+test_str)
        print(str(cp.make_command(test_str)))
        test_str = 'turn left three degrees'
        print("Test string: "+test_str)
        print(str(cp.make_command(test_str)))
        test_str = 'turn right one hundred and sixty degrees'
        print("Test string: "+test_str)
        print(str(cp.make_command(test_str)))
        test_str = 'turn right three hundred five degree'
        print("Test string: "+test_str)
        print(str(cp.make_command(test_str)))
        test_str = 'turn right'
        print("Test string: "+test_str)
        print(str(cp.make_command(test_str)))
        test_str = 'stop'
        print("Test string: "+test_str)
        print(str(cp.make_command(test_str)))
        print("*"*25 + "Done testing command parser" + "*"*25)
    elif test == 'speech' or test == "speech-loop":
        import os
        import time

        def keyword_cb(keyword_str):
            out = ''
            if keyword_str:
                out = "Keyword detected: {}".format(keyword_str)
            else:
                out = "No keyword detected!"
            print(out)

        def command_cb(command_obj):
            out = ''
            if command_obj:
                out = "Command detected: {}".format(command_obj)
            else:
                out = "No command detected!"
            print(out)

        data_path = os.path.join(os.path.dirname(
            os.path.realpath(__file__)), "data")
        keywords_path = os.path.join(data_path, "keywords.txt")
        grammar_path = os.path.join(data_path, "commands.gram")
        loop = test == "speech-loop"
        parser = CmdParser(grammar=grammar_path, keywords=keywords_path,
                           command_callback=command_cb, keyword_callback=keyword_cb, loop_until_grammar=loop)
        parser.start()
        try:
            while True:
                # Simulate bot action
                time.sleep(0.1)
        except KeyboardInterrupt:
            parser.stop()
