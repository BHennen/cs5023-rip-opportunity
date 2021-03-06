from speech_transcriber import SpeechTranscriber
import math
import os

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
        command_str: string - spoken string that generated the command
    """

    def __init__(self, f, b, l, r, dist, command_str=None):
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
        self.command_str = command_str
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
        return '(f:{}, b:{}, l:{}, r:{}); magnitude:{}; str:"{}"'.format(self.f, self.b, self.l, self.r, self.magnitude, self.command_str)

    __str__ = __repr__


class CommandParser:
    """ CommandParser is used to listen for commands and calls two callback when a keyword is heard
    and when a command is heard.

    If a keyword is detected, the keyword callback is executed with the keyword string detected sent to the callback function.
    Then, if valid keyword detected, the command callback is executed with a new command object sent to the callback function.
        If callback returns True, then checks for another command.
    
    Both can call the callbacks with value of None if no keyword or command was detected in time.
    """

    def __init__(self, command_callback, keyword_callback=None, grammar=None, keywords=None):
        """ CommandParser constructor
        Args:
            grammar: string - path to grammar file
            keywords: string - path to keywords file
            command_callback: function - callback function, called when a command is received from the ST model
                callback can optionally return true to indicate to parser to listen for more commands
            keyword_callback: function - callback function, called when a keyword is detected by the ST model
        """
        self.st = SpeechTranscriber(grammar=grammar, keywords=keywords)
        self.keyword_callback = keyword_callback
        self.command_callback = command_callback

    def make_command(self, command_str):
        """Returns command object given a command string"""
        f, b, l, r, dist = (False, False, False, False, None)
        # 1. Separate into words
        words = command_str.split(" ")
        # 2. Check if movement command
        if 'go' not in words and 'turn' not in words:
            pass  # other unrecognized command; use default values set above
        else:
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
                pass # Skip magnitude calculation
            else:
                # Combine strings from end_dir_index + 1 until the end of array - 1 to get magnitude of direction
                # ex : turn right | one hundred and eighty | degrees
                magnitude_str = ' '.join(words[end_dir_index+1:-1])
                # create distance object
                dist = Distance(self.__word_to_num(magnitude_str), words[-1])

        return Command(f, b, l, r, dist, command_str)

    def start(self):
        # Start the listener thread to receive commands from the speech recog model
        self.st.start_listen_thread(self.__listen)

    def stop(self):
        # Stop st listener thread
        self.st.stop_listen_thread()

    def __del__(self):
        # Stop listener thread
        self.stop()

    def __command_callback_st(self, command_str):
        cmd = None
        if command_str is not None:
            # Construct command object
            cmd = self.make_command(command_str)
        # Call client's callback with it.
        # Return value indicates if we want to loop until valid command found
        if callable(self.command_callback):
            return self.command_callback(cmd)
        else:
            return False

    def __keyword_callback_st(self, keyword_str):
        # Call client's keyword callback
        if callable(self.keyword_callback):
            self.keyword_callback(keyword_str)

    def __listen(self, check_run_thread):
        # Thread will continuously listen (until check_run_thread function returns false)
        while check_run_thread():
            # Listen for keyword, then pass it to callback
            keyword = self.st.listen(use_keywords=True, phrase_time_limit=3.0)
            self.__keyword_callback_st(keyword)

            if keyword is not None:
                # Loop until grammar is heard (if desired)
                while True:
                    # listen for command phrase using grammar, then pass it to the callback
                    command = self.st.listen(use_grammar=True, phrase_time_limit=5.0)
                    continue_cmd_chain = self.__command_callback_st(command)
                    if not check_run_thread() or not continue_cmd_chain:
                        # Stop looping if:
                        # We receive signal from st to stop the thread
                        # Client callback signals us not to continue the command chain
                        break   

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
        cp = CommandParser(None, None, None, None)
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
        test_str = 'turn left three foot'
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
        import time
        last_kwd = None
        last_cmd = None

        def keyword_cb(keyword_str):
            global last_kwd # Required to share data between main thread and parse thread
            out = ''
            last_kwd = keyword_str
            if keyword_str:
                out = "Keyword detected: {}".format(keyword_str)                
            else:
                out = "No keyword detected!"
            print("Parse Thread - " + out)


        def command_cb(command_obj):
            global last_cmd  # Required to share data between main thread and parse thread
            loop = test == "speech-loop"
            out = ''
            last_cmd = command_obj
            if command_obj:
                out = "Command detected: {}".format(command_obj)
                loop = False
            else:
                out = "No command detected!"

            print("Parse Thread - " + out)
            # Return value indicates if we want to loop the grammar section
            return loop

        parser = CommandParser(command_callback=command_cb, keyword_callback=keyword_cb)
        parser.start()
        try:
            while True:
                # Simulate bot action
                print("Main Thread - last kwd: {}\t last cmd: {}".format(last_kwd, last_cmd))
                time.sleep(0.5)
        except KeyboardInterrupt:
            parser.stop()
