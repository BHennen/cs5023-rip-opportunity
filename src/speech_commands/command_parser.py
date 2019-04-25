from speech_transcriber import SpeechTranscriber
import threading

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
    def __init__(self, f, b, l, r, dist=None):
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
        self.magnitude = self.parse_distance(dist)
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
        pass

class CmdParser:
    def __init__(self, grammar, keywords, command_callback, keyword_callback=None):
        """ CmdParser constructor
        Args:
            grammar: string - path to grammar file
            keywords: string - path to keywords file
            command_callback: function - callback function, called when a command is received from the ST model
            keyword_callback: function - callback function, called when a keyword is detected by the ST model
        """
        self.st = SpeechTranscriber()
        self.keyword_callback = keyword_callback
        self.command_callback = command_callback
        self.grammar = grammar
        self.keywords = keywords
        """
        Fields:
            sr_thread: Thread - asynchronous, calls ST.start_listening
            run_thread: bool - flag indicating if sr_thread should still be alive
        """
        self.sr_thread = threading.Thread(target=self.__start_listen, args=())
        self.run_thread = True
        pass

    def __del__(self):
        # Stop listener thread
        self.stop()

    def make_command(self, command_str):
        # Returns command derived from parameter
        pass

    def command_callback_st(self, command_str):
        # Construct command object
        cmd = self.make_command(command_str)
        # Call client's callback with it.
        self.command_callback(cmd)
        pass

    def __start_listen(self):
        # Thread definition: run SR listen method until parser object is destroyed
        while self.run_thread:
            self.st.start_listening(grammar=self.grammar,keywords=self.keywords,keyword_cb=self.keyword_callback, command_cb=self.command_callback_st)

    def start(self):
        # Start the listener thread to receive commands from the speech recog model
        self.sr_thread.start()

    def stop(self):
        # Stop SR listener thread
        self.run_thread = False
        self.sr_thread.join()