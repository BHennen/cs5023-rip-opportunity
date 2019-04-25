from speech_transcriber import SpeechTranscriber
from word2number import w2n
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
        self.magnitude = self.parse_distance(dist) if dist is not None else None
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

    def get_distance(self, f, b, l, r, dir):
        if dir == 'forward':
            f = True
        elif dir == 'backward':
            b = True
        elif dir == 'left':
            l = True
        elif dir == 'right':
            r = True
        return (f, b, l, r)

    def make_command(self, command_str):
        """Returns command after deriving from parameter"""
        # 1. Remove "go ", separate into words
        words = command_str[3:].split(" ")
        # 2. Determine grammar based on word count
        w = len(words)
        (f, b, l, r, dist) = (False, False, False, False, None)
        (dir1, dir2, amt, unit) = (None, None, None, None)
        if w == 1:
            # command_str = "go <dir>"
            dir1 = words[0]
        elif w == 3:
            if 'and' in command_str:
                # command_str = "go <dir1> and <dir2>"
                dir1 = words[0]
                dir2 = words[2]
            else:
                # command_str = "go <dir> <amt> <unit>"
                (dir1, amt, unit) = (words[0], words[1], words[2])
        elif w == 5:
            # command_str = "go <dir1> and <dir2> <amt> <unit>"
            (dir1, dir2, amt, unit) = (words[0], words[2], words[3], words[4])
        
        # Get first direction
        (f, b, l, r) = self.get_distance(f, b, l, r, dir1)
        # If second direction exists, get it
        if dir2 is not None:
            (f, b, l, r) = self.get_distance(f, b, l, r, dir2)
        # If magnitude is specified, get it
        if amt is not None and unit is not None:
            dist = Distance(w2n.word_to_num(amt), unit)
        return Command(f, b, l, r, dist)

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




if __name__ == "__main__":
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
    test_str = 'go left three degrees'
    print("Test string: "+test_str)
    print(str(cp.make_command(test_str)))
    test_str = 'go forward five degree'
    print("Test string: "+test_str)
    print(str(cp.make_command(test_str)))
    print("*"*25 + "Done testing command parser" + "*"*25)
