from speech_transcriber import SpeechTranscriber


class SpeechCommands():
    def __init__(self):
        self.st = SpeechTranscriber()
        self.st.start_listening(self.handle_that_audio)
    
    def handle_that_audio(self, speech):
        pass
