from mycroft import MycroftSkill, intent_file_handler


class Mkz(MycroftSkill):
    def __init__(self):
        MycroftSkill.__init__(self)

    @intent_file_handler('mkz.intent')
    def handle_mkz(self, message):
        self.speak_dialog('mkz')


def create_skill():
    return Mkz()

