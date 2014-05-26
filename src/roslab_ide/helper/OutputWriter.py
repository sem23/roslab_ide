__author__ = 'privat'


class OutputWriter():

    def __init__(self, text_edit):
        self.text_edit = text_edit

    def write(self, text):
        self.text_edit.insertPlainText(text)