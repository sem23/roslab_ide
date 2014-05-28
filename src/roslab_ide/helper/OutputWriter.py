__author__ = 'privat'

from PyQt4.QtGui import QTextCursor


class OutputWriter():

    def __init__(self, text_edit):
        self.text_edit = text_edit
        self.text_edit.setPlainText('')

    def write(self, text):
        self.text_edit.moveCursor(QTextCursor.End)
        self.text_edit.insertPlainText(text)