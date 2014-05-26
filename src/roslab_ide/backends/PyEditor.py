__author__ = 'privat'

from PyQt4.QtGui import QColor
from PyQt4.Qsci import QsciScintilla, QsciLexerPython

from roslab_ide.helper.globals import literal_str
from roslab_ide.helper.Workspace import Controller


class PyEditor(QsciScintilla):

    def __init__(self, parent=None):
        QsciScintilla.__init__(self, parent)

        self.setLexer(QsciLexerPython())
        self.setAutoIndent(True)
        self.setCaretForegroundColor(QColor(255, 255, 255))
        self.zoomTo(6)


class PyFunctionEditor(PyEditor):

    def __init__(self, function_data, parent=None):
        PyEditor.__init__(self, parent=parent)
        self._function_data = function_data

        # signals
        self.textChanged.connect(self.update_data)
        self.textChanged.connect(Controller.data_changed)

        # set initial function code
        self.setText(self._function_data['code'])

    def update_data(self):
        self._function_data['code'] = literal_str(self.text())

