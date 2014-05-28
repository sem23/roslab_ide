__author__ = 'privat'

from PyQt4.QtGui import QAction, QColor, QFont, QKeySequence
from PyQt4.Qsci import QsciScintilla, QsciLexerPython

from roslab_ide.helper.globals import literal_str
from roslab_ide.helper.Workspace import Controller


class PyEditor(QsciScintilla):

    def __init__(self, parent=None):
        QsciScintilla.__init__(self, parent)

        lexer = QsciLexerPython()
        lexer.setFont(QFont('DejaVu Sans Mono'))
        self.setLexer(lexer)
        self.setAutoIndent(True)
        self.setCaretForegroundColor(QColor(255, 255, 255))
        self.zoomTo(5)


class PyFunctionEditor(PyEditor):

    def __init__(self, function_data, package, library, parent=None):
        PyEditor.__init__(self, parent=parent)
        self._function_data = function_data

        # vars
        self._package = package
        self._library = library

        # signals
        self.textChanged.connect(self.update_data)
        self.textChanged.connect(self.update_preview)
        self.textChanged.connect(Controller.data_changed)

        # set initial function code
        self.setText(self._function_data['code'])

    def update_data(self):
        self._function_data['code'] = literal_str(self.text())

    def update_preview(self):
        Controller.preview_library(self._package, self._library)