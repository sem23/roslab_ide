__author__ = 'privat'

from PyQt4.QtGui import QAction, QColor, QFont, QKeySequence
from PyQt4.Qsci import QsciScintilla, QsciLexerPython

import roslab_ide.helper.globals as g
from roslab_ide.helper.Workspace import Controller


class PyEditor(QsciScintilla):

    def __init__(self, parent=None):
        QsciScintilla.__init__(self, parent)

        lexer = QsciLexerPython()
        lexer.setFont(QFont('DejaVu Sans Mono'))
        self.setLexer(lexer)
        self.setAutoIndent(True)
        self.setCaretForegroundColor(g.cursor_color)
        self.zoomTo(5)


class PyFunctionEditor(PyEditor):

    def __init__(self, function_data, package, library, parent=None):
        PyEditor.__init__(self, parent=parent)
        self._function_data = function_data

        # vars
        self._package = package
        self._library = library

        # TODO --> Problem: FIX it!
        # If signals are created before text is set, there will be the question for 'save changes' even if nothing
        # changed...

        # set initial function code
        self.setText(self._function_data['code'])

        # signals
        self.textChanged.connect(self.update_data)
        self.textChanged.connect(self.update_preview)
        self.textChanged.connect(Controller.data_changed)

    def update_data(self):
        self._function_data['code'] = g.literal_str(self.text())

    def update_preview(self):
        Controller.preview_library(self._package, self._library)