__author__ = 'privat'

from PyQt4.QtCore import Qt, QEvent, pyqtSlot
from PyQt4.QtGui import QAction, QColor, QFont, QKeySequence, QWidget
from PyQt4.Qsci import QsciScintilla, QsciLexerPython, QsciAPIs

import roslab_ide.helper.globals as g
from roslab_ide.helper.Workspace import Controller


class PyEditor(QsciScintilla):

    def __init__(self, parent=None):
        QsciScintilla.__init__(self, parent)
        self.setTabWidth(4)
        self.setTabIndents(True)
        self.setIndentationsUseTabs(False)

        self._lexer = QsciLexerPython()
        self._lexer.setFont(QFont('DejaVu Sans Mono'))
        self._lexer.setIndentationWarning(QsciLexerPython.Tabs)

        # load current preview to lexer
        api = QsciAPIs(self._lexer)
        api.load('/tmp/preview.py')
        api.prepare()

        self.setLexer(self._lexer)
        self.setAutoCompletionSource(QsciScintilla.AcsAll)
        self.setAutoCompletionThreshold(2)
        self.setAutoIndent(True)
        self.setCaretForegroundColor(g.cursor_color)
        self.zoomTo(5)


class PyFunctionEditor(PyEditor):

    def __init__(self, function_data, package, library, parent=None):
        PyEditor.__init__(self, parent=parent)

        # vars
        self._function_data = function_data
        self._package = package
        self._library = library

        # TODO --> Problem: FIX it!
        # If signals are created before text is set, there will be the question for 'save changes' even if nothing
        # changed...

        # install event filter (to save package with 'CTRL+S')
        self.installEventFilter(self)

        # set initial function code
        self.setText(self._function_data['code'])

        # signals
        self.textChanged.connect(self.update_data)
        self.textChanged.connect(self.update_preview)
        self.textChanged.connect(Controller.data_changed)

    def eventFilter(self, widget, event):
        if event.type() == QEvent.ShortcutOverride:
            if event.matches(QKeySequence.Save):
                self.save_package()
            event.accept()
            return True
        return QWidget.eventFilter(self, widget, event)

    @pyqtSlot()
    def save_package(self):
        Controller.save_package(self._package)

    @pyqtSlot()
    def update_data(self):
        self._function_data['code'] = g.literal_str(self.text())

    @pyqtSlot()
    def update_preview(self):
        Controller.preview_library(self._package, self._library)