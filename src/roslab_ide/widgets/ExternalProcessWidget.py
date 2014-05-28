__author__ = 'privat'
# std imports
import io
import os
import sys
import subprocess
import rospkg
rp = rospkg.RosPack()


# pyqt imports
from python_qt_binding import loadUi
from PyQt4.QtGui import QWidget
from PyQt4.QtCore import QTimer, QProcess, QSettings


# FIXME: If one widget is closed other opened process widgets seem to close too
class ExternalProcessWidget(QWidget):

    def __init__(self, command, working_dir=None, parent=None):
        QWidget.__init__(self, parent=parent)
        # setup user interface
        ui_file = os.path.join(rp.get_path('roslab_ide'), 'resource', 'widgets', 'ExternalProcessWidget.ui')
        self.ui = loadUi(ui_file, self)
        # set command as window title
        self.setWindowTitle(command)
        # set working dir
        if not working_dir:
            working_dir = os.curdir

        args = [
            '-hold',
            '-into', str(self.ui.placeholderWidget.winId()),
            '-e', command
        ]
        self.process = QProcess()
        self.process.setWorkingDirectory(working_dir)
        self.process.start('xterm', args)

    def __del__(self):
        self.process.terminate()
        self.process.waitForFinished()
        self.process = None

    def save_settings(self):
        pass

    def load_settings(self):
        pass

    def closeEvent(self, event):
        self.process.terminate()
        self.process.waitForFinished()
        event.accept()
