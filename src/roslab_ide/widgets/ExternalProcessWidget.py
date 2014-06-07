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
from PyQt4.QtCore import QTimer, QProcess, QSettings, pyqtSlot


# FIXME: If one widget is closed other opened process widgets seem to close too
class ExternalProcessWidget(QWidget):

    def __init__(self, command, working_dir=None, parent=None, keep_open=False):
        QWidget.__init__(self, parent=parent)
        # setup user interface
        ui_file = os.path.join(rp.get_path('roslab_ide'), 'resource', 'widgets', 'ExternalProcessWidget.ui')
        self.ui = loadUi(ui_file, self)
        # set command as window title
        self.setWindowTitle(command)
        # set working dir
        if not working_dir:
            working_dir = os.curdir
        self._process_finished = False
        if keep_open:
            args = [
                '-hold',
                '-into', str(self.ui.placeholderWidget.winId()),
                '-e', command
            ]
        else:
            args = [
                '-into', str(self.ui.placeholderWidget.winId()),
                '-e', command
            ]
        self.process = QProcess()
        self.process.setWorkingDirectory(working_dir)
        self.process.start('xterm', args)

        # signals
        self.process.finished.connect(self.close)

    def __del__(self):
        self.terminate_process()

    def save_settings(self):
        pass

    def load_settings(self):
        pass

    def terminate_process(self):
        self.process.terminate()
        self.process.waitForFinished()
        del self.process
        self._process_finished = True
        self.process = None

    def process_has_finished(self):
        return self._process_finished

    @pyqtSlot()
    def finished_process(self):
        self._process_finished = True
        del self.process
        self.process = None

    def closeEvent(self, event):
        if not self._process_finished:
            self.terminate_process()
        event.accept()