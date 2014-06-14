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
from PyQt4.QtGui import QDockWidget
from PyQt4.QtCore import QTimer, QProcess, QSettings, pyqtSlot


# FIXME: If one widget is closed other opened process widgets seem to close too
class ExternalProcessWidget(QDockWidget):

    def __init__(self, command, working_dir=None, parent=None, keep_open=False):
        QDockWidget.__init__(self, parent=parent)
        # setup user interface
        ui_file = os.path.join(rp.get_path('roslab_ide'), 'resource', 'widgets', 'ExternalProcessWidget.ui')
        self.ui = loadUi(ui_file, self)
        # set command as window title
        self.setWindowTitle(command)
        # set working dir
        if not working_dir:
            working_dir = os.curdir

        # vars
        self._command = command
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

        # print info
        print('starting "{}" in external terminal...'.format(self._command))
        if keep_open:
            print('(terminal will stay open after process has finished. you must close it yourself!)')
        else:
            print('(terminal will close after process has finished.)')

        self.process = QProcess()
        self.process.setWorkingDirectory(working_dir)
        self.process.start('xterm', args)

        # signals
        self.process.finished.connect(self.finished_process)

    def __del__(self):
        self.terminate_process()

    def save_settings(self):
        pass

    def load_settings(self):
        pass

    def terminate_process(self):
        print('terminating "{}" in external terminal...'.format(self._command))
        self.process.terminate()
        self.process.waitForFinished()
        del self.process
        self._process_finished = True
        self.process = None
        # print info
        print('...done!')

    def process_has_finished(self):
        return self._process_finished

    @pyqtSlot()
    def finished_process(self):
        print('...finished "{}" in external terminal.'.format(self._command))
        self._process_finished = True
        del self.process
        self.process = None
        self.close()

    def closeEvent(self, event):
        if not self._process_finished:
            self.terminate_process()
        event.accept()