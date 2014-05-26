__author__ = 'privat'
# std imports
import io
import os
import sys
import yaml
import rospkg
rp = rospkg.RosPack()


# pyqt imports
from python_qt_binding import loadUi
from PyQt4.QtGui import QDialog


class ImportDialog(QDialog):

    def __init__(self, data=None, parent=None):
        QDialog.__init__(self, parent=parent)
        # setup user interface
        ui_file = os.path.join(rp.get_path('roslab_ide'), 'resource', 'dialogs', 'ImportDialog.ui')
        self.ui = loadUi(ui_file, self)
        # data
        if data:
            self.ui.moduleLineEdit.setText(data['module'])
            self.ui.classLineEdit.setText(data['include'])
        self.data = {}

    def accept(self):
        # get module and class
        module = str(self.ui.moduleLineEdit.text())
        class_ = str(self.ui.classLineEdit.text())
        self.data = {'module': module, 'include': class_}
        # accept dialog
        QDialog.accept(self)