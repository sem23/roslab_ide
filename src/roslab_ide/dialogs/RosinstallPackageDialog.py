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


class RosinstallPackageDialog(QDialog):

    def __init__(self, data, parent=None):
        QDialog.__init__(self, parent=parent)
        # setup user interface
        ui_file = os.path.join(rp.get_path('roslab_ide'), 'resource', 'dialogs', 'RosinstallPackageDialog.ui')
        self.ui = loadUi(ui_file, self)

        # vars
        self.data = data

    def accept(self):
        self.data['name'] = str(self.ui.nameLineEdit.text())
        self.data['vcs'] = str(self.ui.vcsComboBox.currentText())
        self.data['uri'] = str(self.ui.uriLineEdit.text())
        self.data['version'] = str(self.ui.versionComboBox.currentText())

        # accept dialog
        QDialog.accept(self)