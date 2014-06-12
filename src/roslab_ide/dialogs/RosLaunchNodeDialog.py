__author__ = 'privat'
# std imports
import os
import rospkg
rp = rospkg.RosPack()

# pyqt imports
from python_qt_binding import loadUi
from PyQt4.QtCore import pyqtSlot
from PyQt4.QtGui import QDialog

import roslab_ide.helper.globals as g


class RosLaunchNodeDialog(QDialog):

    def __init__(self, data, parent=None):
        QDialog.__init__(self, parent=parent)
        # setup user interface
        ui_file = os.path.join(rp.get_path('roslab_ide'), 'resource', 'dialogs', 'RosLaunchNodeDialog.ui')
        self.ui = loadUi(ui_file, self)

        # vars
        self.data = data

        # setup packages
        self.ui.packageComboBox.addItems(g.ROS_PKGS)

        # signals
        self.ui.packageComboBox.currentIndexChanged.connect(self.packageChanged)

    @pyqtSlot()
    def packageChanged(self):
        self.ui.nodeComboBox.clear()
        nodes = g.get_package_executables(self.ui.packageComboBox.currentText())
        if len(nodes):
            self.ui.nodeComboBox.addItems(nodes)
        else:
            self.ui.nodeComboBox.addItem('No nodes inside package!')

    def accept(self):
        # set data
        self.data = {
            'package': str(self.ui.packageComboBox.currentText()),
            'node': str(self.ui.nodeComboBox.currentText()),
            'name': str(self.ui.nameLineEdit.text())
        }

        # accept dialog
        QDialog.accept(self)