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


class ParameterDialog(QDialog):

    def __init__(self, data=None, parent=None):
        QDialog.__init__(self, parent=parent)
        # setup user interface
        ui_file = os.path.join(rp.get_path('roslab_ide'), 'resource', 'dialogs', 'ParameterDialog.ui')
        self.ui = loadUi(ui_file, self)

        # data
        if data:
            self.ui.nameLineEdit.setText(data['name'])
            self.ui.defaultLineEdit.setText(data['default'])
        self.data = {}

    def accept(self):
        # get module and class
        name = str(self.ui.nameLineEdit.text())
        default = str(self.ui.defaultLineEdit.text())
        datatype = str(self.ui.typeComboBox.currentText())
        # if  == 'int':
        #     default = int(default)
        # elif str(self.ui.typeComboBox.currentText()) == 'double':
        #     default = float(default)
        # elif str(self.ui.typeComboBox.currentText()) == 'bool':
        #     if default in ['True', 'true']:
        #         default = True
        #     else:
        #         default = False
        self.data = {'name': name, 'default': default, 'datatype': datatype}
        # accept dialog
        QDialog.accept(self)