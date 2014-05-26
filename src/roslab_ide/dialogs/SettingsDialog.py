__author__ = 'privat'


# std imports
import os

import rospkg
rp = rospkg.RosPack()

# pyqt imports
from python_qt_binding import loadUi
from PyQt4.QtGui import QDialog, QDialogButtonBox
from PyQt4.QtCore import QSettings

import roslab_ide.helper.globals as g


class SettingsDialog(QDialog):

    def __init__(self, parent=None):
        QDialog.__init__(self, parent=parent)
        # setup user interface
        ui_file = os.path.join(rp.get_path('roslab_ide'), 'resource', 'dialogs', 'SettingsDialog.ui')
        self.ui = loadUi(ui_file, self)

        # signals
        apply_button = self.ui.buttonBox.button(QDialogButtonBox.ApplyRole)
        apply_button.clicked.connect(self.applyChanges)

        # fill initials
        self.ui.nameLineEdit.setText(g.name)
        self.ui.emailLineEdit.setText(g.email)
        self.ui.placeLineEdit.setText(g.place)

    def accept(self):
        # save settings
        self.applyChanges()
        # continue
        QDialog.accept(self)

    def applyChanges(self):
        g.name = str(self.ui.nameLineEdit.text())
        g.email = str(self.ui.emailLineEdit.text())
        g.place = str(self.ui.placeLineEdit.text())
        g.store_settings()

