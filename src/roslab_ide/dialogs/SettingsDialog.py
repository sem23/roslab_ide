__author__ = 'privat'


# std imports
import os

import rospkg
rp = rospkg.RosPack()

# pyqt imports
from python_qt_binding import loadUi
from PyQt4.QtGui import QDialog, QDialogButtonBox, QColorDialog
from PyQt4.QtCore import QSettings, pyqtSlot

import roslab_ide.helper.globals as g


class SettingsDialog(QDialog):

    def __init__(self, parent=None):
        QDialog.__init__(self, parent=parent)
        # setup user interface
        ui_file = os.path.join(rp.get_path('roslab_ide'), 'resource', 'dialogs', 'SettingsDialog.ui')
        self.ui = loadUi(ui_file, self)
        self.update_cursor_label_color()

        # signals
        self.ui.changeCursorColorPushButton.clicked.connect(self.change_cursor_color)
        apply_button = self.ui.buttonBox.button(QDialogButtonBox.Apply)
        apply_button.clicked.connect(self.applyChanges)

        # fill initials
        self.ui.nameLineEdit.setText(g.name)
        self.ui.emailLineEdit.setText(g.email)
        self.ui.placeLineEdit.setText(g.place)

    @pyqtSlot()
    def change_cursor_color(self):
        g.cursor_color = QColorDialog.getColor(initial=g.cursor_color, parent=self)
        self.update_cursor_label_color()

    def update_cursor_label_color(self):
        color_arg = 'rgba({r}, {g}, {b}, 255)'.format(
            r=g.cursor_color.red(),
            g=g.cursor_color.green(),
            b=g.cursor_color.blue()
        )
        style = ('QLabel { '
                 'border: 2px solid black; '
                 'border-radius: 4px; '
                 'padding: 2px; '
                 'background-color : ' + color_arg + '; }')
        self.ui.cursorColorLabel.setStyleSheet(style)
        self.ui.cursorColorLabel.update()

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

