# __author__ = 'privat'
#
# import roslab_ide.helper.globals as g
#
# from python_qt_binding.QtGui import QDialog, QInputDialog
#
# from roslab_ide.helper.Workspace import Workspace
# from roslab_ide.helper.TreeHelper import TreeHelper, TreeItem
#
# from roslab_ide.dialogs.ArgumentDialog import ArgumentDialog
# from roslab_ide.dialogs.ImportDialog import ImportDialog
# from roslab_ide.dialogs.ParameterDialog import ParameterDialog
# from roslab_ide.dialogs.CommunicationDialog import CommunicationDialog
#
# from roslab_ide.backends.PyBackend import PyBackend
#
#
# class ROSComm(TreeItem):
#
#     def __init__(self):
#         TreeItem.__init__(self)
#
#     @staticmethod
#     def from_current():
#         pass
#
#     @staticmethod
#     def remove():
#         pass
#
#
# class Import(TreeItem):
#
#     def __init__(self):
#         TreeItem.__init__(self)
#
#     @staticmethod
#     def from_current():
#         pass
#
#     @staticmethod
#     def remove():
#         pass
#
#
# class Parameter(TreeItem):
#
#     def __init__(self):
#         TreeItem.__init__(self)
#
#     @staticmethod
#     def from_current():
#         pass
#
#     @staticmethod
#     def remove():
#         pass
#
#
# class Transform(TreeItem):
#
#     def __init__(self):
#         TreeItem.__init__(self)
#
#     @staticmethod
#     def from_current():
#         pass
#
#     @staticmethod
#     def remove():
#         pass
#
#
# class Function(TreeItem):
#
#     def __init__(self):
#         TreeItem.__init__(self)
#
#     @staticmethod
#     def add_argument():
#         """
#         Add dialog-based an argument to current selected function.
#         """
#         function_name, function_item, function_data = Function.from_current()
#         arg_dialog = ArgumentDialog(data=function_data, parent=g.main_widget)
#         if arg_dialog.exec_():
#             TreeHelper.update_item(function_item, function_data)
#             Workspace.data_changed()
#
#     @staticmethod
#     def remove():
#         pass
#
#     @staticmethod
#     def from_current():
#         function_item = TreeHelper.current_item()
#         function_name = TreeHelper.get_child_item_by_text(function_item, 'name').text(1)
#         library_item = function_item.parent()
#         library_name = library_item.text(0)
#         package_name = TreeHelper.get_package_item(library_item).text(0)
#         function_data = None
#         for data in Workspace.roslab_packages()[package_name]['libraries'][library_name]['function']:
#             if data['name'] == function_name:
#                 function_data = data
#         if not function_data:
#             raise RuntimeError('Something went badly wrong! Never should reach here!')
#         return function_name, function_item, function_data
#
#
# class Library(TreeItem):
#
#     def __init__(self):
#         TreeItem.__init__(self)
#
#     @staticmethod
#     def add_import():
#         """
#         Add dialog-based a module / class import to selected library.
#         """
#         Library.handle_dialog(ImportDialog)
#
#     @staticmethod
#     def add_param():
#         """
#         Add dialog-based a ROS parameter to selected library.
#         """
#         Library.handle_dialog(ParameterDialog)
#
#     @staticmethod
#     def add_function():
#         """
#         Add dialog-based a function to current selected library.
#         """
#         library_name, library_item, library_data = Library.from_current()
#         name, ok = QInputDialog.getText(g.main_widget, 'Add function to library', 'name:')
#         if ok and str(name) != '':
#             if 'function' not in library_data:
#                 library_data['function'] = []
#             function = {'name': str(name), 'args': [], 'code': g.literal_str('# TODO implement me!\npass')}
#             library_data['function'].append(function)
#             TreeHelper.update_item(library_item, library_data)
#             Workspace.data_changed()
#
#     @staticmethod
#     def add_basic_comm():
#         """
#         Add dialog-based basic ROS communication to selected library.
#         Basic ROS communication types are Pubs/Subs, ServiceClients/Servers.
#         """
#         Library.handle_dialog(CommunicationDialog)
#
#     @staticmethod
#     def add_advanced_comm():
#         """
#         Add dialog-based advanced ROS communication to selected library.
#         Advanced ROS communication types are Simple/ActionClients/Servers.
#         """
#         # TODO implement me!
#         pass
#
#     @staticmethod
#     def add_tf_broadcaster():
#         # TODO implement me!
#         pass
#
#     @staticmethod
#     def add_tf_listener():
#         # TODO implement me!
#         pass
#
#     @staticmethod
#     def preview():
#         library_name, library_item, library_data = Library.from_current()
#         backend = PyBackend(library_name, data=library_data)
#         g.preview_widget.setText(backend.generate())
#
#     @staticmethod
#     def handle_dialog(dialog):
#         library_name, library_item, library_data = Library.from_current()
#         library_dialog = dialog(data=library_data, parent=g.main_widget)
#         if library_dialog.exec_() == QDialog.Accepted:
#             TreeHelper.update_item(library_item, library_data)
#             Workspace.data_changed()
#
#     @staticmethod
#     def remove():
#         # get library item and name
#         library_name, library_item, library_data = Library.from_current()
#         # get package item and name
#         package_item = library_item.parent().parent()
#         package_name = package_item.text(0)
#         # remove library item from tree and delete it
#         libraries_item = library_item.parent()
#         libraries_item.removeChild(library_item)
#         del library_item
#         # remove library from package's libraries dict
#         Workspace.roslab_packages()[package_name]['libraries'].pop(library_name)
#         # if removed library was the only one
#         if Workspace.roslab_packages()[package_name]['libraries'] == {}:
#             # remove libraries item from tree and delete it
#             package_item.removeChild(libraries_item)
#             del libraries_item
#             # remove libraries dict from package
#             Workspace.roslab_packages()[package_name].pop('libraries')
#         Workspace.data_changed()
#
#     @staticmethod
#     def from_current():
#         library_item = TreeHelper.current_item()
#         library_name = library_item.text(0)
#         package_name = TreeHelper.get_package_item(library_item).text(0)
#         library_data = Workspace.roslab_packages()[package_name]['libraries'][library_name]
#         return library_name, library_item, library_data