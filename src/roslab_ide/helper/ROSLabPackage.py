# __author__ = 'privat'
#
# import os
# import stat
#
# from python_qt_binding.QtGui import QInputDialog
#
# from roslab_ide.helper.Workspace import Workspace
# from roslab_ide.helper.TreeHelper import TreeHelper, TreeItem
# from roslab_ide.helper.globals import author, email, place, main_widget, ROS_PKGS
# from roslab_ide.generators.CMakeListsTxtGenerator import CMakeListsTxtGenerator
# from roslab_ide.generators.PackageXmlGenerator import PackageXmlGenerator
# from roslab_ide.generators.SetupPyGenerator import SetupPyGenerator
# from roslab_ide.backends.PyBackend import PyBackend
#
#
# class Node(TreeItem):
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
# class ROSLabPackage():
#
#     def __init__(self):
#         pass
#
#     @staticmethod
#     def add_dependency():
#         package_name, package_item, package_data = ROSLabPackage.current_package()
#         dependency, ok = QInputDialog.getItem(main_widget, 'Add dependency to package', 'dependency name', ROS_PKGS)
#         dependency = str(dependency)
#         if not ok:
#             return
#         if 'dependencies' not in package_data:
#             package_data['dependencies'] = []
#         package_data['dependencies'].append(dependency)
#         TreeHelper.update_item(package_item, package_data)
#         Workspace.data_changed()
#
#     @staticmethod
#     def add_library():
#         package_name, package_item, package_data = ROSLabPackage.current_package()
#         library, ok = QInputDialog.getText(main_widget, 'Add library to package', 'library name:')
#         library = str(library)
#         if not ok or library == '':
#             return
#         if 'libraries' not in package_data:
#             package_data['libraries'] = {}
#         package_data['libraries'][library] = {
#             'author': author,
#             'copyright': place,
#             'credits': [],
#             'email': email,
#             'info': 'TODO',
#             'maintainer': author,
#             'status': 'freshly generated',
#             'version': '0.0.1'
#         }
#         TreeHelper.update_item(package_item, package_data)
#         Workspace.data_changed()
#
#     @staticmethod
#     def add_node():
#         pass
#
#     @staticmethod
#     def generate():
#         package_name, package_item, package_data = ROSLabPackage.current_package()
#         package_path = os.path.join(Workspace.path(), 'src', package_name)
#         # update package.xml
#         package_xml = file(os.path.join(package_path, 'package.xml'), 'w+')
#         package_xml.write(PackageXmlGenerator(package_name, build_depends=package_data['dependencies'],
#                           run_depends=package_data['dependencies']).generate())
#         # update CMakeLists.txt
#         cmakelists_txt = file(os.path.join(package_path, 'CMakeLists.txt'), 'w+')
#         cmakelists_txt.write(CMakeListsTxtGenerator(package_name, catkin_depends=package_data['dependencies'],
#                              python_setup=True, python_scripts=package_data['nodes'].keys()).generate())
#         # update setup.py if needed
#         setup_py = file(os.path.join(package_path, 'setup.py'), 'w+')
#         setup_py.write(SetupPyGenerator(package_name, requirements=package_data['dependencies']).generate())
#         # generate libraries
#         if not os.path.exists(os.path.join(package_path, 'src')):
#             os.mkdir(os.path.join(package_path, 'src'))
#         if not os.path.exists(os.path.join(package_path, 'src', package_name)):
#             os.mkdir(os.path.join(package_path, 'src', package_name))
#         open(os.path.join(package_path, 'src', package_name, '__init__.py'), 'w+')
#         libraries = package_data['libraries']
#         for library_name in libraries.keys():
#             library_file = file(os.path.join(package_path, 'src', package_name, '{}.py'.format(library_name)), 'w+')
#             library_file.write(PyBackend(library_name, data=libraries[library_name]).generate())
#         # generate nodes
#         if not os.path.exists(os.path.join(package_path, 'scripts')):
#             os.mkdir(os.path.join(package_path, 'scripts'))
#         nodes = package_data['nodes']
#         for node_name in nodes.keys():
#             node_path = os.path.join(package_path, 'scripts', '{}_node'.format(node_name))
#             node_file = file(node_path, 'w+')
#             node_file.write(PyBackend.generate_node(node_name, package_name, nodes[node_name]))
#             node_stat = os.stat(node_path)
#             os.chmod(node_path, node_stat.st_mode | stat.S_IEXEC)
#
#     @staticmethod
#     def current_package():
#         item = TreeHelper.current_item()
#         name = item.text(0)
#         data = Workspace.roslab_packages()[name]
#         return name, item, data