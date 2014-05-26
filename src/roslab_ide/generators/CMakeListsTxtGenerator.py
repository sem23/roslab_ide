__author__ = 'privat'

from roslab_ide.generators.Generator import Generator


class CMakeListsTxtGenerator(Generator):

    def __init__(self, project, include_dirs=None, libraries=None, catkin_depends=None, depends=None,
                 python_scripts=None, python_setup=False):
        Generator.__init__(self)
        self._project = project
        self._include_dirs = include_dirs
        self._libraries = libraries
        self._catkin_depends = []
        for dep in catkin_depends:
            self._depends.append(dep['name'])
        self._depends = []
        for dep in depends:
            self._depends.append(dep['name'])
        for script in python_scripts:
            self._python_scripts.append(script['name'])
        self._python_setup = python_setup

        self._generator_chain = [
            self.generate_intro,
            self.generate_catkin_required_components,
            self.generate_catkin_python_setup,
            self.generate_catkin_package,
            self.generate_include_directories,
            self.generate_install_scripts
        ]

    def generate_intro(self):
        intro = [
            "cmake_minimum_required(VERSION 2.8.3)",
            "project({0})".format(self._project),
        ]
        return intro

    def generate_catkin_required_components(self):
        required_components = [
            "find_package(catkin REQUIRED COMPONENTS",
            self.intended_join(self._catkin_depends, newline=True),
            ")"
        ]
        return required_components

    def generate_catkin_python_setup(self):
        if self._python_setup:
            return ['catkin_python_setup()']

    def generate_catkin_package(self):
        catkin_package = list()
        catkin_package.append('catkin_package(')
        if self._include_dirs:
            catkin_package.append('  INCLUDE_DIRS ' + ' '.join(self._include_dirs))
        if self._libraries:
            catkin_package.append('  LIBRARIES ' + ' '.join(self._libraries))
        if self._catkin_depends:
            catkin_package.append('  CATKIN_DEPENDS ' + ' '.join(self._catkin_depends))
        if self._depends:
            catkin_package.append('  DEPENDS ' + ' '.join(self._depends))
        catkin_package.append(')')
        return catkin_package

    def generate_include_directories(self):
        include_directories = list()
        if self._include_dirs:
            include_directories.append('include_directories({})'.format(' '.join(self._include_dirs)))
        include_directories.extend([
            'include_directories(',
            '  ${catkin_INCLUDE_DIRS}',
            ')'
        ])
        return include_directories

    def generate_install_scripts(self):
        install_scripts = []
        scripts = []
        if self._python_scripts:
            scripts.extend(self._python_scripts)
        if len(scripts):
            for i in range(len(scripts)):
                scripts[i] = 'scripts/' + scripts[i] + '_node'
            install_scripts.extend([
                'install(PROGRAMS',
                self.intended_join(scripts, newline=True),
                '  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}',
                ')'
            ])
        return install_scripts
