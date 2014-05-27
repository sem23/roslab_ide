__author__ = 'privat'

from roslab_ide.generators.Generator import Generator


class PackageXmlGenerator(Generator):

    def __init__(self, name, version='0.0.0', description=None, maintainer=('TODO', 'todo@todo.com'), license='TODO',
                 build_depends=None, run_depends=None):
        Generator.__init__(self)

        self._name = name
        self._version = version
        if not description:
            description = 'The {} package'.format(name)
        self._description = description
        self._maintainer = maintainer
        self._license = license
        if not build_depends:
            build_depends = []
        self._build_depends = []
        for dep in build_depends:
            self._build_depends.append(dep['name'])
        if not run_depends:
            run_depends = []
        self._run_depends = []
        for dep in run_depends:
            self._run_depends.append(dep['name'])

        self._generator_chain = [
            self.generate_intro,
            self.generate_build_depends,
            self.generate_run_depends,
            self.generate_outro
        ]

    def generate_intro(self):
        intro = [
            '<?xml version="1.0"?>',
            '<package>',
            '  <name>{}</name>'.format(self._name),
            '  <version>{}</version>'.format(self._version),
            '  <description>{}</description>'.format(self._description),
            '  <maintainer email="{1}">{0}</maintainer>'.format(self._maintainer[0], self._maintainer[1]),
            '  <license>{}</license>'.format(self._license),
            '  <buildtool_depend>catkin</buildtool_depend>',
        ]
        return intro

    def generate_build_depends(self):
        build_depends = []
        for build_depend in self._build_depends:
            build_depends.append('  <build_depend>{}</build_depend>'.format(build_depend))
        return build_depends

    def generate_run_depends(self):
        run_depends = []
        for run_depend in self._run_depends:
            run_depends.append('  <run_depend>{}</run_depend>'.format(run_depend))
        return run_depends

    def generate_outro(self):
        outro = [
            '</package>'
        ]
        return outro

