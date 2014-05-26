__author__ = 'privat'

from roslab_ide.generators.Generator import Generator


class SetupPyGenerator(Generator):

    def __init__(self, name, requirements):
        Generator.__init__(self)

        self._name = name
        self._requirements = []
        for req in requirements:
            self._requirements.append(req)

    def generate(self):
        generated = [
            '## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD',
            '',
            'from distutils.core import setup',
            'from catkin_pkg.python_setup import generate_distutils_setup',
            '',
            '# fetch values from package.xml',
            'setup_args = generate_distutils_setup(',
            "    packages=['{}'],".format(self._name),
            "    package_dir={'': 'src'},",
            "    requires=['{}']".format("', '".join(self._requirements)),
            ')',
            'setup(**setup_args)'
        ]
        return '\n'.join(generated)
