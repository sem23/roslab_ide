__author__ = 'privat'


class Generator():
    def __init__(self):
        self._generator_chain = []

    def generate(self):
        generated = []
        for generator in self._generator_chain:
            generated.extend(generator())
        return '\n'.join(generated)

    @staticmethod
    def intended_join(iterable, intend=2, newline=False):
        joiner = intend * ' '
        if newline:
            joiner = '\n' + joiner
        return intend * ' ' + joiner.join(iterable)