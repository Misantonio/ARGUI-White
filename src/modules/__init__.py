from subprocess import call
import importlib

def check_module(name):
    try:
        importlib.import_module(name)
    except ImportError:
        if name == 'pyqtgraph':
            raise ImportError('Python Package {} not found'.format(name))
        print ImportError('Python package {} not found, would like to install '
                          'it? ['
                          'Y/N]'.format(name))
        res = raw_input('');
        if res.capitalize() == 'Y':
            call(['sudo', 'pip', 'install', name])
        else:
            raise SystemExit('Module needs to install all dependencies '
                             'to run')

map(check_module,['numpy','matplotlib','sympy','PySide', 'pyqtgraph'])