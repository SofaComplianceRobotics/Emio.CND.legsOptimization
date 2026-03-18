from importlib import import_module
import pip

def checkModule(module:str,libraryName:str='',gitLink:str=''):
    """
    checks if the module is available, and installs it otherwise
    Args:
        module (str): name of the module.
        libraryName (str, optional): alternative name of the module. Defaults to ''.
        gitLink (str, optional): git repo link. Defaults to ''.
    """
    try:
        import_module(module)
    except ImportError:
        if not libraryName and not gitLink:
            pip.main(['install',module])
        elif not libraryName:
            pip.main(['install','git+'+gitLink])
        elif not gitLink:
            pip.main(['install',libraryName])
    