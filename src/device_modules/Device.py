class DEVICE(port):
    configValidOptions = []
    configIgnoreValueCheck = []

    def __init__(self, port):
        global config
        self.__port = port
        self.__section = config[type(self).__name__ + ':' + self.__port]

    @abstractmethod
    def _connect():
        return None

    @abstractmethod
    def _fetch():
        return None

    @abstractmethod
    def _disconnect():
        return None
