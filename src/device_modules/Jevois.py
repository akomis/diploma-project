import Device

class Jevois(device):
    configValidOptions = ["objects","objectidentified","objectlocation","objectsize"]
    configIgnoreValueCheck = ["objects"]

    def _connect(self):
        try:
            self.__serial = serial.Serial(self.__port, 115200, timeout=1)
            self.__prominit()
            return True
        except Exception as e:
            return False

    def __prominit(self):
        if self.__section.getboolean('ObjectIdentified', fallback=True):
            if self.__section.get('objects') is not None:
                self.__options = self.__section["options"].split()
                self.__objectIdentified = Enum('object_identified', 'Object Identified', states=self.__options, ['device'])
            else:
                print('The \"options\" list is necessary for monitoring identified objects')
                print('Skipping monitoring objects identified for JEVOIS:' + self.__port)

        if self.__section.getboolean('ObjectLocation', fallback=True):
            self.__objectLocationX = Gauge('object_location_x', 'Identified object\'s x position',['device'])
            if int(dimension) > 1:
                self.__objectLocationY = Gauge('object_location_y', 'Identified object\'s y position',['device'])
            if int(dimension) == 3:
                self.__objectLocationZ = Gauge('object_location_z', 'Identified object\'s Z position',['device'])

        if self.__section.getboolean('ObjectSize', fallback=False):
            self.__objectSize = Gauge('object_size','Identified object\'s size', ['device'])

    def _fetch(self):
        line = self.__serial.readline().rstrip()
        tok = line.split()
        dimension = tok[0][1]

        # Abort fetching if timeout or malformed line
        if len(tok) < 1: return
        if dimension == '1' and len(tok) != 4: return
        if dimension == '2' and len(tok) != 6: return
        if dimension == '3' and len(tok) != 8: return

        if self.__section.getboolean('ObjectIdentified', fallback=True):
            if self.__options is not None and tok[1] in self.__options:
                self.__objectIdentified.labels('jevois'+self.__port).state(tok[1])

        if self.__section.getboolean('ObjectLocation', fallback=True):
            self.__objectLocationX.lables('jevois'+self.__port).set(float(tok[2]))

            if int(dimension) > 1:
                self.__objectLocationY.labels('jevois'+self.__port).set(float(tok[3]))

            if int(dimension) == 3:
                self.__objectLOcationZ.labels('jevois'+self.__port).set(float(tok[4]))

        if self.__section.getboolean('ObjectSize', fallback=False):
            if dimension == '1':
                self.__objectSize.labels('jevois'+self.__port).set(float(tok[3]))
            elif dimension == '2':
                self.__objectSize.labels('jevois'+self.__port).set(float(tok[4])*float(tok[5]))
            elif dimension == '3':
                self.__objectSize.labels('jevois'+self.__port).set(float(tok[5])*float(tok[6])*float(tok[7]))

    def _disconnect(self):
        self.__serial.close()
