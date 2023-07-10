from PyQt5 import QtCore

class CommBus(QtCore.QObject):

    closeApp = QtCore.pyqtSignal()
    zeromqSub = QtCore.pyqtSignal(str, int)
