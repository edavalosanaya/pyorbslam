from PyQt5 import QtCore

class CommBus(QtCore.QObject):

    closeApp = QtCore.pyqtSignal()
    zeromqSub = QtCore.pyqtSignal(str, int)
    dataUpdate = QtCore.pyqtSignal(object)
    imageUpdate = QtCore.pyqtSignal(object)
    visualCreate = QtCore.pyqtSignal(str, str)
    visualDelete = QtCore.pyqtSignal(str)
