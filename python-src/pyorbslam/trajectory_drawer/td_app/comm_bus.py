from PyQt5 import QtCore

class CommBus(QtCore.QObject):
    
    # Qt
    closeApp = QtCore.pyqtSignal()

    # Networking
    zeromqSub = QtCore.pyqtSignal(str, int)

    # Display Image
    imageUpdate = QtCore.pyqtSignal(object)

    # Display 3D
    dataUpdate = QtCore.pyqtSignal(object)
    visualCreate = QtCore.pyqtSignal(str, str)
    visualDelete = QtCore.pyqtSignal(str)
    toggleCamera = QtCore.pyqtSignal()

    # Resetting
    resetEvent = QtCore.pyqtSignal()
