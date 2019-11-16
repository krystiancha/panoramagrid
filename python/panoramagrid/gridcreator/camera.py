from urllib.error import URLError

from matplotlib.backends.qt_compat import QtCore, QtWidgets
from panoramagrid.gridcreator.osc_api import OscApi


class Camera(QtWidgets.QWidget):
    def __init__(self, parent):
        super().__init__(parent)

        self.api = OscApi("http://192.168.1.1", timeout=5)

        self.status = QtWidgets.QLabel("Camera: OK")

        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.status)
        self.setLayout(self.layout)

    def take_picture(self):
        try:
            ret = self.api.take_picture()
            self.status.setText("Camera: OK")
            return ret
        except URLError as e:
            self.status.setText("Camera: ERROR")
            raise URLError(e.reason) from e

