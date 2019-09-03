from urllib.error import URLError

import cv2
import numpy as np
from matplotlib.backends.qt_compat import QtCore, QtWidgets

from panoramagrid.gridcreator.osc_api import OscApi


class Camera(QtWidgets.QWidget):
    class CameraThread(QtCore.QThread):
        next_frame = QtCore.Signal(np.ndarray)
        picture_taken = QtCore.Signal(str)

        def __init__(self, parent):
            super().__init__(parent)
            self.api = OscApi("http://192.168.1.1", timeout=5)
            self.picture_name = None
            self.stop = False

        def run(self):
            try:
                stream = self.api.live_preview()
            except URLError:
                return
            while not self.stop:
                if self.picture_name is not None:
                    stream.close()
                    img = self.api.take_picture(delete_local=True)
                    cv2.imwrite(self.picture_name, img)
                    self.picture_taken.emit(self.picture_name)
                    self.picture_name = None
                    stream = self.api.live_preview()
                else:
                    img = cv2.resize(self.api.get_live_frame(stream), (int(1920 / 2), int(960 / 2)))
                    self.next_frame.emit(img)

    def __init__(self, parent):
        super().__init__(parent)

        self.thread = self.CameraThread(self)
        self.thread.start()
        self.thread.next_frame.connect(self.display_frame)

        cv2.namedWindow("GridCreator: Preview", cv2.WINDOW_NORMAL)

    @QtCore.Slot(np.ndarray)
    def display_frame(self, frame):
        cv2.imshow("GridCreator: Preview", frame)
        cv2.waitKey(1)

    @QtCore.Slot(str)
    def take_picture(self, name):
        self.thread.picture_name = name
