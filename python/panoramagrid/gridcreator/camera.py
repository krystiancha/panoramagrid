from enum import unique, Enum, auto
from urllib.error import URLError

import cv2
import numpy as np
from matplotlib.backends.qt_compat import QtCore, QtWidgets

from panoramagrid.gridcreator.osc_api import OscApi


class Camera(QtWidgets.QWidget):
    class CameraThread(QtCore.QThread):
        @unique
        class State(Enum):
            IDLE = auto()
            STARTING_PREVIEW = auto()
            ERROR = auto()
            PREVIEWING = auto()
            TAKING_PICTURE = auto()

        state_changed = QtCore.Signal(State, str)
        next_frame = QtCore.Signal(np.ndarray)
        picture_taken = QtCore.Signal(np.ndarray)

        def __init__(self, parent):
            super().__init__(parent)
            self.api = OscApi("http://192.168.1.1", timeout=5)
            self.picture_name = None
            self.stop = False

        def run(self):
            self.picture_name = None
            self.state_changed.emit(self.State.STARTING_PREVIEW, None)
            try:
                stream = self.api.live_preview()
            except URLError as e:
                self.state_changed.emit(self.State.ERROR, str(e.reason))
                return
            while True:
                self.state_changed.emit(self.State.PREVIEWING, None)
                while self.picture_name is None:
                    if self.stop:
                        self.state_changed.emit(self.State.IDLE, None)
                        return
                    try:
                        img = cv2.resize(self.api.get_live_frame(stream), (int(1920 / 2), int(960 / 2)))
                    except URLError as e:
                        self.state_changed.emit(self.State.ERROR, str(e.reason))
                        return
                    self.next_frame.emit(img)
                self.state_changed.emit(self.State.TAKING_PICTURE, None)
                stream.close()
                try:
                    img = self.api.take_picture(delete_local=True)
                except URLError as e:
                    self.state_changed.emit(self.State.ERROR, str(e.reason))
                    return
                self.picture_taken.emit(img)
                self.picture_name = None
                stream = self.api.live_preview()

    def __init__(self, parent):
        super().__init__(parent)

        self.thread = self.CameraThread(self)
        self.thread.next_frame.connect(self.display_frame)
        self.thread.state_changed.connect(self.update_state)

        self.status = QtWidgets.QLabel("Camera state: IDLE")

        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect)
        self.disconnect_button = QtWidgets.QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self.disconnect)
        self.disconnect_button.setEnabled(False)

        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.status)
        self.layout.addWidget(self.connect_button)
        self.layout.addWidget(self.disconnect_button)
        self.setLayout(self.layout)

        cv2.namedWindow("GridCreator: Preview", cv2.WINDOW_NORMAL)

    @QtCore.Slot(np.ndarray)
    def display_frame(self, frame):
        cv2.imshow("GridCreator: Preview", frame)
        cv2.waitKey(1)

    @QtCore.Slot(str)
    def take_picture(self, ):
        self.thread.picture_name = True

    @QtCore.Slot(CameraThread.State, str)
    def update_state(self, state, data):
        self.status.setText("Camera state: \n\t" + state._name_.replace('_', ' ') + (f": {data}" if data else ""))
        self.connect_button.setEnabled(state in [self.CameraThread.State.IDLE, self.CameraThread.State.ERROR])
        self.disconnect_button.setEnabled(state is self.CameraThread.State.PREVIEWING)

    @QtCore.Slot()
    def connect(self):
        self.thread.stop = False
        self.thread.start()

    @QtCore.Slot()
    def disconnect(self):
        self.thread.stop = True
