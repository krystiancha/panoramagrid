import numpy as np
from matplotlib.backends.qt_compat import QtCore, QtWidgets


class Table(QtWidgets.QWidget):
    def __init__(self, item_model, selection_model, camera):
        super().__init__()

        self.item_model = item_model
        self.selection_model = selection_model

        self.camera = camera
        self.picture_pending_idx = None
        self.camera.thread.picture_taken.connect(self.update_picture)

        self.table_view = QtWidgets.QTableView()
        self.table_view.setModel(item_model)
        self.table_view.setSelectionModel(selection_model)
        self.table_view.sectionSize = lambda idx: 300
        self.selection_model.currentRowChanged.connect(self.update_selected)

        self.add_button = QtWidgets.QPushButton("Add row")
        self.add_button.clicked.connect(self.append_point)

        self.picture_button = QtWidgets.QPushButton("Take picture")
        self.picture_button.setEnabled(False)
        self.picture_button.clicked.connect(self.take_picture)

        self.delete_button = QtWidgets.QPushButton("Delete row")
        self.delete_button.setEnabled(False)
        self.delete_button.clicked.connect(self.remove_point)

        self.deselect_button = QtWidgets.QPushButton("Deselect")
        self.deselect_button.clicked.connect(self.deselect_current)

        self.layout = QtWidgets.QVBoxLayout(self)

        self.button_layout = QtWidgets.QHBoxLayout()
        self.button_layout.addWidget(self.add_button)
        self.button_layout.addWidget(self.picture_button)
        self.button_layout.addWidget(self.delete_button)
        self.button_layout.addWidget(self.deselect_button)

        self.layout.addWidget(self.table_view)
        self.layout.addLayout(self.button_layout)
        self.setLayout(self.layout)

    @QtCore.Slot()
    def append_point(self):
        self.item_model.append_point()

    @QtCore.Slot()
    def remove_point(self):
        current_index = self.selection_model.currentIndex()
        if current_index.isValid():
            self.item_model.remove_point(current_index.row())

    @QtCore.Slot(QtCore.QModelIndex, QtCore.QModelIndex)
    def update_selected(self, current, previous):
        self.delete_button.setEnabled(current.isValid())
        self.picture_button.setEnabled(current.isValid() and self.item_model.is_point_complete(index=current.row()))

    @QtCore.Slot()
    def take_picture(self):
        current_index = self.selection_model.currentIndex()
        if current_index.isValid():
            self.picture_pending_idx = current_index.row()
            self.camera.take_picture()

    @QtCore.Slot(np.ndarray)
    def update_picture(self, img):
        self.item_model.update_image(self.picture_pending_idx, img)
        self.picture_pending_idx = None

    @QtCore.Slot()
    def deselect_current(self):
        self.selection_model.clearCurrentIndex()
        self.selection_model.clearSelection()
