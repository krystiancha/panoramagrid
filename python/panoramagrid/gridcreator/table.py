from matplotlib.backends.qt_compat import QtCore, QtWidgets


class Table(QtWidgets.QWidget):
    def __init__(self, item_model, selection_model, camera):
        super().__init__()

        self.item_model = item_model
        self.selection_model = selection_model

        self.camera = camera
        self.picture_pending_row = None
        self.camera.thread.picture_taken.connect(self.update_picture)

        self.table_view = QtWidgets.QTableView()
        self.table_view.setModel(item_model)
        self.table_view.setSelectionModel(selection_model)
        self.table_view.sectionSize = lambda idx: 300
        self.selection_model.currentRowChanged.connect(self.update_selected)

        self.add_button = QtWidgets.QPushButton("Add row")
        self.add_button.clicked.connect(self.append_point)

        self.picture_button = QtWidgets.QPushButton("Take picture")
        self.picture_button.clicked.connect(self.take_picture)

        self.delete_button = QtWidgets.QPushButton("Delete row")
        self.delete_button.setEnabled(False)
        self.delete_button.clicked.connect(self.remove_point)

        self.deselect_button = QtWidgets.QPushButton("Clear current")
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
        self.item_model.add_point((None, None))

    @QtCore.Slot()
    def remove_point(self):
        current_index = self.selection_model.currentIndex()

        if current_index.isValid():
            self.table_view.model().removeRows(current_index.row(), 1)

    @QtCore.Slot(QtCore.QModelIndex)
    def update_selected(self, current):
        self.delete_button.setEnabled(current.isValid() and len(self.item_model.points))
        self.picture_button.setEnabled(current.isValid() and len(self.item_model.points))

    @QtCore.Slot()
    def take_picture(self):
        current_index = self.selection_model.currentIndex()
        if current_index.isValid():
            point = self.item_model.get_point(current_index.row())
            self.picture_pending_row = current_index.row()
            self.camera.take_picture(f'img/{point[0]}_{point[1]}_1.0.jpg')

    @QtCore.Slot(str)
    def update_picture(self, name):
        self.item_model.setData(self.item_model.createIndex(self.picture_pending_row, 3), name)
        self.picture_pending_row = None

    @QtCore.Slot()
    def deselect_current(self):
        self.selection_model.clearCurrentIndex()
        self.selection_model.clearSelection()
