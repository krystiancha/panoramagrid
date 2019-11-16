from matplotlib.backends.qt_compat import QtCore, QtWidgets


class Stats(QtWidgets.QWidget):
    def __init__(self, parent, item_model):
        super().__init__(parent)

        self.item_model = item_model
        self.item_model.dataChanged.connect(self.update_stats)
        self.item_model.rowsInserted.connect(self.update_stats)
        self.item_model.rowsRemoved.connect(self.update_stats)

        self.count = QtWidgets.QLabel()
        self.images_remaining = QtWidgets.QLabel()
        self.update_stats()

        self.layout = QtWidgets.QFormLayout(self)
        self.layout.addRow(QtWidgets.QLabel("Point count:"), self.count)
        self.layout.addRow(QtWidgets.QLabel("Pts. w/o an image:"), self.images_remaining)
        self.setLayout(self.layout)

    @QtCore.Slot()
    def update_stats(self):
        data = self.item_model.get_data()

        count = len(data)
        images_remaining = len([point for point, image in data if not image])
        points, _ = zip(*data) if count else [[], []]

        x, y, _ = zip(*points) if count else [[], [], []]

        self.count.setText(f'{count or 0}')
        self.images_remaining.setText(f'{images_remaining or 0}')
