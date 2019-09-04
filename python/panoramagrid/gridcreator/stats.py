import alphashape as alphashape
from matplotlib.backends.qt_compat import QtCore, QtWidgets


class Stats(QtWidgets.QWidget):
    def __init__(self, parent, item_model):
        super().__init__(parent)

        self.item_model = item_model
        self.item_model.dataChanged.connect(self.update_stats)
        self.item_model.rowsInserted.connect(self.update_stats)
        self.item_model.rowsRemoved.connect(self.update_stats)

        self.count = QtWidgets.QLabel()
        self.area = QtWidgets.QLabel()
        self.density = QtWidgets.QLabel()
        self.images_remaining = QtWidgets.QLabel()
        self.update_stats()

        self.layout = QtWidgets.QFormLayout(self)
        self.layout.addRow(QtWidgets.QLabel("Point count:"), self.count)
        self.layout.addRow(QtWidgets.QLabel("Convex area:"), self.area)
        self.layout.addRow(QtWidgets.QLabel("Avg. pt. density:"), self.density)
        self.layout.addRow(QtWidgets.QLabel("Pts. w/o an image:"), self.images_remaining)
        self.setLayout(self.layout)

    @QtCore.Slot()
    def update_stats(self):
        data = self.item_model.get_data()

        count = len(data)
        images_remaining = len([point for point, image in data if not image])
        points, _ = zip(*data) if count else [[], []]

        x, y, _ = zip(*points) if count else [[], [], []]
        alpha_shape = alphashape.alphashape(zip(x, y), 0)
        area = alpha_shape.area
        density = (count / area) if area else None

        self.count.setText(f'{count or 0}')
        self.area.setText(f'{area or 0: .3f} m²')
        self.density.setText(f'{density or 0: .3f} 1/m²')
        self.images_remaining.setText(f'{images_remaining or 0}')
