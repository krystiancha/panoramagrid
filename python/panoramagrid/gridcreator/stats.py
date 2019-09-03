import alphashape as alphashape
import numpy as np
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
        self.update_stats()

        self.layout = QtWidgets.QFormLayout(self)
        self.layout.addRow(QtWidgets.QLabel("Point count:"), self.count)
        self.layout.addRow(QtWidgets.QLabel("Convex area:"), self.area)
        self.layout.addRow(QtWidgets.QLabel("Avg. pt. density:"), self.density)
        self.setLayout(self.layout)

    @QtCore.Slot()
    def update_stats(self):
        points = self.item_model.points
        complete_points = [x[1:3] for x in points if None not in x[1:3]]
        count = len(complete_points)

        x, y = zip(*complete_points) if count else [[], []]
        alpha_shape = alphashape.alphashape(complete_points, 0)
        area = alpha_shape.area
        density = (count / area) if area else None

        self.count.setText(f'{count or 0}')
        self.area.setText(f'{area or 0: .3f} m²')
        self.density.setText(f'{density or 0: .3f} 1/m²')
