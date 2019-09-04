from math import sqrt, pow

import alphashape
from descartes import PolygonPatch
from matplotlib.backends.backend_qt5agg import (FigureCanvas)
from matplotlib.backends.qt_compat import QtCore, QtWidgets
from matplotlib.figure import Figure


class Plot(QtWidgets.QWidget):
    def __init__(self, item_model, selection_model):
        super().__init__()

        self.item_model = item_model
        self.item_model.dataChanged.connect(self.update_plot)
        self.item_model.rowsInserted.connect(self.update_plot)
        self.item_model.rowsRemoved.connect(self.update_plot)

        self.selection_model = selection_model
        self.selection_model.currentRowChanged.connect(self.update_plot)

        self.canvas = FigureCanvas(Figure())
        self.canvas.mpl_connect('button_press_event', self.add_or_show)

        self.axes = self.canvas.figure.subplots()
        self.axes.axis('equal')

        self.update_plot()

        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.canvas)
        self.setLayout(self.layout)

    @QtCore.Slot()
    def update_plot(self):
        xlim, ylim = (self.axes.get_xlim(), self.axes.get_ylim())
        self.axes.clear()
        self.axes.autoscale(False)

        data = self.item_model.get_data()
        have_image = []
        no_image = []
        for point, image in data:
            if image:
                have_image.append(point)
            else:
                no_image.append(point)

        if len(have_image) > 0:
            x, y, _ = zip(*have_image)
            self.axes.plot(x, y, 'o', markersize=5, color='#2ca02c')

        if len(no_image) > 0:
            x, y, _ = zip(*no_image)
            self.axes.plot(x, y, 'o', markersize=5, color='#1f77b4')

        current_index = self.selection_model.currentIndex()
        if current_index.isValid() and current_index.row() < len(data):
            x, y, _ = data[current_index.row()][0]
            self.axes.plot([x], [y], 'o', markersize=10, fillstyle='none', markeredgewidth=2, color='#ff7f0e')

        if len(data) >= 3:
            points, _ = zip(*data)
            x, y, _ = zip(*points)
            alpha_shape = alphashape.alphashape(zip(x, y), 0)
            if alpha_shape.geom_type == 'Polygon':
                self.axes.add_patch(PolygonPatch(alpha_shape, alpha=0.2))

        self.axes.set_xlim(xlim)
        self.axes.set_ylim(ylim)
        self.canvas.draw()

    @QtCore.Slot()
    def add_or_show(self, event):
        if self.canvas.toolbar.mode != '' or event.button != 1:
            return

        click_x, click_y = (event.xdata, event.ydata)
        if None in [click_x, click_y]:
            return

        scale = self.axes.get_xlim()[1] - self.axes.get_xlim()[0]
        for idx, ((x, y, _), _) in enumerate(self.item_model.get_data()):
            distance = sqrt(pow(x - click_x, 2) + pow(y - click_y, 2))
            if distance < scale * 0.01:
                self.selection_model.setCurrentIndex(
                    self.item_model.createIndex(idx, 0),
                    QtCore.QItemSelectionModel.Current,
                )
                return

        # Add point by clicking on the plot
        # self.item_model.append_point([round(click_x, 3), round(click_y, 3), 0])
