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

        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.canvas)
        self.setLayout(self.layout)

    @QtCore.Slot()
    def update_plot(self, *args):
        xlim, ylim = (self.axes.get_xlim(), self.axes.get_ylim())
        self.axes.clear()
        self.axes.autoscale(False)

        try:
            x, y = zip(*[x[1:3] for x in self.item_model.points if not x[3]])
            self.axes.plot(x, y, 'o', markersize=5)
        except ValueError:
            pass

        complete_points = [x[1:3] for x in self.item_model.points if None not in x[1:3]]
        alpha_shape = alphashape.alphashape(complete_points, 0)
        if alpha_shape.geom_type == 'Polygon':
            self.axes.add_patch(PolygonPatch(alpha_shape, alpha=0.2))

        current_index = self.selection_model.currentIndex()
        if current_index.isValid():
            point = self.item_model.points[current_index.row()]
            self.axes.plot([point[1]], [point[2]], 'o', markersize=10, fillstyle='none', markeredgewidth=2)

        try:
            x, y = zip(*[x[1:3] for x in self.item_model.points if x[3]])
            self.axes.plot(x, y, 'o', markersize=5)
        except ValueError:
            pass

        self.axes.set_xlim(xlim)
        self.axes.set_ylim(ylim)
        self.canvas.draw()

    @QtCore.Slot()
    def add_or_show(self, event):
        if self.canvas.toolbar.mode != '' or event.button != 1:
            return

        click_x, click_y = (event.xdata, event.ydata)

        data_lim = self.axes.dataLim
        distance_x, distance_y = ((data_lim.x1 - data_lim.x0) / 25, (data_lim.y1 - data_lim.y0) / 25)
        for idx, (_, x, y, _) in enumerate(self.item_model.points):
            if not x or not y:
                continue
            if abs(click_x - x) < distance_x and abs(click_y - y) < distance_y:
                self.selection_model.setCurrentIndex(
                    self.item_model.createIndex(idx, 0),
                    QtCore.QItemSelectionModel.Current,
                )
                return

        self.item_model.add_point((round(click_x, 3), round(click_y, 3)))
