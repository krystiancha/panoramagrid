import argparse
import sys
from urllib.parse import urlparse

from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)
from matplotlib.backends.qt_compat import QtWidgets, QtCore

from panoramagrid.gridcreator.camera import Camera
from panoramagrid.gridcreator.plot import Plot
from panoramagrid.gridcreator.stats import Stats
from panoramagrid.gridcreator.table import Table
from panoramagrid.gridcreator.table_model import TableModel


class MainWidget(QtWidgets.QWidget):
    def __init__(self, parent, grid_path):
        super().__init__(parent)

        self.camera = Camera(self)

        self.item_model = TableModel(grid_path)

        self.selection_model = QtCore.QItemSelectionModel(self.item_model)
        self.table = Table(self.item_model, self.selection_model, self.camera)

        self.plot = Plot(self.item_model, self.selection_model)

        self.stats = Stats(self, self.item_model)

        self.layout = QtWidgets.QVBoxLayout(self)

        self.bottom_layout = QtWidgets.QHBoxLayout()
        self.bottom_layout.addWidget(self.table, 3)

        self.bottom_right_layout = QtWidgets.QVBoxLayout()
        self.bottom_right_layout.addWidget(self.stats)
        self.bottom_right_layout.addWidget(self.camera)
        self.bottom_layout.addLayout(self.bottom_right_layout, 1)

        self.layout.addWidget(self.plot, 1)
        self.layout.addLayout(self.bottom_layout, 1)
        self.setLayout(self.layout)


class ApiBaseAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        values = urlparse(values, scheme='http').geturl()
        setattr(namespace, self.dest, values)


parser = argparse.ArgumentParser(description='Build a grid of equirectangular images.')
parser.add_argument('path', type=str)
parser.add_argument('--api-base', '-b', type=str, default='http://192.168.1.1', action=ApiBaseAction)
parser.add_argument('--timeout', '-t', type=float, default=None)
args = parser.parse_args()

app = QtWidgets.QApplication(sys.argv)

window = QtWidgets.QMainWindow()
window.setWindowTitle("GridCreator")

widget = MainWidget(window, args.path)
window.setCentralWidget(widget)
window.addToolBar(NavigationToolbar(widget.plot.canvas, window))
window.show()

sys.exit(app.exec_())
