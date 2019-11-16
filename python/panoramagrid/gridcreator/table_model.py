from enum import Enum, unique

from matplotlib.backends.qt_compat import QtCore


class TableModel(QtCore.QAbstractItemModel):
    @unique
    class Columns(Enum):
        X = 0
        Y = 1
        IMAGE = 2

    def __init__(self, file):
        super().__init__()
        self._points = []
        self._images = []
        
        self.file = file
        try:
            with open(file, 'r') as f:
                for line in f:
                    columns = line.split()
                    if len(columns) < 2:
                        columns = [None] + columns
                    camera, standard = columns
                    self.append_point(self.std_to_point(standard), camera, noflush=True)
        except FileNotFoundError:
            pass

    def index(self, row, column, parent=QtCore.QModelIndex()):
        return self.createIndex(row, column)

    def parent(self, index):
        return QtCore.QModelIndex()

    def rowCount(self, parent=QtCore.QModelIndex()):
        return len(self._points)

    def columnCount(self, parent=QtCore.QModelIndex()):
        # noinspection PyTypeChecker
        return len(self.Columns)

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if role == QtCore.Qt.DisplayRole:
            column = self.Columns(index.column())
            if column == self.Columns.X:
                return self._points[index.row()][0]
            if column == self.Columns.Y:
                return self._points[index.row()][1]
            if column == self.Columns.IMAGE:
                return self._images[index.row()]
        return None

    def setData(self, index, value, role=QtCore.Qt.EditRole):
        def success():
            self.dataChanged.emit(index, index, [role])
            return True

        if role == QtCore.Qt.EditRole:
            column = self.Columns(index.column())
            if column in [self.Columns.X, self.Columns.Y]:
                element = [self.Columns.X, self.Columns.Y].index(column)
                if self.is_point_complete(self._points[index.row()]):
                    return False
                if value is None:
                    self._points[index.row()][element] = None
                    return success()
                try:
                    self._points[index.row()][element] = float(value)
                except ValueError:
                    return False
                
                if self.is_point_complete(self._points[index.row()]):
                    self.flush_file()
                return success()

    def flags(self, index=QtCore.QModelIndex()):
        column = self.Columns(index.column())
        if column in [self.Columns.X, self.Columns.Y]:
            if self.is_point_complete(self._points[index.row()]):
                return QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled
            return QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsEditable
        if column == self.Columns.IMAGE:
            return QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if role == QtCore.Qt.DisplayRole:
            if orientation == QtCore.Qt.Horizontal:
                return self.Columns(section).name.lower().replace('_', " ")
            if orientation == QtCore.Qt.Vertical:
                return section + 1

    def get_data(self):
        return [(point, image) for point, image in zip(self._points, self._images) if None not in point]

    def append_point(self, point=(None, None, 1.0), image=None, noflush=False):
        self.beginInsertRows(QtCore.QModelIndex(), len(self._points), len(self._points))
        self._points.append(list(point))
        self._images.append(image)
        self.endInsertRows()
        if self.is_point_complete(point) and not noflush:
            self.flush_file()

    def update_image(self, index, image):
        if index >= len(self._points):
            raise Exception()
        if not isinstance(image, str):
            raise Exception()
        self._images[index] = image
        self.flush_file()
        self.dataChanged.emit(
            self.createIndex(index, self.Columns.IMAGE.value),
            self.createIndex(index, self.Columns.IMAGE.value),
            [],
        )

    def remove_point(self, index):
        self.beginRemoveRows(QtCore.QModelIndex(), index, index)
        self._points.pop(index)
        self._images.pop(index)
        self.endRemoveRows()
        self.flush_file()

    def is_point_complete(self, point=None, index=None):
        if point is not None and index is not None:
            raise Exception()
        if index is not None:
            point = self._points[index]
        return None not in point
    
    def flush_file(self):
        with open(self.file, "w") as f:
            f.writelines([
                f"{image}\t{self.point_to_std(point)}\n" if image else f"{self.point_to_std(point)}\n"
                for point, image in zip(self._points, self._images)
            ])

    @staticmethod
    def std_to_point(name):
        return map(lambda x: float(x), name.replace('.jpg', '').split('_'))

    @staticmethod
    def point_to_std(point):
        x, y, z = point
        return f"{x:.1f}_{y:.1f}_{z:.1f}.jpg"
