from enum import Enum, unique

from matplotlib.backends.qt_compat import QtCore


class TableModel(QtCore.QAbstractItemModel):
    @unique
    class Columns(Enum):
        ID = 0
        X = 1
        Y = 2
        IMAGE = 3

    def __init__(self):
        super().__init__()
        self.points = []
        self.id_counter = 0

    def index(self, row, column, parent=QtCore.QModelIndex()):
        return self.createIndex(row, column)

    def parent(self, index):
        return QtCore.QModelIndex()

    def rowCount(self, parent=QtCore.QModelIndex()):
        return len(self.points)

    def columnCount(self, parent=QtCore.QModelIndex()):
        # noinspection PyTypeChecker
        return len(self.Columns)

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if role == QtCore.Qt.DisplayRole:
            return self.points[index.row()][index.column()]
        return None

    def setData(self, index, value, role=QtCore.Qt.EditRole):
        def success():
            self.dataChanged.emit(index, index, [role])
            return True

        if role == QtCore.Qt.EditRole:
            column = self.Columns(index.column())
            if column in (self.Columns.X, self.Columns.Y):
                if value is None:
                    self.points[index.row()][index.column()] = None
                    return success()
                try:
                    self.points[index.row()][index.column()] = float(value)
                except ValueError:
                    return False
                return success()
            if column in (self.Columns.ID, self.Columns.IMAGE):
                self.points[index.row()][index.column()] = value
                return success()

    def flags(self, index=QtCore.QModelIndex()):
        column = self.Columns(index.column())
        if column in (self.Columns.X, self.Columns.Y):
            return QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsEditable
        elif column in (self.Columns.ID, self.Columns.IMAGE):
            return QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if role == QtCore.Qt.DisplayRole:
            if orientation == QtCore.Qt.Horizontal:
                return ("id", "x", "y", "image")[section]
            else:
                return section + 1

    def insertRows(self, row, count, parent=QtCore.QModelIndex()):
        if row != self.rowCount():
            raise NotImplemented()

        self.beginInsertRows(parent, row, row + count - 1)
        self.points += [[None, None, None, None] for _ in range(count)]
        self.endInsertRows()

    def removeRows(self, row, count, parent=QtCore.QModelIndex()):
        self.beginRemoveRows(parent, row, row + count - 1)
        for _ in range(count):
            self.points.pop(row)
        self.endRemoveRows()

    def add_point(self, point):
        x, y = point
        row = self.rowCount()
        self.insertRows(row, 1)
        self.setData(self.createIndex(row, self.Columns.ID._value_), self.id_counter)
        self.setData(self.createIndex(row, self.Columns.X._value_), x)
        self.setData(self.createIndex(row, self.Columns.Y._value_), y)
        self.id_counter += 1

    def get_point(self, index):
        return [
            self.data(self.createIndex(index, self.Columns.X)),
            self.data(self.createIndex(index, self.Columns.Y)),
        ]
