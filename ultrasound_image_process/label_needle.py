import sys
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QFileDialog, QAction,
                             QStatusBar)
from PyQt5.QtGui import QPixmap, QPainter, QPen, QColor, QFont
from PyQt5.QtCore import Qt, QPoint


class ImageViewer(QWidget):
    def __init__(self, parent=None):
        super(ImageViewer, self).__init__(parent)
        self.parent = parent
        self.path = None
        self.pixmap = None
        self.points = []  # 存储两个点的坐标
        self.setMinimumSize(600, 400)  # 最小尺寸不影响坐标，仅限制窗口大小
        self.setStyleSheet("background-color: #f0f0f0;")

    def loadImage(self, path):
        """加载图像"""
        self.path = path
        self.pixmap = QPixmap(path)
        self.points = []  # 重置点
        self.update()  # 重绘窗口
        self.parent.updateInfo()  # 更新信息显示

    def getScaledImageInfo(self):
        """获取缩放后图像的信息（尺寸和居中位置），统一计算逻辑"""
        if not self.pixmap:
            return None, 0, 0, 0, 0

        # 缩放图像（保持比例）
        scaled_pixmap = self.pixmap.scaled(
            self.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        # 计算图像在窗口中的居中位置（左上角坐标）
        x = (self.width() - scaled_pixmap.width()) // 2
        y = (self.height() - scaled_pixmap.height()) // 2

        return scaled_pixmap, x, y, scaled_pixmap.width(), scaled_pixmap.height()

    def paintEvent(self, event):
        """绘制事件，用于显示图像和点"""
        if self.pixmap:
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)

            # 获取缩放后图像信息（与点击事件共用同一套计算逻辑）
            scaled_pixmap, x, y, scaled_w, scaled_h = self.getScaledImageInfo()
            painter.drawPixmap(x, y, scaled_pixmap)

            # 绘制点和向量
            if len(self.points) >= 1:
                # 绘制起点（红色）
                pen = QPen(QColor(255, 0, 0), 6)
                painter.setPen(pen)
                painter.drawPoint(x + self.points[0].x() * (scaled_w / self.pixmap.width()),
                                  y + self.points[0].y() * (scaled_h / self.pixmap.height()))

                # 标记起点文本
                font = QFont()
                font.setPointSize(10)
                painter.setFont(font)
                painter.drawText(x + self.points[0].x() * (scaled_w / self.pixmap.width()) + 10,
                                 y + self.points[0].y() * (scaled_h / self.pixmap.height()) - 10, "起点")

                # 绘制终点和向量（如果有）
                if len(self.points) == 2:
                    # 绘制终点（蓝色）
                    pen = QPen(QColor(0, 0, 255), 6)
                    painter.setPen(pen)
                    painter.drawPoint(x + self.points[1].x() * (scaled_w / self.pixmap.width()),
                                      y + self.points[1].y() * (scaled_h / self.pixmap.height()))

                    # 标记终点文本
                    painter.drawText(x + self.points[1].x() * (scaled_w / self.pixmap.width()) + 10,
                                     y + self.points[1].y() * (scaled_h / self.pixmap.height()) - 10, "终点")

                    # 绘制向量（绿色虚线）
                    pen = QPen(QColor(0, 255, 0), 2, Qt.DashLine)
                    painter.setPen(pen)
                    painter.drawLine(
                        x + self.points[0].x() * (scaled_w / self.pixmap.width()),
                        y + self.points[0].y() * (scaled_h / self.pixmap.height()),
                        x + self.points[1].x() * (scaled_w / self.pixmap.width()),
                        y + self.points[1].y() * (scaled_h / self.pixmap.height())
                    )

    def mousePressEvent(self, event):
        """鼠标点击事件，修正坐标转换逻辑"""
        if self.pixmap and event.button() == Qt.LeftButton:
            # 获取缩放后图像的信息（与绘制事件共用同一套计算逻辑，避免偏差）
            scaled_pixmap, x, y, scaled_w, scaled_h = self.getScaledImageInfo()

            # 检查点击是否在图像区域内（防止点击空白区域）
            if (event.x() >= x and event.x() <= x + scaled_w and
                    event.y() >= y and event.y() <= y + scaled_h):

                # 计算点击位置在缩放图像中的相对坐标（0~1范围）
                rel_x = (event.x() - x) / scaled_w  # 相对宽度比例
                rel_y = (event.y() - y) / scaled_h  # 相对高度比例

                # 转换为原始图像的像素坐标（使用浮点数计算提高精度）
                original_x = int(rel_x * self.pixmap.width())
                original_y = int(rel_y * self.pixmap.height())

                # 管理标记点（最多两个点，超过则替换）
                if len(self.points) < 2:
                    self.points.append(QPoint(original_x, original_y))
                else:
                    self.points[0] = self.points[1]
                    self.points[1] = QPoint(original_x, original_y)

                self.update()  # 重绘
                self.parent.updateInfo()  # 更新信息
                self.parent.statusBar().showMessage(f'已标记点: ({original_x}, {original_y})')

    def resizeEvent(self, event):
        """窗口大小改变时重绘（确保图像和点位置同步更新）"""
        self.update()
        super(ImageViewer, self).resizeEvent(event)


class MainWindow(QMainWindow):
    # 此处代码与之前一致，省略（功能不变）
    def __init__(self):
        super(MainWindow, self).__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('图像向量标记工具')
        self.setGeometry(100, 100, 1000, 700)

        # 菜单栏
        menubar = self.menuBar()
        file_menu = menubar.addMenu('文件')

        open_action = QAction('打开图像', self)
        open_action.setShortcut('Ctrl+O')
        open_action.triggered.connect(self.openImage)
        file_menu.addAction(open_action)

        clear_action = QAction('清除标记', self)
        clear_action.setShortcut('Ctrl+C')
        clear_action.triggered.connect(self.clearMarks)
        file_menu.addAction(clear_action)

        exit_action = QAction('退出', self)
        exit_action.setShortcut('Ctrl+Q')
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # 中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        self.image_viewer = ImageViewer(self)
        main_layout.addWidget(self.image_viewer)

        # 信息栏
        info_widget = QWidget()
        info_widget.setStyleSheet("background-color: #e0e0e0; padding: 10px;")
        info_layout = QHBoxLayout(info_widget)
        info_layout.setSpacing(20)

        font = QFont()
        font.setPointSize(10)

        self.start_point_label = QLabel('初始点: 未设置')
        self.start_point_label.setFont(font)

        self.end_point_label = QLabel('终点: 未设置')
        self.end_point_label.setFont(font)

        self.vector_label = QLabel('向量: 未设置')
        self.vector_label.setFont(font)

        info_layout.addWidget(self.start_point_label)
        info_layout.addWidget(self.end_point_label)
        info_layout.addWidget(self.vector_label)

        main_layout.addWidget(info_widget)
        self.statusBar().showMessage('就绪 - 请打开图像并点击标记两个点')

    def openImage(self):
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择图像文件", "",
            "图像文件 (*.png *.jpg *.jpeg *.bmp *.gif);;所有文件 (*)",
            options=options
        )
        if file_path:
            self.image_viewer.loadImage(file_path)
            self.statusBar().showMessage(f'已加载: {os.path.basename(file_path)}')

    def clearMarks(self):
        self.image_viewer.points = []
        self.image_viewer.update()
        self.updateInfo()
        self.statusBar().showMessage('已清除标记')

    def updateInfo(self):
        points = self.image_viewer.points
        if len(points) >= 1:
            self.start_point_label.setText(f'初始点: ({points[0].x()}, {points[0].y()})')
        else:
            self.start_point_label.setText('初始点: 未设置')

        if len(points) == 2:
            self.end_point_label.setText(f'终点: ({points[1].x()}, {points[1].y()})')
            vector_x = points[1].x() - points[0].x()
            vector_y = points[1].y() - points[0].y()
            self.vector_label.setText(f'向量: ({vector_x}, {vector_y})')
        else:
            self.end_point_label.setText('终点: 未设置')
            self.vector_label.setText('向量: 未设置')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())