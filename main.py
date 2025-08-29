import sys
from PyQt5.QtWidgets import QApplication
from ui.main_window import RobotControlWindow

def main():
    """
    程序主入口。
    创建 QApplication 实例，然后创建并显示主窗口，最后启动事件循环。
    """
    app = QApplication(sys.argv)
    window = RobotControlWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()