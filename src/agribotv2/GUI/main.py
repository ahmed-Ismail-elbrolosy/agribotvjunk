import sys
import os

# Fix Qt platform issues when launched from subprocess
os.environ['QT_QPA_PLATFORM'] = 'xcb'
os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)  # Remove if set by ROS

from PyQt5.QtWidgets import QApplication
from gui import MainWindow

def main():
    app = QApplication(sys.argv)
    
    # Create Main Window (this initializes Controller, Database, MapWidget)
    win = MainWindow()
    win.show()
    
    # Run Event Loop
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
