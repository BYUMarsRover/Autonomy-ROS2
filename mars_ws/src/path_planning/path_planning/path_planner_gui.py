import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, 
                           QLineEdit, QPushButton, QLabel)
from PyQt5.QtCore import Qt

class PathPlannerGUI(Node):
    def __init__(self):
        # Initialize ROS2 node
        super().__init__('path_planner_gui')

        self.start = None
        self.goal = None

        ################# ROS Communication #################

        # Publishers
        self.mapviz_path = self.create_publisher(Path, '/mapviz/path', 10)

        # Subscribers

        # Services

        # Clients
        self.plan_path_client = self.create_client(PlanPath, 'plan_path')

        ################# GUI Creation #################
        
        # Create Qt application and main window
        self.app = QApplication([])
        self.window = QWidget()
        self.window.setWindowTitle('Path Planner GUI')
        
        # Create main layout
        layout = QVBoxLayout()
        
        # Create input fields (red in template)
        self.latitude_input = QLineEdit()
        self.latitude_input.setPlaceholderText('Enter Latitude')
        layout.addWidget(self.latitude_input)
        
        self.longitude_input = QLineEdit()
        self.longitude_input.setPlaceholderText('Enter Longitude')
        layout.addWidget(self.longitude_input)
        
        # Create error message label (blue in template)
        self.error_label = QLabel()
        self.error_label.setStyleSheet('color: red;')
        self.error_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.error_label)
        
        # Create buttons
        buttons = [
            ('Update Start', self.update_start),
            ('Update Goal', self.update_goal),
            ('Plan Path', self.plan_path),
            ('Clear Mapviz', self.clear_mapviz)
        ]
        
        for button_text, callback in buttons:
            button = QPushButton(button_text)
            button.clicked.connect(callback)
            layout.addWidget(button)
        
        # Set layout and display window
        self.window.setLayout(layout)
        self.window.show()
        
        # Start Qt event loop
        self.app.exec_()

    # Callback functions for buttons
    def update_start(self):
        try:
            lat = float(self.latitude_input.text())
            lon = float(self.longitude_input.text())
            self.error_label.setText('Start position updated')
        except ValueError:
            self.error_label.setText('Invalid coordinates')

    def update_goal(self):
        try:
            lat = float(self.latitude_input.text())
            lon = float(self.longitude_input.text())
            self.error_label.setText('Goal position updated')
        except ValueError:
            self.error_label.setText('Invalid coordinates')

    def plan_path(self):
        self.error_label.setText('Planning path...')
        # Add path planning logic here
        

    def clear_mapviz(self):
        self.error_label.setText('Mapviz cleared')
        # Add mapviz clearing logic here

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create and run GUI
    gui = PathPlannerGUI()
    
    # Spin ROS2 node
    rclpy.spin(gui)
    
    # Cleanup
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()