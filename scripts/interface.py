#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu
from PyQt5 import QtWidgets, QtGui, QtCore

class SensorDataNode(QtWidgets.QWidget):
    def __init__(self):
        super(SensorDataNode, self).__init__()

        rospy.init_node('sensor_data_gui', anonymous=True)
        
        # GUI Layout
        self.layout = QtWidgets.QVBoxLayout()

        # Create widgets
        self.odom_label = QtWidgets.QLabel("Odometry: ")
        self.imu_label = QtWidgets.QLabel("IMU: ")
        self.scan_label = QtWidgets.QLabel("Laser Scan: ")
        
        self.map_view = QtWidgets.QLabel("Map View: ")
        self.map_view.setPixmap(QtGui.QPixmap(500, 500))  # Placeholder for map visualization

        # Create a layout for the map view
        self.map_layout = QtWidgets.QVBoxLayout()
        self.map_layout.addWidget(self.map_view)

        # Create a layout for sensor data
        self.data_layout = QtWidgets.QVBoxLayout()
        self.data_layout.addWidget(self.odom_label)
        self.data_layout.addWidget(self.imu_label)
        self.data_layout.addWidget(self.scan_label)

        # Add layouts to main layout
        self.layout.addLayout(self.data_layout)
        self.layout.addLayout(self.map_layout)

        self.setLayout(self.layout)
        
        # ROS Subscribers
        rospy.Subscriber("/odom", Odometry, self.update_odom)
        rospy.Subscriber("/imu_data", Imu, self.update_imu)
        rospy.Subscriber("/scan", LaserScan, self.update_scan)
        rospy.Subscriber("/map", OccupancyGrid, self.update_map)  # Example for map

    def update_odom(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.odom_label.setText("Odometry: Position ({:.2f}, {:.2f}, {:.2f}), Orientation ({:.2f}, {:.2f}, {:.2f}, {:.2f})".format(
    position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w))

    def update_imu(self, msg):
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        self.imu_label.setText("IMU: Orientation ({:.2f}, {:.2f}, {:.2f}, {:.2f}), Angular Velocity ({:.2f}, {:.2f}, {:.2f}), Linear Acceleration ({:.2f}, {:.2f}, {:.2f})".format(
    orientation.x, orientation.y, orientation.z, orientation.w,
    angular_velocity.x, angular_velocity.y, angular_velocity.z,
    linear_acceleration.x, linear_acceleration.y, linear_acceleration.z))


    def update_scan(self, msg):
          self.scan_label.setText("Scan: Min Range: {:.2f} m, Max Range: {:.2f} m, Angle Min: {:.2f} rad, Angle Max: {:.2f} rad".format(
            min(msg.ranges), max(msg.ranges), msg.angle_min, msg.angle_max))



    def update_map(self, msg):
        # Assuming you have a function to convert OccupancyGrid to QPixmap
        map_pixmap = self.convert_map_to_pixmap(msg)
        self.map_view.setPixmap(map_pixmap)

    def convert_map_to_pixmap(self, msg):
        # Placeholder function to convert OccupancyGrid to QPixmap
        # Implement map conversion logic here
        return QtGui.QPixmap(500, 500)  # Placeholder image

    def closeEvent(self, event):
        rospy.signal_shutdown("GUI closed")
        event.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = SensorDataNode()
    window.setWindowTitle('Sensor Data Display')
    window.show()
    sys.exit(app.exec_())
