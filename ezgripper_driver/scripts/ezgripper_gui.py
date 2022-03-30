#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
EZGripper GUI for testing the ezgripper_driver
"""

import sys
import rospy
from ezgripper_libs.ezgripper_interface import EZGripper
from PyQt5.QtWidgets import QMainWindow, QPushButton, QApplication

rospy.init_node('ezgripper_gui_node')
gripper = EZGripper('dual_gen2_single_mount', \
   '/ezgripper_dual_gen2_single_mount/ezgripper_controller/gripper_cmd')


class GripperGUI(QMainWindow):
    """
    EZGripper GUI Class
    """

    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        """
        Initialize User Interface
        """

        calibrate_button = QPushButton("Calibrate",self)
        calibrate_button.resize(100,30)
        calibrate_button.clicked.connect(gripper.calibrate)
        calibrate_button.move(50,10)
        calibrate_button.show()

        release_button = QPushButton("Release",self)
        release_button.resize(200,200)
        release_button.clicked.connect(gripper.release)
        release_button.move(50,50)

        hard_close_button = QPushButton("Hard Close",self)
        hard_close_button.resize(200,200)
        hard_close_button.clicked.connect(self.hard_close)
        hard_close_button.move(250,50)

        open_button = QPushButton("Open", self)
        open_button.clicked.connect(self.open)
        open_button.resize(200,200)
        open_button.move(450,50)

        goto_button = QPushButton("0% Torque Mode", self)
        goto_button.resize(100,200)
        goto_button.clicked.connect(self.zero_torque_mode)
        goto_button.move(50,250)

        goto_button = QPushButton("10%", self)
        goto_button.resize(100,200)
        goto_button.clicked.connect(self.ten_percent_open)
        goto_button.move(150,250)

        goto_button = QPushButton("20%", self)
        goto_button.resize(100,200)
        goto_button.clicked.connect(self.twenty_percent_open)
        goto_button.move(250,250)

        goto_button = QPushButton("30%", self)
        goto_button.resize(100,200)
        goto_button.clicked.connect(self.thirty_percent_open)
        goto_button.move(350,250)

        goto_button = QPushButton("40%", self)
        goto_button.resize(100,200)
        goto_button.clicked.connect(self.forty_percent_open)
        goto_button.move(450,250)

        goto_button = QPushButton("50%", self)
        goto_button.resize(100,200)
        goto_button.clicked.connect(self.fifty_percent_open)
        goto_button.move(550,250)

        goto_button = QPushButton("60%", self)
        goto_button.resize(100,200)
        goto_button.clicked.connect(self.sixty_percent_open)
        goto_button.move(150,450)

        goto_button = QPushButton("70%", self)
        goto_button.resize(100,200)
        goto_button.clicked.connect(self.seventy_percent_open)
        goto_button.move(250,450)

        goto_button = QPushButton("80%", self)
        goto_button.resize(100,200)
        goto_button.clicked.connect(self.eighty_percent_open)
        goto_button.move(350,450)

        goto_button = QPushButton("90%", self)
        goto_button.resize(100,200)
        goto_button.clicked.connect(self.ninety_percent_open)
        goto_button.move(450,450)

        goto_button = QPushButton("100%", self)
        goto_button.resize(100,200)
        goto_button.clicked.connect(self.hundred_percent_open)
        goto_button.move(550,450)

        self.statusBar()

        self.setGeometry(300, 200, 800, 850)
        self.setWindowTitle("EZGripper GUI")
        self.show()

    def hard_close(self):
        """
        Hard Close the gripper
        """
        gripper.goto_position(0.0, 100.0)

    def open(self):
        """
        Open the gripper
        """
        gripper.goto_position(100.0, 100.0)

    def zero_torque_mode(self):
        """
        Enable Zero Torque Mode
        """
        gripper.goto_position(0.0, 10.0)

    def ten_percent_open(self):
        """
        10% gripper open
        """
        gripper.goto_position(10.0, 100.0)

    def twenty_percent_open(self):
        """
        20% gripper open
        """
        gripper.goto_position(20.0, 100.0)

    def thirty_percent_open(self):
        """
        30% gripper open
        """
        gripper.goto_position(30.0, 100.0)

    def forty_percent_open(self):
        """
        40% gripper open
        """
        gripper.goto_position(40.0, 100.0)

    def fifty_percent_open(self):
        """
        50% gripper open
        """
        gripper.goto_position(50.0, 100.0)

    def sixty_percent_open(self):
        """
        60% gripper open
        """
        gripper.goto_position(60.0, 100.0)

    def seventy_percent_open(self):
        """
        70% gripper open
        """
        gripper.goto_position(70.0, 100.0)

    def eighty_percent_open(self):
        """
        80% gripper open
        """
        gripper.goto_position(80.0, 100.0)

    def ninety_percent_open(self):
        """
        90% gripper open
        """
        gripper.goto_position(90.0, 100.0)

    def hundred_percent_open(self):
        """
        100% gripper open
        """
        gripper.goto_position(100.0, 100.0)


def main():
    """
    Main Function
    """
    ezgripper_app = QApplication(sys.argv)
    ex = GripperGUI()
    sys.exit(ezgripper_app.exec_())

if __name__== '__main__':
    main()
