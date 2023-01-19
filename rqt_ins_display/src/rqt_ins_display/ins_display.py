import os
import rospy
import rospkg
import datetime

from inertiallabs_msgs.msg import ins_data, sensor_data, gps_data
 
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot, QTimer
from python_qt_binding.QtWidgets import QWidget, QScrollArea, QVBoxLayout
 
class InsDisplay(Plugin):

    def __init__(self, context):
        super(InsDisplay, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('InsDisplay')
        self.ins_data = None
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        #args, unknowns = parser.parse_known_args(context.argv())
        # if not args.quiet:
        #    print 'arguments: ', args
        #    print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_ins_display'), 'resource', 'InsDisplay.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('InsDisplayUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.

        self._widget.message.setText("Hello ROS!")
        self._widget.message.adjustSize()
        self._widget.message2.setText("Hello ROS!")
        self._widget.message2.adjustSize()

        rospy.Subscriber("/Inertial_Labs/sensor_data", sensor_data, self.get_sensor_data)
        rospy.Subscriber("/Inertial_Labs/ins_data", ins_data, self.get_ins_data)
        rospy.Subscriber("/Inertial_Labs/gps_data", gps_data, self.get_gps_data)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        timer = QTimer(self)
        timer.timeout.connect(self.update_msg)
        timer.start(20)  # every 20 milliseconds
        
        
        # Add widget to the user interface
        context.add_widget(self._widget)

    @Slot()
    def update_msg(self):
        # lets get INS ROS Data:
        # print("YAS!")
        
        #current_time = str(datetime.datetime.now().time())
        #current_time = str(datetime.datetime.fromtimestamp(rospy.get_time()))
        if self.ins_data != None:
            current_time = str(datetime.datetime.fromtimestamp(self.sensor_data.header.stamp.secs))
            Latitude = str(self.ins_data.LLH.x)
            Longitude = str(self.ins_data.LLH.y)
            Altitude = str(self.ins_data.LLH.z)
            Vol_E = str(self.ins_data.Vel_ENU.x)
            Vol_N = str(self.ins_data.Vel_ENU.y)
            Vol_U = str(self.ins_data.Vel_ENU.z)
            Accel_X = str(self.sensor_data.Accel.x)
            Accel_Y = str(self.sensor_data.Accel.y)
            Accel_Z = str(self.sensor_data.Accel.z)
            Temperature = str(round(self.sensor_data.Temp,3))
            sensor_msg = str(self.sensor_data)
            gps_msg = str(self.gps_data)
            ins_msg = str(self.ins_data)
        else:
            current_time = "Time not found!"
            Latitude = "Latitude not found!"
            Longitude = "Longitud not found!"
            Altitude = "Altitude not found!"
            Vol_E = "Velocity not found!"
            Vol_N = "Velocity not found!"
            Vol_U = "Velocity not found!"
            Accel_X = "Acceleration not found!"
            Accel_Y = "Acceleration not found!"
            Accel_Z = "Acceleration not found!"
            Temperature = "Temperature not found!"
            sensor_msg = "Sensor data not found!"
            gps_msg = "GPS data not found!"
            ins_msg = "INS data not found!"
        
        display_msg = "Date: " + current_time + "\n\n" + "Location: \n   Latitude: " + Latitude + "\n   Longitude: " + Longitude + "\n   Altitude: " + Altitude + "\n\nTemperature: " + Temperature 
        display_msg2 = "Velocity: \n   East: " + Vol_E + "\n   North: " + Vol_N + "\n   Up: " + Vol_U + "\n\nAcceleration: \n   X: " + Accel_X + "\n   Y: " + Accel_Y + "\n   Z: " + Accel_Z
        self._widget.message.setText(display_msg)
        self._widget.message.adjustSize()
        self._widget.message2.setText(display_msg2)
        self._widget.message2.adjustSize()
        self._widget.ins_label.setText(ins_msg)
        self._widget.ins_label.adjustSize()
        self._widget.gps_label.setText(gps_msg)
        self._widget.gps_label.adjustSize()
        self._widget.sensor_label.setText(sensor_msg)
        self._widget.sensor_label.adjustSize()

    def get_sensor_data(self,data):
        self.sensor_data = data

    def get_ins_data(self,data):
        self.ins_data = data

    def get_gps_data(self,data):
        self.gps_data = data
    
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass
 
    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass
 
    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
 
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
