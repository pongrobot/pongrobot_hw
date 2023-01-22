import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from enum import Enum
import math
import serial
import pyvesc
import sys

class CommandMode(Enum):
    NO_COMMAND=0
    DUTY_CYCLE_COMMAND=1
    RPM_COMMAND=2

class VescHandler:

    def __init__(self, port1, port2):
        self.load_config()

        # ROS data
        self.duty_cycle_sub = rospy.Subscriber("duty_cycle_cmd", Float32, self.duty_cycle_callback)
        self.rpm_sub = rospy.Subscriber("rpm_cmd", Float32, self.rpm_callback)
        self.vel_sub = rospy.Subscriber("velocity_cmd", Float32, self.vel_callback)
        self.trigger_sub = rospy.Subscriber("/hw/trigger", Empty, self.trigger_callback)
        self.ready_pub = rospy.Publisher("vesc_ready", Bool, queue_size=10)

        # Port data
        self.port1_name = port1
        self.port2_name = port2
        self.port1_open = False
        self.port2_open = False
        self.open_port1()
        self.open_port2()

        # Command Data
        self.command_mode = CommandMode.NO_COMMAND
        self.last_command_time = rospy.get_rostime()

        # Motor data
        self.at_setpoint = False
        self.cooling_down = False
        self.trigger_time = rospy.get_rostime()

        # RPM interface
        self.target_rpm = 0
        self.rpm_cmd = 0
        self.initial_rpm = 0

        # Duty cycle interface
        self.target_duty_cycle = 0
        self.duty_cycle_cmd = 0
        self.initial_duty_cycle = 0

    def load_config(self):
        # VESC PARAMS
        self.rate = rospy.Rate( rospy.get_param("rate") ) 
        self.ramp_time = rospy.get_param("ramp_time") # time it takes for vesc to get up to speed (sec)
        self.cooldown_time = rospy.get_param("cooldown_time") # time to wait after trigger to shut down motor (sec)
        self.command_timeout = rospy.get_param("command_timeout")
    
        # Geometry params
        self.WHEEL_RADIUS = rospy.get_param("geometry/wheel_radius") # radius of launcher wheel in meters
        self.NUM_MOTOR_POLES = rospy.get_param("geometry/num_motor_poles")

        # Limits params
        self.limits.RPM_ACCEL = rospy.get_param("limits/rpm_accel") # rpm/sec
        self.limits.DUTY_CYCLE_ACCEL = rospy.get_param("limits/duty_cycle_accel") # %/sec
        self.limits.MAX_RPM = rospy.get_param("limits/max_rpm")

        # Calibration params
        self.calibration.rpm_m = rospy.get_param("calibration/rpm_slope")
        self.calibration.rpm_b = rospy.get_param("calibration/rpm_offset")
        self.calibration.fudge_weight = rospy.get_param("calibration/fudge_weight")
        self.calibration.fudge_weight = rospy.get_param("calibration/fudge_bias") 

    def duty_cycle_callback(self, msg):
        if msg.data > 100:
            self.target_duty_cycle = 100
        else:
            self.target_duty_cycle = msg.data

        self.initial_duty_cycle = self.duty_cycle_cmd
        self.command_mode = CommandMode.DUTY_CYCLE_COMMAND
        self.last_command_time = rospy.get_rostime() 
        self.cooling_down = False # un-schedule cooldown condition
        self.target_rpm = 0
        rospy.loginfo('Received DUTY_CYCLE_COMMAND: ' + str(self.duty_cycle) + '%')

    def rpm_callback(self, msg):
        if msg.data > self.limits.MAX_RPM:
            self.target_rpm = self.limits.MAX_RPM
        else:
            self.target_rpm = msg.data

        self.target_rpm = msg.data 
        self.initial_rpm = self.rpm_cmd
        self.command_mode = CommandMode.RPM_COMMAND
        self.last_command_time = rospy.get_rostime() 
        self.cooling_down = False # un-schedule cooldown condition
        self.duty_cycle = 0
        rospy.loginfo('Received RPM_COMMAND: ' + str(self.target_rpm) + ' rpm')

    def vel_callback(self, msg):
        self.target_rpm = self.velocity_to_rpm(msg.data)
        self.initial_rpm = self.rpm_cmd
        self.command_mode = CommandMode.RPM_COMMAND
        self.last_command_time = rospy.get_rostime() 
        self.cooling_down = False # un-schedule cooldown condition
        self.duty_cycle = 0
        rospy.loginfo('Received VELOCITY_COMMAND: ' + "{:.4f}".format(msg.data) + ' m/s')

    def trigger_callback(self, msg):
        if self.command_mode != CommandMode.NO_COMMAND and not self.cooling_down:
            self.cooling_down = True
            self.trigger_time = rospy.get_rostime() 
            self.last_command_time = rospy.get_rostime() # extend command timeout
            rospy.logdebug('Recieved trigger signal, starting cooldown timer')

    def open_port1(self):
        try:
            self.port1 = serial.Serial(self.port1_name)
            self.port1_open = True
        except:
            rospy.logerr('Unable to open port1:' + self.port1_name) 
            self.port1_open = False

    def open_port2(self):
        try:
            self.port2 = serial.Serial(self.port2_name)
            self.port2_open = True
        except:
            rospy.logerr('Unable to open port2:' + self.port2_name) 
            self.port2_open = False

    def send_duty_cycle_command(self):
        # TODO: Apply decceleration curve
        if self.port1_open and self.port2_open:
            if self.duty_cycle_cmd < self.target_duty_cycle:
                self.duty_cycle_cmd = self.initial_duty_cycle + (rospy.get_rostime() - self.last_command_time).to_sec() * self.limits.DUTY_CYCLE_ACCEL
            else:
                self.duty_cycle_cmd = self.target_duty_cycle

            rospy.logdebug('Sending DUTY_CYCLE_COMMAND = ' + str(self.duty_cycle_cmd))
            self.port1.write( pyvesc.encode( pyvesc.SetDutyCycle( int((self.duty_cycle_cmd) * 1000) )) )
            self.port2.write( pyvesc.encode( pyvesc.SetDutyCycle( int((self.duty_cycle_cmd) * 1000) )) )

    def send_rpm_command(self):

        self.calibration.rpm_m = rospy.get_param("calibration/rpm_m")
        self.calibration.rpm_b = rospy.get_param("calibration/rpm_b")
        self.calibration.fudge_m = rospy.get_param("calibration/fudge_m")
        self.calibration.fudge_b = rospy.get_param("calibration/fudge_b")

        if self.port1_open and self.port2_open:
            if self.rpm_cmd < self.target_rpm:
                self.rpm_cmd = self.initial_rpm + (rospy.get_rostime() - self.last_command_time).to_sec() * self.limits.RPM_ACCEL
            else:
                self.rpm_cmd = self.target_rpm
            
            # Apply linear RPM calibration
            m = self.calibration.rpm_m * self.calibration.fudge_m
            b = self.calibration.rpm_b + self.calibration.fudge_b
            actual_cmd = self.rpm_cmd*m + b

            rospy.logdebug('Sending RPM_COMMAND = ' + str(self.rpm_cmd))
            self.port1.write( pyvesc.encode( pyvesc.SetRPM( int(actual_cmd * self.NUM_MOTOR_POLES)) ) )
            self.port2.write( pyvesc.encode( pyvesc.SetRPM( int(actual_cmd * self.NUM_MOTOR_POLES)) ) )

    def velocity_to_rpm(self, v):
        # TODO: Calibrate experimentally and add a calibration function
        rpm = v * 30.0/(self.WHEEL_RADIUS * math.pi)

        # clamp value to MAX_RPM
        if rpm > self.limits.MAX_RPM:
            rpm = self.limits.MAX_RPM

        return rpm 

    def run(self):
        while not rospy.is_shutdown():
            # If the serial port is not open, attempt to reconnect
            if not self.port1_open:
                self.open_port1()

            if not self.port2_open:
                self.open_port2()

            # Check if we have a valid command
            if self.command_mode != CommandMode.NO_COMMAND:
                if (rospy.get_rostime() - self.last_command_time).to_sec() > self.command_timeout:
                    # Command has timed out, shut down motor
                    self.at_setpoint = False
                    self.command_mode = CommandMode.NO_COMMAND
                    self.target_rpm = 0
                    self.rpm_cmd = 0

                elif self.cooling_down and (rospy.get_rostime() - self.trigger_time).to_sec() > self.cooldown_time:
                    # ball has been launched, shut down motor
                    self.cooling_down = False
                    self.at_setpoint = False
                    self.command_mode = CommandMode.NO_COMMAND
                    self.target_rpm = 0
                    self.rpm_cmd = 0
                    rospy.logdebug('Cooled down after trigger')

                else:
                    self.at_setpoint = (rospy.get_rostime() - self.last_command_time).to_sec() > self.ramp_time and ( self.target_rpm == self.rpm_cmd )
                    if self.command_mode == CommandMode.DUTY_CYCLE_COMMAND:
                        self.send_duty_cycle_command()
                    elif self.command_mode == CommandMode.RPM_COMMAND:
                        self.send_rpm_command()

            # Report status 
            self.ready_pub.publish(Bool(self.at_setpoint))
            self.rate.sleep()

if __name__ =='__main__':
    rospy.init_node('vesc_node')
    myargv = rospy.myargv(argv=sys.argv)

    if len(myargv) >= 2:
        vesc_handler = VescHandler(sys.argv[1], sys.argv[2])
        vesc_handler.run()
    else:
        print("Invalid number of args")