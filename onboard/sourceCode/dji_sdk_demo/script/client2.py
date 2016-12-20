#!/usr/bin/env python
import rospy
import std_msgs.msg 
import dji_sdk.msg 
import nav_msgs.msg 
import dji_sdk.srv 
import math
import time
import actionlib
import roslib

class DJIDrone:
    HORIZ_ATT  = 0x00
    HORIZ_VEL  = 0x40
    HORIZ_POS  = 0x80
    VERT_VEL   = 0x00
    VERT_POS   = 0x10
    VERT_TRU   = 0x20
    YAW_ANG    = 0x00
    YAW_RATE   = 0x08
    HORIZ_GND  = 0x01
    HORIZ_BODY = 0x01
    STABLE_OFF = 0x00
    STABLE_ON  = 0x01

    def acceleration_subscriber_callback(self, acceleration):
        self.acceleration = acceleration

    def attitude_quaternion_subscriber_callback(self, attitude_quaternion):
        self.attitude_quaternion = attitude_quaternion

    def compass_subscriber_callback(self, compass):
        self.compass = compass

    def flight_control_info_subscriber_callback(self, flight_control_info):
        self.flight_control_info = flight_control_info

    def flight_status_subscriber_callback(self, flight_status):
        self.flight_status = flight_status

    def gimbal_subscriber_callback(self, gimbal):
        self.gimbal = gimbal

    def global_position_subscriber_callback(self, global_position):
        self.global_position = global_position

    def local_position_subscriber_callback(self, local_position):
        self.local_position = local_position

    def power_status_subscriber_callback(self, power_status):
        self.power_status = power_status

    def rc_channels_subscriber_callback(self, rc_channels):
        self.rc_channels = rc_channels

    def velocity_subscriber_callback(self, velocity):
        self.velocity = velocity

    def activation_subscriber_callback(self, activation):
        self.activation = activation

    def odometry_subscriber_callback(self, odometry):
        self.odometry = odometry

    def time_stamp_subscriber_callback(self, time_stamp):
        self.time_stamp = time_stamp

    def init_subscribers(self):
        self.acceleration_subscriber = rospy.Subscriber("dji_sdk/acceleration", dji_sdk.msg.Acceleration, self.acceleration_subscriber_callback)
        self.attitude_quaternion_subscriber = rospy.Subscriber("dji_sdk/attitude_quaternion", dji_sdk.msg.AttitudeQuaternion, self.attitude_quaternion_subscriber_callback)
        self.compass_subscriber = rospy.Subscriber("dji_sdk/compass", dji_sdk.msg.Compass, self.compass_subscriber_callback)
        self.flight_control_info_subscriber = rospy.Subscriber("dji_sdk/flight_control_info", dji_sdk.msg.FlightControlInfo, self.flight_control_info_subscriber_callback)
        self.flight_status_subscriber = rospy.Subscriber("dji_sdk/flight_status", std_msgs.msg.UInt8, self.flight_status_subscriber_callback)
        self.gimbal_subscriber = rospy.Subscriber("dji_sdk/gimbal", dji_sdk.msg.Gimbal, self.gimbal_subscriber_callback)
        self.global_position_subscriber = rospy.Subscriber("dji_sdk/global_position", dji_sdk.msg.GlobalPosition, self.global_position_subscriber_callback)
        self.local_position_subscriber = rospy.Subscriber("dji_sdk/local_position", dji_sdk.msg.LocalPosition, self.local_position_subscriber_callback)
        self.power_status_subscriber = rospy.Subscriber("dji_sdk/power_status", dji_sdk.msg.PowerStatus, self.power_status_subscriber_callback)
        self.rc_channels_subscriber = rospy.Subscriber("dji_sdk/rc_channels", dji_sdk.msg.RCChannels, self.rc_channels_subscriber_callback)
        self.velocity_subscriber = rospy.Subscriber("dji_sdk/velocity", dji_sdk.msg.Velocity, self.velocity_subscriber_callback)
        self.activation_subscriber = rospy.Subscriber("dji_sdk/activation", std_msgs.msg.UInt8, self.activation_subscriber_callback)
        self.odometry_subscriber = rospy.Subscriber("dji_sdk/odometry", nav_msgs.msg.Odometry, self.odometry_subscriber_callback)
        self.time_stamp_subscriber = rospy.Subscriber("dji_sdk/time_stamp", dji_sdk.msg.TimeStamp, self.time_stamp_subscriber_callback)

    def init_services(self):
        rospy.wait_for_service("dji_sdk/attitude_control")
        rospy.wait_for_service("dji_sdk/camera_action_control")
        rospy.wait_for_service("dji_sdk/drone_task_control")
        rospy.wait_for_service("dji_sdk/gimbal_angle_control")
        rospy.wait_for_service("dji_sdk/gimbal_speed_control")
        rospy.wait_for_service("dji_sdk/global_position_control")
        rospy.wait_for_service("dji_sdk/local_position_control")
        rospy.wait_for_service("dji_sdk/sdk_permission_control")
        rospy.wait_for_service("dji_sdk/velocity_control")
        rospy.wait_for_service("dji_sdk/drone_arm_control")
        rospy.wait_for_service("dji_sdk/virtual_rc_enable_control")
        rospy.wait_for_service("dji_sdk/virtual_rc_data_control")
        rospy.wait_for_service("dji_sdk/sync_flag_control")

        self.attitude_control_service = rospy.ServiceProxy("dji_sdk/attitude_control", dji_sdk.srv.AttitudeControl)
        self.camera_action_control_service = rospy.ServiceProxy("dji_sdk/camera_action_control", dji_sdk.srv.CameraActionControl)
        self.drone_task_control_service = rospy.ServiceProxy("dji_sdk/drone_task_control", dji_sdk.srv.DroneTaskControl)
        self.gimbal_angle_control_service = rospy.ServiceProxy("dji_sdk/gimbal_angle_control", dji_sdk.srv.GimbalAngleControl)
        self.gimbal_speed_control_service = rospy.ServiceProxy("dji_sdk/gimbal_speed_control", dji_sdk.srv.GimbalSpeedControl)
        self.global_position_control_service = rospy.ServiceProxy("dji_sdk/global_position_control", dji_sdk.srv.GlobalPositionControl)
        self.local_position_control_service = rospy.ServiceProxy("dji_sdk/local_position_control", dji_sdk.srv.LocalPositionControl)
        self.sdk_permission_control_service = rospy.ServiceProxy("dji_sdk/sdk_permission_control", dji_sdk.srv.SDKPermissionControl)
        self.velocity_control_service = rospy.ServiceProxy("dji_sdk/velocity_control", dji_sdk.srv.VelocityControl)
        self.drone_arm_service = rospy.ServiceProxy("dji_sdk/drone_arm_control", dji_sdk.srv.DroneArmControl)
        self.drone_vrc_enable_service = rospy.ServiceProxy("dji_sdk/virtual_rc_enable_control", dji_sdk.srv.VirtualRCEnableControl)
        self.drone_vrc_data_service = rospy.ServiceProxy("dji_sdk/virtual_rc_data_control", dji_sdk.srv.VirtualRCDataControl)
        self.sync_timestamp_service = rospy.ServiceProxy("dji_sdk/sync_flag_control", dji_sdk.srv.SyncFlagControl)

    def init_actions(self):
        self.local_position_navigation_action_client = actionlib.SimpleActionClient("dji_sdk/local_position_navigation_action", dji_sdk.msg.LocalPositionNavigationAction)
        self.local_position_navigation_action_client.wait_for_server()
        self.global_position_navigation_action_client = actionlib.SimpleActionClient("dji_sdk/global_position_navigation_action", dji_sdk.msg.GlobalPositionNavigationAction)
        self.global_position_navigation_action_client.wait_for_server()
        self.waypoint_navigation_action_client = actionlib.SimpleActionClient("dji_sdk/waypoint_navigation_action", dji_sdk.msg.WaypointNavigationAction)
        self.waypoint_navigation_action_client.wait_for_server()

    def local_position_navigation_send_request(self, x, y, z):
        goal = dji_sdk.msg.LocalPositionNavigationGoal(x = x, y = y, z = z)
        self.local_position_navigation_action_client.send_goal(goal)

    def global_position_navigation_send_request(self, latitude, longitude, altitude):
        goal = dji_sdk.msg.GlobalPositionNavigationGoal(latitude = latitude, longitude = longitude, altitude = altitude)
        self.global_position_navigation_action_client.send_goal(goal)

    def waypoint_navigation_send_request(self, waypoint_list):
        goal = dji_sdk.msg.WaypointNavigationGoal(waypoint_list=dji_sdk.msg.WaypointList(waypoint_list=waypoint_list))
        self.waypoint_navigation_action_client.send_goal(goal)

    def arm_drone(self):
        self.drone_arm_service(arm=1)

    def disarm_drone(self):
        self.drone_arm_service(arm=0)

    def takeoff(self):
        self.drone_task_control_service(task=4)

    def landing(self):
        self.drone_task_control_service(task=6)

    def gohome(self):
        self.drone_task_control_service(task=1)

    def take_picture(self):
        self.camera_action_control_service(camera_action=0)

    def vrc_start(self):
        self.drone_vrc_enable_service(enable=1, if_back_to_real=1)

    def vrc_stop(self):
        self.drone_vrc_enable_service(enable=0, if_back_to_real=1)

    def vrc_control(self, roll=1024, pitch=1024, throttle=1024, yaw=1024, gear=1324, mode=496):
        self.drone_vrc_data_service(channel_data = [roll, pitch, throttle, yaw, gear, 0, mode, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    def sync_timestamp(self, frequency=0):
        self.sync_timestamp_service(frequency=frequency)

    def start_video(self):
        self.camera_action_control_service(camera_action=1)

    def stop_video(self):
        self.camera_action_control_service(camera_action=2)

    def gimbal_angle_control(self, yaw = 0, roll = 0, pitch = 0, duration = 0, absolute_or_incremental = True, yaw_cmd_ignore = False, roll_cmd_ignore = False, pitch_cmd_ignore = False):
        self.gimbal_angle_control_service(yaw = yaw, roll = roll, pitch = pitch, duration = duration,
            absolute_or_incremental = absolute_or_incremental, yaw_cmd_ignore = yaw_cmd_ignore, roll_cmd_ignore = roll_cmd_ignore, pitch_cmd_ignore = pitch_cmd_ignore)

    def gimbal_speed_control(self, yaw_rate = 0, roll_rate = 0, pitch_rate = 0):
        self.gimbal_speed_control_service(yaw_rate = yaw_rate, roll_rate = roll_rate, pitch_rate = pitch_rate)

    def request_sdk_permission_control(self):
        self.sdk_permission_control_service(control_enable = 1)

    def release_sdk_permission_control(self):
        self.sdk_permission_control_service(control_enable = 0)

    def attitude_control(self, flag, x, y, z, yaw):
        self.attitude_control_service(flag = flag, x = x, y = y, z = z, yaw = yaw)

    def velocity_control(self, frame, vx, vy, vz, yawRate):
        self.velocity_control_service(frame = frame, vx = vx, vy = vy, vz = vz, yawRate = yawRate)

    def local_position_control(self, x, y, z, yaw):
        self.local_position_control_service(x = x, y = y, z = z, yaw = yaw)

    def global_position_control(self, latitude, longitude, altitude, yaw):
        self.global_position_control_service(latitude = latitude, longitude = longitude, altitude = altitude, yaw = yaw)

    def lookat(self, x, y, z, duration):
        x, y, z
        self.local_position

        self.gimbal_angle_control_service(flag = flag, yaw = yaw, roll = 0, pitch = 0, duration = duration)
        self.local_position_control_service(x = x, y = y, z = z, yaw = yaw)

    def __init__(self, namespace=''):
        rospy.init_node('dji_sdk_connector')
        self.namespace = namespace

        self.acceleration = dji_sdk.msg.Acceleration()
        self.attitude_quaternion = dji_sdk.msg.AttitudeQuaternion()
        self.compass = dji_sdk.msg.Compass()
        self.flight_control_info = dji_sdk.msg.FlightControlInfo()
        self.flight_status = std_msgs.msg.UInt8()
        self.gimbal = dji_sdk.msg.Gimbal()
        self.global_position = dji_sdk.msg.GlobalPosition()
        self.global_position_ref = dji_sdk.msg.GlobalPosition()
        self.local_position = dji_sdk.msg.LocalPosition()
        self.local_position_ref = dji_sdk.msg.LocalPosition()
        self.power_status = dji_sdk.msg.PowerStatus()
        self.rc_channels = dji_sdk.msg.RCChannels() 
        self.time_stamp = dji_sdk.msg.TimeStamp()
        self.velocity = dji_sdk.msg.Velocity()
        self.odometry = nav_msgs.msg.Odometry()

        self.activation = False
        self.localposbase_use_height = True

        self.init_actions()
        self.init_subscribers()
        self.init_services()


import dji_sdk.msg
import time
import sys
import math

def display_main_menu():
    print "----------- < Main menu > ----------"
    print "[a] Request to obtain control"
    print "[b] Release control"
    print "[c] Takeoff"
    print "[d] Landing"
    print "[e] Go home"
    print "[f] Gimbal control sample"
    print "[g] Attitude control sample"
    print "[h] Draw circle sample"
    print "[i] Draw square sample"
    print "[j] Take a picture"
    print "[k] Start video"
    print "[l] Stop video"
    print "[m] Local Navi Test"
    print "[n] GPS Navi Test"
    print "[o] Waypoint List Test"
    print "[p] Arm Test"
    print "[q] Disarm Test"
    print "[r] Vrc Test"
    print "[s] Sync Test"
    print "[t] Exit"
    print "[y] Forward 1m body frame"
    print "[w] Backward 1m body frame"
    print "[x] Left 1m body frame"
    print "[z] Right 1m body frame"
    print "[0] Rotate 90 degrees"
    print "\ninput a/b/c etc..then press enter key"
    print "\nuse `rostopic echo` to query drone status"
    print "----------------------------------------"
    print "input: "

def main():
    currentRotation = 0
    distance = 0.5
    drone = DJIDrone()
    display_main_menu()
    while True:
        main_operate_code = sys.stdin.read(1)
        if main_operate_code == 'a':
            drone.request_sdk_permission_control()
        elif main_operate_code == 'b':
            drone.release_sdk_permission_control()
        elif main_operate_code == 'c':
            drone.takeoff()
        elif main_operate_code == 'd':
            drone.landing()
        elif main_operate_code =='e':
            drone.gohome()
        elif main_operate_code == 'f':
            drone.gimbal_angle_control(0, 0, 0, 20)
            time.sleep(2)
            drone.gimbal_angle_control(0, 0, 1800, 20)
            time.sleep(2)
            drone.gimbal_angle_control(0, 0, -1800, 20)
            time.sleep(2)
            drone.gimbal_angle_control(300, 0, 0, 20)
            time.sleep(2)
            drone.gimbal_angle_control(-300, 0, 0, 20)
            time.sleep(2)
            drone.gimbal_angle_control(0, 300, 0, 20)
            time.sleep(2)
            drone.gimbal_angle_control(0, -300, 0, 20)
            time.sleep(2)
            drone.gimbal_speed_control(100, 0, 0)
            time.sleep(2)
            drone.gimbal_speed_control(-100, 0, 0)
            time.sleep(2)
            drone.gimbal_speed_control(0, 0, 200)
            time.sleep(2)
            drone.gimbal_speed_control(0, 0, -200)
            time.sleep(2)
            drone.gimbal_speed_control(0, 200, 0)
            time.sleep(2)
            drone.gimbal_speed_control(0, -200, 0)
            time.sleep(2)
            drone.gimbal_angle_control(0, 0, 0, 20)
        elif main_operate_code == 'g':
            drone.takeoff()
            time.sleep(5)

            for i in range(100):
                if i < 90:
                    drone.attitude_control(0x40, 0, 2, 0, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 2, 0, 0, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)


            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, -2, 0, 0, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 0, 2, 0, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 0, -2, 0, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 0, 0, 0.5, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 0, 0, -0.5, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 0, 0, 0, 90)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 0, 0, 0, -90)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            drone.landing()

        elif main_operate_code == 'h':
            R = 2
            V = 2
            # start to draw circle
            for i in range(300):
                vx = V * math.sin((V/R)*i/50.0)
                vy = V * math.cos((V/R)*i/50.0)

                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, vx, vy, 0, 0)
                time.sleep(0.02)
        elif main_operate_code == 'i':
            # draw square sample
            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, 3, 3, 0, 0)
                time.sleep(0.02)
            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, -3, 3, 0, 0)
                time.sleep(0.02)
            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, -3, -3, 0, 0)
                time.sleep(0.02)
            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, 3, -3, 0, 0)
                time.sleep(0.02)
        elif main_operate_code == 'j':
            # take a picture
            drone.take_picture()
        elif main_operate_code == 'k':
            # start video
            drone.start_video()
        elif main_operate_code == 'l':
            # stop video
            drone.stop_video()
        elif main_operate_code == 'm':
            # Local Navi Test
            drone.local_position_navigation_send_request(-10, -10, 2)
        elif main_operate_code == 'n':
            # GPS Navi Test
            drone.global_position_navigation_send_request(22.535, 113.95, 100)
        elif main_operate_code == 'o':
            # Waypoint List Navi Test
            newWaypointList = [
                dji_sdk.msg.Waypoint(latitude = 22.535, longitude = 113.95, altitude = 100, staytime = 5, heading = 0),
                dji_sdk.msg.Waypoint(latitude = 22.535, longitude = 113.96, altitude = 100, staytime = 0, heading = 90),
                dji_sdk.msg.Waypoint(latitude = 22.545, longitude = 113.96, altitude = 100, staytime = 4, heading = -90),
                dji_sdk.msg.Waypoint(latitude = 22.545, longitude = 113.96, altitude = 10, staytime = 2, heading = 180),
                dji_sdk.msg.Waypoint(latitude = 22.525, longitude = 113.93, altitude = 50, staytime = 0, heading = -180)]
            drone.waypoint_navigation_send_request(newWaypointList)
        elif main_operate_code == 'p':
            drone.arm_drone()
        elif main_operate_code == 'q':
            drone.disarm_drone()
        elif main_operate_code == 'r':
            drone.vrc_start();
            for i in range(100):
                drone.vrc_control(1024-660, 1024-660, 1024-660, 1024+660)
                time.sleep(0.02)
            for i in range(1000):
                drone.vrc_control(1024, 1024, 1024+660, 1024+660)
                time.sleep(0.02)
            for i in range(1000):
                drone.vrc_control(1024-660, 1024-660)
                time.sleep(0.02)
            drone.vrc_stop()
        elif main_operate_code == 's':
            drone.sync_timestamp(50)
        elif main_operate_code == 't':
            return
    #forwards
	elif main_operate_code == 'y':
            # draw square sample
            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, distance * math.cos(math.radians(currentRotation)), distance * math.sin(math.radians(currentRotation)), 0, currentRotation)
                time.sleep(0.02)
    #backwards
	elif main_operate_code == 'w':
            # draw square sample
            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, distance * math.cos(math.radians(currentRotation+180)), distance *math.sin(math.radians(currentRotation+180)), 0, currentRotation)
                time.sleep(0.02)

     #left
	elif main_operate_code == 'x':

            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, distance *math.cos(math.radians(currentRotation-90)), distance *math.sin(math.radians(currentRotation-90)), 0, currentRotation)
                time.sleep(0.02)
	elif main_operate_code == 'z':

            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, distance *math.cos(math.radians(currentRotation+90)), distance * math.sin(math.radians(currentRotation+90)), 0, currentRotation)
                time.sleep(0.02)
	elif main_operate_code == '0':

            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, 0, 0, 0, (currentRotation + 15))



                time.sleep(0.02)
            currentRotation += 15
            if currentRotation >= 360:
                currentRotation -= 360

	elif main_operate_code == '1':

            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, 0, 0, 0, (currentRotation + 90))



                time.sleep(0.02)
            currentRotation += 90
            if currentRotation >= 360:
                currentRotation -= 360

        else:
            display_main_menu()

if __name__ == "__main__":
    main()
