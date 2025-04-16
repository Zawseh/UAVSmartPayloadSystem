#!/usr/bin/env python

import rospy
import serial
from esp32_imu.msg import ImuWithTension  #custom message
import numpy as np
import time
import math

def try_open_serial(retries=100, delay_between=1.0):
    expected_response = "ESP32_READY"
    handshake_message = "HELLO\n"
    reset_message = "RESET_HANDSHAKE\n"
    attempt = 0

    while not rospy.is_shutdown():
        for i in range(retries):
            port = f"/dev/ttyACM{i}"
            try:
                ser = serial.Serial(port, 115200, timeout=1)
                rospy.loginfo(f"trying handshake on {port}...")
                time.sleep(0.3)

                ser.write(handshake_message.encode())
                time.sleep(0.2)

                response = ser.readline().decode().strip()
                if expected_response in response:
                    rospy.loginfo(f"handshake success on {port}")
                    rospy.set_param("/esp32_serial_port", port)
                    ser.close()
                    return port
                rospy.logwarn(f"no handshake match on {port} (got: '{response}')")
                ser.close()

            except serial.SerialException:
                continue
            except Exception as e:
                rospy.logwarn(f"failed on {port}: {e}")
                continue

        attempt += 1
        if attempt % 2 == 0:
            rospy.logwarn("RESETTING HANDSHAKE")
            for i in range(100): 
                for j in range(retries):
                    try:
                        port = f"/dev/ttyACM{j}"
                        ser = serial.Serial(port, 115200, timeout=0.05)
                        ser.write(reset_message.encode())
                        ser.close()
                    except:
                        continue
                time.sleep(0.001)

        rospy.loginfo("no ESP32 detected retrying in %.1fs.." % delay_between)
        time.sleep(delay_between)

    return None


SERIAL_PORT = try_open_serial()
BAUD_RATE = 115200


def compensate_tension(raw_weight, pitch_deg, roll_deg):

    pitch_rad = math.radians(pitch_deg)
    roll_rad = math.radians(roll_deg)

    tilt_rad = math.sqrt(pitch_rad**2 + roll_rad**2)
    tilt_deg = math.degrees(tilt_rad)

    if tilt_deg > 80:
        tilt_rad = math.radians(80)

    compensation_factor = math.cos(tilt_rad)

    if compensation_factor < 0.01:
        compensation_factor = 0.01  

    compensated_weight = raw_weight / compensation_factor
    return compensated_weight

def exponential_filter(alpha=0.1):
    state = {"last": None}

    def filter(value):
        if state["last"] is None:
            state["last"] = value
        else:
            state["last"] = alpha * value + (1 - alpha) * state["last"]
        return state["last"]

    return filter


def imu_smoother(window_size=10):
    pitch_history = []
    roll_history = []

    def filter(pitch, roll):
        pitch_history.append(pitch)
        roll_history.append(roll)

        if len(pitch_history) > window_size:
            pitch_history.pop(0)
        if len(roll_history) > window_size:
            roll_history.pop(0)

        avg_pitch = sum(pitch_history) / len(pitch_history)
        avg_roll = sum(roll_history) / len(roll_history)

        return avg_pitch, avg_roll

    return filter


smooth_imu = imu_smoother()

filter_fn = exponential_filter()

def tension_smoother(window_size=10):
    history = []

    def filter(value):
        history.append(value)
        if len(history) > window_size:
            history.pop(0)
        return sum(history) / len(history)

    return filter
smooth_tension = tension_smoother()


def estimate_object_position(swiv_pitch, swiv_roll, base_pitch=0.0, base_roll=0.0,
                             tether_length=2.0, base_influence=0.2):

    swiv_pitch_rad = math.radians(swiv_pitch)
    swiv_roll_rad = math.radians(swiv_roll)
    base_pitch_rad = math.radians(base_pitch)
    base_roll_rad = math.radians(base_roll)

    corrected_pitch = swiv_pitch_rad - base_influence * base_pitch_rad
    corrected_roll  = swiv_roll_rad - base_influence * base_roll_rad

    x = tether_length * math.sin(corrected_pitch) * math.cos(corrected_roll)  # forward/backward
    y = tether_length * math.sin(corrected_roll)                              # left/right
    z = -tether_length * math.cos(corrected_pitch) * math.cos(corrected_roll)  # downward

    return x, y, z  # forward, right, down









def read_imu_tension_data():
    """ Reads IMU + tension sensor data from the serial port and publishes it to ROS"""
    rospy.init_node("imu_serial_node", anonymous=True)
    imu_pub = rospy.Publisher("/imu_tension_data", ImuWithTension, queue_size=10)
    last_tension = 0.0 

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        rospy.loginfo(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")

        while not rospy.is_shutdown():
            try:
                line = ser.readline().decode().strip()  #read line from serial
                values = line.split(" ")

                if len(values) == 8:  # imu and tension
                    az, gx, gy, gz, gx2, gy2, gz2, tension = map(float, values)
                    last_tension = tension

                elif len(values) == 7:  #imu if tension fails to send, uses last tension value for fallback
                    az, gx, gy, gz, gx2, gy2, gz2 = map(float, values)
                    tension = last_tension

                else:
                    rospy.logwarn(f"Incorrect data format ({len(values)} values): {line}")
                    continue
                
                

                smoothed_pitch, smoothed_roll = smooth_imu(gy, gz)
                x_pred, y_pred, z_pred = estimate_object_position(gy2, gz2, smoothed_pitch, smoothed_roll)

                imu_msg = ImuWithTension()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = "imu_tension_link"

                imu_msg.swiv_pitch = gy2
                imu_msg.swiv_roll = gz2

                imu_msg.base_pitch = smoothed_pitch
                imu_msg.base_roll = smoothed_roll
                imu_msg.tension_value = filter_fn(tension)
                #filter_fn(compensate_tension(tension,gy,gz))
                smoothed_val = compensate_tension(tension,smoothed_pitch,smoothed_roll)
                imu_msg.compensated_tension = smooth_tension(smoothed_val)
                imu_msg.predict_yaw = x_pred
                imu_msg.predict_pitch = y_pred
                imu_msg.predict_roll = z_pred
                imu_pub.publish(imu_msg)

            except Exception as e:
                rospy.logwarn(f"error {e}")

    except serial.SerialException as e:
        rospy.logerr(f"cannot open {SERIAL_PORT}: {e}")

if __name__ == "__main__":
    try:
        read_imu_tension_data()
    except rospy.ROSInterruptException:
        pass