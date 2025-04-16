#!/usr/bin/env python3

import rospy
import serial
import subprocess
import time
from esp32_imu.srv import SetServo, SetServoResponse  

def get_current_port():
    if rospy.has_param("/esp32_serial_port"):
        return rospy.get_param("/esp32_serial_port")
    else:
        rospy.logwarn("ESP32 serial port param not found.")
        return None


suspended_pids = []

def suspend_busy_processes():
    global suspended_pids
    suspended_pids.clear()
    try:
        port = get_current_port()
        if not port: return
        output = subprocess.check_output(["lsof", port]).decode("utf-8")
        lines = output.split("\n")[1:]
        for line in lines:
            if line.strip() and "python" in line:
                pid = int(line.split()[1])
                rospy.logwarn(f"suspending process {pid} using {port}")
                subprocess.run(["kill", "-STOP", str(pid)], check=True)
                suspended_pids.append(pid)
    except subprocess.CalledProcessError:
        pass

import os

def resume_suspended_processes():
    global suspended_pids
    for pid in suspended_pids:
        if os.path.exists(f"/proc/{pid}"):
            try:
                rospy.loginfo(f"resuming process {pid}")
                subprocess.run(["kill", "-CONT", str(pid)], check=True)
                time.sleep(0.1)  
            except subprocess.CalledProcessError:
                rospy.logwarn(f"failed to resume PID {pid}")
        else:
            rospy.logwarn(f"PID {pid} does not exist anymore")
    suspended_pids.clear()

def handle_servo_request(req):
    """called when the service is triggered"""
    suspend_busy_processes()

    try:
        port = get_current_port()
        if not port:
            rospy.logerr("ESP32 port not available.")
            resume_suspended_processes()
            return SetServoResponse(success=False)
        ser = serial.Serial(port, 115200, timeout=1)

        time.sleep(0.1) 
        command = f"{req.angle}\n"
        rospy.loginfo(f"sending angle {req.angle} to {port}")

        ser.write(command.encode())
        ser.flush()
        ser.close()
        resume_suspended_processes()
        return SetServoResponse(success=True)
    except serial.SerialException as e:
        rospy.logerr(f"error: {e}")
        resume_suspended_processes()
        return SetServoResponse(success=False)

def servo_service_node():
    rospy.init_node("servo_control_service_node")
    service = rospy.Service("set_servo", SetServo, handle_servo_request)
    rospy.loginfo("Servo service ready.")
    rospy.spin()

if __name__ == "__main__":
    try:
        servo_service_node()
    except rospy.ROSInterruptException:
        pass
