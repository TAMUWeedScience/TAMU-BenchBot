from datetime import date
import os
import threading
import time
from pathlib import Path

import numpy as np
# Raspberry pi
import RPi.GPIO as GPIO
# For checking camera connection
import usb.core
from usb.util import get_string

from utils.bbot_utils import find_orientation
from utils.logger import log
from utils.MachineMotion import *
from utils.RPI_Sensors import UltrasonicSensor

GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)


class CollectImages:

    def __init__(self, cfg):
        """Main BBot operation class that use configuration information to 
        moves the benchbot wheel motors, camera sliders, and triggers camera.
        Gets configuration information from .env file located in the project
        root and a single user defined input from the command line (rows to traverse).
        """
        # General config
        self.cfg = cfg
        self.offsets = [cfg.FRONT_OFFSET, cfg.BACK_OFFSET]
        self.state_id = cfg.STATE_ID
        self.pots_per_row = 4
        self.rows = int(
            input(
                "How many rows do you want the BBot to move? You must provide a single whole number (e.g. 45)."
            ))

        # Final and temp (for renaming and checking) image save directories
        self.final_batch_dir = Path(f"{self.state_id}_{date.today()}","SONY")
        self.final_batch_dir.mkdir(exist_ok=True, parents=True)

        # Camera axis
        self.camera_motor = 1
        self.camera_home_speed = 55
        self.camera_home_acc = 30
        self.camera_capture_speed = 50
        self.camera_capture_acc = 30
        self.from_home = True
        self.cam_dist = 0

        # Wheel axes
        self.wheel_motors = [2, 3]
        self.move_wheel_distance = cfg.DISTANCE_TRAVELED
        self.directions = cfg.DIRECTIONS
        self.mm_wheel_acc = 50
        self.mm_wheel_speed = 80

        # File stem (filename str without extension)
        self.temp_fstem = None

    ################# Initialize Machine Motion

    def init_mm(self):
        # Init bbot
        self.mm = MachineMotion(DEFAULT_IP)
        self.mm.isReady()
        self.mm.releaseEstop()
        self.mm.resetSystem()
        # self.mm.bindeStopEvent(self.e_stop_callback)
        time.sleep(1)  # in seconds

    ################## Callbacks and signal handlers

    def gcode_callback(self, data):
        """Callback to process controller gCode responses at
        at Machine Motion initialization."""
        print("Controller gCode responses " + data)
        print("Controller connected")

    def e_stop_callback(estop_status):
        """Define a callback to process estop status"""
        print("eStop status is : " + str(estop_status))
        if estop_status == True:
            # executed when you enter estop
            print("MachineMotion is in estop state.")
        elif estop_status == False:
            # executed when you exit estop
            print("MachineMotion is not in estop state.")

    def sigint_handler(self, signum, frame):
        """Keyboard interuption handling. 
        1. Returns camera slider to home position
        2. Enables MM Estop and
        3. Exits process (1)
        """
        log.warning("Python SIGINT detected. Homing camera and triggering Estop.")
        if hasattr(self, "mm"):
            self.home_camera()
            self.mm.triggerEstop()
        exit(1)

    def sigterm_handler(self, signum, frame):
        """Same as signit_handler"""
        # Terminate signal handling.
        log.warning("Python SIGTERM detected. Homing camera and triggering Estop.")
        if hasattr(self, "mm"):
            self.home_camera()
            self.mm.triggerEstop()
        exit(1)

    ################## Axis configurations

    def config_camera_axis(self, home=False):
        """Configers camera slider  stepper motor by setting 
        camera axis, speed, and acceleration. Sets speed and acceleration
        based on 'homing' or 'capture' status."""

        self.mm.configAxis(self.camera_motor, MICRO_STEPS.ustep_8,
                           MECH_GAIN.ballscrew_10mm_turn)
        self.mm.configAxisDirection(self.camera_motor, DIRECTION.POSITIVE)
        if home:
            self.mm.configHomingSpeed(self.camera_motor,
                                      self.camera_home_speed)
            self.mm.emitAcceleration(self.camera_capture_acc)
        else:  # capture
            # Global speed and acceleration
            self.mm.emitSpeed(self.camera_capture_speed)
            self.mm.emitAcceleration(self.camera_capture_acc)

    def config_wheel_axes(self):
        """Configures machine motion wheel motor axes, 
        directions, acceleration, and speed."""
        # config machine motion wheel motors
        for idx, axis in enumerate(self.cfg.WHEEL_MOTORS):
            self.mm.configAxis(axis, MICRO_STEPS.ustep_8,
                               MECH_GAIN.enclosed_timing_belt_mm_turn)
            self.mm.configAxisDirection(axis, self.directions[idx])

        self.mm.emitAcceleration(self.mm_wheel_acc)
        self.mm.emitSpeed(self.mm_wheel_speed)

    ################ Camera slider movement (motor #1)
    def get_cam_dist(self):
        self.cam_dist = int(self.cfg.HOME_TO_END_SENSOR_DISTANCE /
                                (self.pots_per_row - 1))
        return self.cam_dist
    def home_camera(self):
        """Homes camera slider."""
        self.config_camera_axis(home=True)  # Faster
        self.mm.moveToHome(self.camera_motor)
        time.sleep(1)
        self.mm.waitForMotionCompletion()
    
    def prompt_camera_home(self):
        input_status = True
        while input_status:
            home_cam = input("Home camera slider (y/n)?  ").lower().strip()
            if home_cam == "y":
                self.home_camera()
                input_status = False
            elif home_cam == "n":
                input_status = False
            else:
                print("You haven't provided a valid string")

    def move_camera_slider(self, pot_num):
        """Moves camera slider a given distance.

        Args:
            distance (int): distance based on distance between
            proximity sensors and remaining images to capture. 
        """
        self.config_camera_axis()
        enstopstate = self.mm.getEndStopState()
        home_sensor_status = enstopstate["x_min"].strip() 
        end_sensor_status = enstopstate["x_max"].strip()
        
        log.info(f"Home sensor status {home_sensor_status}")
        log.info(f"End sensor status {end_sensor_status}")
        
        if pot_num == 1:
            if home_sensor_status == "TRIGGERED":
                self.cam_dist = self.get_cam_dist()
                self.from_home = True
            elif end_sensor_status == "TRIGGERED":
                self.from_home = False
                self.cam_dist = self.get_cam_dist()* -1

        elif pot_num != 1:
            if self.from_home:
                self.cam_dist = self.get_cam_dist()
            elif self.from_home == False:
                self.cam_dist = self.get_cam_dist()* -1
         
        log.info(f"Camera distance: {self.cam_dist}")
        log.info(f"Position of camera slider: {self.mm.getActualPositions()}")
        self.mm.moveRelative(self.camera_motor, self.cam_dist)
        time.sleep(1)
        self.mm.waitForMotionCompletion()

    ################ Wheel movement (motor #2 and #3)

    def move_wheels_combined(self):
        """Move wheel stepper motors simultaneously 
        a given travel distance. 
        """
        self.config_wheel_axes()
        self.mm.moveRelativeCombined(
            self.wheel_motors,
            [self.move_wheel_distance, self.move_wheel_distance])
        time.sleep(1)
        self.mm.waitForMotionCompletion()

    def move_wheels_seperate(self, angle, correction_dist):
        """For correcting path. 

        Args:
            angle (float): angle
            travel_distance (float): distance corrections from sensor in millimeters
        """
        if angle > 0.5 or angle < -0.5:
            if angle > 0.5:
                motor = self.wheel_motors[0]
            elif angle < -0.5:  # TODO verify this is right
                motor = self.wheel_motors[1]
            self.config_wheel_axes()
            self.mm.moveRelative(motor, correction_dist)
            time.sleep(1)
            self.mm.waitForMotionCompletion()
        else:
            log.info(f"BBot angle is fine ({angle}), adjusting position is not necessary.  ")
        
    ################ Capture image and naming

    def check_cam_connection(self):
        # find our device
        try:
            cam = usb.core.find(idVendor=0x054c, idProduct=0x0d9f)
            log.info(
                f"Camera found. Manufacturer:{get_string(cam, 1)}, Product: {get_string(cam, 2)}, Serial: {get_string(cam, 3)}"
            )
        except AttributeError:
            log.error(
                "Camera could not connect. Check cable, camera power, and battery. Exiting."
            )
            exit(1)

    def monitor_trigger(self):
        """Logs if both JPG and RAW images were saved and present on directory. 
        Triggers camera a second time if RAW image was not saved. Exits if 
        second trigger fails to save RAW again. Logs error if RAW or both 
        are missing. Logs warning if only JPG is missing.
        """
        jpg = Path(self.final_batch_dir, self.temp_fstem + ".JPG").exists()
        raw = Path(self.final_batch_dir, self.temp_fstem + ".ARW").exists()

        if jpg and raw:
            log.info(f"JPG and Raw images exists for {self.temp_fstem}")
        elif not jpg and raw:
            log.warning(f"Only RAW for {self.temp_fstem} was captured.")

        elif not raw:
            log.warning(
                f"Only JPG for {self.temp_fstem} was captured. Triggering camera a second time."
            )
            self.trigger_camera()
            raw = Path(self.temp_fstem + ".ARW").exists()
            if not raw:
                log.error(
                    "Camera failed to capture the RAW image a second time. Exiting."
                )
        else:
            log.error(f"{self.temp_fstem} was not captured.")

    def trigger_camera(self):
        start = time.time()
        ts = int(time.time())
        CAM_PATH = os.getcwd() + "/support/SONY_rpi/RemoteCli"
        os.system(CAM_PATH)
        time.sleep(2)
        self.temp_fstem = f"{self.state_id}_{ts}"
        threading.Thread(target=self.file_rename(self.temp_fstem)).start()
        end = time.time()
        elapsed_time = round(end - start, 3)
        log.info(f"{elapsed_time} sec for capture and renaming.")

    def file_rename(self, file_stem):
        time.sleep(1)
        for file_name in os.listdir("."):
            if file_name.startswith('DSC'):
                if file_name.endswith('.JPG'):
                    new_name = f"{file_stem}.JPG"
                elif file_name.endswith('.ARW'):
                    new_name = f"{file_stem}.ARW"
                os.rename(file_name, Path(self.final_batch_dir,new_name))

    ################ GPIO and Ultrasonic sensors

    def check_gpio_status(self):
        GPIO.setwarnings(False)
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        trigger_list = self.cfg.TRIGGER_PINS
        outs = []
        for trig in trigger_list:
            GPIO.setup(trig, GPIO.OUT)
            GPIO.output(trig, True)
            time.sleep(0.00001)
            GPIO.output(trig, False)
            out = GPIO.input(trig)
            outs.append(out)
            # return outs
            # if out == 0:
            #     raise ValueError(
                    # f"Trigger pin {trig} is not recieving input ({out})")

    def get_distances_rpi(self):
        """Calculates distance of ultrasonic sensor to rail.
        Args:
            cfg (dataclass): BBotConfig dataclass
            offsets (array): array of sensor offsets
        Returns:
            list: list of distance offsets for each wheel
        """
        trigger_list = self.cfg.TRIGGER_PINS
        echo_list = self.cfg.ECHO_PINS
        dist_list = np.zeros(
            (self.cfg.NUMBER_OF_SENSORS, self.cfg.NUMBER_OF_SENSORS))
        sensor = []
        # Creates UltrasonicSensor instances for each sensor
        for num in range(0, self.cfg.NUMBER_OF_SENSORS):
            sensor.append(UltrasonicSensor(trigger_list[num], echo_list[num]))

        for k in range(0, self.cfg.NUMBER_OF_SENSORS):
            for i in range(0, self.cfg.NUMBER_OF_SENSORS):
                dist_raw = round(sensor[i].distance(), 1)
                dist_list[k, i] = dist_raw - self.offsets[i]

        dist_list = np.median(dist_list, 0)
        return [dist_list[1], dist_list[0]]

    # Helper function for getting distance measurements from sensor
    def correct_path(self):
        """Makes small adjustments based on ultrasonic sensors 
        proximity to rail. Makes adjust one wheel at a time. 
        """
        log.info("Calculating distances from ultrasonic sensor")
        corrected_distance = self.get_distances_rpi()
        # Getting angle
        ang = find_orientation(self.cfg.ROBOT_LENGTH, corrected_distance)
        # Calculate how much distance motor needs to move to align platform
        d_correction_mm = 2 * np.pi * self.cfg.ROBOT_WIDTH * (abs(ang) /
                                                              360) * 10
        adjust_thresh = 150
        if d_correction_mm > adjust_thresh:
            log.warning(f"Distance to align platforms was found to be greater than {adjust_thresh}mm at {d_correction_mm}. No adjustment is being applied.")
            d_correction_mm = 0.0
        # Create if statement to indicate which motor moves
        log.info(f"Angle: {ang}")
        log.info(
            f"Distance to align platforms {d_correction_mm}"
        )
        self.move_wheels_seperate(ang, d_correction_mm)