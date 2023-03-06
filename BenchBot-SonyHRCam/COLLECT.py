import logging
from signal import SIGINT, SIGTERM, signal

from utils.bbot_utils import get_config
from utils.collect_images import CollectImages
from utils.logger import log

cfg = get_config()

try:
    log.info("Starting image collection.")
    ci = CollectImages(cfg)
except Exception as e:
    log.exception("Could not initialize CollectImage class. Exiting.")
    exit(1)

try:
    log.info("Checking camera connection.")
    ci.check_cam_connection()
except Exception as e:
    log.exception("Could not check camera connection. Exiting.")
    exit(1)

try:
    log.info("Initializing Machine Motion (MM) controller.")
    ci.init_mm()
    # Keyboard interuption or termination handling
    signal(SIGINT, ci.sigint_handler)
    signal(SIGTERM, ci.sigterm_handler)
except Exception as e:
    log.exception("Could not initialize MM. Exiting.")
    exit(1)

try:
    log.info("Checking GPIO status")
    ci.check_gpio_status()
except Exception as e:
    log.exception("Checking GPIO status failed.")
    exit(1)

try:
    log.info("Prompting for camera slider homing.")
    ci.prompt_camera_home()
except Exception as e:
    log.error("Prompting for homing camera slider failed. Exiting.")
    exit(1)

# Start image collection
try:
    log.info("Starting image collection.")
    for row in range(1, ci.rows + 1):
        log.info("Correcting bbot position.")
        ci.correct_path()
        if row != 1:  # Don't move bbot at start
            log.info(f"Moving BenchBot to the next row ({row}).")
            ci.move_wheels_combined()
        # Take images per row
        for pot in range(1, ci.pots_per_row + 1):
            log.info(f"----------------------- Capturing image for pot {pot}/{ci.pots_per_row} - row {row}/{ci.rows}")
            ci.trigger_camera()
            ci.monitor_trigger()
            log.info("Moving camera slider for capture")
            ci.move_camera_slider(pot)
except Exception as e:
    log.exception("Could not collect images.")
    exit(1)

try:
    log.info("Image collection completed. Triggering estop.")
    ci.mm.triggerEstop()
except Exception as e:
    log.exception("Could not trigger estop. Exiting.")
    exit(1)
