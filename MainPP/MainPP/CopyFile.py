from logzero import logger, logfile
from sense_hat import SenseHat
from picamera import PiCamera
from gpiozero import CPUTemperature
import cv2 as cv
import ephem
import time
import os

# get time of the mission start
start_time = time.time()

MAXIMUM_FOLDER_SIZE = 2900000000 # bytes (Maximum 3GB)
DELAY_BETWEEN_PHOTOS = 9 # seconds
TIME_OF_MISSION = 10650 # seconds (Maximum 10800s)
CPU_TEMPERATURE_LIMIT = 85 # °C (Maximum 95°C)
MINIMUM_BRIGHTNESS = 55 # byte value, 255 - bright, 0 - dark
PIXEL_STEP = 20 # shortens the calculation time of pixel brightness
WIDTH_PHOTO_BORDER = 500 # pixel offset from edges used when counting pixel brightness
HEIGHT_PHOTO_BORDER = 200 # pixel offset from edges used when counting brightness
CPU_COOLDOWN_DELAY = 3 # seconds

# set the latest TLE data for ISS location
name = "ISS (ZARYA)"
l1 = "1 25544U 98067A   22287.73395616  .00015277  00000-0  27471-3 0  9997"
l2 = "2 25544  51.6422 104.3123 0003273 302.9418 167.8363 15.50041295363725"

# read and save the path to the main.py
dir_path = os.path.dirname(os.path.realpath(__file__))

# create logfile
logfile(dir_path + "/pardubicepi.log")

# setup an object for CPU temperature control
cpu = CPUTemperature()

# prepare for reading location
iss = ephem.readtle(name, l1, l2)

# setup camera 
cam = PiCamera()
cam.resolution = (2592, 1944)

def write_north():
    """Gets the orientation to north magnetic pole of Earth and writes it into EXIF of upcoming photo."""
    try:
        sense = SenseHat()
        cam.exif_tags["GPS.GPSImgDirection"] = "%d/100"  % (100 * round(sense.get_compass(), 2))
        logger.info("\tnorth: " + str(round(sense.get_compass(), 2)) + "°")
    except Exception as e:
        logger.error("{}: {})".format(e.__class__.__name__, e))
        logger.info("\tnorth: Failed")

def write_latlon():
    """Gets the longitude and latitude, changes its format and writes it into EXIF of upcoming photo (used example from Astro Pi Mission Space Lab Phase 2 guide)."""
    try:
        iss.compute()

        long_value = [float(i) for i in str(iss.sublong).split(":")]
        if long_value[0] < 0:
            long_value[0] = abs(long_value[0])
            cam.exif_tags["GPS.GPSLongitudeRef"] = "W"
        else:
            cam.exif_tags["GPS.GPSLongitudeRef"] = "E"
        cam.exif_tags["GPS.GPSLongitude"] = "%d/1, %d/1, %d/10" % (long_value[0], long_value[1], long_value[2]*10)
        logger.info("\tlongitude: " + str(int(long_value[0])) + "°" + str(int(long_value[1])) + "'" + str(int(long_value[2])) + "''")
    except Exception as e:
        logger.error("{}: {})".format(e.__class__.__name__, e))
        logger.info("\tlongitude: Failed")

    try:
        lat_value = [float(i) for i in str(iss.sublat).split(":")]
        if lat_value[0] < 0:
            lat_value[0] = abs(lat_value[0])
            cam.exif_tags["GPS.GPSLatitudeRef"] = "S"
        else:
            cam.exif_tags["GPS.GPSLatitudeRef"] = "N"
        cam.exif_tags["GPS.GPSLatitude"] = "%d/1, %d/1, %d/10" % (lat_value[0], lat_value[1], lat_value[2]*10)
        logger.info("\tlatitude: " + str(int(lat_value[0])) + "°" + str(int(lat_value[1])) + "'" + str(int(lat_value[2])) + "''")
    except Exception as e:
        logger.error("{}: {})".format(e.__class__.__name__, e))
        logger.info("\tlatitude: Failed")

def get_brightness(image):
    """Returns the brightness of taken image."""
    try:
        image_in_hsv = cv.cvtColor(cv.imread(image), cv.COLOR_BGR2HSV)
        brightness, pixel_counter = [0] * 2
        width = WIDTH_PHOTO_BORDER
        height = HEIGHT_PHOTO_BORDER

        while height < len(image_in_hsv) - 1 - HEIGHT_PHOTO_BORDER:
            while width < len(image_in_hsv[height]) - 1 - WIDTH_PHOTO_BORDER:
                brightness += image_in_hsv[height][width][2]
                pixel_counter += 1
                width += PIXEL_STEP
            height += PIXEL_STEP
            width = 0
        return brightness / pixel_counter

    except Exception as e:
        logger.error("{}: {})".format(e.__class__.__name__, e))
        logger.info("\tGetting brightness problem")
        return 255

def get_size(start_path = "."):
    """Returns the calculated size of the used memory space."""
    try:
        total_size = 0
        for dirpath, dirnames, filenames in os.walk(start_path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                total_size += os.path.getsize(fp)
        return total_size

    except Exception as e:
        logger.error('{}: {})'.format(e.__class__.__name__, e))
        return (photo_counter - 1) * 3500000

# start the mission
logger.info("Mission started")

photo_counter = 1

# measure size of folder at the beginning
initial_size = get_size(dir_path)
logger.info("Initial folder size: " + str(initial_size))

# repeat this code until the time or folder size exceed
while time.time() - start_time < TIME_OF_MISSION and get_size(dir_path) < initial_size + MAXIMUM_FOLDER_SIZE:
    try:
        # save the full path od upcoming photo
        image_name = dir_path + "/pardubicepi_" + str(photo_counter).zfill(4) + ".jpg"

        # measure the wanted information (the direction of North, longitude and latitude)
        write_north() 
        write_latlon()

        # start measuring time of last photo taken, take photo and increase the photo counter
        time_last = time.time()
        cam.capture(image_name)
        photo_counter += 1

        logger.info("Captured %s" % image_name)

        # if the photo is too datk, delete it
        if get_brightness(image_name) < MINIMUM_BRIGHTNESS:
            os.remove(image_name)
            photo_counter -= 1
            logger.info("Deleted %s" % image_name)

        # wait until ISS changes its position
        while time.time() - time_last < DELAY_BETWEEN_PHOTOS:
            time.sleep(0.3)

        # while the CPU is too hot, just wait
        while(cpu.temperature > CPU_TEMPERATURE_LIMIT):
            # if it is too late leave the CPUloop"
            if time.time() + CPU_COOLDOWN_DELAY - start_time > TIME_OF_MISSION:
                break

            logger.info("CPU is too hot, CPU temperature: " + str(round(cpu.temperature, 1)) + "°C")
            time.sleep(CPU_COOLDOWN_DELAY)

    except Exception as e:
        logger.error("{}: {})".format(e.__class__.__name__, e))

if get_size(dir_path) > initial_size + MAXIMUM_FOLDER_SIZE:
    logger.info("Folder size exceeded")
else:
    logger.info("Time has exceeded")

logger.info("Mission has ended")