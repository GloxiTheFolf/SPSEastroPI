import gzip
import os
import time
import cv2
import threading
from logzero import logger, logfile
from picamera import PiCamera
from gpiozero import CPUTemperature

cam = PiCamera()
cam.resolution = (4056,3040)

cpu = CPUTemperature

dirPath = os.path.dirname(os.path.realpath(__file__))

logfile(dirPath + "Awooo.log")


cam.start_preview

time.sleep(15)
cam.stop_preview()
time.sleep(1)
cam.start_preview()
fileNum = 0

def CompressWithGzip(name):
    startTime = time.time_ns
    compG = gzip.compress(img)
    f = open("GzipComp" + name + ".gz", "wb")
    f.write(compG)
    f.close()
    logger.info("Gzip " + name +" time: " + (startTime - time.time_ns) + " ns")


mainStartTime = time.time_ns
for x in range(0,10):
    startTime = time.time_ns
    cam.capture("Original" + fileNum + ".jpg")
    logger.info("Original " + fileNum +" time: " + (startTime - time.time_ns) + " ns")

    img = cv2.imread("Original" + fileNum + ".jpg")

    compressThread = threading.Thread(target=CompressWithGzip, args=(fileNum))
    compressThread.start()
    CompressWithGzip(fileNum)

    time.sleep(2)
    while(cpu.temperature > 80):
        logger.info("CPU is too hot, CPU temperature: " + str(round(cpu.temperature, 1)) + "°C")
        time.sleep(3)
        fileNum++
    pass
logger.info("mainTime: " + (mainStartTime - time.time_ns) )
cam.stop_preview()

