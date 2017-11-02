#!/usr/bin/env python
# coding: Latin-1

############################################################
# This is the start-up script for the Formula Pi Race Code #
#                                                          #
# It is responsible for managing the threads which control #
# the MonsterBorg during a race                            #
############################################################

# Load all the library functions we want
import time
import os
import sys
import threading
import cv2
import numpy
import ThunderBorg
import random
import inspect
import Globals
print 'Libraries loaded'

# Change the current directory to where this script is
scriptDir = os.path.dirname(sys.argv[0])
os.chdir(scriptDir)
print 'Running script in directory "%s"' % (scriptDir)

# Setup the ThunderBorg
TB = ThunderBorg.ThunderBorg()
#TB.i2cAddress = 0x15                  # Uncomment and change the value if you have changed the board address
TB.Init()
if not TB.foundChip:
	boards = ThunderBorg.ScanForThunderBorg()
	if len(boards) == 0:
		print 'No ThunderBorg found, check you are attached :)'
	else:
		print 'No ThunderBorg at address %02X, but we did find boards:' % (TB.i2cAddress)
		for board in boards:
			print '    %02X (%d)' % (board, board)
		print 'If you need to change the I²C address change the setup line so it is correct, e.g.'
		print 'TB.i2cAddress = 0x%02X' % (boards[0])
	sys.exit()
TB.SetCommsFailsafe(False)
TB.SetLedShowBattery(False)
TB.SetLeds(0,0,0)

# Functions used by the processing to control the MonsterBorg
def MonsterLed(r, g, b):
	global TB
	TB.SetLeds(r, g, b)

def MonsterMotors(driveLeft, driveRight):
	global TB
	TB.SetMotor1(driveRight * Settings.maxPower) # Right
	TB.SetMotor2(driveLeft  * Settings.maxPower) # Left


# Add the cross-module functions to the global list
Globals.MonsterLed = MonsterLed
Globals.MonsterMotors = MonsterMotors

# Setup synchronisation locks
Globals.frameLock = threading.Lock()

# Load the standard processing code after the built-ins
import ImageProcessor
from RaceCodeFunctions import *
print 'Image processor and Race Code Functions loaded'

# Function for displaying the current settings
def ShowSettings():
	print '=== NEW SETTINGS ==='
	print

	print '[Image setup]'
	print 'Camera %d x %d at %d fps' % (Settings.imageWidth, Settings.imageHeight, Settings.frameRate)
	print 'X cropped from %d to %d' % (Settings.cropX1, Settings.cropX2)
	print 'Y cropped from %d to %d' % (Settings.cropY1, Settings.cropY2)
	print 'Image processing threads: %d' % (Settings.processingThreads)
	print

	print '[Startup]'
	print 'Initial mode: %d' % (Settings.startupMode)
	print

	print '[Y scan lines]'
	for Y in Settings.croppedYScan:
		print '%d (%d)' % (Y + Settings.cropY1, Y)
	print

	print '[Colour identification]'
	print 'Black limit: %d %d %d' % (Settings.blackMaxR, Settings.blackMaxG, Settings.blackMaxB)
	print 'Green gain: %.2f' % (Settings.greenGain)
	print 'Blue gain: %.2f' % (Settings.blueGain)
	print 'Target level for auto-gain: %.0f' % (Settings.targetLevel)
	print 'Erosion factor for colour channels: %.0f' % (Settings.erodeChannels)
	print 'Final colour minimums: %d %d %d' % (Settings.redMin, Settings.greenMin, Settings.blueMin)
	print

	print '[Line correction]'
	print 'Colour edge gap: %.0f pixels' % (Settings.maxSepX)
	print 'Lane gap: %.0f pixels' % (Settings.trackSepX)
	print 'Offset Y calculation target %d: (%d)' % (Settings.offsetTargetY, Settings.croppedTargetY)
	print 'Corrective gain for derivative: %.3f' % (Settings.gainCorrection)
	print

	print '[Start marker detection]'
	print 'Levels: Min Red = %d, Max Green = %d, Max Blue = %d' % (Settings.startMinR, Settings.startMaxG, Settings.startMaxB)
	print 'Start minimum match ratio: %.1f %%' % (Settings.startRatioMin * 100.0)
	print 'Start crossed delay %.2f s (%d frames)' % (Settings.startCrossedSeconds, Settings.startCrossedFrames)
	print 'Start re-detection delay: %.1f s' % (Settings.startRedetectionSeconds)
	print 'Start detection zone X limits: %d to %d' % (Settings.startX1, Settings.startX2)
	print 'Start detection zone Y position: %d' % (Settings.startY)

	print '[PID values]'
	print '    P0: %f   I0:%f   D0: %f' % (Settings.Kp0, Settings.Ki0, Settings.Kd0)
	print '    P1: %f   I1:%f   D1: %f' % (Settings.Kp1, Settings.Ki1, Settings.Kd1)
	print '    P2: %f   I2:%f   D2: %f' % (Settings.Kp2, Settings.Ki2, Settings.Kd2)
	print

	print '[FIR filter]'
	print '    Taps: %d' % (Settings.firTaps)
	print

	print '[Drive settings]'
	steeringMin = Settings.steeringOffset - Settings.steeringGain
	steeringMax = Settings.steeringOffset + Settings.steeringGain
	print 'Maximum output: %.1f %%' % (Settings.maxPower * 100.0)
	print 'Steering %+.1f %% to %+.1f %% (central %+.1f %%)' % (steeringMin * 100.0, steeringMax * 100.0, Settings.steeringOffset * 100.0)
	print 'Missing frames before stopping: %d' % (Settings.maxBadFrames)
	print

	print '[Override settings]'
	print 'Stuck detection threshold: %.2f' % (Settings.stuckIdenticalThreshold)
	print 'Stuck detection time: %.2f s (%d frames)' % (Settings.stuckIdenticalSeconds, Settings.stuckIdenticalFrames)
	print 'Stuck reversing time: %.2f s (%d frames)' % (Settings.stuckOverrideSeconds, Settings.stuckOverrideFrames)
	print 'Stuck hunting time: %.2f s (%d frames)' % (Settings.stuckHuntSeconds, Settings.stuckHuntFrames)
	print 'Stuck colour detection at %d x %d' % (Settings.stuckDetectColourX, Settings.stuckDetectColourY)
	print 'Wrong way detection threshold: %d' % (Settings.wrongWayThreshold)
	print 'Wrong way spin time: %.2f s (%d frames)' % (Settings.wrongWaySpinSeconds, Settings.wrongWaySpinFrames)
	print 'Robot in front detection threshold: %d' % (Settings.overtakeThreshold)
	print 'Overtaking lane shift: %.2f' % (Settings.overtakeLaneOffset)
	print 'Overtaking time: %.2f s (%d frames)' % (Settings.overtakeDurationSeconds, Settings.overtakeDurationFrames)
	print

	print '===================='
	print

# Function to load settings overrides
def SettingsOverrides():
	pass

# Function for auto-loading the settings file when it has changed
global modificationStamp
modificationStamp = 0
def AutoReloadSettings():
	global modificationStamp
	newModificationStamp = os.path.getmtime('Settings.py')
	if newModificationStamp != modificationStamp:
		reload(Settings)
		SettingsOverrides()
		ImageProcessor.Settings = Settings
		if Globals.controller:
			Globals.controller.Reset()
		ShowSettings()
		Globals.startLights = ImageProcessor.READY_OFF
		modificationStamp = newModificationStamp

# Load the initial settings (does a reload)
import Settings
AutoReloadSettings()
Globals.imageMode = Settings.startupMode
Globals.lastImageMode = ImageProcessor.READY_TO_RACE

# Image processing debugging settings
ImageProcessor.filePattern = './test-%05d-%s.jpg'
ImageProcessor.writeRawImages = False 
ImageProcessor.writeImages = False
ImageProcessor.debugImages = False
ImageProcessor.showProcessing = False
ImageProcessor.showFps = True
ImageProcessor.showUnknownPoints = False
ImageProcessor.predatorView = False
ImageProcessor.scaleFinalImage = 1.0
ImageProcessor.scaleDebugImage = 1.0
ImageProcessor.dPlotY = -10

# Thread for running Race.py
raceGlobals = globals().copy()
raceLocals = locals().copy()
class RaceLoop(threading.Thread):
	def __init__(self):
		super(RaceLoop, self).__init__()
		self.start()

	def run(self):
		# This method runs in a separate thread, but shares global and local values / functions
		execfile('Race.py', raceGlobals.copy(), raceLocals.copy())

# Startup sequence
os.system('sudo modprobe bcm2835-v4l2')
Globals.capture = cv2.VideoCapture(0) 
Globals.capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, Settings.imageWidth);
Globals.capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, Settings.imageHeight);
Globals.capture.set(cv2.cv.CV_CAP_PROP_FPS, Settings.frameRate);
if not Globals.capture.isOpened():
	Globals.capture.open()
	if not Globals.capture.isOpened():
		print 'Failed to open the camera'
		sys.exit()

print 'Setup stream processor threads'
Globals.processorPool = [ImageProcessor.StreamProcessor(i+1) for i in range(Settings.processingThreads)]
allProcessors = Globals.processorPool[:]

print 'Setup control loop'
Globals.controller = ImageProcessor.ControlLoop()

print 'Wait ...'
time.sleep(2)
captureThread = ImageProcessor.ImageCapture()

print 'Start Race.py'
raceThread = RaceLoop()

try:
	print 'Press CTRL+C to quit'
	TB.MotorsOff()
	if ImageProcessor.showProcessing:
		cv2.namedWindow('Processed', cv2.WINDOW_NORMAL)
	if ImageProcessor.predatorView:
		cv2.namedWindow('Predator', cv2.WINDOW_NORMAL)
	# Loop indefinitely
	while Globals.running:
		# See if there is a frame to show
		if Globals.displayFrame != None:
			if ImageProcessor.scaleFinalImage != 1.0:
				size = (int(Globals.displayFrame.shape[1] * ImageProcessor.scaleFinalImage), 
						int(Globals.displayFrame.shape[0] * ImageProcessor.scaleFinalImage))
				processed = cv2.resize(Globals.displayFrame, size, interpolation = cv2.INTER_CUBIC)
			else:
				processed = Globals.displayFrame
			cv2.imshow('Processed', processed)
		if Globals.displayPredator != None:
			if ImageProcessor.scaleFinalImage != 1.0:
				size = (int(Globals.displayPredator.shape[1] * ImageProcessor.scaleFinalImage), 
						int(Globals.displayPredator.shape[0] * ImageProcessor.scaleFinalImage))
				predator = cv2.resize(Globals.displayPredator, size, interpolation = cv2.INTER_CUBIC)
			else:
				predator = Globals.displayPredator
			cv2.imshow('Predator', predator)
		# Wait either way
		if (Globals.displayFrame != None) or (Globals.displayPredator != None):
			cv2.waitKey(100)
		else:
			# Wait for the interval period
			time.sleep(0.1)
		# Check if on-the-fly settings have been changed
		AutoReloadSettings()
	# Disable all drives
	TB.MotorsOff()
except KeyboardInterrupt:
	# CTRL+C exit, disable all drives
	TB.SetLeds(0.5,0.5,0.5)
	print '\nUser shutdown'
	TB.MotorsOff()
except:
	# Unexpected error, shut down!
	TB.SetLeds(0.5,0.5,0.5)
	e = sys.exc_info()
	print
	print e
	print '\nUnexpected error, shutting down!'
	TB.MotorsOff()
# Tell each thread to stop, and wait for them to end
Globals.running = False
while allProcessors:
	with Globals.frameLock:
		processor = allProcessors.pop()
	processor.terminated = True
	processor.event.set()
	processor.join()
Globals.controller.terminated = True
raceThread.terminated = True
Globals.controller.join()
captureThread.join()
raceThread.join()
Globals.capture.release()
del Globals.capture
TB.MotorsOff()
TB.SetLeds(Globals.colour[0] * 0.3, Globals.colour[1] * 0.3, Globals.colour[2] * 0.3)
print 'Program terminated.'
