#!/usr/bin/env python
# coding: Latin-1

############################################################
# This is a simulation script for the Formula Pi Race Code #
#                                                          #
# This simulation uses a folder of images to run the image #
# processing manually                                      #
############################################################

# Load all the library functions we want
import time
import os
import sys
import threading
import cv2
import numpy
import random
import inspect
import Globals
print 'Libraries loaded'

# Simulation only parameters
autoRun = True							# True to run a directory of images, False to load a single image
fileIn = 'test-00029-raw.jpg'			# Single image file to load
#fileIn = raw_input('? ')				# Uncomment to be asked for the single image file each time
autoPath = r'C:\Temp\TrackImages'		# Directory to load images from
#autoPath = raw_input('? ')				# Uncomment to be asked for the directory each time
autoDelay = 1							# Delay in milliseconds between images, 0 will pause each frame until a key is pressed

# Change the current directory to where this script is
scriptDir = os.path.dirname(sys.argv[0])
os.chdir(scriptDir)
print 'Running script in directory "%s"' % (scriptDir)

# Functions used by the processing to control the MonsterBorg
def MonsterLed(r, g, b):
	print '>>> LED: %.1f %.1f %.1f' % (r, g, b)

def MonsterMotors(driveLeft, driveRight):
	print '>>> MOTORS: %.3f | %.3f' % (driveLeft, driveRight)

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
	print '    P0: %f	I0:%f	D0: %f' % (Settings.Kp0, Settings.Ki0, Settings.Kd0)
	print '    P1: %f	I1:%f	D1: %f' % (Settings.Kp1, Settings.Ki1, Settings.Kd1)
	print '    P2: %f	I2:%f	D2: %f' % (Settings.Kp2, Settings.Ki2, Settings.Kd2)
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
	# Force detailed image display each frame
	Settings.fpsInterval = 1

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
ImageProcessor.debugImages = True
ImageProcessor.showProcessing = True
ImageProcessor.showFps = False
ImageProcessor.showUnknownPoints = True
ImageProcessor.predatorView = False
ImageProcessor.scaleFinalImage = 4.0
ImageProcessor.scaleDebugImage = 2.0
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

# Dummy startup
print 'Setup processor'
processor = ImageProcessor.StreamProcessor('ImageSequence', False)

print 'Setup control loop'
Globals.controller = ImageProcessor.ControlLoop()

print 'Start Race.py'
raceThread = RaceLoop()

try:
	if autoRun:
		print 'Scanning directory'
		files = os.listdir(autoPath)
		t = []
		for i in range(100):
			t.extend(files)
		files = t
		for filepath in files:
			if filepath.endswith('.jpg'):
				print 'Loading %s' % filepath
				filepath = os.path.join(autoPath, filepath)
				image = cv2.imread(filepath)
				if (image.shape[1] != Settings.imageWidth) or (image.shape[0] != Settings.imageHeight):
					ratioIn = image.shape[1] / float(image.shape[0])
					ratioOut = Settings.imageWidth / float(Settings.imageHeight)
					if abs(ratioIn - ratioOut) < 0.01:
						# Straight resize
						pass
					elif ratioIn > ratioOut:
						# Crop width
						width = image.shape[1]
						cropWidth = ratioOut / ratioIn
						cropOffset = (1.0 - cropWidth) / 2.0
						cropWidth = int(cropWidth * width)
						cropOffset = int(cropOffset * width)
						image = image[:, cropOffset : cropOffset + cropWidth, :]
					else:
						# Crop height
						height = image.shape[0]
						cropHeight = ratioIn / ratioOut
						cropOffset = (1.0 - cropHeight) / 2.0
						cropHeight = int(cropHeight * height)
						cropOffset = int(cropOffset * height)
						image = image[cropOffset : cropOffset + cropHeight, :, :]
					image = cv2.resize(image, (Settings.imageWidth, Settings.imageHeight), interpolation = cv2.INTER_CUBIC)
				if Settings.flippedImage:
					image = cv2.flip(image, -1)
				print 'Running processor'
				processor.shownCount = 0
				processor.ProcessImage(image)
				if Globals.displayFrame != None:
					cv2.namedWindow('Processed', cv2.WINDOW_AUTOSIZE)
					if ImageProcessor.scaleFinalImage != 1.0:
						size = (int(Globals.displayFrame.shape[1] * ImageProcessor.scaleFinalImage), 
								int(Globals.displayFrame.shape[0] * ImageProcessor.scaleFinalImage))
						processed = cv2.resize(Globals.displayFrame, size, interpolation = cv2.INTER_CUBIC)
					else:
						processed = Globals.displayFrame
					cv2.imshow('Processed', processed)
					cv2.waitKey(autoDelay)
				AutoReloadSettings()
				if not Globals.running:
					# Race has been ended
					print '============ RACE OVER ==============='
					raw_input('Press ENTER to finish')
					break
	else:
		print 'Load the image'
		image = cv2.imread(fileIn)
		if (image.shape[1] != Settings.imageWidth) or (image.shape[0] != Settings.imageHeight):
			ratioIn = image.shape[1] / float(image.shape[0])
			ratioOut = Settings.imageWidth / float(Settings.imageHeight)
			if abs(ratioIn - ratioOut) < 0.01:
				# Straight resize
				pass
			elif ratioIn > ratioOut:
				# Crop width
				width = image.shape[1]
				cropWidth = ratioOut / ratioIn
				cropOffset = (1.0 - cropWidth) / 2.0
				cropWidth = int(cropWidth * width)
				cropOffset = int(cropOffset * width)
				image = image[:, cropOffset : cropOffset + cropWidth, :]
			else:
				# Crop height
				height = image.shape[0]
				cropHeight = ratioIn / ratioOut
				cropOffset = (1.0 - cropHeight) / 2.0
				cropHeight = int(cropHeight * height)
				cropOffset = int(cropOffset * height)
				image = image[cropOffset : cropOffset + cropHeight, :, :]
			image = cv2.resize(image, (Settings.imageWidth, Settings.imageHeight), interpolation = cv2.INTER_CUBIC)
		if Settings.flippedImage:
			image = cv2.flip(image, -1)
		
		print 'Running processor'
		processor.ProcessImage(image)
		if Globals.displayFrame != None:
			cv2.namedWindow('Processed', cv2.WINDOW_AUTOSIZE)
			if ImageProcessor.scaleFinalImage != 1.0:
				size = (int(Globals.displayFrame.shape[1] * ImageProcessor.scaleFinalImage), 
						int(Globals.displayFrame.shape[0] * ImageProcessor.scaleFinalImage))
				processed = cv2.resize(Globals.displayFrame, size, interpolation = cv2.INTER_CUBIC)
			else:
				processed = Globals.displayFrame
			cv2.imshow('Processed', processed)
			cv2.waitKey(0)
except KeyboardInterrupt:
	# CTRL+C exit, disable all drives
	print '\nUser shutdown'

# Tell each thread to stop, and wait for them to end
Globals.running = False
Globals.controller.terminated = True
raceThread.terminated = True
Globals.controller.join()
raceThread.join()
print 'Program terminated.'
