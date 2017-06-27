#!/usr/bin/env python
# coding: Latin-1

#############################################################
# This is the functions script for the Formula Pi Race Code #
#                                                           #
# It is responsible for supplying the Race Code Functions   #
# available to Race.py during the race                      #
#############################################################

import inspect
import time
import os
import math
import cv2
import numpy
import Settings
import ImageProcessor
import Globals

RAD_TO_DEG = 180.0 / math.pi

def LogUserCall(paramFormat, paramTupple):
	calleeName = inspect.stack()[1][3] # Calling functions name
	if paramFormat:
		paramString = paramFormat % paramTupple
		logLine = '%s %s(%s)\n' % (ImageProcessor.TimeOnlyStamp(), calleeName, paramString)
	else:
		logLine = '%s %s()\n' % (ImageProcessor.TimeOnlyStamp(), calleeName)
	if Globals.userLogFile:
		Globals.userLogFile.write(logLine)
	print logLine

# Race Code Functions
def WaitForGo():
	LogUserCall(None, None)
	if Globals.running:
		ImageProcessor.SetImageMode(ImageProcessor.WAIT_FOR_LIGHTS)
	while Globals.running and (Globals.imageMode == ImageProcessor.WAIT_FOR_LIGHTS):
		time.sleep(Globals.pollDelay)

def FinishRace():
	LogUserCall(None, None)
	Globals.running = False

def WaitForSeconds(seconds):
	LogUserCall('%.3f', (seconds))
	time.sleep(seconds)

def Speed(speed):
	LogUserCall('%.3f', (speed))
	if speed > 100.0:
		speed = 100.0
	elif speed < 0.0:
		speed = 0.0
	Globals.userSpeed = float(speed) / 100.0

def AimForLane(position):
	LogUserCall('%.3f', (position))
	Globals.userTargetLane = float(position)

def WaitForWaypoint(pointNumber):
	LogUserCall('%d', (pointNumber))
	if Globals.running:
		if pointNumber < 1:
			# Not valid
			return
		elif pointNumber > len(Settings.distanceBetweenWaypoints):
			# Not valid
			return
		elif pointNumber == 1:
			# Start marker
			distance = 0.0			
		else:
			# Other waypoint, lookup distance from start
			index = pointNumber - 2
			distance = Settings.waypointDistances[index]
		# Wait for the distance to match / exceed the waypoint distance
		WaitForDistance(distance)

def WaitForDistance(distance):
	LogUserCall('%.3f', (distance))
	if Globals.running:
		lap = LapCount()
		# Check if we already passed the marker this lap, if so wait for the next one
		if (GetDistance() - distance) > Settings.waitForNextLapAfter:
			while lap == LapCount():
				time.sleep(Globals.pollDelay)
		# Wait until we reach or passed the correct distance
		while GetDistance() < distance:
			time.sleep(Globals.pollDelay)
			# If we complete a lap while waiting it means we messed up the distance :(
			if lap != LapCount():
				break

def LapCount():
	LogUserCall(None, None)
	if Globals.running:
		return Globals.lapCount
	else:
		return 99999

def GetDistance():
	LogUserCall(None, None)
	if Globals.running:
		return Globals.lapTravelled
	else:
		return 99999.9

def TrackFound():
	LogUserCall(None, None)
	return Globals.trackFound

def CurrentTrackPosition():
	LogUserCall(None, None)
	calculatedTrackValues = Globals.controller.lastSample
	return calculatedTrackValues[0] + calculatedTrackValues[1]

def CurrentAngle():
	LogUserCall(None, None)
	radians = math.atan(Globals.controller.lastD1 / Settings.angleCorrection)
	return radians * RAD_TO_DEG

def TrackCurve():
	LogUserCall(None, None)
	return Globals.controller.lastD2

def TrackLines():
	LogUserCall(None, None)
	return Globals.lastLines

def GetLatestImage():
	LogUserCall(None, None)
	return Globals.lastRawFrame

def PerformProcessingStage(stage, image):
	LogUserCall('%d, <image>', (stage))
	if stage < 1:
		return None
	elif stage > len(processingStages):
		return None
	else:
		func = processingStages[stage - 1]
		return func(image)
	
def SaveImage(image, name):
	LogUserCall('<image>, "%s"', (name))
	if not os.path.isdir('./logs'):
		os.mkdir('./logs')
	cv2.imwrite('./logs/%s.jpg' % (name), image)

def StartDetailedLoging():
	LogUserCall(None, None)
	if Globals.processingLogFile:
		Globals.processingLogFile.close()
		Globals.processingLogFile = None
	logPath = './logs/Processing %s.txt' % (ImageProcessor.FullTimeStamp())
	try:
		if not os.path.isdir('./logs'):
			os.mkdir('./logs')
		logFile = open(logPath, 'w')
		Globals.processingLogFile = logFile
	except:
		print 'Failed to start processing logging!'
	
def StartUserLog():
	if Globals.userLogFile:
		Globals.userLogFile.close()
		Globals.userLogFile = None
	logPath = './logs/User %s.txt' % (ImageProcessor.FullTimeStamp())
	try:
		if not os.path.isdir('./logs'):
			os.mkdir('./logs')
		logFile = open(logPath, 'w')
		Globals.userLogFile = logFile
	except:
		print 'Failed to start user logging!'
	LogUserCall(None, None)

def EndDetailedLog():
	LogUserCall(None, None)
	if Globals.processingLogFile:
		Globals.processingLogFile.close()
		Globals.processingLogFile = None
	
def EndUserLog():
	LogUserCall(None, None)
	if Globals.userLogFile:
		Globals.userLogFile.close()
		Globals.userLogFile = None

#############################################################
# Below are the image processing stages                     #
# They are replicas of the standard functionality and are   #
# called from the PerformProcessingStage fuction            #
#############################################################

# Stage 1 - Get maximum ranges from the image
def GetRanges(image):
	bAll, gAll, rAll = cv2.split(image)
	return rAll.min(), rAll.max(), gAll.min(), gAll.max(), bAll.min(), bAll.max(), 

# Stage 2 - Crop the image to the desired size
def CropFrame(image):
	cropped = image[Settings.cropY1 : Settings.cropY2, Settings.cropX1 : Settings.cropX2, :]
	return cropped

# Stage 3 - Automatic brightness adjustment
def AutoBrightness(image):
	maximum = numpy.max(image)
	adjustment = 255.0 / maximum
	if adjustment > Settings.autoGainMax:
		adjustment = Settings.autoGainMax
	corrected = image * adjustment
	corrected = numpy.clip(corrected, 0, 255)
	corrected = numpy.array(corrected, dtype = numpy.uint8)
	return corrected

# Stage 4 - Find the black parts of the image
def ExtractBlackMask(image):
	black = cv2.inRange(image, numpy.array((0, 0, 0)),
			            numpy.array((Settings.blackMaxB, Settings.blackMaxG, Settings.blackMaxR)))
	return black

# Stage 5 - Erode a channel
def ErodeChannel(channel):
	erodeKernel = numpy.ones((Settings.erodeChannels, Settings.erodeChannels), numpy.uint8)
	eroded = cv2.erode(channel, erodeKernel)
	return eroded

# Stage 6 - Split colour channels
def SplitRGB(image):
	blue, green, red = cv2.split(image)
	return red, green, blue

# Stage 7 - Auto level single channel based on settings
def AutoLevel((channel, minimum, maximum, gain)):
	autoGain = Settings.targetLevel / maximum
	adjusted = (channel - minimum) * gain * autoGain
	adjusted = numpy.clip(adjusted, 0, 255)
	adjusted = numpy.array(adjusted, dtype = numpy.uint8)
	return adjusted

# Stage 8 - Get maximums for each pixel
def MaxImage((red, green, blue)):
	return numpy.maximum(numpy.maximum(blue, green), red)

# Stage 9 - Exclude the unwanted ares on a channel
def ExcludeSections((channel, maxImage, black)):
	sectionOnly = channel[:]
	sectionOnly[sectionOnly < maxImage] = 0
	exclude = black > 0
	sectionOnly[exclude] = 0
	return sectionOnly

# Stage 10 - Build single view from channels
def BuildView((red, green, blue, black)):
	viewImage = cv2.merge([blue, green, red])
	walls = cv2.merge([black, black, black])
	viewImage = cv2.addWeighted(viewImage, 1.0, walls, 1.0, 0)
	return viewImage
	
# The stage list for processing images
processingStages = [
	GetRanges,
	CropFrame, 
	AutoBrightness, 
	ExtractBlackMask, 
	ErodeChannel,
	SplitRGB, 
	AutoLevel,
	MaxImage,
	ExcludeSections,
	BuildView
]

