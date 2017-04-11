#!/usr/bin/env python
# coding: Latin-1

# Load library functions we want
import time
import datetime
import threading
import cv2
import numpy
import math
import random
import Globals

# Mode values
READY_TO_RACE = 1
WAIT_FOR_LIGHTS = 2
FOLLOW_TRACK = 3
CRASHED = 4
FLIPPED = 5
WRONG_WAY = 6
RACE_OVER = 7
FIRST_STRAIGHT = 8

# Light sequence
READY_OFF = 1
FIRST_GREEN = 2
SECOND_RED = 3
THIRD_GREEN_GO = 4

# Logging levels
LOG_CRITICAL = 1
LOG_MAJOR = 2
LOG_MINOR = 3

# Mappings for output
lineIndexToName = {
	0 : 'wall | red',
	1 : 'red | blue',
	2 : 'blue | red',
	3 : 'red | green',
	4 : 'green | blue',
	5 : 'blue | green',
	6 : 'green | wall'
}
lineIndexToColour = {
	0 : (255,   0,   0),
	1 : (255,   0, 127),
	2 : (127,   0, 255),
	3 : (255, 255,   0),
	4 : (  0, 127, 255),
	5 : (  0, 255, 127),
	6 : (  0, 255,   0),
	7 : (  0,   0,   0)
}
lineIndexToOffset = {
	0 : +3.0,
	1 : +2.0,
	2 : +1.0,
	3 :  0.0,
	4 : -1.0,
	5 : -2.0,
	6 : -3.0
}

def rgb2bgr((r, g, b)):
	return b, g, r

def SetImageMode(newMode):
	Globals.lastImageMode = Globals.imageMode
	Globals.imageMode = newMode

def FullTimeStamp():
	stamp = datetime.datetime.now()
	return stamp.strftime('%Y %m %d - %H-%M-%S')

def TimeOnlyStamp():
	stamp = datetime.datetime.now()
	return stamp.strftime('%H:%M:%S.%f')

def LogData(logLevel, logString):
	logLine = '%s %s\n' % (TimeOnlyStamp(), logString)
	if logLevel <= Globals.processingPrintLogLevel:
		print logLine
	if Globals.processingLogFile:
		if logLevel <= Globals.processingWriteLogLevel:
			Globals.processingLogFile.write(logLine)

# PID processing thread
class ControlLoop(threading.Thread):
	def __init__(self, autoRun = True):
		super(ControlLoop, self).__init__()
		self.event = threading.Event()
		self.lock = threading.Lock()
		self.terminated = False
		self.eventWait = 2.0 / Settings.frameRate
		self.userTargetLane = 0.0
		self.lastStartMarker = time.time()
		self.Reset()
		if autoRun:
			LogData(LOG_CRITICAL, 'Control loop thread started with idle time of %.2fs' % (self.eventWait))
			self.start()
		else:
			LogData(LOG_CRITICAL, 'Control loop loaded and waiting for commands')

	def run(self):
		# This method runs in a separate thread
		while not self.terminated:
			# Wait for an image to be written to the stream
			self.event.wait(self.eventWait)
			if self.event.isSet():
				if self.terminated:
					break
				try:
					# Read the next set of values
					sample = self.nextSample
					self.RunLoop(sample)
					self.lastSample = sample
				finally:
					# Reset the event trigger
					self.event.clear()
		LogData(LOG_CRITICAL, 'Control loop thread terminated')

	def Reset(self):
		with self.lock:
			self.__Reset__()

	def __Reset__(self):
		# Set everything to a clean starting state
		self.moving = False
		self.i0 = 0.0
		self.i1 = 0.0
		self.i2 = 0.0
		self.lastD0 = 0.0
		self.lastD1 = 0.0
		self.lastD2 = 0.0
		self.clipMax = Settings.clipI
		self.clipMin = -Settings.clipI
		self.lastSample = (0.0, 0.0, 0.0, 0.0)
		self.firTaps = Settings.firTaps
		self.firHistorySpeed = []
		self.firHistorySteering = []
		self.autoTargetLane = 0.0
		self.stuckFrameCount = 0
		self.overrideStuckTicks = 0
		self.firstStraightTicks = None
		self.wrongWayCount = 0
		self.wrongWayTicks = 0
		self.huntColours = (False, False, False)
		self.huntLeft = False
		self.flippedImageCount = 0
		self.stuckSpeed = -1.0
		self.overtaking = False
		self.overtakeRemainingTicks = 0
		self.overtakeBrakingTicks = 0
		self.unknownPointCount = 0
		self.unknownPointAverage = 0.5
		self.lastSpeed = 0.0
		self.lastSteering = 0.0
		self.accumulateDistance = False
		self.SetDrive(0.0, 0.0)
	
	def SetDrive(self, speed, steering):
		# Make sure speed and steering are within limits
		if steering < -1.0:
			steering = -1.0
		elif steering > 1.0:
			steering = 1.0
		if speed < -1.0:
			speed = -1.0
		elif speed > 1.0:
			speed = 1.0
		# Final steering corrections
		steering *= Settings.steeringGain
		steering += Settings.steeringOffset
		if steering < -Settings.steeringClip:
			steering = -Settings.steeringClip
		elif steering > Settings.steeringClip:
			steering = Settings.steeringClip
		# Determine the individual drive power levels
		driveLeft  = speed
		driveRight = speed
		if steering < -0.01:
			# Turning left
			driveLeft *= 1.0 + steering
		elif steering > 0.01:
			# Turning right
			driveRight *= 1.0 - steering
		# Set the motors to the new speeds
		Globals.MonsterMotors(driveLeft, driveRight)
		if (Globals.frameAnnounce == 0):
			LogData(LOG_MINOR, 'Motors: %+.3f, %+.3f' % (driveLeft, driveRight))
		# Calculate the travelled distance between the last two frames
		self.IncreaseDistance()
		self.lastSpeed = speed
		self.lastSteering = steering
	
	def IncreaseDistance(self):
		if self.accumulateDistance and (self.lastSpeed != 0):
			# Work out motor speed based on steering and power applied
			turningSpeed = abs(self.lastSteering)
			straightSpeed = 1.0 - turningSpeed
			calculatedSpeed = (turningSpeed * Settings.monsterSpeedFullSteering)
			calculatedSpeed += (straightSpeed * Settings.monsterSpeed)
			if calculatedSpeed < 0.0:
				calculatedSpeed = 0.0
			calculatedSpeed *= self.lastSpeed
			# Work out the distance along the track axis based on the angle of travel
			hyp = calculatedSpeed * Settings.monsterDistancePerFrame
			opp = self.changeD0 * Settings.laneWidth
			hyp2 = hyp ** 2
			opp2 = opp **2
			if hyp2 >= opp2:
				adj = math.sqrt(hyp2 - opp2)
			else:
				# Offset change is too sharp, larger than the speed!
				# This usually indicates being knocked or processing errors
				adj = 0.0
			# Work out the speed correction for the 'lane' over the last couple of frames
			laneLength = (self.distanceD0 * Settings.trackChangePerLane) + Settings.trackLengthCenter
			laneSpeedCorrection = Settings.trackLengthCenter / laneLength
			# Apply the speed correction to the calculated distance along the track
			trackDistanceTravelled = adj * laneSpeedCorrection
			# Add the distance travelled to the distance so far
			Globals.lapTravelled += trackDistanceTravelled

	def FirFilter(self, speed, steering):
		# Filtering for speed and steering
		self.firHistorySpeed.append(speed)
		self.firHistorySteering.append(steering)
		self.firHistorySpeed = self.firHistorySpeed[-self.firTaps:]
		self.firHistorySteering = self.firHistorySteering[-self.firTaps:]
		filteredSpeed = numpy.mean(self.firHistorySpeed)
		filteredSteering = numpy.mean(self.firHistorySteering)
		# Run any override conditions
		filteredSpeed, filteredSteering = self.PerformOverrides(filteredSpeed, filteredSteering)
		self.SetDrive(filteredSpeed, filteredSteering)

	def PerformOverrides(self, filteredSpeed, filteredSteering):
		# Default to keeping the filtered values
		overrideSpeed = filteredSpeed
		overrideSteering = filteredSteering
		checkForOverrides = False
		calculateDistance = True
		# Check for existing override conditions
		if Globals.imageMode == READY_TO_RACE:
			# Waiting for commands, set motors off
			overrideSpeed = 0.0
		elif Globals.imageMode == WAIT_FOR_LIGHTS:
			# Waiting for the lights sequence, set motors off
			overrideSpeed = 0.0
		elif Globals.imageMode == FOLLOW_TRACK:
			# Normal driving, proceed to standard code
			checkForOverrides = True
		elif Globals.imageMode == CRASHED:
			# Running the reversing procedure for being stuck or lost
			calculateDistance = False
			if self.overrideStuckTicks < Settings.stuckOverrideFrames:
				# Keep reversing at full speed
				overrideSteering = 0.0
				overrideSpeed = self.stuckSpeed
				# Check for left or right hunt
				if self.huntColours[0] > 0:
					# Red, turn left
					self.huntLeft = True
				elif self.huntColours[1] > 0:
					# Green, turn left
					self.huntLeft = False
				self.overrideStuckTicks += 1
			elif self.overrideStuckTicks < (Settings.stuckOverrideFrames + Settings.stuckHuntFrames):
				# Move forward at an angle to try and re-catch the track
				overrideSpeed = -self.stuckSpeed
				if self.huntLeft:
					overrideSteering = +1.0
				else:
					overrideSteering = -1.0
				if overrideSpeed < 0.0:
					overrideSteering *= -1.0
				self.overrideStuckTicks += 1
			else:
				# We have finished hunting, stop and reset
				LogData(LOG_MAJOR, '< END OVERRIDE: STUCK  >')
				overrideSpeed = 0.0
				self.__Reset__()
				SetImageMode(Globals.lastImageMode)
		elif Globals.imageMode == FLIPPED:
			# Flipped over, invert control
			overrideSpeed = -overrideSpeed
			overrideSteering = -overrideSteering
			checkForOverrides = True
		elif Globals.imageMode == WRONG_WAY:
			# Driving the wrong way, we need to spin around
			calculateDistance = False
			if self.wrongWayTicks < Settings.wrongWaySpinFrames:
				overrideSpeed = 1.0
				overrideSteering = +1.0
				self.wrongWayTicks += 1
			else:
				LogData(LOG_MAJOR, '< END OVERRIDE: WRONG WAY >')
				self.wrongWayCount = 0
				overrideSpeed = 0.0
				self.__Reset__()
				SetImageMode(Globals.lastImageMode)
			pass
		elif Globals.imageMode == RACE_OVER:
			# Done racing, set motors off
			overrideSpeed = 0.0
		elif Globals.imageMode == FIRST_STRAIGHT:
			# Initial straight, full speed forward :)
			if self.firstStraightTicks == None:
				# Pick a time between the limits
				self.firstStraightTicks = random.random() * Settings.frameRate
				self.firstStraightTicks *= (Settings.firstStraightMax - Settings.firstStraightMin)
				self.firstStraightTicks += Settings.firstStraightMin
			if self.firstStraightTicks > 0:
				# Override control with straight forward
				overrideSpeed = Globals.userSpeed
				overrideSteering = 0.0
				self.firstStraightTicks -= 1
			else:
				# Completed our time, return to normal control
				SetImageMode(FOLLOW_TRACK)
		if checkForOverrides:
			# Check for new override conditions
			if self.stuckFrameCount >= Settings.stuckIdenticalFrames:
				# We are stuck or lost, engage the stuck override
				LogData(LOG_MAJOR, '< START OVERRIDE: STUCK >')
				calculateDistance = False
				Globals.lapTravelled -= Settings.stuckDetectedDistanceCorrection
				self.overrideStuckTicks = 0
				if Globals.imageMode == FLIPPED:
					self.stuckSpeed = +1.0
				else:
					self.stuckSpeed = -1.0
				SetImageMode(CRASHED)
				overrideSpeed, overrideSteering = self.PerformOverrides(overrideSpeed, overrideSteering)
			elif self.wrongWayCount > Settings.wrongWayThreshold:
				# We seem to be facing the wrong way
				LogData(LOG_MAJOR, '< START OVERRIDE: WRONG WAY >')
				calculateDistance = False
				self.wrongWayTicks = 0
				SetImageMode(WRONG_WAY)
				overrideSpeed = 0.0
				overrideSteering = 0.0
			elif (not self.overtaking) and (self.unknownPointCount >= Settings.overtakeThreshold):
				# There may be a car in front, try and overtake
				LogData(LOG_MAJOR, '< START OVERRIDE: OVERTAKE >')
				if self.unknownPointAverage < 0.5:
					# Robot to the left, overtake to the right
					self.autoTargetLane = -Settings.overtakeLaneOffset
				else:
					# Robot to the right, overtake to the left
					self.autoTargetLane = +Settings.overtakeLaneOffset
				self.overtakeRemainingTicks = Settings.overtakeDurationFrames
				self.overtakeBrakingTicks = Settings.overtakeBrakingFrames
				self.overtaking = True
				LogData(LOG_MAJOR, 'Overtaking at lane offset %.2f' % (self.autoTargetLane))
		if self.overtaking:
			# Count down the remaining overtake distance
			self.overtakeRemainingTicks -= 1
			self.overtakeBrakingTicks -= 1
			if self.overtakeRemainingTicks < 0:
				# Overtaking complete, reset the target lane for the next loop
				LogData(LOG_MAJOR, '< END OVERRIDE: OVERTAKE >')
				self.autoTargetLane = 0.0
				self.overtaking = False
			elif self.overtakeBrakingTicks >= 0:
				# Initial part of the overtake, apply braking
				overrideSpeed *= Settings.overtakeBrakingSpeed
			print overrideSpeed
		self.accumulateDistance = calculateDistance
		return overrideSpeed, overrideSteering

	def RunLoop(self, (line0, local0, d1, d2)):
		with self.lock:
			# Select the correct d0 depending on nearest line or track position
			full0 = line0 + local0
			d0 = full0 # red / green line
			d0 = d0 - (Settings.targetTrackPosition + Globals.userTargetLane + self.autoTargetLane)
			# Track offset loop (d0)
			self.p0 = Settings.Kp0 * d0
			self.i0 += Settings.Ki0 * d0
			if self.i0 > self.clipMax:
				self.i0 = self.clipMax
			elif self.i0 < self.clipMin:
				self.i0 = self.clipMin
			self.d0 = Settings.Kd0 * (d0 - self.lastD0)
			self.pid0 = self.p0 + self.i0 + self.d0
			if self.pid0 > self.clipMax:
				self.pid0 = self.clipMax
			elif self.pid0 < self.clipMin:
				self.pid0 = self.clipMin
			# Track angle loop (d1)
			self.p1 = Settings.Kp1 * d1
			self.i1 += Settings.Ki1 * d1
			if self.i1 > self.clipMax:
				self.i1 = self.clipMax
			elif self.i1 < self.clipMin:
				self.i1 = self.clipMin
			self.d1 = Settings.Kd1 * (d1 - self.lastD1)
			self.pid1 = self.p1 + self.i1 + self.d1
			if self.pid1 > self.clipMax:
				self.pid1 = self.clipMax
			elif self.pid1 < self.clipMin:
				self.pid1 = self.clipMin
			# Track curvature loop (d2)
			self.p2 = Settings.Kp2 * d2
			self.i2 += Settings.Ki2 * d2
			if self.i2 > self.clipMax:
				self.i2 = self.clipMax
			elif self.i2 < self.clipMin:
				self.i2 = self.clipMin
			self.d2 = Settings.Kd2 * (d2 - self.lastD2)
			self.pid2 = self.p2 + self.i2 + self.d2
			if self.pid2 > self.clipMax:
				self.pid2 = self.clipMax
			elif self.pid2 < self.clipMin:
				self.pid2 = self.clipMin
			# Speed setting
			if self.moving:
				speed = Globals.userSpeed
			else:
				speed = 0.0
			# Note the average and change in d0 for the last two frames for distance calculations
			self.distanceD0 = (self.lastD0 + d0) / 2.0
			self.changeD0 = d0 - self.lastD0
			# Note the old values
			self.lastD0 = d0
			self.lastD1 = d1
			self.lastD2 = d2
			# Set the final drive
			steering = self.pid0 + self.pid1 + self.pid2
			self.FirFilter(speed, steering)
	

# Image stream processing thread
class StreamProcessor(threading.Thread):
	def __init__(self, name, autoRun = True):
		super(StreamProcessor, self).__init__()
		self.event = threading.Event()
		self.terminated = False
		self.name = str(name)
		self.shownCount = 0
		self.lastFrame = None
		self.lastLightsFrame = None
		self.olderLightsFrame = None
		self.burnCount = Settings.lightsBurnFrames
		if self.burnCount < 2:
			self.burnCount = 2
		self.eventWait = (2.0 * Settings.processingThreads) / Settings.frameRate
		if autoRun:
			LogData(LOG_CRITICAL, 'Processor thread %s started with idle time of %.2fs' % (self.name, self.eventWait))
			self.start()
		else:
			LogData(LOG_CRITICAL, 'Processor thread %s loaded and waiting for instructions' % (self.name))

	def run(self):
		# This method runs in a separate thread
		while not self.terminated:
			# Wait for an image to be written to the stream
			self.event.wait(self.eventWait)
			if self.event.isSet():
				if self.terminated:
					break
				try:
					# grab the image and do some processing on it
					if Settings.flippedImage and Settings.horizontalFlip:
						image = cv2.flip(self.nextFrame, 0)
					elif Settings.horizontalFlip:
						image = cv2.flip(self.nextFrame, 1)
					elif Settings.flippedImage:
						image = cv2.flip(self.nextFrame, -1)
					else:
						image = self.nextFrame
					self.ProcessImage(image)
				finally:
					# Reset the event
					self.nextFrame = None
					self.event.clear()
					# Return ourselves to the pool at the back
					with Globals.frameLock:
						Globals.processorPool.insert(0, self)
		LogData(LOG_CRITICAL, 'Processor thread %s terminated' % (self.name))
	
	# Helper function to show images
	def ShowImage(self, name, image):
		self.shownCount += 1
		name = '%d - %s' % (self.shownCount, name)
		if scaleDebugImage != 1.0:
			size = (int(image.shape[1] * scaleDebugImage), int(image.shape[0] * scaleDebugImage))
			image = cv2.resize(image, size, interpolation = cv2.INTER_CUBIC)
		cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
		cv2.imshow(name, image)
		cv2.waitKey(1)
	
	# Helper function for plotting
	def DrawCross(self, image, x, y, (r, g, b)):
		points = [(x,y), (x-1,y), (x+1,y), (x,y-1), (x,y+1)]
		for point in points:
			x = point[0]
			y = point[1]
			if (x >= 0) and (y >= 0) and (x < Settings.imageWidth) and (y < Settings.imageHeight):
				if b != None:
					image.itemset((y, x, 0), b)
				if g != None:
					image.itemset((y, x, 1), g)
				if r != None:
					image.itemset((y, x, 2), r)
	
	# Helper function for plotting
	def DrawPoints(self, image, points, xOffset, yOffset, (r, g, b)):
		for point in points:
			x = point[0] + xOffset
			y = point[1] + yOffset
			self.DrawCross(image, x, y, (r, g, b))

	# Find edges in a boolean image
	def SweepLine(self, image, Y):
		risingX = []
		fallingX = []
		line = image[Y, :]
		width = len(line)
		changed = numpy.where(line[:-1] != line[1:])[0]
		current = line.item(0)
		for i in changed:
			if current:
				typeX = fallingX
			else:
				typeX = risingX
			# Filter out changes at the edge of the image
			if i < 2:
				pass
			elif i > (width - 3):
				pass
			else:
				typeX.append(i)
			current = not current
		return risingX, fallingX

	# Remove matches from the non-target lists
	def EliminateMatches(self, Y, valsTarget, valsA, valsB):
		for Xt in valsTarget:
			if valsA:
				for Xa in valsA:
					newDistance = abs(Xt - Xa)
					if newDistance <= Settings.maxSepX:
						valsA.remove(Xa)
			if valsB:
				for Xb in valsB:
					newDistance = abs(Xt - Xb)
					if newDistance <= Settings.maxSepX:
						valsB.remove(Xb)

	# Find matches
	def FindMatches(self, Y, valsTarget, valsA, valsB, valsC, matchNone, matchA, matchB, matchC):
		while len(valsTarget) > 0:
			Xt = valsTarget.pop()
			if valsA:
				matchDistance = 999999
				matchedX = None
				for Xa in valsA:
					newDistance = abs(Xt - Xa)
					if newDistance < matchDistance:
						matchDistance = newDistance
						matchedX = Xa
				if matchDistance <= Settings.maxSepX:
					X = (Xt + matchedX) / 2
					matchA.append((X, Y))
					valsA.remove(matchedX)
					continue
			if valsB:
				matchDistance = 999999
				matchedX = None
				for Xb in valsB:
					newDistance = abs(Xt - Xb)
					if newDistance < matchDistance:
						matchDistance = newDistance
						matchedX = Xb
				if matchDistance <= Settings.maxSepX:
					X = (Xt + matchedX) / 2
					matchB.append((X, Y))
					valsB.remove(matchedX)
					continue
			if valsC:
				matchDistance = 999999
				matchedX = None
				for Xc in valsC:
					newDistance = abs(Xt - Xc)
					if newDistance < matchDistance:
						matchDistance = newDistance
						matchedX = Xc
				if matchDistance <= Settings.maxSepX:
					X = (Xt + matchedX) / 2
					matchC.append((X, Y))
					valsC.remove(matchedX)
					continue
			matchNone.append((Xt, Y))
	
	# Image processing function
	def ProcessImage(self, image):
		# Frame rate counter
		with Globals.frameLock:
			Globals.lastRawFrame = image
			self.frame = Globals.frameCounter
			Globals.frameAnnounce += 1
			Globals.frameCounter += 1
			if Globals.frameAnnounce == Settings.fpsInterval:
				frameStamp = time.time()
				if showFps:
					fps = Settings.fpsInterval / (frameStamp - Globals.lastFrameStamp)
					fps = '%.1f FPS' % (fps)
					print fps
				saveImages = writeImages
				showAll = debugImages
				saveAll = saveImages and debugImages
				saveRaw = writeRawImages
				Globals.frameAnnounce = 0
				Globals.lastFrameStamp = frameStamp
			else:
				saveImages = False
				showAll = False
				saveAll = False
				saveRaw = False
		if showAll:
			self.ShowImage('raw', image)
		if saveAll or saveRaw:
			cv2.imwrite(filePattern % (self.frame, 'raw'), image)
		# See what mode we are in
		checkStuck = False
		checkForStart = False
		if Globals.imageMode == READY_TO_RACE:
			# Waiting for commands, process to determine grid position
			pass
		elif Globals.imageMode == WAIT_FOR_LIGHTS:
			# Waiting for the lights sequence
			# Grab the lights section
			lightsFrame = image[Settings.lightsY1 : Settings.lightsY2, Settings.lightsX1 : Settings.lightsX2]
			if showProcessing:
				Globals.displayFrame = lightsFrame
			# Get a difference from and to the new frame verses the frame two previous
			if self.burnCount > 0:
				self.burnCount -= 1
				diffLights = numpy.zeros_like(lightsFrame)
				diff2Lights = numpy.zeros_like(lightsFrame)
			else:
				diffLights = cv2.subtract(lightsFrame, self.olderLightsFrame)
				diff2Lights = cv2.subtract(self.olderLightsFrame, lightsFrame)
			# Maintain a history of two previous frames
			self.olderLightsFrame = self.lastLightsFrame
			self.lastLightsFrame = lightsFrame
			# Split out the three channels
			diffBlue, diffGreen, diffRed = cv2.split(diffLights)
			diff2Blue, diff2Green, diff2Red = cv2.split(diff2Lights)
			redLevel   = diffRed.mean() * Settings.lightsRedGain
			greenLevel = diffGreen.mean()
			red2Level   = diff2Red.mean() * Settings.lightsRedGain
			green2Level = diff2Green.mean()
			# Work out on and off
			redOnLevel = redLevel - red2Level
			greenOnLevel = greenLevel - green2Level
			redOffLevel = red2Level - redLevel
			greenOffLevel = green2Level - greenLevel
			# Check the levels
			if (redOnLevel > Settings.lightsChangeThreshold) or (greenOnLevel > Settings.lightsChangeThreshold):
				if redOnLevel > greenOnLevel:
					lightOn = 'R'
				else:
					lightOn = 'G'
			else:
				lightOn = None
			if (redOffLevel > Settings.lightsChangeThreshold) or (greenOffLevel > Settings.lightsChangeThreshold):
				if redOffLevel > greenOffLevel:
					lightOff = 'R'
				else:
					lightOff = 'G'
			else:
				lightOff = None
			# State machine for lights
			if Globals.startLights == READY_OFF:
				# Move on if we changed to green
				if lightOn == 'G':
					Globals.startLights = FIRST_GREEN
					LogData(LOG_MAJOR, 'Lights: 1 - Green')
			elif Globals.startLights == FIRST_GREEN:
				# Move on if we changed to red
				if lightOn == 'R': #and lightOff == 'G':
					Globals.startLights = SECOND_RED
					LogData(LOG_MAJOR, 'Lights: 2 - Red')
			elif Globals.startLights == SECOND_RED:
				# Move on if we changed to green
				if lightOn == 'G': #and lightOff == 'R':
					Globals.startLights = THIRD_GREEN_GO
					LogData(LOG_MAJOR, 'Lights: 3 - Green')
			elif Globals.startLights == THIRD_GREEN_GO:
				# Ready, start racing
				LogData(LOG_CRITICAL, 'Lights: GO')
				self.lastLightsFrame = None
				self.olderLightsFrame = None
				self.burnCount = Settings.lightsBurnFrames
				if self.burnCount < 2:
					self.burnCount = 2
				if Settings.firstStraightOverride:
					SetImageMode(FIRST_STRAIGHT)
				else:
					SetImageMode(FOLLOW_TRACK)
				Globals.controller.lastStartMarker = time.time()
			else:
				LogData(LOG_CRITICAL, '! BAD LIGHTS STATE !')
				Globals.startLights = READY_OFF
			return
		elif Globals.imageMode == FOLLOW_TRACK:
			# Normal driving, proceed to standard code
			checkStuck = True
			checkForStart = True
		elif Globals.imageMode == CRASHED:
			# Crash detected, keep processing normally as this is handled in the control loop
			if Globals.lastImageMode == FLIPPED:
				# Flipped over, invert camera
				image = cv2.flip(image, -1)
		elif Globals.imageMode == FLIPPED:
			# Flipped over, invert camera
			checkStuck = True
			checkForStart = True
			image = cv2.flip(image, -1)
		elif Globals.imageMode == WRONG_WAY:
			# Driving the wrong way, keep processing normally as this is handled in the control loop
			checkStuck = True
			if Globals.lastImageMode == FLIPPED:
				# Flipped over, invert camera
				image = cv2.flip(image, -1)
		elif Globals.imageMode == RACE_OVER:
			# Done racing, finish the processing loop here
			Globals.running = False
			return
		elif Globals.imageMode == FIRST_STRAIGHT:
			# First straight dash, keep processing normally as this is handled in the control loop
			# We do not want to check for the start line here, it may confuse things!
			pass
		else:
			# Unexpected mode!!!
			LogData(LOG_CRITICAL, '! BAD IMAGE STATE !')
			SetImageMode(READY_TO_RACE)

		# Quick check for a flipped robot
		bAll, gAll, rAll = cv2.split(image)
		maxImage = numpy.maximum(numpy.maximum(bAll, gAll), rAll)
		topCrop = maxImage[Settings.flipDetectionY1 : Settings.flipDetectionY2, :]
		bottomCrop = maxImage[Settings.flipDetectionY3 : Settings.flipDetectionY4, :]
		bottomVar = bottomCrop.var()
		topVar = topCrop.var()
		if topVar > 0.0:
			flippedDetectionLevel = bottomVar / topVar
		else:
			flippedDetectionLevel = 0.0
		if flippedDetectionLevel > Settings.flipDetectionThreshold:
			# We may have a flipped image
			Globals.controller.flippedImageCount += 1
			if Globals.controller.flippedImageCount > Settings.flipDetectionFrames:
				# We think this is a flipped image!
				if Globals.imageMode == FOLLOW_TRACK:
					# In normal mode, swap to flipped
					LogData(LOG_MAJOR, '< START OVERRIDE: FLIPPED >')
					SetImageMode(FLIPPED)
					Globals.controller.flippedImageCount = 0
					image = cv2.flip(image, -1)
				elif Globals.imageMode == FLIPPED:
					# In flipped mode, swap back to normal
					LogData(LOG_MAJOR, '< END OVERRIDE: FLIPPED >')
					SetImageMode(FOLLOW_TRACK)
					Globals.controller.flippedImageCount = 0
					image = cv2.flip(image, -1)
				else:
					# Busy doing some other override, let this pass for now
					pass
		else:
			# We think this image is fine, reset the counter
			Globals.controller.flippedImageCount = 0
		# Crop the frame
		cropped = image[Settings.cropY1 : Settings.cropY2, Settings.cropX1 : Settings.cropX2, :]
		# Check if the frame is not changing much (we are stuck)
		if checkStuck:
			if self.lastFrame != None:
				differences = cv2.absdiff(cropped, self.lastFrame)
				differenceLevel = numpy.mean(differences)
				if differenceLevel < Settings.stuckIdenticalThreshold:
					# Stuck frame
					Globals.controller.stuckFrameCount += 1
				else:
					# Not a stuck frame
					Globals.controller.stuckFrameCount = 0
		else:
			# Checking off, reset counter
			Globals.controller.stuckFrameCount = 0
		self.lastFrame = cropped
		# Auto-brighten
		maximum = numpy.max(cropped)
		adjustment = 255.0 / maximum
		if adjustment > Settings.autoGainMax:
			adjustment = Settings.autoGainMax
		corrected = cropped * adjustment
		corrected = numpy.clip(corrected, 0, 255)
		corrected = numpy.array(corrected, dtype = numpy.uint8)
		if showAll:
			self.ShowImage('corrected', corrected)
		if saveAll:
			cv2.imwrite(filePattern % (self.frame, 'corrected'), corrected)
		# Find the dark portions for exclusion
		black = cv2.inRange(corrected, numpy.array((0, 0, 0)),
				            numpy.array((Settings.blackMaxB, Settings.blackMaxG, Settings.blackMaxR)))
		# Erode the black detection to remove noise
		if Settings.erodeChannels > 1:
			erodeKernel = numpy.ones((Settings.erodeChannels, Settings.erodeChannels), numpy.uint8)
			black = cv2.erode(black, erodeKernel)
		if showAll:
			self.ShowImage('black', black)
		if saveAll:
			cv2.imwrite(filePattern % (self.frame, 'black'), black)
		# Grab the section of the image used for the start line
		lineR = image[Settings.startY, Settings.startX1 : Settings.startX2, 2]
		lineG = image[Settings.startY, Settings.startX1 : Settings.startX2, 1]
		lineB = image[Settings.startY, Settings.startX1 : Settings.startX2, 0]
		# Check each pixel for a start marker colour match
		matchR = lineR > Settings.startMinR
		matchG = lineG < Settings.startMaxG
		matchB = lineB < Settings.startMaxB
		match = numpy.logical_and(numpy.logical_and(matchR, matchG), matchB)
		matchRatio = numpy.count_nonzero(match) / float(len(match))
		if matchRatio >= Settings.startRatioMin:
			startDetected = True
		else:
			startDetected = False
		if checkForStart:
			# Start line testing logic
			if Globals.startWaitCount > 0:
				# Countdown to announcement
				Globals.startWaitCount -= 1
				if Globals.startWaitCount <= 0:
					# Crossed line
					LogData(LOG_MAJOR, '--- START-LINE CROSSED ---')
					Globals.lapTravelled = 0.0
					Globals.lapCount += 1
			elif Globals.seenStart:
				# We have seen the start line, wait for it to move out of sight
				if not startDetected:
					LogData(LOG_MAJOR, '--- START-LINE OUT OF SIGHT ---')
					Globals.seenStart = False
			else:
				# We are still waiting to see the start line
				if startDetected:
					detectionTime = time.time()
					if (detectionTime - Globals.controller.lastStartMarker) < Settings.startRedetectionSeconds:
						LogData(LOG_MINOR, '--- START-LINE SEEN BUT TOO SOON ---')
					else:
						Globals.controller.lastStartMarker = detectionTime
						LogData(LOG_MAJOR, '--- START-LINE DETECTED ---')
						Globals.seenStart = True
						if Settings.startCrossedFrames < 1:
							# No need to wait, we have already crossed the line
							LogData(LOG_MAJOR, '--- START-LINE CROSSED ---')
							Globals.lapTravelled = 0.0
							Globals.lapCount += 1
						else:
							# Wait for a countdown before we announce crossing
							Globals.startWaitCount = Settings.startCrossedFrames
		# Convert to red, green, and blue sections
		blue, green, red = cv2.split(cropped)
		## Work out some automatic levels and gains based on the whole image
		dropR = rAll.min()
		dropG = gAll.min()
		dropB = bAll.min()
		autoGainR = Settings.targetLevel / rAll.max()
		autoGainG = Settings.targetLevel / gAll.max()
		autoGainB = Settings.targetLevel / bAll.max()
		# Apply gains to make the channels roughly equal
		red   = (red   - dropR) * Settings.redGain   * autoGainR
		green = (green - dropG) * Settings.greenGain * autoGainG
		blue  = (blue  - dropB) * Settings.blueGain  * autoGainB
		# Clamp the values to the standard range
		red   = numpy.clip(red,   0, 255)
		green = numpy.clip(green, 0, 255)
		blue  = numpy.clip(blue,  0, 255)
		red   = numpy.array(red,   dtype = numpy.uint8)
		green = numpy.array(green, dtype = numpy.uint8)
		blue  = numpy.array(blue,  dtype = numpy.uint8)
		# Remove any section where a different channel is stronger
		maxImage = numpy.maximum(numpy.maximum(blue, green), red)
		red  [red   < maxImage] = 0
		green[green < maxImage] = 0
		blue [blue  < maxImage] = 0
		exclude = black > 0
		red  [exclude] = 0
		green[exclude] = 0
		blue [exclude] = 0
		# Erode each channel to remove noise
		if Settings.erodeChannels > 1:
			red   = cv2.erode(red,   erodeKernel)
			green = cv2.erode(green, erodeKernel)
			blue  = cv2.erode(blue,  erodeKernel)
		# Extract the closest track colours for hunting override
		Globals.controller.huntColours = (
			red[Settings.stuckDetectColourY, Settings.stuckDetectColourX],
			green[Settings.stuckDetectColourY, Settings.stuckDetectColourX],
			blue[Settings.stuckDetectColourY, Settings.stuckDetectColourX]
		)
		# Display colours 
		if showAll or saveAll or predatorView:
			adjusted = cv2.merge([blue, green, red])
			walls = cv2.merge([black, black, black])
			adjusted = cv2.addWeighted(adjusted, 1.0, walls, 1.0, 0)
		if showAll:
			self.ShowImage('r', red)
			self.ShowImage('g', green)
			self.ShowImage('b', blue)
		if showAll:
			self.ShowImage('adjusted', adjusted)
		if predatorView:
			Globals.displayPredator = adjusted
		if saveAll:
			cv2.imwrite(filePattern % (self.frame, 'adjusted'), adjusted)
		# Find image contours and plot
		if showAll or saveAll:
			displayImage = image.copy()
			rContours, hierarchy = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
			gContours, hierarchy = cv2.findContours(green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
			bContours, hierarchy = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
			kContours, hierarchy = cv2.findContours(black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
			cv2.drawContours(displayImage, rContours, -1, (0,0,255), 1, offset = (Settings.cropX1, Settings.cropY1))
			cv2.drawContours(displayImage, gContours, -1, (0,255,0), 1, offset = (Settings.cropX1, Settings.cropY1))
			cv2.drawContours(displayImage, bContours, -1, (255,0,0), 1, offset = (Settings.cropX1, Settings.cropY1))
			cv2.drawContours(displayImage, kContours, -1, (0,0,0), 1, offset = (Settings.cropX1, Settings.cropY1))
		if showAll:
			self.ShowImage('contours', displayImage)
		if saveAll:
			cv2.imwrite(filePattern % (self.frame, 'contours'), displayImage)
		# Make thresholded arrays into boolean arrays
		red   = red   > 0
		green = green > 0
		blue  = blue  > 0
		black = black > 0
		# Find the edges
		krLine = []
		rbLine = []
		brLine = []
		rgLine = []
		gbLine = []
		bgLine = []
		gkLine = []
		others = []
		wrongWay = []
		for Y in Settings.croppedYScan:
			rRisingX, rFallingX = self.SweepLine(red, Y)
			gRisingX, gFallingX = self.SweepLine(green, Y)
			bRisingX, bFallingX = self.SweepLine(blue, Y)
			kRisingX, kFallingX = self.SweepLine(black, Y)
			# Match based on pairings
			self.FindMatches(Y, rRisingX,  bFallingX, kFallingX, gFallingX, others, brLine, krLine, wrongWay)
			self.FindMatches(Y, rFallingX, bRisingX,  gRisingX,	 None,      others, rbLine, rgLine, None)
			self.FindMatches(Y, gFallingX, bRisingX,  kRisingX,  None,      others, gbLine, gkLine, None)
			self.FindMatches(Y, bFallingX, gRisingX,  None,    	 None,      others, bgLine, None,   None)
			for values in [rRisingX, rFallingX, 
					       gRisingX, gFallingX,
						   bRisingX, bFallingX,
						   kRisingX, kFallingX]:
				for X in values:
					others.append((X, Y))
		lines = [krLine,
				 rbLine,
				 brLine,
				 rgLine,
				 gbLine,
				 bgLine,
				 gkLine]
		Globals.controller.wrongWayCount = len(wrongWay)
		if len(others) > 0:
			Globals.controller.unknownPointAverage = numpy.array(others)[:,0].mean() / (Settings.cropX2 - Settings.cropX1)
		else:
			Globals.controller.unknownPointAverage = 0.5
		Globals.controller.unknownPointCount = len(others)
		# Plot the lines
		buildDisplay = showProcessing or saveImages
		if buildDisplay:
			displayImage = image.copy()
			self.DrawPoints(displayImage, wrongWay, Settings.cropX1, Settings.cropY1, (255, 255, 255))
			for i in range(len(lines)):
				self.DrawPoints(displayImage, lines[i], Settings.cropX1, Settings.cropY1, lineIndexToColour[i])
			if showUnknownPoints:
				self.DrawPoints(displayImage, others, Settings.cropX1, Settings.cropY1, (0, 0, 0))
			if saveImages:
				cv2.imwrite(filePattern % (self.frame, 'final'), displayImage)
			self.SetSpeedFromLines(lines, True, displayImage)
		else:
			self.SetSpeedFromLines(lines, False, None)


	# Set the drive speed and steering from the detected lines
	def SetSpeedFromLines(self, lines, buildDisplay, displayImage):
		Globals.lastLines = lines
		# Defaults
		hasValues = False
		# Find the best line
		count = 0
		index = 0
		for i in range(len(lines)):
			if len(lines[i]) > count:
				index = i
				count = len(lines[i])
		if count < 3:
			# No valid lines to process
			status = 'No lines!'
			bestLine = []
		else:
			# Found our line,
			status = 'Using line %d (%s)' % (index, lineIndexToName[index])
			bestLine = lines[index]
			lineColour = rgb2bgr(lineIndexToColour[index])
			white = (255,255,255)
			dLine = lineIndexToOffset[index]
			# Scan for and remove any points with a duplicate Y (usually double-matched markings)
			newBestLine = []
			for i in range(len(bestLine)):
				if i == 0:
					newBestLine.append(bestLine[i])
				elif bestLine[i][1] - bestLine[i-1][1] == 0:
					pass
				else:
					newBestLine.append(bestLine[i])
			bestLine = newBestLine
		if len(bestLine) < 3:
			# Not enough points left to calculate derivatives
			status = 'No lines!'
		else:
			# calculate the derivatives and offset
			offsetIndex = 0
			offsetErrorY = abs(Settings.croppedTargetY - bestLine[0][1])
			dXdY = []
			for i in range(1, len(bestLine)):
				dX = float(bestLine[i-1][0] - bestLine[i][0])
				dY = float(bestLine[i][1] - bestLine[i-1][1])
				errorY = abs(Settings.croppedTargetY - bestLine[i][1])
				if errorY < offsetErrorY:
					offsetErrorY = errorY
					offsetIndex = i
				dXdY.append((dX, dY))
			offsetX = bestLine[offsetIndex][0]
			offsetY = bestLine[offsetIndex][1]
			# Calculate the offset from the line
			targetX = (Settings.cropX2 - Settings.cropX1) / 2.0
			d0 = (offsetX - targetX) / float(Settings.trackSepX)
			# Calculate the line gain correction
			adjustD1 = Settings.gainCorrection * d0
			# Calculate the second derivatives and the average derivative
			lastd = dXdY[0][0] / dXdY[0][1]
			d1 = lastd
			d2XdY2 = []
			for i in range(1, len(dXdY)):
				nextd = dXdY[i][0] / dXdY[i][1]
				d1 += nextd
				d2X = lastd - nextd
				d2Y = dXdY[i][1]
				d2XdY2.append((d2X, d2Y))
				lastd = nextd
			d1 /= len(dXdY)
			d1 += adjustD1
			# Calculate the average second derivative
			d2 = 0.0
			for change in d2XdY2:
				d2 += change[0] / change[1]
			d2 /= len(d2XdY2)
			status += '\n    line = %.3f, d0 = %.3f, d1 = %.3f, d2 = %.3f' % (dLine, d0, d1, d2)
			hasValues = True
			# Mark the offset and derivatives
			if buildDisplay:
				if d2 < 0:
					d2Colour = (0,0,255) # Red  - left bend
				else:
					d2Colour = (0,255,0) # Green - right bend
				offsetPoint = (offsetX + Settings.cropX1, offsetY + Settings.cropY1)
				centralPoint = (int(offsetX + Settings.cropX1 - (d0 * Settings.trackSepX)), int(offsetY + Settings.cropY1))
				dPoint = (int(centralPoint[0] + dPlotY * d1), int(centralPoint[1] - dPlotY))
				d2PointA = (int(Settings.imageWidth / 2 + dPlotY * d2), Settings.imageHeight - 10)
				d2PointB = (int(d2PointA[0] + dPlotY * dPlotY * d2), d2PointA[1])
				cv2.line(displayImage, offsetPoint, centralPoint, lineColour, 3) 
				cv2.line(displayImage, centralPoint, dPoint, lineColour, 3, lineType = cv2.CV_AA) 
				cv2.line(displayImage, d2PointA, d2PointB, d2Colour, 3, lineType = cv2.CV_AA) 
				cv2.line(displayImage, offsetPoint, centralPoint, white, 1) 
				cv2.line(displayImage, centralPoint, dPoint, white, 1, lineType = cv2.CV_AA) 
				cv2.line(displayImage, d2PointA, d2PointB, white, 1, lineType = cv2.CV_AA) 
				# Offset plot
				d0Sum = dLine + d0
				fat = Settings.thickLineScale
				thin = Settings.thinLineScale
				white = (255,255,255)
				black = (0,0,0)
				blue = (255,0,0)
				green = (0,255,0)
				red = (0,0,255)
				orange = (63,170,255)
				darkRed = (0,0,64)
				section = Settings.imageWidth / 8
				trackParts = [(0, black),
						 (1, red),
						 (2, blue),
						 (3, red),
						 (4, green),
						 (5, blue),
						 (6, green),
						 (7, black)]
				y = Settings.offsetAxis
				for line in trackParts:
					x1 = section * line[0]
					x2 = x1 + section
					cv2.line(displayImage, (x1, y), (x2, y), line[1], fat, lineType = cv2.CV_AA)
				y1 = y - Settings.offsetHeight
				y2 = y + Settings.offsetHeight
				x = int(section * (4 - d0Sum))
				cv2.line(displayImage, (x, y1), (x, y2), black, fat, lineType = cv2.CV_AA)
				cv2.line(displayImage, (x, y1), (x, y2), orange, thin, lineType = cv2.CV_AA)
				# Overtaking status
				font = cv2.FONT_HERSHEY_SIMPLEX
				if Globals.controller.overtaking:
					text = 'Overtaking: %+.1f' % (Globals.controller.autoTargetLane)
					cv2.putText(displayImage, text, (5, 20), font, 0.25, black, 1, cv2.CV_AA)
		# Display status
		if (Globals.frameAnnounce == 0):
			LogData(LOG_MINOR, status)
		# Build and display the final frame output
		if buildDisplay:
			Globals.displayFrame = displayImage
		# Pass to the control loop
		if hasValues:
			Globals.controller.moving = True
			Globals.controller.nextSample = (dLine, d0, d1, d2)
			Globals.badFrameCount = 0
			Globals.trackFound = True
		elif Globals.badFrameCount < Settings.maxBadFrames:
			# For now stick with the previous results
			Globals.controller.nextSample = Globals.controller.lastSample
			Globals.badFrameCount += 1
			Globals.trackFound = True
		else:
			# Too long between good frames, stop and wait
			Globals.controller.moving = False
			Globals.trackFound = False
		Globals.controller.event.set()
			

# Image capture thread
class ImageCapture(threading.Thread):
	def __init__(self):
		super(ImageCapture, self).__init__()
		self.start()

	# Stream delegation loop
	def run(self):
		while Globals.running:
			# Grab the oldest unused processor thread
			with Globals.frameLock:
				if Globals.processorPool:
					processor = Globals.processorPool.pop()
				else:
					processor = None
			if processor:
				# Grab the next frame and send it to the processor
				ret, frame = Globals.capture.read()
				if ret:
					processor.nextFrame = frame
					processor.event.set()
				else:
					LogData(LOG_CRITICAL, 'Capture stream lost...')
					Globals.running = False
					break
			else:
				# When the pool is starved we wait a while to allow a processor to finish
				time.sleep(0.01)
		LogData(LOG_CRITICAL, 'Streaming terminated.')

# Boot-up code
Globals.lastFrameStamp = time.time()
try:
	# get our assigned colour
	fileIn = open('/colour.txt', 'r')
	dataIn = fileIn.read()
	fileIn.close()
	dataIn = dataIn.split('\n')[0]
	dataIn.strip()
	dataIn = dataIn.split(',')
	if len(dataIn) == 3:
		# Attempt to extract the values
		print 'Team colour read'
		r = float(dataIn[0])
		g = float(dataIn[1])
		b = float(dataIn[2])
	else:
		# Badly formatted, use white to indicate a problem
		print 'Team colour formatted badly'
		r = 1
		g = 1
		b = 1
except:
	# Failed to access the file or data for some reason
	print 'Error reading team colour'
	r = 1
	g = 1
	b = 1
print 'Colour: %.2f, %.2f, %.2f' % (r, g, b)
Globals.colour = (r, g, b)
Globals.MonsterLed(r, g, b)

