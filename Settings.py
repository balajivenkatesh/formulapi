#!/usr/bin/env python
# coding: Latin-1

#######################################################################################
# These settings are looked at once at startup and should not be changed when running #
# Any runtime changes will either not work or have strange side effects               #
#######################################################################################

# Threading settings
processingThreads = 1				# Number of processing threads to work with

# Camera settings
imageWidth   = 160					# Camera image width
imageHeight  = 120					# Camera image height
frameRate    = 5					# Camera image capture frame rate
fpsInterval  = frameRate			# Number of frames to average FPS over
flippedImage = True					# True if the camera needs to be rotated

# Startup
startupMode = 1						# Startup mode, use 1 for Race.py calling WaitForGo
firstStraightOverride = True		# If True we will use FIRST_STRAIGHT mode after the start lights
									# This will move straight ahead for the intervals below
									# If False we will go to FOLLOW_TRACK directly instead
firstStraightMin = 2.0				# Minimum number of seconds straight forward
firstStraightMax = 4.0				# Maximum number of seconds straight forward

#######################################################################################
# These settings define the layout of the track and are used for working out how far  #
# the YetiBorg has travelled, they only affect the Race Code Functions                #
#######################################################################################

# YetiBorg movement details
yetiSpeed = 1.15					# Speed the YetiBorg moves at with 100% power in m/s
yetiSpeedFullSteering = 0.0			# Speed the YetiBorg moves at with 100% steering in m/s

# Adjustments used for the simulation
simulationDrivePower = 1.0			# Speed multiplier for the simulation mode
simulationSteeringGain = 0.50		# Override for the steering gain in simulation mode
simulationYetiSpeed = 0.78			# Override for the YetiBorg speed in simulation mode (used for distance calculations)
simulationLagFrames = 1				# Number of frames to delay the processing by to simulate camera lag

# Track dimensions
trackLengthCenter = 22.9			# Length of the center line in meters
trackLengthOuter = 28.6				# Length of the outer wall in meters
trackLengthInner = 17.1				# Length of the inner wall in meters
laneWidth = 0.305					# Width of the coloured lanes in meters
lanes = 6							# Number of coloured lanes

# Waypoint separations in meters
distanceBetweenWaypoints = [		# The first distance is from the starting waypoint, #1
		3.47, # Start to Turn 1
		3.00, # Turn 1
		2.00, # Straight 2
		1.65, # Turn 2
		2.95, # Turn 3
		3.25, # Turn 4
		1.70, # Straight 3
		2.85, # Turn 5
		2.40  # Turn 5 to Start
]

# Other settings
waitForNextLapAfter = 1.0			# If we are more than this many meters ahead of the WaitFor*
									# position then the call will wait for the next lap

#######################################################################################
# These settings are dynamic and should be automatically corrected for when changed   #
# Some parts of the logic get restarted on these changes, the car will briefly stop   #
#######################################################################################

# Power settings
maxPower = 1.0						# Maximum of allowed drive output (1.0 is 100%)
#maxPower = 0.95					# Maximum of allowed drive output - Approximate calibration level
#maxPower = 0.0						# Maximum of allowed drive output - Force stationary

# Crop settings
cropX1 = int(imageWidth  * 0.00)	# Left edge of the image to use
cropX2 = int(imageWidth  * 1.00)	# Right edge of the image to use
cropY1 = int(imageHeight * 0.45)	# Top edge of the image to use
cropY2 = int(imageHeight * 1.00)	# Bottom edge of the image to use

# Image black identification tuning
autoGainMax = 3.0					# Maximum allowed gain for the brightness auto correction
blackMaxR = 100						# Maximum red level for black detection (0-255)
blackMaxG = 100						# Maximum green level for black detection (0-255)
blackMaxB = 60						# Maximum blue level for black detection (0-255)

# Image start line identification tuning
startCrossedSeconds = 0.5			# Number of seconds to wait after the start line is seen before announcing crossing
startX1 = int(imageWidth  * 0.40)	# Lower X boundary for start line detection
startX2 = int(imageWidth  * 0.60)	# Upper X boundary for start line detection
startY  = int(imageHeight * 0.34)	# Y target for start line detection
startMinR = 45						# Minimum red level in the start detection zone
startMaxG = 50						# Maximum green level in the start detection zone
startMaxB = 50						# Maximum blue level in the start detection zone
startRatioMin = 0.90				# Minimum number of matching pixels per pixel in the detection zone
startRedetectionSeconds = 10.0		# Number of seconds before we will detect another start marker

# Image colour identification tuning
targetLevel = 200.0					# Level to auto-tune channels for
redGain   = 1.00					# Gain to make the red channel equal
greenGain = 1.10					# Gain to make the green channel equal
blueGain  = 1.35					# Gain to make the blue channel equal
erodeChannels = 2					# Amount to erode each channel to remove noise (1 is off)

# Image colour final thresholding
redMin   = 1						# Minimum red level after normalization (0-255)
greenMin = 1						# Minimum green level after normalization (0-255)
blueMin  = 1						# Minimum blue level after normalization (0-255)

# Positions in the image to look for lines
imagePositions = []
grid = 40
for i in range(grid + 1):
	imagePositions.append(i / float(grid))

# Camera perspective correction settings
trackSepX = 85						# Number of pixels between two lines at offsetTargetY
gainCorrection = 0.620 / 0.100		# Corrective gain per track width off central
	
# Other image processing settings
maxSepX = 0.15 * imageWidth			# Furthest two edges can be apart for merging
offsetTargetY = 0.25				# Y level from the top of the crop to use for offset calculations

# PID control values
Kp0 = 0.16							# P term for d0 input (Offset)
Ki0 = 0.00							# I term for d0 input (Offset)
Kd0 = 0.20							# D term for d0 input (Offset)
Kp1 = 0.004							# P term for d1 input (Angle)
Ki1 = 0.00							# I term for d1 input (Angle)
Kd1 = 0.004							# D term for d1 input (Angle)
Kp2 = 0.00							# P term for d2 input (Curvature)
Ki2 = 0.00							# I term for d2 input (Curvature)
Kd2 = 0.00							# D term for d2 input (Curvature)
clipI = 100							# Clipping limit for the integrators

# FIR filter settings
firTaps = 3							# Number of readings (taps) the filter is working over

# Final drive settings
steeringGain = 2.0					# Steering range correction value
steeringClip = 0.99					# Maximum steering value
steeringOffset = 0.0				# Steering centre correction value
maxBadFrames = frameRate / 2		# Number of poor frames before stopping
targetTrackPosition = 0.0			# Target position on the track, 0 is the centre

# Override system settings
stuckIdenticalSeconds = 1.0			# Number of seconds with near identical frames before deciding we are stuck
stuckIdenticalThreshold = 2.00		# Level at which two frames are seen as identical
stuckOverrideSeconds = 1.5			# Number of seconds to reverse for when stuck
stuckHuntSeconds = 0.8				# Number of seconds to hunt for the track after reversing when stuck
stuckDetectColourWidth = 0.5		# Position in the image along X to look for the track colour
stuckDetectColourHeight = 0.9		# Position in the image along Y to look for the track colour
flipDetectionSeconds = 0.3			# Number of seconds with frames which seem flipped before inverting movement
flipDetectionThreshold = 12.00		# Minimum gain between the background and the track
wrongWayThreshold = 10				# Number of wrong-way points before deciding to turn around
wrongWaySpinSeconds = 1.2			# Number of seconds to spin when the wrong way around
overtakeThreshold = 38				# Number of unexpected points before deciding there is a robot in front
overtakeLaneOffset = 1.0			# Lane shift away from the robot in front when overtaking
overtakeDurationSeconds = 4.0		# Number of seconds to overtake for

# Traffic light settings
lightsChangeThreshold = 50.0		# Minimum colour difference to flag as a change in light colour 
lightsRedGain = 2.0					# Gain term used to make the red and green levels consistent
lightsX1 = int(imageWidth  * 0.45)	# Lower X boundary for light detection
lightsX2 = int(imageWidth  * 0.55)	# Upper X boundary for light detection
lightsY1 = int(imageHeight * 0.25)	# Lower Y boundary for light detection
lightsY2 = int(imageHeight * 0.30)	# Upper Y boundary for light detection
lightsBurnFrames = frameRate * 2	# Number of initial frames where we throw the image away because it is still settling

# Plotting settings
offsetAxis = 10
offsetHeight = 5
thickLineScale = 2
thinLineScale = 1

#######################################################################################
# These settings are automatically calculated from the settings above                 #
# While the logic here can be changed do so carefully, some changes are not dynamic   #
#######################################################################################

# Work out the cropped image locations
croppedYScan = []
for position in imagePositions:
	Y = int(position * imageHeight)
	if Y < cropY1:
		# Too high
		pass
	elif Y >= cropY2:
		# Too low
		pass
	else:
		Y = Y - cropY1
		croppedYScan.append(Y)
croppedTargetY = (cropY2 - cropY1) * offsetTargetY

# Setup the power limits
if maxPower > 1.0:
	maxPower = 1.0
elif maxPower < -1.0:
	maxPower = -1.0

# Work out times in frame counts
stuckIdenticalFrames = int(stuckIdenticalSeconds * frameRate)
stuckOverrideFrames = int(stuckOverrideSeconds * frameRate)
stuckHuntFrames = int(stuckHuntSeconds * frameRate)
flipDetectionFrames = int(flipDetectionSeconds * frameRate)
wrongWaySpinFrames = int(wrongWaySpinSeconds * frameRate)
overtakeDurationFrames = int(overtakeDurationSeconds * frameRate)
startCrossedFrames = int(startCrossedSeconds * frameRate)

# Work out other positions in final X and Y
stuckDetectColourX = int(imageWidth  * stuckDetectColourWidth) - cropX1
stuckDetectColourY = int(imageHeight * stuckDetectColourHeight) - cropY1
flipDetectionY1 = 0
flipDetectionY2 = int(imageHeight * 0.2)
flipDetectionY3 = int(imageHeight * 0.8)
flipDetectionY4 = imageHeight

# Work out the absolute distance from the start to each waypoint
waypointDistances = []
distanceSoFar = 0.0
for i in range(len(distanceBetweenWaypoints) - 1):
	distanceSoFar += distanceBetweenWaypoints[i]
	waypointDistances.append(distanceSoFar)
del distanceSoFar

# Work out speeds per frame and corrections for lane
yetiDistancePerFrame = yetiSpeed / float(frameRate)
trackChangePerLane = (trackLengthOuter - trackLengthInner) / float(lanes)
stuckDetectedDistanceCorrection = (stuckIdenticalFrames + stuckOverrideFrames) * (yetiSpeed / 4.0)
