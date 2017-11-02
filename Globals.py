#!/usr/bin/env python
# coding: Latin-1

#############################################################
# This is the module which holds all the global values      #
#############################################################

# General values
badFrameCount = 0
frameCounter = 0
frameAnnounce = 0
running = True
frameTimes = []
lastFrameStamp = 0
trackFound = False
lastLines = []
lapCount = 0
lapTravelled = 0.0
colour = (0, 0, 0)

# Defaults for values set from the Race Code Functions
userSpeed = 1.0
userTargetLane = 0.0

# States
startLights = 0
imageMode = 0
lastImageMode = 0
seenStart = False
startWaitCount = 0
pollDelay = 0.25
processingWriteLogLevel = 3
processingPrintLogLevel = 2

# Images
displayFrame = None
displayPredator = None
lastRawFrame = None

# Threads and locks
processorPool = []
controller = None
frameLock = None

# Functions
MonsterLed = None
MonsterMotors = None

# Other resources
capture = None
userLogFile = None
processingLogFile = None
