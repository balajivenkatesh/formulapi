import time

### Enable logging ###
StartDetailedLoging()
StartUserLog()

### Get the starting lane ###
WaitForSeconds(2.0)
startLane = round(CurrentTrackPosition())
AimForLane(startLane)

### Start of the race ###
WaitForGo()

# Starting straight
AimForLane(startLane)
Speed(100)
WaitForWaypoint(2)

# First corner of the race
AimForLane(0)

### Racing for 10 minutes ###
endTime = time.time() + 10 * 60
while time.time() < endTime:
	# Drive in the center until turn 2
	AimForLane(0)
	WaitForWaypoint(4)
	# Drive on the middle of the inner blue lane
	# until the back straight
	AimForLane(-1.5)
	WaitForWaypoint(7)
	# Move towards the center of the track
	# so we do not try and turn too sharply
	AimForLane(0)
	WaitForSeconds(1.0)
	# Drive on the middle of the outer blue lane
	# until the start / finish line
	AimForLane(+1.5)
	WaitForWaypoint(1)

### Continue racing for a few seconds ###
WaitForSeconds(4)

### End of the race ###
EndDetailedLog()
EndUserLog()
FinishRace()
