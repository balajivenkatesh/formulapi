import time

### Settings for this race ###
secondsRacing = 10 * 60

### Start of the race ###
# Wait until we can see the track
while not TrackFound():
	WaitForSeconds(1.0)
# We can see the track now, start by following the lane we are on
trackLane = round(CurrentTrackPosition())
AimForLane(trackLane)
# Save a start-line image
photo = GetLatestImage()
SaveImage(photo, 'Start-line')
# Start logging what happens
StartUserLog()
StartDetailedLoging()
# Wait for the go signal from the start/stop lights.
WaitForGo()
# Go at max speed
Speed(100)

### During the race ###
# Keep going until we have fished all of the time
startTime = time.time()
endTime = startTime + secondsRacing
while time.time < endTime:
	# Full speed to the first corner
	Speed(100)
	WaitForWaypoint(2)
	# Slow down, move to the inside in stages and wait for the apex
	Speed(80)
	AimForLane(0)
	WaitForSeconds(1.0)
	AimForLane(-2)
	WaitForWaypoint(3)
	# Speed up and move to the center until the S curve starts
	Speed(100)
	AimForLane(0)
	WaitForWaypoint(4)
	# Move towards the outside until the S curve changes
	AimForLane(+1)
	WaitForWaypoint(5)
	# Move towards the inside until the S curve ends
	AimForLane(-1)
	WaitForWaypoint(6)
	# Slow down and move to the inside around the corner
	Speed(70)
	AimForLane(-2)
	WaitForWaypoint(7)
	# Speed up for the back straight along the center
	Speed(100)
	AimForLane(0)
	WaitForWaypoint(8)
	# High speed for the last corner on the inside
	Speed(90)
	AimForLane(-2)
	WaitForWaypoint(9)
	# Full speed until the start/finish line along the outside
	Speed(100)
	AimForLane(+2)
	WaitForWaypoint(1)

### End of the race ###
# Save a finish-line image
photo = GetLatestImage()
SaveImage(photo, 'Finished')
# Slow the YetiBorg down gradually from 100% to 0%
for slowing in range(99, -1, -1):
	Speed(slowing)
	WaitForSeconds(0.01)
# Stop the logging
EndUserLog()
EndDetailedLog()
# End the race (will stop the robot and end the program)
FinishRace()
