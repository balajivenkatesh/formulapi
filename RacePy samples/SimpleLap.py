### Start of the race ###
WaitForGo()
Speed(100)

### Racing for 23 laps ###
while LapCount() < 23:
	# Drive in the center until turn 2
	AimForLane(0)
	WaitForWaypoint(4)
	# Drive on the middle of the inner blue lane
	# until the back straight
	AimForLane(-1.5)
	WaitForWaypoint(7)
	# Drive on the middle of the outer blue lane
	# until the start / finish line
	AimForLane(+1.5)
	WaitForWaypoint(1)

### End of the race ###
FinishRace()
