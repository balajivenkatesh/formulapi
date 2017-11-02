import time

### Enable logging ###
StartDetailedLoging()
StartUserLog()

### Wait for the lights ###
WaitForGo()

### Racing for 10 minutes ###
endTime = time.time() + 10 * 60
while time.time() < endTime:
	# Wait until we reach the first corner
	WaitForWaypoint(2)
	# Wait until we reach the start / finish line
	WaitForWaypoint(1)

### Slow the MonsterBorg down gradually from 100% to 0% ###
for slowing in range(99, -1, -1):
	Speed(slowing)
	WaitForSeconds(0.01)

### Disable logging ###
EndDetailedLog()
EndUserLog()

### End of the race ###
FinishRace()
