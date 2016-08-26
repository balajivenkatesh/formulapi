### Demo of image processing routines
WaitForSeconds(2.5)

# Grab the image
image = GetLatestImage()
SaveImage(image, '00-raw')

# Get range levels for later
rMin, rMax, gMin, gMax, bMin, bMax = PerformProcessingStage(1, image)
print rMin, rMax, gMin, gMax, bMin, bMax

# Crop the image
image = PerformProcessingStage(2, image)
SaveImage(image, '02-cropped')

# Auto brightness
image = PerformProcessingStage(3, image)
SaveImage(image, '03-auto-bright')

# Find black areas
black = PerformProcessingStage(4, image)
SaveImage(black, '04-black-raw')

# Erode the black areas
black = PerformProcessingStage(5, black)
SaveImage(black, '05-black')

# Split the colours
red, green, blue = PerformProcessingStage(6, image)
SaveImage(red, '06-red-raw')
SaveImage(green, '06-green-raw')
SaveImage(blue, '06-blue-raw')

# Auto level channels
red = PerformProcessingStage(7, (red, rMin, rMax, Settings.redGain))
SaveImage(red, '07-red-leveled')
green = PerformProcessingStage(7, (green, gMin, gMax, Settings.greenGain))
SaveImage(green, '07-green-leveled')
blue = PerformProcessingStage(7, (blue, bMin, bMax, Settings.blueGain))
SaveImage(blue, '07-blue-leveled')

# Get the maximums
maxImage = PerformProcessingStage(8, (red, green, blue))
SaveImage(maxImage, '08-max')

# Exclude unwanted parts of each channel
red = PerformProcessingStage(9, (red, maxImage, black))
SaveImage(red, '09a-red-section')
green = PerformProcessingStage(9, (green, maxImage, black))
SaveImage(green, '09a-green-section')
blue = PerformProcessingStage(9, (blue, maxImage, black))
SaveImage(blue, '09a-blue-section')

# Erode the channels
red = PerformProcessingStage(5, red)
SaveImage(red, '09b-red')
green = PerformProcessingStage(5, green)
SaveImage(green, '09b-green')
blue = PerformProcessingStage(5, blue)
SaveImage(blue, '09b-blue')

# Merge channels to a single image to make them easy to see
viewImage = PerformProcessingStage(10, (red, green, blue, black))
SaveImage(viewImage, '10-final-view')

### End of the race ###
FinishRace()
