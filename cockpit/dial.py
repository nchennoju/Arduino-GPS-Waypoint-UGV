import math
import serial
import pygame
from pygame.locals import *
from pygame import mixer
import sys

#import threading
import textToSpeech

pygame.init()

class Dial:
   """
   Generic dial type.
   """
   def __init__(self, image, frameImage, x=0, y=0, w=0, h=0):
       """
       x,y = coordinates of top left of dial.
       w,h = Width and Height of dial.
       """
       self.x = x 
       self.y = y
       self.image = image
       self.frameImage = frameImage
       self.dial = pygame.Surface(self.frameImage.get_rect()[2:4])
       self.dial.fill(0xFFFF00)
       if(w==0):
          w = self.frameImage.get_rect()[2]
       if(h==0):
          h = self.frameImage.get_rect()[3]
       self.w = w
       self.h = h
       self.pos = self.dial.get_rect()
       self.pos = self.pos.move(x, y)

   def position(self, x, y):
       """
       Reposition top,left of dial at x,y.
       """
       self.x = x 
       self.y = y
       self.pos[0] = x 
       self.pos[1] = y 

   def position_center(self, x, y):
       """
       Reposition centre of dial at x,y.
       """
       self.x = x
       self.y = y
       self.pos[0] = x - self.pos[2]/2
       self.pos[1] = y - self.pos[3]/2

   def rotate(self, image, angle):
       """
       Rotate supplied image by "angle" degrees.
       This rotates round the centre of the image. 
       If you need to offset the centre, resize the image using self.clip.
       This is used to rotate dial needles and probably doesn't need to be used externally.
       """
       tmpImage = pygame.transform.rotate(image ,angle)
       imageCentreX = tmpImage.get_rect()[0] + tmpImage.get_rect()[2]/2
       imageCentreY = tmpImage.get_rect()[1] + tmpImage.get_rect()[3]/2

       targetWidth = tmpImage.get_rect()[2]
       targetHeight = tmpImage.get_rect()[3]

       imageOut = pygame.Surface((targetWidth, targetHeight))
       imageOut.fill(0xFFFF00)
       imageOut.set_colorkey(0xFFFF00)
       imageOut.blit(tmpImage,(0,0), pygame.Rect( imageCentreX-targetWidth/2,imageCentreY-targetHeight/2, targetWidth, targetHeight ) )
       return imageOut

   def clip(self, image, x=0, y=0, w=0, h=0, oX=0, oY=0):
       """
       Cuts out a part of the needle image at x,y position to the correct size (w,h).
       This is put on to "imageOut" at an offset of oX,oY if required.
       This is used to centre dial needles and probably doesn't need to be used externally.       
       """
       if(w==0):
           w = image.get_rect()[2]
       if(h==0):
           h = image.get_rect()[3]
       needleW = w + 2*math.sqrt(oX*oX)
       needleH = h + 2*math.sqrt(oY*oY)
       imageOut = pygame.Surface((needleW, needleH))
       imageOut.fill(0xFFFF00)
       imageOut.set_colorkey(0xFFFF00)
       imageOut.blit(image, (needleW/2-w/2+oX, needleH/2-h/2+oY), pygame.Rect(x,y,w,h))
       return imageOut

   def overlay(self, image, x, y, r=0):
       """
       Overlays one image on top of another using 0xFFFF00 (Yellow) as the overlay colour.
       """
       x -= (image.get_rect()[2] - self.dial.get_rect()[2])/2
       y -= (image.get_rect()[3] - self.dial.get_rect()[3])/2
       image.set_colorkey(0xFFFF00)
       self.dial.blit(image, (x,y))




class Horizon(Dial):
   """
   Artificial horizon dial.
   """
   def __init__(self, x=0, y=0, w=0, h=0):
       """
       Initialise dial at x,y.
       Default size of 300px can be overidden using w,h.
       """
       self.image = pygame.image.load('resources/Horizon_GroundSky2.png').convert()
       self.frameImage = pygame.image.load('resources/Horizon_Background.png').convert()
       self.maquetteImage = pygame.image.load('resources/Maquette_Avion.png').convert()
       Dial.__init__(self, self.image, self.frameImage, x, y, w, h)
   def update(self, screen, angleX, angleY):
       """
       Called to update an Artificial horizon dial.
       "angleX" and "angleY" are the inputs.
       "screen" is the surface to draw the dial on.
       """
       angleX %= 360
       angleY %= 360
       if (angleX > 180):
           angleX -= 360 
       if (angleY > 90)and(angleY < 270):
           angleY = 180 - angleY 
       elif (angleY > 270):
           angleY -= 360
       tmpImage = self.clip(self.image, 0, (59-angleY)*720/180, 250, 250)
       tmpImage = self.rotate(tmpImage, angleX)
       self.overlay(tmpImage, 0, 0)
       self.overlay(self.frameImage, 0,0)
       self.overlay(self.maquetteImage, 0,0)
       self.dial.set_colorkey(0xFFFF00)
       screen.blit( pygame.transform.scale(self.dial,(self.w,self.h)), self.pos )

class TurnCoord(Dial):
   """
   Turn Coordinator dial.
   """
   def __init__(self, x=0, y=0, w=0, h=0):
       """
       Initialise dial at x,y.
       Default size of 300px can be overidden using w,h.
       """
       self.image = pygame.image.load('resources/TurnCoordinatorAircraft.png').convert()
       self.frameImage = pygame.image.load('resources/TurnCoordinator_Background.png').convert()
       self.marks = pygame.image.load('resources/TurnCoordinatorMarks.png').convert()
       self.ball = pygame.image.load('resources/TurnCoordinatorBall.png').convert()
       Dial.__init__(self, self.image, self.frameImage, x, y, w, h)
   def update(self, screen, angleX, angleY):
       """
       Called to update a Turn Coordinator dial.
       "angleX" and "angleY" are the inputs.
       "screen" is the surface to draw the dial on.       
       """
       angleX %= 360 
       angleY %= 360
       if (angleX > 180):
           angleX -= 360 
       if (angleY > 180):
           angleY -= 360
       if(angleY > 14): 
           angleY = 14
       if(angleY < -14): 
           angleY = -14
       tmpImage = self.clip(self.image, 0, 0, 0, 0, 0, -12)
       tmpImage = self.rotate(tmpImage, angleX)
       self.overlay(self.frameImage, 0,0)
       self.overlay(tmpImage, 0, 0)
       tmpImage = self.clip(self.marks, 0, 0, 0, 0, 0, 0)
       self.overlay(tmpImage, 0, 80)
       tmpImage = self.clip(self.ball, 0, 0, 0, 0, 0, 300)
       tmpImage = self.rotate(tmpImage, angleY)
       self.overlay(tmpImage, 0, -220)
       self.dial.set_colorkey(0xFFFF00)
       screen.blit( pygame.transform.scale(self.dial,(self.w,self.h)), self.pos )

class Generic(Dial):
   """
   Generic Dial. This is built on by other dials.
   """
   def __init__(self, x=0, y=0, w=0, h=0):
       """
       Initialise dial at x,y.
       Default size of 300px can be overidden using w,h.       
       """
       self.image = pygame.image.load('resources/AirSpeedNeedle.png').convert()
       self.frameImage = pygame.image.load('resources/Indicator_Background.png').convert()
       Dial.__init__(self, self.image, self.frameImage, x, y, w, h)
   def update(self, screen, angleX, iconLayer=0):
       """
       Called to update a Generic dial.
       "angleX" and "angleY" are the inputs.
       "screen" is the surface to draw the dial on.       
       """
       angleX %= 360
       angleX = 360 - angleX
       tmpImage = self.clip(self.image, 0, 0, 0, 0, 0, -35)
       tmpImage = self.rotate(tmpImage, angleX)
       self.overlay(self.frameImage, 0,0)
       if iconLayer:
          self.overlay(iconLayer[0],iconLayer[1],iconLayer[2])
       self.overlay(tmpImage, 0, 0)
       self.dial.set_colorkey(0xFFFF00)
       screen.blit( pygame.transform.scale(self.dial,(self.w,self.h)), self.pos )

class HeadingGeneric(Dial):
   """
   Generic Dial. This is built on by other dials.
   """
   def __init__(self, x=0, y=0, w=0, h=0):
       """
       Initialise dial at x,y.
       Default size of 300px can be overidden using w,h.
       """
       self.image = pygame.image.load('resources/HeadingIndicator_Aircraft.png').convert()
       self.image2 = pygame.image.load('resources/LongNeedleAltimeter.png').convert()
       self.frameImage = pygame.image.load('resources/HeadingIndicator_Background.png').convert()
       Dial.__init__(self, self.image, self.frameImage, x, y, w, h)
   def update(self, screen, angleX, iconLayer=0):
       """
       Called to update a Generic dial.
       "angleX" and "angleY" are the inputs.
       "screen" is the surface to draw the dial on.
       """
       angleX %= 360
       angleX = 360 - angleX
       tmpImage = self.clip(self.image, 0, 0, 0, 0, 0, -35)
       tmpImage = self.rotate(tmpImage, angleX)
       self.overlay(self.frameImage, 0,0)
       if iconLayer:
          self.overlay(iconLayer[0],iconLayer[1],iconLayer[2])
       self.overlay(tmpImage, 0, 0)
       self.dial.set_colorkey(0xFFFF00)
       screen.blit( pygame.transform.scale(self.dial,(self.w,self.h)), self.pos )

class Battery(Generic):
   """
   Battery dial.
   """
   def __init__(self, x=0, y=0, w=0, h=0):
       """
       Initialise dial at x,y.
       Default size of 300px can be overidden using w,h.
       """
       self.icon = pygame.image.load('resources/battery2.png').convert()
       Generic.__init__(self, x, y, w, h)
       self.frameImage = pygame.image.load('resources/ledgend.png').convert()
   def update(self, screen, angleX):
       """
       Called to update a Battery dial.
       "angleX" is the input.
       "screen" is the surface to draw the dial on.
       """
       if angleX > 100:
          angleX = 100
       elif angleX < 0:
          angleX = 0
       angleX *= 2.7
       angleX -= 135
       Generic.update(self, screen, angleX, (self.icon, 0, 100))
class Heading(HeadingGeneric):
   """
   Battery dial.
   """
   def __init__(self, x=0, y=0, w=0, h=0):
       """
       Initialise dial at x,y.
       Default size of 300px can be overidden using w,h.
       """
       HeadingGeneric.__init__(self, x, y, w, h)
       self.icon = pygame.image.load('resources/HeadingWeel.png').convert()

   def update(self, screen, angleX):
       """
       Called to update a Battery dial.
       "angleX" is the input.
       "screen" is the surface to draw the dial on.
       """
       HeadingGeneric.update(self, screen, angleX, (self.icon, 0, 0))

class RfSignal(Generic):
   """
   RF Signal dial.
   """
   def __init__(self, x=0, y=0, w=0, h=0):
       """
       Initialise dial at x,y.
       Default size of 300px can be overidden using w,h.
       """
       self.image = pygame.Surface((0,0))
       self.frameImage = pygame.image.load('resources/RF_Dial_Background.png').convert()
       Dial.__init__(self, self.image, self.frameImage, x, y, w, h)
   def update(self, screen, inputA, inputB, scanPos):
       """
       "screen" is the surface to draw the dial on.       
       """

       top = self.dial.get_rect()[0] +60
       left = self.dial.get_rect()[1] +30
       bottom = self.dial.get_rect()[0] + self.dial.get_rect()[2] -60
       right = self.dial.get_rect()[1] + self.dial.get_rect()[3] -30
       height = bottom - top
       middle = height/2 + top

       scanPos %= right -30
       scanPos += 30
       inputA %= 100
       inputB %= 100
       inputA = height * inputA / 200
       inputB = height * inputB / 200

       pygame.draw.line(self.dial, 0xFFFFFF, (scanPos,top), (scanPos,bottom), 1)
       pygame.draw.line(self.dial, 0x222222, (scanPos-1,top), (scanPos-1,bottom), 1)

       pygame.draw.line(self.dial, 0x00FFFF, (scanPos-1,middle-inputA), (scanPos-1,middle),4)
       pygame.draw.line(self.dial, 0xFF00FF, (scanPos-1,bottom-inputB), (scanPos-1,bottom),4)
       pygame.draw.line(self.dial, 0xFFFF00, (scanPos-1,middle), (scanPos-1,middle))

       self.overlay(self.frameImage, 0,0)

       self.dial.set_colorkey(0xFFFF00)
       screen.blit( pygame.transform.scale(self.dial,(self.w,self.h)), self.pos )

class GenericText():
   """
   Generic Dial. This is built on by other dials.
   """
   def __init__(self, x=0, y=0, font=0):
       """
       Initialise dial at x,y.
       Default size of 300px can be overidden using w,h.
       """
       self.X = x
       self.Y = y
       self.font = font
       print()
       #self.frameImage = pygame.image.load('resources/Indicator_Background.png').convert()
   def update(self, screen, textStr, textNum, threshold):

       red = (255, 0, 0)
       green = (0, 255, 0)
       black = (0, 0, 0)

       # assigning values to X and Y variable
       #self.X = 700
       #self.Y = 255


       # set the pygame window name
       pygame.display.set_caption('Show Text')

       font = pygame.font.Font('freesansbold.ttf', self.font)

       # create a text suface object,
       # on which text is drawn on it.
       if(textNum < threshold):
           text = font.render(textStr + str(textNum), True, green, black)
           #tts.speak("Waypoint")
       else:
           text = font.render(textStr + str(textNum), True, red, black)

       numChars = len(textStr) + len(str(textNum))


       textRect = pygame.Rect((self.X, self.Y), (numChars*self.font, self.font))
       pygame.draw.rect(pygame.display.get_surface(), black, textRect)

       screen.blit(text, (self.X, self.Y))








import serial.tools.list_ports
import keyboard


# Returns list of all accessible serial ports
def getPorts():
    portData = serial.tools.list_ports.comports()
    return portData

# Returns COM port of Arduino if detected by computer. User for switchbox
def findArduino(portsFound):
    numConnections = len(portsFound)
    for i in range(0, numConnections):
        if ('Uno' in str(portsFound[i]) or 'Nano' in str(portsFound[i]) or 'CH340' in str(portsFound[i])):
            return str(portsFound[i])
    return "None"


def conv(str):
    return str[2:len(str) - 5]



tts = textToSpeech.TextToSpeech()
tts.setRate(210)
tts.setMF("female")

# GET ARDUINO STATUS / Update on GUI connection label
status = findArduino(getPorts())
ser = serial.Serial()
test = 1
if (not (status == "None")):
    ser = serial.Serial(status.split()[0], 115200)
    test = 0
    file = open("positions.kml", "r+")
    file.truncate(0)
    file.close()


arr = []
i = 0

fileName = ""
if(test == 0):  # change 1 to 0 once done
    fileName = input("Enter file name (without file extension): ")
    fileName = "logs/" + fileName + ".txt"
    f = open(fileName, "a")
    # thetaScaled angle distance Heading Roll Pitch Voltage
    f.write("Theta\tAngle\tDistance\tHeading\tRoll\tPitch\tVoltage\n")
    f.close()



# Initialise screen.
screen = pygame.display.set_mode((1000, 1050), pygame.FULLSCREEN)
screen.fill(0x000)

   
# Initialise Dials.
horizon = Horizon(10,10, 500, 500)
turn = TurnCoord(10,520,250,250)
throttle = Generic(260,520,250,250)
RXbattery = Battery(520,780,250,250)
heading = Heading(520,520,250,250)
rfSignal = RfSignal(10,780,500,250)
dist = GenericText(530,55,30)

pps = GenericText(530,110,30)
speed = GenericText(530,165,30)

a=0


while 1:
   # Main program loop.
   if (keyboard.is_pressed('q')):
       print("Exiting....")
       ser.close()  # close serial port.
       pygame.quit()
       sys.exit()

   for event in pygame.event.get():
       if event.type == QUIT:
           print("Exiting....")
           ser.close()	# close serial port.
           pygame.quit()
           sys.exit()   # end program.
       if event.type == pygame.VIDEORESIZE:
           surface = pygame.display.set_mode((event.w, event.h),
                                             pygame.RESIZABLE)


   i += 1

   if(test == 1):
      # Use dummy test data
      curPos = pygame.mouse.get_pos()
      rf_data = [0, 0, i, 0, curPos[0]/2, curPos[1]/2, 0, 0, 0, 0, 0]
      pygame.time.delay(100)
      if(i > 50):
          i = 0
   else: 
      # Get real data from USB port.
      rf_data = conv(str(ser.readline())).split("\\t")
      if (i > 50):
          i = 0
          ser.flushInput()

   if(len(rf_data) >= 11):
      # We have data.
      a+=1

      #PRINT ALL SERIAL DATA RECEIVED
      #print(rf_data)

      # Update dials.
      try:
          #String(currentTheta.value) + "\t" + String(headingReq.value) + "\t" + String(distanceTo.value) + "\t" + String(currentHeading.value) + "\t" + String(currentRoll.value) + "\t" + String(currentPitch.value) + "\t" + String(currentVoltage.value) + "\t" + String(currentLat.value, 7) + "\t" + String(currentLon.value, 7) + "\t" + String(currentSpeed.value);
          horizon.update(screen, int(rf_data[4]), int(rf_data[5]) - 8 )
          turn.update(screen, -int(rf_data[4])/2, -int(rf_data[4])/4)
          throttle.update(screen, int(rf_data[0]))
          RXbattery.update(screen, 100*((float(rf_data[6]) - 7.22)/1.18) )
          rfSignal.update(screen, 0, 0, a)
          heading.update(screen, int(rf_data[3]))
          dist.update(screen, "Distance: ", float(rf_data[2]), 3)
          pps.update(screen, "PPS: ", float(rf_data[10]), 3)
          speed.update(screen, "Speed (mph): ", float(rf_data[9]), 20)

          if(test == 0):
              f = open(fileName, "a")
              # thetaScaled angle distance Heading Roll Pitch Voltage
              f.write(str(rf_data[0]) + "\t" + str(rf_data[1]) + "\t" + str(rf_data[2]) + "\t" + str(
                  rf_data[3]) + "\t" + str(rf_data[4]) + "\t" + str(rf_data[5]) + "\t" + str(rf_data[6]) + "\t" + str(rf_data[7]) + "\t" + str(rf_data[8]) + "\t" + str(rf_data[9]) + "\t" + str(rf_data[10]) + "\n")
              f.close()

              if((float(rf_data[7]) > 5 or float(rf_data[7]) < -5) and (float(rf_data[8]) > 5 or float(rf_data[8]) < -5)):
                  with open("positions.kml", "w") as pos:
                      pos.write("""<kml xmlns="http://www.opengis.net/kml/2.2"
                   xmlns:gx="http://www.google.com/kml/ext/2.2"><Placemark>
                                    <name>UGV</name>
                                    <description>GPS Data from UGV</description>
                                    <Point>
                                      <coordinates>%s,%s,%s</coordinates>
                                    </Point>
                                    <Style id="mystyle">
                                      <IconStyle>
                                        <scale>1</scale>
                                        <Icon>
                                          <href>http://maps.google.com/mapfiles/kml/shapes/track.png</href>
                                        </Icon>
                                      </IconStyle>
                                    </Style>
                                    <LookAt>       
                                      <longitude>%s</longitude>       
                                      <latitude>%s</latitude>       
                                      <altitude>2000</altitude>
                                      <altitudeMode>clampToGround</altitudeMode>
                                      <tilt>45</tilt>
                                      <range>15</range>
                                      <heading>%s</heading>
                                    </LookAt>
                                  </Placemark></kml>""" % (rf_data[8], rf_data[7], 0, rf_data[8], rf_data[7], rf_data[3]))
              else:
                  print("INVALID GPS")


      except:
          print("SERIAL ERROR")

      pygame.display.update()



