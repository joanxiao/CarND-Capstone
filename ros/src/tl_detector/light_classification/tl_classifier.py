from styx_msgs.msg import TrafficLight
import cv2
import math
import numpy as np
import matplotlib.pyplot as plt

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass


    ###############################################
    def get_light_color(self, img, x, y, lower_HSV, upper_HSV):
        colorID = TrafficLight.UNKNOWN
        # median blur the image
        img = cv2.medianBlur(img, 5)
        # Convert image to HSV
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Threshold the HSV image to get only selected(red, greeen, or yellow) colors
        mask = cv2.inRange(hsvImg, lower_HSV, upper_HSV) 
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img,img, mask= mask)

        #mask out the area in image that has no traffic lights
        #create a black image
        polygon_img = np.zeros(img.shape, np.uint8)
      
        #draw a polygon
        pts = np.array([[x-275, y], [x+275, y], [x+275, y+150], [x-275, y+150]])
        cv2.fillPoly(polygon_img, pts=[pts], color=(255,255,255))
        res = cv2.bitwise_and(res,res,mask=polygon_img[:,:,1])

        g=res
        #brightest spot
        a = np.array(g)
        print(a.max(), np.unravel_index(a.argmax(), a.shape))
        brighty = np.unravel_index(a.argmax(), a.shape)[0]
        brightx = np.unravel_index(a.argmax(), a.shape)[1]
        #print("Brightest spot, brightx: {}, birghty: {}".format(brightx, brighty)) 


	#color hsv range boolean
        greenColor = np.all(lower_HSV == np.array([60, 125, 125])) and np.all(upper_HSV == np.array([120,255,255]))
        redColor = np.all(lower_HSV == np.array([170, 125, 125])) and np.all(upper_HSV == np.array([179,255,255]))
        yellowColor = np.all(lower_HSV == np.array([5, 150, 150])) and np.all(upper_HSV == np.array([40,255,255]))

        if (((brightx == 0) and (brighty == 0)) == False):
            if (greenColor == True):
		print("*******************Green Traffic Light**************")
                cv2.rectangle(img, (brightx -15, brighty - 15), (brightx + 15, brighty + 15), (255,0,0),2)
                cv2.putText(img, "green traffic light", (brightx-15, brighty -27), 0, 1.2, (255,0,0),2)
                colorID = TrafficLight.GREEN
                print("colorID: TrafficLight.GREEN or color ID index: {}".format(TrafficLight.GREEN))
            elif (redColor == True):
		print("*******************Red Traffic Light**************")
                cv2.rectangle(img, (brightx -15, brighty - 15), (brightx + 15, brighty + 15), (255,0,0),2)
                cv2.putText(img, "red traffic light", (brightx-15, brighty -27), 0, 1.2, (255,0,0),2)
                colorID = TrafficLight.RED
                print("colorID: TrafficLight.RED or color ID index: {}".format(TrafficLight.RED))
            elif (yellowColor == True):
		print("*******************Yellow Traffic Light**************")
                cv2.rectangle(img, (brightx -15, brighty - 15), (brightx + 15, brighty + 15), (255,0,0),2)
                cv2.putText(img, "yellow traffic light", (brightx-15, brighty -27), 0, 1.2, (255,0,0),2)
              
                colorID = TrafficLight.YELLOW
                print("colorID: TrafficLight.YELLOW or color ID index: {}".format(TrafficLight.YELLOW))
        return colorID

    #######################################################

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction


        ########################################################
	cv_image = image
        #initialize the color ID and boolean light colors
        clrID = TrafficLight.UNKNOWN
        yellowLight = False
        greenLight = False
        redLight = False

	#The size of one traffic light is about 50 in x direction,125 in y direction
	#The center of the image is:
	x = cv_image.shape[1]/2 
        y = cv_image.shape[0]/2 
        

        ###################green color detection##########
        img = cv_image
        imgOrig = img

        # median blur the image
        img = cv2.medianBlur(img, 5)
        # Convert image to HSV
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # define range of green color in HSV
        lower_green = np.array([60,125,125]) #100,100])
        upper_green = np.array([120,255,255])
        clr_ID = self.get_light_color(img, x, y, lower_green, upper_green)
        if (clr_ID == TrafficLight.GREEN):
            greenLight = True
        ##################red color detection#################
        # define range of red color in HSV
        lower_red = np.array([170,125,125]) 
        upper_red = np.array([179,255,255])
        clr_ID = self.get_light_color(img, x, y, lower_red, upper_red)
        if (clr_ID == TrafficLight.RED):
           redLight = True


        ###############yellow traffic light detection###########
        # define range of orange color in HSV
        lower_yellow = np.array([5,150,150]) 
        upper_yellow = np.array([40,255,255]) #real amber traffic light works 15,255,255])
        clr_ID = self.get_light_color(img, x, y, lower_yellow, upper_yellow)
        if (clr_ID == TrafficLight.YELLOW):
            yellowLight = True
	
        if ((yellowLight == True) and (redLight == False) 
             and (greenLight == False)):
            clr_ID = TrafficLight.YELLOW
        elif ((yellowLight == False) and (redLight == True) 
             and (yellowLight == False)):
            clr_ID = TrafficLight.RED
        elif ((yellowLight == False) and (redLight == False) 
             and (greenLight == True)):
            clr_ID = TrafficLight.GREEN
	else:
            clr_ID = TrafficLight.UNKNOWN
        
        #print("Traffic Light color_ID: {}".format(clr_ID))
        
        ########################################################

        return clr_ID #TrafficLight.UNKNOWN
