from styx_msgs.msg import TrafficLight
import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
import time 
import tensorflow as tf

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        PATH_TO_MODEL = 'frozen_inference_graph.pb'
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            # Works up to here.
            with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
        pass


    ###############################################
    def get_light_color(self, img, tl_box, lower_HSV, upper_HSV):
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
        left_x = tl_box[0]
        top_y = tl_box[1]
        right_x = tl_box[2]
        bot_y = tl_box[3]
        pts = np.array([[left_x, top_y], [right_x, top_y], [right_x, bot_y], [left_x, bot_y]])
        cv2.fillPoly(polygon_img, pts=[pts], color=(255,255,255))
        res = cv2.bitwise_and(res,res,mask=polygon_img[:,:,1])
        
        # Debug.
        #cv2.imwrite('img.png',img)
        #cv2.imwrite('poly.png',polygon_img)
        #cv2.imwrite('and.png',cv2.bitwise_and(img,img,mask=polygon_img[:,:,1]))

        g=res
        #brightest spot
        a = np.array(g)
        #print(a.max(), np.unravel_index(a.argmax(), a.shape))
        brighty = np.unravel_index(a.argmax(), a.shape)[0]
        brightx = np.unravel_index(a.argmax(), a.shape)[1]
        #print("Brightest spot, brightx: {}, birghty: {}".format(brightx, brighty)) 


	#color hsv range boolean
        greenColor = np.all(lower_HSV == np.array([60, 125, 125])) and np.all(upper_HSV == np.array([120,255,255]))
        redColor = np.all(lower_HSV == np.array([170, 125, 125])) and np.all(upper_HSV == np.array([179,255,255]))
        yellowColor = np.all(lower_HSV == np.array([5, 150, 150])) and np.all(upper_HSV == np.array([40,255,255]))

        if (((brightx == 0) and (brighty == 0)) == False):
            if (greenColor == True):
		print("********* G R E E N *********")
                #cv2.rectangle(img, (brightx -15, brighty - 15), (brightx + 15, brighty + 15), (255,0,0),2)
                #cv2.putText(img, "green traffic light", (brightx-15, brighty -27), 0, 1.2, (255,0,0),2)
                colorID = TrafficLight.GREEN
                #print("At time: {} sec, colorID: TrafficLight.GREEN or color ID index: {}".format(str(time.clock()), TrafficLight.GREEN))
            elif (redColor == True):
                
		print("*********   R E D   *********")
                #cv2.rectangle(img, (brightx -15, brighty - 15), (brightx + 15, brighty + 15), (255,0,0),2)
                #cv2.putText(img, "red traffic light", (brightx-15, brighty -27), 0, 1.2, (255,0,0),2)
                colorID = TrafficLight.RED
                #print("At time: {} sec, colorID: TrafficLight.RED or color ID index: {}".format(str(time.clock()), TrafficLight.RED))
            elif (yellowColor == True):
		print("******** Y E L L O W ********")
                #cv2.rectangle(img, (brightx -15, brighty - 15), (brightx + 15, brighty + 15), (255,0,0),2)
                #cv2.putText(img, "yellow traffic light", (brightx-15, brighty -27), 0, 1.2, (255,0,0),2)
              
                colorID = TrafficLight.YELLOW
                #print("At time: {} sec, colorID: TrafficLight.YELLOW or color ID index: {}".format(str(time.clock()), TrafficLight.YELLOW))
        return colorID

    def traffic_light_location(self, boxes, scores, classes, img_size=[600, 800, 3], score_thresh=0.2):
        # tensorflow usually operates on a set of images but since we are
        # working with a single image we only need first index.
        boxes = boxes[0]
        scores = scores[0]
        classes = classes[0]
        
        output_boxes = []
        # For now only do box around highest score. NOT ROBUST.
        for i in range(len(scores)):
            # Must be a traffic light and meet threshold.
            if scores[i] > score_thresh and classes[i] == 10:
                # Box values are between 0-1.
                left_x = int(boxes[i][1]*img_size[1])
                top_y = int(boxes[i][0]*img_size[0])
                right_x = int(boxes[i][3]*img_size[1])
                bot_y = int(boxes[i][2]*img_size[0])
                output_boxes.append([left_x, top_y, right_x, bot_y])
                break
            else:
                break
        return output_boxes

    #######################################################

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        #print("At time: {} sec, Start classification.".format(str(time.clock())))

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
        

        
        img = cv_image
        # Bounding Box Detection.
        #print("At time: {} sec, Start tf.".format(str(time.clock())))
        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                # Expand dimension since the model expcts image to have shape [1, None, None, 3].
                img_expanded = np.expand_dims(img, axis=0)  
                (boxes, scores, classes, num) = sess.run(
                    [self.d_boxes, self.d_scores, self.d_classes, self.num_d],
                    feed_dict={self.image_tensor: img_expanded})
        #print("At time: {} sec, End tf.".format(str(time.clock())))

        # Turn detection into pixel values.
        tl_loc = self.traffic_light_location(boxes, scores, classes, img.shape)
        #print(tl_loc[0])
        # No traffic lights found, look in Bernards original location.
        if len(tl_loc) == 0:
            tl_loc = [[x-275, y, x+275, y+150]]
            print("No Lights found by NN!")
        #imgOrig = img

        # median blur the image
        img = cv2.medianBlur(img, 5)
        # Convert image to HSV
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        ###################green color detection##########
        # define range of green color in HSV
        lower_green = np.array([60,125,125]) #100,100])
        upper_green = np.array([120,255,255])
        clr_ID = self.get_light_color(img, tl_loc[0], lower_green, upper_green)
        if (clr_ID == TrafficLight.GREEN):
            greenLight = True
        ##################red color detection#################
        # define range of red color in HSV
        lower_red = np.array([170,125,125]) 
        upper_red = np.array([179,255,255])
        clr_ID = self.get_light_color(img, tl_loc[0], lower_red, upper_red)
        if (clr_ID == TrafficLight.RED):
           redLight = True


        ###############yellow traffic light detection###########
        # define range of orange color in HSV
        lower_yellow = np.array([5,150,150]) 
        upper_yellow = np.array([40,255,255]) #real amber traffic light works 15,255,255])
        clr_ID = self.get_light_color(img, tl_loc[0], lower_yellow, upper_yellow)
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
        #print("At time: {} sec, End classification.".format(str(time.clock())))
        return clr_ID
