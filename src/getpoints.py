#!/usr/bin/env python
import sip
sip.setapi('QVariant', 2)
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtWebKit import *
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError;
import numpy as np
from rospy.numpy_msg import numpy_msg
import yaml


refPt =[]

pointsworld=[]
i=0
n=0
camerano=1
brushregion=0
cv_image1 = None 
cv_image2 = None
cv_image3 = None
cv_image4 = None
cv_image5 = None
cv_image6 = None

class get_points :
  
 
 
  def __init__(self, source1, source2, source3, source4, source5, source6, sink1, sink2, sink3, sink4, sink5, sink6) :
    self.image_pub1 = rospy.Publisher(sink1, Image, queue_size=25)
    self.image_pub2 = rospy.Publisher(sink2, Image, queue_size=25)
    self.image_pub3 = rospy.Publisher(sink3, Image, queue_size=25)
    self.image_pub4 = rospy.Publisher(sink4, Image, queue_size=25)
    self.image_pub5 = rospy.Publisher(sink5, Image, queue_size=25)
    self.image_pub6 = rospy.Publisher(sink6, Image, queue_size=25)
  
    self.bridge = CvBridge()
    self.image_sub1 = rospy.Subscriber(source1, Image, self.callback1)
    self.image_sub2 = rospy.Subscriber(source2, Image, self.callback2)
    self.image_sub3 = rospy.Subscriber(source3, Image, self.callback3)
    self.image_sub4 = rospy.Subscriber(source4, Image, self.callback4)
    self.image_sub5 = rospy.Subscriber(source5, Image, self.callback5)
    self.image_sub6 = rospy.Subscriber(source6, Image, self.callback6)
 
  def get_cordinates(self):
    global pointsworld, n 
    if n<4:  
     frame = self.view.page().mainFrame()
     cords=frame.evaluateJavaScript('getCordinate();')
     worldxy=cords.split(" ")
     myStrList = [str(x) for x in worldxy]
     myfloat=[np.float32(j) for j in myStrList]
     pointsworld.append(myfloat)
     print pointsworld   
     n=n+1
     print n

    elif n==4:
        print "you cant have more points"
            


  def getPos(self , event):
       global refPt,i, camerano
       if i<4:
         x = event.pos().x()
         y = event.pos().y()
         refPt.append((x,y))
         i=i+1 
         
         print refPt
         print i
       else:
         print "you can only choose four, click reset to change"
         
  def brushregions(self):
         
      self.label.mouseMoveEvent = self.getzone

  def getpoints(self):
         
     self.label.mousePressEvent = self.getPos

      
  def getzone(self , event):
         global camerano
          
         x = event.pos().x()
         y = event.pos().y()
     
         if camerano == 4:  
          
          self.paint4.drawPoint(x,y)
          self.label.setPixmap(self.pixtemp4) 
         elif camerano ==5 :
          
          self.paint5.drawPoint(x,y)
          self.label.setPixmap(self.pixtemp5) 
         elif camerano ==6 :
  
          
          self.paint6.drawPoint(x,y)
          self.label.setPixmap(self.pixtemp6) 
         elif camerano ==3 :
          
          
          self.paint3.drawPoint(x,y)
          self.label.setPixmap(self.pixtemp3) 
         elif camerano == 2:
          
          
          self.paint2.drawPoint(x,y)
          self.label.setPixmap(self.pixtemp2) 
         else:
          
          
          self.paint.drawPoint(x,y)
          self.label.setPixmap(self.pixtemp)
         
   
  def reset_imgcordinates(self):
     global refPt, i
     i=0
     refPt[:]=[]
 

  def reset_worldcordinates(self):
      global  pointsworld, n
      n=0
      pointsworld[:]=[]
 
  def camera_one(self):
      global camerano
      camerano=1
      self.reset_imgcordinates()
      self.reset_worldcordinates()
      self.label.setPixmap(self.pix)

  def camera_two(self):
      global camerano
      camerano=2
      self.reset_imgcordinates()
      self.reset_worldcordinates()
      self.label.setPixmap(self.pix2)

  def camera_three(self):
      global camerano
      camerano=3
      self.reset_imgcordinates()
      self.reset_worldcordinates()
      self.label.setPixmap(self.pix3)

  def camera_four(self):
      global camerano
      camerano=4
      self.reset_imgcordinates()
      self.reset_worldcordinates()
      self.label.setPixmap(self.pix4)

  def camera_five(self):
      global camerano
      camerano=5
      self.reset_imgcordinates()
      self.reset_worldcordinates()
      self.label.setPixmap(self.pix5)

  def camera_six(self):
      global camerano
      camerano=6
      self.reset_imgcordinates()
      self.reset_worldcordinates()
      self.label.setPixmap(self.pix6)

  def cal_hom(self):
      global  pointsworld, refPt, camerano 
      if n==4 & i==4:
        points=np.array(refPt,dtype='float32')
        pointsworld2=np.array(pointsworld,dtype='float32') 
        #M = cv2.getPerspectiveTransform(points,pointsworld2)
        M, mask=cv2.findHomography(points,pointsworld2,cv2.RANSAC,5.0)
        print M
        
        if camerano == 4: 
           fname = '/home/enas/cam4.yaml'
        elif camerano ==5 :
           fname = '/home/enas/cam5.yaml'
        elif camerano ==6 :
           fname = '/home/enas/cam6.yaml'
        elif camerano ==3 :
           fname = '/home/enas/cam3.yaml'
        elif camerano == 2:
           fname = '/home/enas/cam2.yaml'
        else:
           fname = '/home/enas/cam1.yaml'
        print fname
        stream = open(fname, 'r')
        data = yaml.load(stream)

        data['instances'][0]['R'] = M.tolist()
        data['instances'][0]['P'] = points.tolist()

        with open(fname, 'w') as yaml_file:
           yaml_file.write( yaml.dump(data, default_flow_style=False))
       
        print points
        print pointsworld
        print pointsworld2
      else:
        print "not enough points"        
       
  def callback1(self, data1) :
    global cv_image1, cv_image2, cv_image3, cv_image4, cv_image5, cv_image6, camerano, brushregion
    try :
     
      cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
      
    except CvBridgeError, e:
      print  e
   
       
            
     # And a window
    self.app = QApplication([])
    self.win = QWidget()
    self.win.setWindowTitle('Get Coordinates')
    self.win.resize(800,800)
    
 
    # And give it a layout
    self.layout = QGridLayout() 
    self.win.setLayout(self.layout)
 
    cv2.cvtColor(cv_image1, cv2.COLOR_BGR2RGB, cv_image1)
    self.img= QImage(cv_image1, cv_image1.shape[1],cv_image1.shape[0], QImage.Format_RGB888)
    self.pix = QPixmap.fromImage(self.img)
    self.pixtemp = QPixmap.fromImage(self.img)
    self.paint= QPainter(self.pixtemp)
    self.brush= QBrush(QColor(255,0,0,255))
    self.pen=QPen(QColor(255,0,0,255),20)
    self.paint.setPen(self.pen)

    if cv_image2 != None: 
     cv2.cvtColor(cv_image2, cv2.COLOR_BGR2RGB, cv_image2)
     self.img2= QImage(cv_image2, cv_image2.shape[1],cv_image2.shape[0], QImage.Format_RGB888)
     self.pix2 = QPixmap.fromImage(self.img2)
     self.pixtemp2 = QPixmap.fromImage(self.img2)
     self.paint2= QPainter(self.pixtemp2)
     self.paint2.setPen(self.pen)


    if cv_image3 != None: 
     cv2.cvtColor(cv_image3, cv2.COLOR_BGR2RGB, cv_image3)
     self.img3= QImage(cv_image3, cv_image3.shape[1],cv_image3.shape[0], QImage.Format_RGB888)
     self.pix3 = QPixmap.fromImage(self.img3)
     self.pixtemp3 = QPixmap.fromImage(self.img3)
     self.paint3= QPainter(self.pixtemp3)
     self.paint3.setPen(self.pen)


    if cv_image4 != None: 
     cv2.cvtColor(cv_image4, cv2.COLOR_BGR2RGB, cv_image4)
     self.img4= QImage(cv_image4, cv_image4.shape[1],cv_image4.shape[0], QImage.Format_RGB888)
     self.pix4 = QPixmap.fromImage(self.img4)
     self.pixtemp4 = QPixmap.fromImage(self.img4)
     self.paint4= QPainter(self.pixtemp4)
     self.paint4.setPen(self.pen)

    if cv_image5 != None: 
     cv2.cvtColor(cv_image5, cv2.COLOR_BGR2RGB, cv_image5)
     self.img5= QImage(cv_image5, cv_image5.shape[1],cv_image5.shape[0], QImage.Format_RGB888)
     self.pix5 = QPixmap.fromImage(self.img5)
     self.pixtemp5 = QPixmap.fromImage(self.img5)
     self.paint5= QPainter(self.pixtemp5)
     self.paint5.setPen(self.pen)

    if cv_image6 != None: 
     cv2.cvtColor(cv_image6, cv2.COLOR_BGR2RGB, cv_image6)
     self.img6= QImage(cv_image6, cv_image6.shape[1],cv_image6.shape[0], QImage.Format_RGB888)
     self.pix6 = QPixmap.fromImage(self.img6)
     self.pixtemp6 = QPixmap.fromImage(self.img6)
     self.paint6= QPainter(self.pixtemp6)
     self.paint6.setPen(self.pen)

    


   
    self.label = QLabel()
    self.label.setPixmap(self.pix)
    self.view = QWebView()  
    self.view.setHtml("""
   <html>
       <head>
         <title>A Demo Page</title>
 
         <script language="javascript">
           // Completes the full-name control and
           // shows the submit button
           function getCordinate() {
             var xcor = document.getElementById('xcor').value;
             var ycor = document.getElementById('ycor').value;
             var wc = xcor +" "+ ycor;
             document.getElementById("myForm").reset();
             return wc;
           }

         </script>
       </head>
 
       <body>
         <form id="myForm">
           <label for="xcor">X cordinate:</label>
           <input type="number" name="xcor" id="xcor"></input>
          <br />
           <label for="ycor">Y cordiante:</label>
           <input type="number" name="ycor" id="ycor"></input>
      
         </form>
       </body>
     </html>
    """)
 
    # A button to call our JavaScript
    self.button = QPushButton('Save world Cordinates')
    # Connect 'complete_name' to the button's 'clicked' signal
    self.button.clicked.connect(self.get_cordinates)
    # A button to call our JavaScript
    self.button2 = QPushButton('reset image coordnates')
    # Connect 'complete_name' to the button's 'clicked' signal
    self.button2.clicked.connect(self.reset_imgcordinates)
    # A button to call our JavaScript
    self.button3 = QPushButton('Reset World Cordinates')
    # Connect 'complete_name' to the button's 'clicked' signal
    self.button3.clicked.connect(self.reset_worldcordinates)
    # A button to call our JavaScript
    self.button4 = QPushButton('calculate Homagraphy')
    # Connect 'complete_name' to the button's 'clicked' signal
    self.button4.clicked.connect(self.cal_hom)

    self.button5 = QPushButton('camera one')
    self.button5.clicked.connect(self.camera_one)
    self.button6 = QPushButton('camera two')
    self.button6.clicked.connect(self.camera_two)
    self.button7 = QPushButton('camera three')
    self.button7.clicked.connect(self.camera_three)
    self.button8 = QPushButton('camera four')
    self.button8.clicked.connect(self.camera_four)
    self.button9 = QPushButton('camera five')
    self.button9.clicked.connect(self.camera_five)
    self.button10 = QPushButton('camera six')
    self.button10.clicked.connect(self.camera_six)
    self.button11 = QPushButton('Brush regions')
    self.button11.clicked.connect(self.brushregions)
    self.button12 = QPushButton('Get Points')
    self.button12.clicked.connect(self.getpoints)
    # Add the QWebView and button to the layout
    self.layout.addWidget(self.label,0,0)
    self.layout.addWidget(self.view,1,0)
    self.layout.addWidget(self.button,2,0)
    self.layout.addWidget(self.button2,3,0)
    self.layout.addWidget(self.button3,4,0)
    self.layout.addWidget(self.button4,5,0)
    self.layout.addWidget(self.button5,2,1)
    self.layout.addWidget(self.button6,3,1)
    self.layout.addWidget(self.button7,4,1)
    self.layout.addWidget(self.button8,5,1)
    self.layout.addWidget(self.button9,6,1)
    self.layout.addWidget(self.button10,7,1)
    self.layout.addWidget(self.button11,6,0)
    self.layout.addWidget(self.button12,7,0)
    
   
   
     
      
    # Show the window and run the app
    self.win.show()
    self.app.exec_()


  
    try :
      
       
        self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.pixtemp, "bgr8"))
       
       
    except CvBridgeError, e:
        print e
    try :
      
       
        self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.pixtemp2, "bgr8"))
       
       
    except CvBridgeError, e:
        print e 
    try :
      
       
        self.image_pub3.publish(self.bridge.cv2_to_imgmsg(self.pixtemp3, "bgr8"))
       
       
    except CvBridgeError, e:
        print e
    try :
      
       
        self.image_pub4.publish(self.bridge.cv2_to_imgmsg(self.pixtemp4, "bgr8"))
       
       
    except CvBridgeError, e:
        print e  
    try :
      
       
        self.image_pub5.publish(self.bridge.cv2_to_imgmsg(self.pixtemp5, "bgr8"))
       
       
    except CvBridgeError, e:
        print e

    try :
      
       
        self.image_pub6.publish(self.bridge.cv2_to_imgmsg(self.pixtemp6, "bgr8"))
       
       
    except CvBridgeError, e:
        print e  
 
    
         
  def callback2(self, data2) :
    global  cv_image2 
    try :
      
      cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
     
    except CvBridgeError, e:
      print e
   
    

  def callback3(self, data3) :
    global  cv_image3
    try :
      cv_image3 = self.bridge.imgmsg_to_cv2(data3, "bgr8")
     
    except CvBridgeError, e:
      print e
    
   

  def callback4(self, data4) :
    global  cv_image4
    try :
      cv_image4 = self.bridge.imgmsg_to_cv2(data4, "bgr8")
      
    except CvBridgeError, e:
      print e
   

  def callback5(self, data5) :
    global  cv_image5
    try :
      cv_image5 = self.bridge.imgmsg_to_cv2(data5, "bgr8")
      
    except CvBridgeError, e:
      print e


  def callback6(self, data6) :
    global  cv_image6
    try :
      cv_image6 = self.bridge.imgmsg_to_cv2(data6, "bgr8")
      
    except CvBridgeError, e:
      print e          
            
          

def main(args) :
 
  rospy.init_node('get_points')

  arg_defaults = {
        'source1': '/watcher1/image_raw',
        'source2': '/watcher2/image_raw',
        'source3': '/watcher3/image_raw',
        'source4': '/watcher4/image_raw',
        'source5': '/watcher5/image_raw',
        'source6': '/watcher6/image_raw',
        'sink1': '/nozone1/image_raw',
        'sink2': '/nozone2/image_raw',
        'sink3': '/nozone3/image_raw',
        'sink4': '/nozone4/image_raw',
        'sink5': '/nozone5/image_raw',
        'sink6': '/nozone6/image_raw'       
        }
  args = updateArgs(arg_defaults)
 
  get_points(**args)
  try :
    rospy.spin()
  except KeyBoardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

def updateArgs(arg_defaults):
    '''Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces.  '''
    args = {}
    for name, val in arg_defaults.iteritems():
        full_name = rospy.search_param(name)
        print "name ", name, full_name
        if full_name is None:
            args[name] = val
        else:
            args[name] = rospy.get_param(full_name, val)
            print "We have args " + val + " value " + args[name]
    return(args)

if __name__ == '__main__':
  main(sys.argv)
