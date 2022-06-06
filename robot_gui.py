
from tkinter import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from threading import *

class GuiContoller(Node):

    def moveForward(self):
        msg = String()
        print("Forward")
        msg.data = "MOVEF"
        self.publisher.publish(msg)

    def moveBackward(self):
        msg = String()
        print("Back")
        msg.data = "MOVEB"
        self.publisher.publish(msg)

    def moveLeft(self):
        msg = String()
        print("Left")
        msg.data = "TURNL"
        self.publisher.publish(msg)

    def moveRight(self):
        msg = String()
        print("Right")
        msg.data = "TURNR"
        self.publisher.publish(msg)
        
    def speedUp(self):
        global speed
        msg = String()
        print("Speed Up")
        msg.data = "SPEEU"
        if (speed < 250):
             speed += 10
        
        self.publisher.publish(msg)
        
    def speedDown(self):
        global speed
        msg = String()
        print("Speed Down")
        msg.data = "SPEED"
        if (speed > 110):
             speed -= 10
        
        self.publisher.publish(msg)

    def left_callback(self, msg):
        global left
        try:
            left = int(msg.data[:-2]) #remove the 'cm' and convert to int.
        except:
            print("error")

    def right_callback(self, msg):
        global right
        try:
            right = int(msg.data[:-2]) #remove the 'cm' and convert to int.
        except:
            print("error")

    def front_callback(self, msg):
        global front
        try:
            front = int(msg.data[:-2]) #remove the 'cm' and convert to int.
        except:
            print("error")
        
    def auto(self):
    	msg = String()
    	global auto
    	auto = 1
    	print("Auto Mode")
    	msg.data = "AUTOY"
    	self.publisher.publish(msg)
        
    def manual(self):
    	msg = String()
    	global auto
    	auto = 0
    	print("Manual Mode")
    	msg.data = "AUTON"
    	self.publisher.publish(msg)
    	

    def __init__(self):
        super().__init__('GuiContoller')
        self.publisher = self.create_publisher(String, '/robot/control', 1)

        self.subscription = self.create_subscription(
            String,
            '/robot/left',
            self.left_callback,
            1)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            String,
            '/robot/front',
            self.front_callback,
            1)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            String,
            '/robot/right',
            self.right_callback,
            1)
        self.subscription  # prevent unused variable warning




def main(args=None):
    global left
    global front
    global right
    global speed
    global auto
    
    speed = 180

    left = 0
    right = 0
    front = 0
    
    auto = 0

    rclpy.init(args=args)
    controller = GuiContoller()

    root = Tk()
    root.geometry('550x270')
    root.title("Robot Controller")

    btn = Button(root, text = 'Forward', command =  controller.moveForward)
    btn.place(x=93, y=25)

    btn2 = Button(root, text = 'Left', command =  controller.moveLeft)
    btn2.place(x=25, y=60)

    btn4 = Button(root, text = 'Backward', command =  controller.moveBackward)
    btn4.place(x=85, y=60)

    btn3 = Button(root, text = 'Right', command = controller.moveRight)
    btn3.place(x=185, y=60)
    
    sensorLabel = Label(root) 
    sensorLabel.place(x=330, y=15)
    sensorLabel.config(text="Sensor Readings:")

    leftLabel = Label(root) 
    leftLabel.place(x=330, y=55)
    leftLabel.config(text="Left Reading: %d" % left)

    rightLabel = Label(root) 
    rightLabel.place(x=330, y=90)
    rightLabel.config(text="Right Reading: %d" % right)

    frontLabel = Label(root) 
    frontLabel.place(x=330, y=125)
    frontLabel.config(text="Front Reading: %d" % front)
    
    controlLabel = Label(root) 
    controlLabel.place(x=85, y=105)
    controlLabel.config(text="Control Modes:")

    btn5 = Button(root, text = 'Auto Mode', command = controller.auto)
    btn5.place(x=25, y=140)
    btn5 = Button(root, text = 'Manual Mode', command = controller.manual)
    btn5.place(x=140, y=140)
    
    speedLabel = Label(root) 
    speedLabel.place(x=60, y=185)
    speedLabel.config(text="Robot Speed: %d" % speed)
    
    btn6 = Button(root, text = 'Speed Up', command =  controller.speedUp) 
    btn6.place(x=35, y=220)
    btn7 = Button(root, text = 'Speed Down', command =  controller.speedDown)
    btn7.place(x=140, y=220)
    
    autoLabel = Label(root) 
    autoLabel.place(x=330, y=165)
    
    autoLabel.config(text="Operating Mode:")
    

    def spinCheck():
        rclpy.spin_once(controller)
        root.after(50, spinCheck)  # reschedule event in 2 seconds
        frontLabel.config(text="Front Reading: %d" % front, foreground = "black")
        rightLabel.config(text="Right Reading: %d" % right, foreground = "black")
        leftLabel.config(text="Left Reading: %d" % left, foreground = "black")
        speedLabel.config(text="Robot Speed: %d" % speed)
        
        if(front <= 30):
            frontLabel.config(text="Front Reading: %d" % front, foreground = "red")
        if (left <= 20):
            leftLabel.config(text="Left Reading: %d" % left, foreground = "red")
        if (right <= 20):
            rightLabel.config(text="Right Reading: %d" % right, foreground = "red")
            
        global auto
        
        if (auto == 1):
            autoLabel.config(text="Operating Mode: Automatic")
        else:
            autoLabel.config(text="Operating Mode: Manual")
        
        

    root.after(50, spinCheck)
    root.mainloop()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
