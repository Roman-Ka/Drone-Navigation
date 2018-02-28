#!/usr/bin/env python

# Every Python ROS Node will have this declaration at the top.
# The first line makes sure your script is executed as a Python script.

# import statements:
import rospy
import math
import sys
import time

# ROS stuff:
# from mavros_msgs.msg import OpticalFlowRad  # import optical flow message structure
# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import Pose  # import position message structures
# from geometry_msgs.msg import Point

from mavros_msgs.msg import State  # import state message structure
from sensor_msgs.msg import Range  # import range message structure
from sensor_msgs.msg import Imu  # IMU readings
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped  # used to set velocity messages
from mavros_msgs.srv import *  # import for arm and flight mode setting


from tf.transformations import euler_from_quaternion  # angle transformation


class velControl:
    def __init__(self, attPub):  # attPub = attitude publisher
        self._attPub = attPub
        self._setVelMsg = TwistStamped()
        self._targetVelX = 0
        self._targetVelY = 0
        self._targetVelZ = 0

        self._targetAngVelX = 0
        self._targetAngVelY = 0
        self._targetAngVelZ = 0

    def setVel(self, coordinates):
        self._targetVelX = float(coordinates[0])
        self._targetVelY = float(coordinates[1])
        self._targetVelZ = float(coordinates[2])

    def setAngVel(self, coordinates):
        self._targetAngVelX = float(coordinates[0])
        self._targetAngVelY = float(coordinates[1])
        self._targetAngVelZ = float(coordinates[2])

    def publishTargetPose(self, stateManagerInstance):
        self._setVelMsg.header.stamp = rospy.Time.now()  # construct message to publish with time, loop count and id
        self._setVelMsg.header.seq = stateManagerInstance.getLoopCount()
        self._setVelMsg.header.frame_id = 'fcu'

        self._setVelMsg.twist.linear.x = self._targetVelX
        self._setVelMsg.twist.linear.y = self._targetVelY
        self._setVelMsg.twist.linear.z = self._targetVelZ

        self._setVelMsg.twist.angular.x = self._targetAngVelX
        self._setVelMsg.twist.angular.y = self._targetAngVelY
        self._setVelMsg.twist.angular.z = self._targetAngVelZ

        self._attPub.publish(self._setVelMsg)


class stateManager:  # class for monitoring and changing state of the controller
    def __init__(self, rate):
        self._rate = rate
        self._loopCount = 0
        self._isConnected = 0
        self._isArmed = 0
        self._mode = None

    def incrementLoop(self):
        self._loopCount = self._loopCount + 1

    def getLoopCount(self):
        return self._loopCount

    def stateUpdate(self, msg):
        self._isConnected = msg.connected
        self._isArmed = msg.armed
        self._mode = msg.mode
        rospy.logwarn("Connected is {}, armed is {}, mode is {} ".format(self._isConnected, self._isArmed,
                                                                         self._mode))  # some status info

    def armRequest(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            modeService = rospy.ServiceProxy('/mavros/set_mode',
                                             mavros_msgs.srv.SetMode)  # get mode service and set to offboard control
            modeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("Service mode set faild with exception: %s" % e)

    def offboardRequest(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm = rospy.ServiceProxy('/mavros/cmd/arming',
                                     mavros_msgs.srv.CommandBool)  # get arm command service and arm
            arm(True)
        except rospy.ServiceException as e:  # except if failed
            print("Service arm failed with exception :%s" % e)

    def waitForPilotConnection(self):  # wait for connection to flight controller
        rospy.logwarn("Waiting for pilot connection")
        while not rospy.is_shutdown():  # while not shutting down
            if self._isConnected:  # if state isConnected is true
                rospy.logwarn("Pilot is connected")
                return True
            self._rate.sleep
        rospy.logwarn("ROS shutdown")
        return False
###############################################################################
class PID:
    def __init__(self,P,I,D,_integrator=0,_derivator=0,integrator_max=500,integrator_min=-500):
        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=_derivator #stores derivative of the error --> multiply this by Kd
        self.Integrator=_integrator #stores integral of the error --> multiply this by Ki
        self.set_point=0.0
        self.error=0.0 # --> multiply this by Kp
        self.velToSet=0

    def setPoint(self,set_point):
	self.set_point = set_point
	self.Integrator=0
	self.Derivator=0

    def update(self,position):
        self.error=self.set_point - position
        #set the P, I and D
        self.propTerm=self.Kp*self.error
        self.derTerm=self.Kd*(self.error-self.Derivator)
        self.Derivator=self.error #for the next loop

        self.Integrator=self.Integrator + self.error
        self.intTerm=self.Ki*self.Integrator
        self.velToSet=self.propTerm+self.intTerm+self.derTerm
        return self.velToSet

    def getSetpoint(self) :
        return self.set_point

        

###############################################################################
def distanceCheck(msg):
    global grange  # import global range
    # print(msg.range)  # for debugging
    grange = msg.range  # set range = recieved range


def angleCheck(msg):
    global roll
    global pitch
    global yaw
    (roll, pitch, yaw) = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    # rospy.loginfo("\nIMU roll: {} \nIMU pitch: {} \nIMU yaw: {} \n ---".format(roll, pitch, yaw))


def callback(msg):
    global x
    global y
    global z
    x = msg.integrated_x
    y = msg.integrated_y
    z = msg.distance
    # rospy.loginfo("\nOptical Flow x: {} \nOptical Flow y: {} \nOptical Flow z: {} \n ---".format(x, y, z))


def positionCheck(msg):
    global posxx
    global posyy
    posxx = msg.pose.position.x
    posyy = msg.pose.position.y
    # rospy.loginfo("\nPosition x: {} \nPosition y: {} \n ---".format(posxx, posyy))


def headingCheck(msg):
    global headx
    global heady
    global headz
    headx = msg.magnetic_field.x
    heady = msg.magnetic_field.y
    headz = msg.magnetic_field.z
    # rospy.loginfo("\nHeading x: {} \nHeading y: {} \nHeading z: {} \n ---".format(headx, heady, headz))

def velocityCheck(msg):
    global linx
    global liny
    global linz
    linx = msg.twist.linear.x
    liny = msg.twist.linear.y
    linz = msg.twist.linear.z
    # rospy.loginfo("\nVelocity x: {} \nVelocity y: {} \nVelocity z: {} \n ---".format(linx, liny, linz))


############################################################# FUNCTIONS RUNNING IN MAIN #####################################################################
        
def startup() : #getting off the ground phase
    global goto1st
    global has_started_flying
    global xpositionControl014
    global zpositionControl014
    global yawControl
    global xVelocity
    global zVelocity
    global counter
   
    if trueheight<=0.2 :
        counter=counter+1
        xpositionControl014.setPoint(0.0) #setting them within the while loop as it doesnt really matter where we do it
        zpositionControl014.setPoint(1.5) #this only changes in phase4()
        ypositionControl.setPoint(0.0)
        yawControl.setPoint(0.0) #never changes

    else :
        goto1st=1
        has_started_flying=1
    xVelocity=xpositionControl014.update(posxx) #need to update xVelocity in each stage seperately as some use PID and others don't.
    zVelocity=zpositionControl014.update(trueheight)
    if zVelocity>0.5:
        zVelocity=0.5
    
def phase1() : #trying to hover at 1.5m before moving forward phase
    global posxx
    global goto1st
    global goto2nd
    global currentime
    global prevtime
    global currentpos
    global previouspos
    global check
    global i
    global is_hovering
    global firstphase1run
    global xpositionControl014
    global xVelocity
    global zVelocity
    global j
    global zpositionControl014
    xpositionControl014.setPoint(0)
    zpositionControl014.setPoint(1.5)
    rospy.loginfo("\zsetpoint: {} \n ---".format(zpositionControl014.getSetpoint()))

    if(firstphase1run): #only runs the first time phase1() runs so that hover checking variables are initialised
        is_hovering=0
        check=[0]
        i=0
        prevtime=time.time()
        previouspos=[posxx,posyy,grange]
        firstphase1run=0
        j=0
        

    currenttime=time.time()
    if (currenttime-prevtime>0.4) :
        i=i+1
        previouspos=currentpos
        currentpos=[posxx,posyy,grange]
        prevtime=currenttime #prev time records 0.4 second intervals
        if abs(currentpos[0]-previouspos[0])>0.05 or abs(currentpos[1]-previouspos[1])>0.05 or abs(currentpos[2]-previouspos[2])>0.05 :
            check.append(0) 
        else :
            check.append(1)
            j=j+1

        if (j>=4): #checking how the drone has behaved in the last 5 intervals of 0.4 seconds (i.e over the last 2 seconds)
            checkfiltered=[x for x in check if (i-4)<=x<=i] #taking the last 5 values of check 
            if all(x==1 for x in checkfiltered) : # if drone has moved minimally in the given time period then checkfiltered=[1,1,1,1,1]
                is_hovering=1
                goto2nd=1 #stop returning to phase1() and proceed
                goto1st=0
                
   #set points for PID in this stage are exactly the same as for startup so no need to modify
    xVelocity=xpositionControl014.update(posxx) #need to update xVelocity in each stage seperately as some use PID and others don't.
    zVelocity=zpositionControl014.update(trueheight)
    if zVelocity>0.5:
        zVelocity=0.5
        
        
def phase2() : #moving over ramp at steady speed phase
    global posxx #don't actually need to import global as we're not modifying
    global goto2nd
    global goto3rd
    global xVelocity
    global zVelocity
    global zpositionControl2
    zpositionControl2.setPoint(1.5)
    #set points are still the same for PID, although in the next line we opt not to use PID
    xVelocity=0.3
    zVelocity=zpositionControl2.update(trueheight)
    if trueheight>1.7:
        xVelocity=0
    if posxx>3.5 :
        goto2nd=0
        goto3rd=1 #stop returning to phase2() and proceed

def phase3() : #moving to 6m from starting position and hovering for a second phase
    global posxx
    global goto3rd
    global goto4th
    global currentime
    global prevtime
    global currentpos
    global previouspos
    global check
    global i
    global is_hovering
    global firstphase3run
    global xpositionControl3
    global xVelocity
    global zVelocity
    global j
    global zpositionControl3
    xpositionControl3.setPoint(6.0)
    zpositionControl3.setPoint(1.5)
    

    if(firstphase3run): #only runs the first time phase3() runs so that hover checking variables are initialised
        is_hovering=0
        check=[0]
        i=0
        prevtime=time.time()
        previouspos=[posxx,posyy,grange]
        firstphase3run=0
        j=0

    currenttime=time.time()
    if (currenttime-prevtime>0.4) :
        i=i+1
        previouspos=currentpos
        currentpos=[posxx,posyy,grange]
        prevtime=currenttime #prev time records 0.4 second intervals
        if abs(currentpos[0]-previouspos[0])>0.025 or abs(currentpos[1]-previouspos[1])>0.025 or abs(currentpos[2]-previouspos[2])>0.01 :
            check.append(0) 
        else :
            check.append(1)
            j=j+1

        if (j>=4):#checking how the drone has behaved in the last 5 intervals of 0.4 seconds (i.e over the last 2 seconds)
            checkfiltered=[x for x in check if (i-4)<=x<=i] #taking the last 5 values of check 
            if all(x==1 for x in checkfiltered) : # if drone has moved minimally in the given time period then checkfiltered=[1,1,1,1,1]
                is_hovering=1
                goto4th=1 #stop returning to phase3() and proceed
                goto3rd=0
    xVelocity=xpositionControl3.update(posxx)
    zVelocity=zpositionControl3.update(trueheight)

    if xVelocity>0.4 :
        xVelocity=0.4
    if trueheight>2 :
        xVelocity=0
    if zVelocity<-0.5:
        zVelocity=-0.5
    


def phase4() : #landing phase
    global posxx
    global goto4th
    global currentime
    global prevtime
    global currentpos
    global previouspos
    global check
    global i
    global is_hovering
    global firstphase4run
    global zpositionControl014
    global xpositionControl014
    global xVelocity
    global yVelocity
    global zVelocity
    global want_to_land
    global j
    xpositionControl014.setPoint(6.0)
    zpositionControl014.setPoint(0.12)
    

    if(firstphase4run): #only runs the first time phase3() runs so that hover checking variables are initialised
        is_hovering=0
        check=[0]
        i=0
        prevtime=time.time()
        previouspos=[posxx,posyy,grange]
        firstphase4run=0
        j=0

    currenttime=time.time()
    if (currenttime-prevtime>0.4) :
        i=i+1
        previouspos=currentpos
        currentpos=[posxx,posyy,grange]
        prevtime=currenttime #prev time records 0.4 second intervals
        if abs(currentpos[0]-previouspos[0])>0.05 or abs(currentpos[1]-previouspos[1])>0.05 or abs(currentpos[2]-previouspos[2])>0.05 :
            check.append(0) 
        else :
            check.append(1)
            j=j+1

        if (j>=4): #checking how the drone has behaved in the last 5 intervals of 0.4 seconds (i.e over the last 2 seconds)
            checkfiltered=[x for x in check if (i-4)<=x<=i] #taking the last 5 values of check 
            if all(x==1 for x in checkfiltered) : # if drone has moved minimally in the given time period then checkfiltered=[1,1,1,1,1]
                is_hovering=1
                want_to_land=1
                goto4th=0 #stop returning to phase4()
                xVelocity=0
                yVelocity=0
                zVelocity=0
                

    if(not is_hovering) :
         xVelocity=xpositionControl014.update(posxx) #set point is still 6m
         zVelocity=zpositionControl014.update(trueheight) #set point is still 6m
         if zVelocity>0.5:
             zVelocity=0.5
    





##################################################################################################################################################

def main():

    rospy.init_node('node')

    # Global variables
    global grange
    global roll
    global pitch
    global yaw
    global posxx
    global posyy
    global headx
    global heady
    global headz
    global linx
    global liny
    global linz
    global trueheight
    #global xVelocity
    #global yVelocity
    global zVelocity
    (posxx, posyy, roll, pitch, pitch, headx, heady, headz, grange, linx, liny, linz, yaw) = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    rate = rospy.Rate(20)  # rate will update publisher
    stateManagerInstance = stateManager(rate)  # create new statemanager

    # Subscriptions
    rospy.Subscriber("/mavros/state", State, stateManagerInstance.stateUpdate)
    rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range, distanceCheck)
    # Do we actually need the optical flow?
    # rospy.Subscriber("/mavros/px4flow/raw/optical_flow_rad", OpticalFlowRad, callback)
    rospy.Subscriber("/mavros/imu/data", Imu, angleCheck)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, positionCheck)
    rospy.Subscriber("/mavros/imu/mag", MagneticField, headingCheck)
    rospy.Subscriber("mavros/local_position/velocity", TwistStamped, velocityCheck)

    # Publishers
    velPub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2)

    controller = velControl(velPub)  # create new controller class and pass in publisher and state manager
    stateManagerInstance.waitForPilotConnection()  # wait for connection to flight controller

    global zpositionControl014
    zpositionControl014 = PID(1.2, 0.00005, 0.55) #create PID controller objects with specified gains
    
    global xpositionControl014
    xpositionControl014 = PID(1.2, 0.000015, 0.6)
    
    global ypositionControl
    ypositionControl = PID(0.9, 0.000015, 0.3)
    
    global yawControl
    yawControl = PID(1, 0, 0)

    global zpositionControl2
    zpositionControl2 = PID(1.1, 0.00002, 1) #create PID controller objects with specified gains #0.6 deriv
    
    global zpositionControl3
    zpositionControl3 = PID(1.1, 0.00002, 1) #create PID controller objects with specified gains #0.8 deriv
    
    global xpositionControl3
    xpositionControl3 = PID(0.9, 0.000015, 0.6)


    

    global i
    i=0
    global is_hovering
    is_hovering=0
    global has_started_flying
    has_started_flying=0
    global goto1st
    goto1st=0
    global goto2nd
    goto2nd=0 #variable is 1 if second stage has started
    global goto3rd
    goto3rd=0
    global goto4th
    goto4th=0
    global start_time
    start_time=time.time()
    global currenttime
    currentime=start_time
    global prevtime
    prevtime=start_time
    global currentpos
    currentpos=[0,0,0]
    global previouspos
    previouspos=[0,0,0]
    global check
    check=[0] #if all elements are 1, then check is complete and the rover has hovered for 2 seconds (i.e every check is at 0.4s, 0.8s, 1.2s...)
    global firstphase1run
    firstphase1run=1 #variables that are true for a first run of the phase, so that hover checking variables are reinitialised as required within the phase
    global firstphase3run
    firstphase3run=1
    global firstphase4run
    firstphase4run=1
    global counter
    counter=0
    global want_to_land
    want_to_land=0
    global j
    j=0
    
    
    while not rospy.is_shutdown():


        # Run a short loop to ensure all of the subscriptions are initiated
        if any(v == 0 for v in (posxx, posyy, roll, pitch, pitch, headx, heady, headz, grange)):
            print 'Waiting for all of the subscriptions'
            rate.sleep()
        trueheight=grange*math.cos(pitch)
##########################################################################

        if(has_started_flying==0):
            startup() #set goto1st as true after grange>0.2 or so. Use PID for height control. Set has_started_flying=1
        if(goto1st):
            phase1() #Still using PID for height control but checking for 2sec hover. Set goto2nd=1 when is_hovering=true and goto1st=0 
        elif(goto2nd):
            phase2() #change xvelocity to desired value over the ramp. Set goto3rd=1 when posxx>3.5 and goto2nd=0
        elif(goto3rd):
            phase3() #Use PID to get posxx=6. Check for 1 second hover. When is_hovering=true set goto3rd=0 and goto4th=1 AND is_hovering=false
        elif(goto4th):
            phase4() #Use PID to reduce height (make sure overdamped) to about 0.05 m. Then set all velocities to 0
            if(want_to_land):
                break
        if(posxx>6.5):
            break
        #xVelocity isnt always controlled by PID (see phase2()) so need to set it individually in each case
	#global xVelocity
        #global yVelocity
        yVelocity=ypositionControl.update(posyy) #yVelocity, zVelocity and yawRate are always set using PID
        yawRate = yawControl.update(yaw)
	#global zVelocity
        
        #if trueheight>1.7 :
            #zVelocity=-0.4

        if want_to_land :
            zVelocity=0
            yVelocity=0
            
            
            
	#global yawRate
       




################################################################        
        rospy.loginfo("\nTarget Velocity Z: {} \n ---".format(zVelocity))
        rospy.loginfo("\ntrueheight: {} \n ---".format(trueheight))
        rospy.loginfo("\ngrange: {} \n ---".format(grange))
        #rospy.loginfo("\nYposition: {} \n ---".format(posyy))
        #rospy.loginfo("\nTarget YAW: {} \n ---".format(yaw))
        rospy.loginfo("\nXposition: {} \n ---".format(posxx))
        #rospy.loginfo("\ntime: {} \n ---".format(currenttime-start_time))
        rospy.loginfo("\nXvelocity: {} \n ---".format(xVelocity))
        #rospy.loginfo("\nis_hovering: {} \n ---".format(is_hovering))
        #rospy.loginfo("\nisphase1: {} \n ---".format(goto1st))
	#rospy.loginfo("\nisphase2: {} \n ---".format(goto2nd))
        #rospy.loginfo("\ncounter : {} \n ---".format(counter))
	#rospy.loginfo("\nisphase3: {} \n ---".format(goto3rd))
        #rospy.loginfo("\nisphase4: {} \n ---".format(goto4th))
       
        #rospy.loginfo("\ni: {} \n ---".format(i))
        #rospy.loginfo("\nj: {} \n ---".format(j))

        controller.setVel([xVelocity,yVelocity, zVelocity])
        controller.setAngVel([0, 0, yawRate])
            

        controller.publishTargetPose(stateManagerInstance)
        stateManagerInstance.incrementLoop()

        rate.sleep()  # sleep at the set rate

        # Don't know what that is, but better not to touch it
        if stateManagerInstance.getLoopCount() > 100 :  # to prevent offboard rejection
            stateManagerInstance.offboardRequest()  # request control from external computer
            stateManagerInstance.armRequest()  # arming must take place after offboard is requested

    rospy.spin()


if __name__ == '__main__':
    main()

############################## DELETE EVERYTHING AFTER THIS LINE, USE FOR COMING UP WITH IDEAS
