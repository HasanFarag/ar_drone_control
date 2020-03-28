#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist,Pose
from gazebo_msgs.msg import ContactsState
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import signal
import sys
import time
matplotlib.interactive(True)
first_time = False
first_time2 = False
def callback(msg):
    global first_time
    global ax
    #global ax2
    if first_time == False:
        fig = plt.figure()
        #ax = fig.add_subplot(1, 2, 1, projection='3d')
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim3d([3.5, 4.8])
	ax.set_xlabel('X [m]')
	ax.set_ylim3d([-2.0, 2.0])
	ax.set_ylabel('Y [m]')
	ax.set_zlim3d([0.0, 2.0])
	ax.set_zlabel('Z [m]')
        ax.set_title('drone contact points')    
    #    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    #    ax2.set_xlim3d([0, 5])
	#ax2.set_xlabel('FX [N]')
	#ax2.set_ylim3d([-2.0,  2.0])
	#ax2.set_ylabel('FY [N]')
	#ax2.set_zlim3d([0.0, 5.0])
	#ax2.set_zlabel('FZ [N]')
        #ax2.set_title('drone contact Forces')   
        
        first_time = True
    
    if len(msg.states) > 1:
        #rospy.loginfo("x1: %f, y1 = %f , z1= %f , x2: %f, y2 = %f , z2= %f",msg.states[0].contact_positions[0].x,msg.states[0].contact_positions[0].y,msg.states[0].contact_positions[0].z,msg.states[1].contact_positions[0].x,msg.states[1].contact_positions[0].y,msg.states[1].contact_positions[0].z)
        #rospy.loginfo("fx: %f, fy = %f , fz= %f",msg.states[0].total_wrench.force.x,msg.states[0].total_wrench.force.y,msg.states[0].total_wrench.force.z)
        plot3d = ax.plot([msg.states[0].contact_positions[0].x], [msg.states[0].contact_positions[0].y], msg.states[0].contact_positions[0].z,'ro')
	plot3d2 = ax.plot([msg.states[1].contact_positions[0].x], [msg.states[1].contact_positions[1].y], msg.states[1].contact_positions[0].z,'go')
        #plot3d3 = ax2.plot([msg.states[0].total_wrench.force.x], [msg.states[0].total_wrench.force.y], msg.states[0].total_wrench.force.z,'ro')
        #plot3d4 = ax2.plot([msg.states[1].total_wrench.force.x], [msg.states[1].total_wrench.force.y], msg.states[1].total_wrench.force.z,'ro')
        plt.draw()                     
        plt.pause(0.0001)
    if len(msg.states) == 1:
        #rospy.loginfo("x1: %f, y1 = %f , z1= %f ",msg.states[0].contact_positions[0].x,msg.states[0].contact_positions[0].y,msg.states[0].contact_positions[0].z)
        plot3d = ax.plot([msg.states[0].contact_positions[0].x], [msg.states[0].contact_positions[0].y], msg.states[0].contact_positions[0].z,'ro')
        #rospy.loginfo("fx: %f, fy = %f , fz= %f",msg.states[0].total_wrench.force.x,msg.states[0].total_wrench.force.y,msg.states[0].total_wrench.force.z)
        #plot3d3 = ax2.plot([msg.states[0].total_wrench.force.x], [msg.states[0].total_wrench.force.y], msg.states[0].total_wrench.force.z,'ro')
        plt.draw()                     
        plt.pause(0.0001)


  


def signal_handler(sig, frame):
    matplotlib.pyplot.close("all")
    sys.exit(0)
def listener():
    
    rospy.init_node('listener', anonymous=True)

    #signal.signal(signal.SIGINT, signal_handler)

    rospy.Subscriber("/link_0_contact", ContactsState, callback)

    #rospy.Subscriber("/drone/gt_pose", Pose, callback2)

   
    rospy.spin()

if __name__ == '__main__':
    listener()
else:
    print ('Program ended')
