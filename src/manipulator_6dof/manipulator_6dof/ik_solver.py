import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
from math import *
import numpy as np

l1,l2,l34,l56=116, 105,133,126

class IkSolver(Node):
    def __init__(self):
        super().__init__("ik_solver")
        self.data = JointState()
        self.data.name = ["revolute1", "revolute2", "revolute3", "revolute4", "revolute5", "revolute6"]
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.x=0
        self.create_timer(0.1, self.timer_callback)
        

    def ik(self):
        px, py, pz=250, 0, 150
        self.x+=pi/20
        # Rst=np.array([[0,1,0],[1,0,0],[0,0,-1]])
        Rst=np.array([[0,0,1],[1,0,0],[0,1,0]])
        Rz=np.array([[cos(self.x), -sin(self.x),0],[sin(self.x),cos(self.x),0],[0,0,1]])
        Rx=np.array([[1,0,0],[0,cos(pi/4),-sin(pi/4)],[0,sin(pi/4),cos(pi/4)]])
        R=Rst@Rz@Rx
        r11, r12, r13, r21, r22, r23, r31, r32, r33=R[0,0],R[0,1],R[0,2],R[1,0],R[1,1],R[1,2],R[2,0],R[2,1],R[2,2]
        wx, wy, wz=px-l56*r13, py-l56*r23, pz-l56*r33
        theta1=atan2(wy,wx)
        r=sqrt(wx**2+wy**2)
        s=wz-l1

        c3=(r**2+s**2-l2**2-l34**2)/(2*l2*l34)
        s3=-sqrt(1-c3**2)
        
        theta2=atan2(s,r)-atan2(l34*s3,l2+l34*c3)
        theta3=atan2(s3,c3)

        theta5=atan2(-sqrt(1-(cos(theta1)*cos(theta2+theta3)*r13+sin(theta1)*cos(theta2+theta3)*r23+sin(theta2+theta3)*r33)**2), cos(theta1)*cos(theta2+theta3)*r13+sin(theta1)*cos(theta2+theta3)*r23+sin(theta2+theta3)*r33)
        s4=(sin(theta1)*r13-cos(theta1)*r23)/sin(theta5)
        c4=(-cos(theta1)*sin(theta2+theta3)*r13-sin(theta1)*sin(theta2+theta3)*r23+cos(theta2+theta3)*r33)/sin(theta5)
        theta4=atan2(s4,c4)
        s6=(cos(theta1)*cos(theta2+theta3)*r11+sin(theta1)*cos(theta2+theta3)*r21+sin(theta2+theta3)*r31)/(-sin(theta5))
        c6=(cos(theta1)*cos(theta2+theta3)*r12+sin(theta1)*cos(theta2+theta3)*r22+sin(theta2+theta3)*r32)/(-sin(theta5))
        theta6=atan2(s6,c6)
        self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6 =theta1, theta2-pi/2, theta3, theta4, theta5, theta6
        print("theta1 : ",theta1*180/pi)
        print("theta2 : ",theta2*180/pi)
        print("theta3 : ",theta3*180/pi)
        print("theta4 : ",theta4*180/pi)
        print("theta5 : ",theta5*180/pi)
        print("theta6 : ",theta6*180/pi)
        
    def timer_callback(self):
        self.ik()
        self.data.position = [self.theta1,self.theta2, self.theta3,self.theta4, self.theta5, self.theta6]
        self.data.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.data)

    
 
def main():
    rp.init()
    ik_solver = IkSolver()
    rp.spin(ik_solver)
    ik_solver.destroy_node()
    rp.shutdown()
 
if __name__ == "__main__":

    main()
