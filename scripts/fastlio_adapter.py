"""ROS 2 sensor/odometry adapter. Only range samples and IMU enter FAST-LIO."""
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg, CustomPoint


class FastLIOAdapter:
    def __init__(self,owner):
        rclpy.init()
        self.owner=owner
        self.node=Node('mid360_sim_bridge')
        self.lidar=self.node.create_publisher(CustomMsg,'/livox/lidar',10)
        self.imu=self.node.create_publisher(Imu,'/livox/imu',200)
        self.node.create_subscription(Odometry,'/Odometry',self.odometry,100)
        self.alignment=np.eye(3)
        self.gyro=np.zeros(3)
        self.frames=0; self.points=0; self.starts={}
        self.thread=threading.Thread(target=self.spin,daemon=True)
        self.thread.start()

    def spin(self):
        try: rclpy.spin(self.node)
        except ExternalShutdownException: pass

    @staticmethod
    def set_time(header,t):
        ns=round(t*1e9)
        header.stamp.sec=ns//1000000000; header.stamp.nanosec=ns%1000000000
        header.frame_id='mid360_link'

    def publish_cloud(self,xyz,times):
        msg=CustomMsg(); self.set_time(msg.header,times[0])
        msg.timebase=round(times[0]*1e9); msg.point_num=len(xyz)
        offsets=np.rint((times-times[0])*1e9).astype(int)
        msg.points=[CustomPoint(x=float(p[0]),y=float(p[1]),z=float(p[2]),offset_time=int(offset),
                                reflectivity=100,tag=0x10,line=0) for p,offset in zip(xyz,offsets)]
        self.starts[round(times[-1],5)]=time.monotonic()
        self.points=len(xyz); self.lidar.publish(msg)

    def publish_imu(self,message):
        msg=Imu()
        t=message.header.stamp.sec+message.header.stamp.nsec*1e-9
        self.set_time(msg.header,t)
        # Orientation intentionally not copied: it is simulator truth. FAST-LIO
        # integrates only raw angular velocity and specific force.
        msg.orientation_covariance[0]=-1.
        for axis in 'xyz':
            sign=1. if axis=='x' else -1.
            setattr(msg.angular_velocity,axis,sign*getattr(message.angular_velocity,axis))
            setattr(msg.linear_acceleration,axis,sign*getattr(message.linear_acceleration,axis))
        msg.header.frame_id='mid360_imu_body_aligned'
        self.gyro=np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
        self.imu.publish(msg)

    def odometry(self,message):
        try:
            t=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            p=message.pose.pose.position; q=message.pose.pose.orientation
            R=self.alignment@Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            lever=np.array([0,0,.18])
            position=self.alignment@np.array([p.x,p.y,p.z])+lever-R@lever
            linear=message.twist.twist.linear
            velocity=R@(np.array([linear.x,linear.y,linear.z])-np.cross(self.gyro,lever))
            quaternion=np.roll(Rotation.from_matrix(R).as_quat(),1)
            if not np.all(np.isfinite(position)): raise ValueError('Non-finite FAST-LIO estimate')
            self.frames+=1
            processing=time.monotonic()-self.starts.pop(round(t,5),time.monotonic())
            self.owner.writer.writerow([t,*position,*quaternion,*velocity,80,self.points,processing])
            with self.owner.lock:
                self.owner.state={'t':t,'position':position,'rotation':R,'velocity':velocity,'quality':80,'frames':self.frames}
        except Exception as exc:
            self.owner.error=repr(exc)

    def close(self):
        rclpy.shutdown()
        self.thread.join(timeout=5)
        self.node.destroy_node()
