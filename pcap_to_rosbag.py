import numpy as np
from ouster import client, pcap
import open3d as o3d

import os
import math 
import rosbag, rospy
from std_msgs.msg import Float64, UInt16, Float64MultiArray, MultiArrayDimension, MultiArrayLayout, Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatStatus, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from cv_bridge import CvBridge

import numpy as np
import yaml




def write_imu_txt(imu_file, imu_packet):
    imu_file.write(str(client.packet_ts(imu_packet)))  # timestamp of imu msg, the unit is nanosecond
    print(client.packet_ts(imu_packet))
    imu_file.write(' ')
    imu_file.write(str(imu_packet.accel[0]))    # x component of linear acceleration，the unit should be m/s，https://static.ouster.dev/sdk-docs/python/api/client.html?highlight=packet%20accel#ouster.client.ImuPacket.accel
    imu_file.write(' ')
    imu_file.write(str(imu_packet.accel[1]))    # y component of linear acceleration
    imu_file.write(' ')
    imu_file.write(str(imu_packet.accel[2]))    # z component of linear acceleration
    imu_file.write(' ')
    imu_file.write(str(imu_packet.angular_vel[0]))  # x component of angular velocity，the unit is deg/second.
    imu_file.write(' ')
    imu_file.write(str(imu_packet.angular_vel[1]))  # y component of angular velocity
    imu_file.write(' ')
    imu_file.write(str(imu_packet.angular_vel[2]))  # z component of angular velocity
    imu_file.write('\n')


def write_imu_bag(imu_packet, bag):
    imu_msg = Imu()
    imu_msg.header.frame_id = 'ouster'
    imu_msg.header.stamp = rospy.Time.from_sec(float(client.packet_ts(imu_packet))/1e9) 
    imu_msg.linear_acceleration.x = float(imu_packet.accel[0])
    imu_msg.linear_acceleration.y = float(imu_packet.accel[1])
    imu_msg.linear_acceleration.z = float(imu_packet.accel[2])
    imu_msg.angular_velocity.x = float(imu_packet.accel[0])
    imu_msg.angular_velocity.y = float(imu_packet.accel[1])
    imu_msg.angular_velocity.z = float(imu_packet.accel[2])
    bag.write('ouster/imu', imu_msg, imu_msg.header.stamp)


def write_ouster_bag(xyz, time_stamp, bag):
    header = Header()
    header.frame_id = 'ouster'
    header.stamp = rospy.Time.from_sec(float(time_stamp)/1e9)    # change the unit of timestamp from ns to s
    fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            ]
    pcl_msg = pcl2.create_cloud(header, fields, xyz)
    pcl_msg.is_dense = True
    bag.write("ouster/points", pcl_msg, t=pcl_msg.header.stamp)




def pcap_read_packets(
        source: client.PacketSource,
        metadata: client.SensorInfo,
        save_pcd: bool,
        pcd_output_folder: str,
        save_imu_txt: bool,
        imu_filepath: str,
        bag: rosbag.Bag
) -> None:
    if save_imu_txt:
        imu_file = open(imu_filepath, 'w')
    if save_pcd:
        if pcd_output_folder[-1] != '/':
            pcd_output_folder = pcd_output_folder + '/'
    ranges_hub = []
    xyzlut = client.XYZLut(metadata)
    laserscan_timestamp = 0
    for packet in source:
        if isinstance(packet, client.ImuPacket):
            write_imu_bag(packet, bag)
            if save_imu_txt:
                write_imu_txt(imu_file, packet)  # write the imu msg into txt file
        elif isinstance(packet, client.LidarPacket):
            # Expand a laser scan into a two-dimensional image. The number of rows is the number of lidar lines, here it is 128, and the number of columns is related to the horizontal resolution, here it is 1024
            # A laser scan consists of many lidar packets
            # A lidar packet contains 128x16 data, that is, a packet is 16 columns in a scan
            # It is necessary to splice 64 packets into a 128x1024 picture in order to use the xyzlut function to calculate the xyz coordinates of the corresponding points of each pixel.
            ranges = packet.field(client.ChanField.RANGE)
            if ranges_hub.__len__() == 0:
                ranges_hub = ranges.copy()
                # Each packet contains a timestamp. The timestamp of the first packet of each scan is used as the timestamp of the scan. The unit is nanosecond.
                laserscan_timestamp = client.packet_ts(packet)  
            else:
                ranges_hub = np.hstack([ranges_hub,ranges]) # Splicing packets
            
            if ranges_hub.shape[1] == 1024:
                xyz = xyzlut(ranges_hub)
                write_ouster_bag(xyz.reshape(-1,3), laserscan_timestamp, bag)
                if save_pcd:
                    pcd = o3d.geometry.PointCloud()  
                    pcd.points = o3d.utility.Vector3dVector(xyz.reshape(-1,3))  
                    o3d.io.write_point_cloud(pcd_output_folder+str(laserscan_timestamp)+'.pcd', pcd)  # Timestamp as file name of pcd file
                # o3d.visualization.draw_geometries([pcd])
                # print(laserscan_timestamp)
                ranges_hub = []
    if save_imu_txt:
        imu_file.close()




if __name__ == '__main__':
    f = open("/home/lyxichigoichie/SSD/Dataset/zed_test/config.yaml", 'r', encoding='utf-8')
    cfg = yaml.safe_load(f.read())
    
    with open(cfg["meta_path"], 'r') as f:
        metadata = client.SensorInfo(f.read())
    source = pcap.Pcap(cfg["pcap_path"], metadata)

    bag = rosbag.Bag(cfg["bag"]["path"], cfg["bag"]["mode"])

    pcap_read_packets(source=source, 
                      metadata=metadata,
                      save_pcd=cfg["pointcloud"]["save_pcd"],
                      pcd_output_folder=cfg["pointcloud"]["save_pcd_folder"], 
                      save_imu_txt=cfg["imu"]["save_txt"],
                      imu_filepath=cfg["imu"]["save_txt_path"],
                      bag=bag
                      )
    
    bag.close()
