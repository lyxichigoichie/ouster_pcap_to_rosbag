# Pcap to rosbag

This tool converts Ouster pcaps to rosbags using `ouster-sdk-python`. 

This tool only can extract

- the `xyz` coordinate of the pointcloud
- the unix timestamp of the pointcloud msg and imu msg
- the linear acceleration and angular velocity measurement of imu

## Requirement

- [ROS](http://wiki.ros.org/noetic/Installation)
- `ouster-sdk`: `pip3 install ouster-sdk`

## Running

alert the path in `config.yaml`

```yaml
pcap_path: "/home/lyxichigoichie/SSD/Dataset/zed_test/data2/20240311_1611_OS-1-128_992029000020.pcap"  # path of pcap file
meta_path: "/home/lyxichigoichie/SSD/Dataset/zed_test/data2/20240311_1611_OS-1-128_992029000020.json"

bag:
  save_bag: true  # whether save into the bag
  path: "/home/lyxichigoichie/SSD/Dataset/zed_test/data2/zed_test.bag" # path of rosbag file
  mode: "a"  # The mode can be 'r', 'w', or 'a' for reading (default), writing or appending. 

pointcloud:
  save_pcd: false  # whether save the pointcloud as pcd file
  save_pcd_folder: "/home/lyxichigoichie/SSD/Dataset/zed_test/data2/pcd" # the folder to save the pcd file

imu:
  save_txt: false  # whether save the imu measurement into a txt file
  save_txt_path: "/home/lyxichigoichie/SSD/Dataset/zed_test/data2/imu.txt" # the imu txt file path
```

and run

```bash
python pcap_to_rosbag.py
```

