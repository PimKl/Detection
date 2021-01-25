# Detection
VLP-16_Detection

This is a piece of software using PCL to detect obstacles from data comming from the Velodyne VLP16 3D-LiDAR in real time, which was used as a proof of concept for a project. 
This software is by no means fully tested nor optimalised, as a large knowledge gap, limited hands on time with the sensor and overall time constraints were present during the (final) weeks of the project.

This software was heavily based of off 3 other other programs found around the internet. These 3 pieces of software are linked below:


1: Visualising a point cloud retreived from the VLP16 via PCL, https://gist.github.com/UnaNancyOwen/9f9459d3c10f7a6325ebebabda9865f7

2: Saving a point cloud to a PCD file, https://gist.github.com/UnaNancyOwen/d0b60eff8e1c8e52b080

3: Obstacle detection from PCD files, https://github.com/olpotkin/Lidar-Obstacle-Detection


Installation commands for Ubuntu (not tested, should be same as software #3):
1. $> sudo apt install libpcl-dev
2. $> cd ~
3. $> git clone https://github.com/PimKl/Detection.git
4. $> cd Detection/VLP16_Detection
5. $> mkdir build && cd build
6. $> cmake ..
7. $> make
8. $> ./environment
