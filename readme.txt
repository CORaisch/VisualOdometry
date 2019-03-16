Build Project on Debian-Linux
#############################

0.1) ensure to have opencv installed.Iif not type: sudo apt-get install libopencv-dev. Note that it's tested only
     on version 3.3.1, so building OpenCV from https://github.com/opencv/opencv/tree/3.3.1 is recommended.
0.2) additionally cmake is required: sudo apt-get install cmake
1) mkdir build
2) cd build
3) cmake ..
4) make

Launch Project on Debian-Linux:
##################################

0) the project so far only supports the sequences of the KITTI benchmark suite. It can be downloaded at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
1) KITTI benchmark suite provides calibration files for the cameras, which needs to be passed to the project binary. But before the provided KITTI calibration
   files needs to be converted to the YAML specification used in OpenCV. Therefor you can use the python scribt at <BAS>/scripts/KITTICalib2OCVYAML.py. 
   Type: python <BASE>/scripts/KITTICalib2OCVYAML.py PATH/TO/KITTI/CALIB/FILE/calib_cam_to_cam.txt. The script has only been tested on python 2.7! It will create
   a file named "calib_cam_to_cam_converted.txt" in the same directory, which needs to be passed to the projects binary.
2) to finally launch the programm switch to the build directory and type: 
      ./visodom [-v] -i PATH/TO/KITTI/DATASET/sequences/XX PATH/TO/CONVERTED/KITTI/CALIB/FILE/calib_cam_to_cam_converted.txt
   where XX is a number within [00-21] and indicates the KITTI sequence.

Programm Usage:
###############

When focussed on the Birdview window (the white one):
     - Press <SPACE> to toggle start/pause the sequence
     - Press <ESC> to exit

When foccused on the 3D trajectory visualization window:
     - press <q> or <e> to exit
     - press <p> to toggle between following the latest frame and free movment with keyboard and mouse
     - when in free movemnt mode (after pressing <p> once from the beginning):
       - translate camera with <w>,<a>,<s> and <d> keys
       - press left mouse button while moving the mouse will rotate the camera
       
