# VisualOdometry
Lightweight indirect stereo visual odometry implementation with OpenCV. It based on ORB point-features and uses RANSAC to become robust for outliers during feature matching/tracking.

# Build Project on Ubuntu

1. ensure to have opencv installed.Iif not type: sudo apt-get install libopencv-dev. Note that it's tested only on version 3.3.1, so building OpenCV from https://github.com/opencv/opencv/tree/3.3.1 is recommended.
2. additionally cmake is required: `sudo apt-get install cmake`
```bash
mkdir build
cd build
cmake ..
make
```

# Launch Project on Ubuntu
0. download the [KITTI odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) for testing.
1. the KITTI benchmark suite provides calibration files for the cameras, which need to be passed when runnding the program. But beforehand, the calibration files need to be converted to the YAML specification used in OpenCV. Therefor use the provided python script at `scripts/KITTICalib2OCVYAML.py`. 
```bash
python <BASE>/scripts/KITTICalib2OCVYAML.py PATH/TO/KITTI/CALIB/FILE/calib_cam_to_cam.txt.
```
The script has only been tested on python 2.7! It will create a file named `calib_cam_to_cam_converted.txt` in the same directory.
2. to finally launch the programm switch to the build directory and type: 
```bash
./visodom [-v] -i PATH/TO/KITTI/DATASET/sequences/XX PATH/TO/CONVERTED/KITTI/CALIB/FILE/calib_cam_to_cam_converted.txt
```
where `XX` is a number within [00-21] and indicates the KITTI sequence.

# Programm Usage
* when focussing the birdview window (the white one):
  * Press `SPACE` to toggle start/pause of the sequence
  * Press `ESC` to exit
* When foccussing the 3D trajectory visualization window:
  * press `q` or `e` to exit
  * press `p` to toggle between following the latest frame and free movment with keyboard and mouse
  * when in free movemnt mode (after pressing `p` once from the beginning):
    * translate camera with `w`, `a`, `s` and `d` keys
    * pressing the left mouse button while moving the mouse will rotate the camera
