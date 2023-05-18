# Graph-based Stairway Detection using PointCloud Data -- GUI Version

![alt text](https://github.com/ThomasWestfechtel/StairwayDetectionGUI/blob/master/pics/StairDetectionGUI.png "Staiway Detection and its GUI example")

To install the repository:

Go to main directory

```
mkdir build
cd build

cmake ..
make -j8
./pcl_visualizer
```

"Depending on the version there might be an linker error - in this case try replacing CMakeLists.txt with CMakeLists_V2.txt."
is from the README.md of original repo.
In this repo, we renamed `CMakeLists_V2.txt` to `CMakeLists.txt` and `CMakeLists.txt` to `CMakeLists.txt.bak` as only V2 works.


There are 2 example pcd files in the example folder to test the algorihtm. When loading point cloud, the main window does not change its view automatically and has to be changed using keyboard/mouse to alighn the point cloud in the window.

Depending on the accuracy of the employed LIDAR some parameters need to be tuned accordingly -- especially the parameters for the segmentation process.

This is the GUI version of the Stairway Detection. A stand alone version can be found at:

https://github.com/ThomasWestfechtel/StairwayDetection

The processing steps of the algorihm are explained in our papers:

"Robust stairway-detection and localization method for mobile robots using a graph-based model and competing initializations" (https://journals.sagepub.com/doi/full/10.1177/0278364918798039) (IJRR)

"3D graph based stairway detection and localization for mobile robots" (http://ieeexplore.ieee.org/document/7759096/) (IROS).

![alt text](https://github.com/ThomasWestfechtel/StairwayDetection/blob/master/pics/stairGraph.png "Graph-based Detection")


# Note
To extract `.pcd` files from `.bag` files, use [this command](http://wiki.ros.org/pcl_ros#Usage).


## To test the files provided in ths repo with your own stair localizer
This repo provides some test data in `./examples` as a color image (.png) and XYZ pointcloud (.pcd).

Follow this instruction to test your own stair localizer:

### Step 1: transform the pointcloud saved in .pcd (if necessary)
NOTICE: for https://github.com/SunggooJung/stair_detection/pull/1,
the converted pcd file is already provided as `./examples/13_Depth_converted.pcd`.
You can skip the Step 1 if you need that.

You can publish a pointcloud topic (name: `cloud_pcd`) using `pcl_ros` as follows.
Note that you can specify the frame id as well.
```
rosrun pcl_ros pcd_to_pointcloud examples/13_Depth.pcd 0.1 _frame_id:=spot1/base_link
```
Note that the stairway in the original .pcd file seems like follow the "ENU" (EAST/NORTH/UP)-like frame.

Generate a virtual base link for the transform.

```
python3 virtual_base_link.py
```

While running the above node, convert the published pointcloud to a desired frame as follows.
```
python3 convert_frame.py spot1/camera_front_color_optical_frame
```

Then, save it as a .pcd file using `pcl_ros`.

```
rosrun pcl_ros pointcloud_to_pcd input:=/cloud_pcd_converted
```

This is because transforming pointclouds in Python is very slow (e.g. `do_transform_cloud` in `./convert_frame.py`); save and republish it.

Close the above running nodes and rename the saved pcd file as `examples/13_Depth_converted.pcd`,


### Step 2: Republish the converted pcd file
Republishing the saved pcd file can be done as follows.

```
rosrun pcl_ros pcd_to_pointcloud examples/13_Depth_converted.pcd 0.1 _frame_id:=spot1/camera_front_color_optical_frame
```

