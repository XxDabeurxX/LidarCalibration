# Lidar Calibration

The goal of this project is to use the data from a 2D Lidar in order to re-calibrate the, less precise, ZED2's depth map algorithm.
The program gets the coordinate of the pixels of the ZED2 depth map into the lidar's plane, then checks which Z coordinates are closest to 0 on each line and which pixel is the closest to the lidar's data for each line too. All this is to determine where the lidar is scanning when looking at the map. Then, a metric can be determined to know how much the algorithm varied from the lidar.

## Getting Started
 - Get the latest [ZED SDK](https://www.stereolabs.com/developers/release/) and [pyZED Package](https://www.stereolabs.com/docs/app-development/python/install/)
 - The code works with a .svo file provided by a ZED2 Camera. Check the documentation in order to record your .svo file
 - A .txt file is also needed. This one is provided by a rplidar A1. Nevertheless, every lidar export as a .txt file in yaml format is supported, as long as you modify the data to extract (default: "ranges").

## Parameters
 - Camera's resolution
 - Camera's FOV
 - Data to extract from .txt file (default: "ranges)
 - Distance (x,y,z) from the left camera to the lidar output
 - Orientation of the lidar from the camera (default: 0). (Where does the recording begins for the lidar)
                        ![image](https://user-images.githubusercontent.com/58843318/176413913-05cf9f89-dfd3-46fa-9f89-d90e2e107a9f.png)


## Run the program

    python "LidarCalibration.py" my_svo_file.svo my_lidar_file.txt
    
## Support
 - Check the [Documentation](https://www.stereolabs.com/docs/)
 - If you need assistance go to the Community site at https://community.stereolabs.com/
