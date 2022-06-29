# Lidar Calibration

The goal of this project is to use the data from a 2D Lidar in order to re-calibrate the, less precise, ZED2's depth map algorithm.
The program gets the coordinate of the pixels of the ZED2 depth map into the lidar's plane, then checks which Z coordinates are closest to 0 on each line and which pixel is the closest to the lidar's data for each line too.

## Getting Started
 - Get the latest [ZED SDK](https://www.stereolabs.com/developers/release/) and [pyZED Package](https://www.stereolabs.com/docs/app-development/python/install/)

## Run the program

    python "LidarCalibration.py" my_svo_file.svo my_lidar_file.txt
    
## Support
 - Check the [Documentation](https://www.stereolabs.com/docs/)
 - If you need assistance go to the Community site at https://community.stereolabs.com/
