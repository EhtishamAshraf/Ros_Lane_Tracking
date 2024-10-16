# Lane Tracking using Turtlebot3
This repository features a custom ROS package, lane_tracking_pkg, designed to enable Turtlebot3 to autonomously navigate through autorace environment by tracking the lane. The package equips Turtlebot3 with the ability to track the lane using camera sensor data, utilizing the cv_bridge package to process images and detect lines (Yellow and White).

### Demo Video
You can watch the demo video by clicking on the below image
[![Watch the video](https://github.com/EhtishamAshraf/Turtlebot3_line_wall_following/blob/main/src/follower_robot_pkg/Images/2.png)](https://www.youtube.com/watch?v=d6_b9ii4WU4&t=2s)

## Gazebo World
Below image shows the Gazebo world used in this project. The world contains white and yellow lines.
![Gazebo World](https://github.com/EhtishamAshraf/Turtlebot3_line_wall_following/blob/main/src/follower_robot_pkg/Images/1.png)

### Note 
1.  Details about cloning the repository are given at the end of this **readme file**

## Lane Tracking Logic
  - Convert RGB image to HSV
  - Define the HSV range for detecting white and yellow color lines in the image
  - Create and display the masked image (highlighting the white and yellow detected lines)
  - Combine the white and yellow masks using a bitwise OR operation, allowing both lane lines to be   
    visible in a single image
  - Find the contours in the masked image
  - If contours exist:
      - Get the second and third biggest contour based on the area
      - apply bitwise_and to extract the region where yellow and white lines are present in the 
        original image
      - draw the contour for visualization
      - find the moments of the selected contours
      - find the centroids of the selected contours from the moments
      - draw a circle to highlight the centroid
        
## Robot's Movement Control using PD Controller
  - calculate the error by finding the difference b/w camera_center and the x position of the   
    centroids of both lines
  - calculate the PD controller output based on the proportional, and derivative terms
  - Track the lane using PD controller
### Formula for Desired Center

The desired center is calculated as the midpoint between the left and right lines. The formula is:

\[
\text{desired\_center} = \frac{{\text{line\_left}['x'] + \text{line\_right}['x']}}{2}
\]

Where:
- `line_left['x']` is the x-coordinate of the left line.
- `line_right['x']` is the x-coordinate of the right line.
![Wall Following](https://github.com/EhtishamAshraf/Turtlebot3_line_wall_following/blob/main/src/follower_robot_pkg/Images/3.png)

## Running the Simulation
To run the simulation, launch the wall_line_following launch file, In order to launch the launch file (you should first navigate inside the launch folder of the package and then, use the following command): 
```bash
roslaunch wall_line_following.launch
```

## Create Ros Workspace
Open shell and execute the following commands:
```bash
mkdir wall_line_following_ws
```
```bash
cd wall_line_following_ws
```
# Clone the repository
```bash
sudo apt-get update
```
```bash
sudo apt-get install git
```
```bash
git clone https://github.com/EhtishamAshraf/Turtlebot3_line_wall_following.git
```
```bash
cd Turtlebot3_line_wall_following
```
Run the below commands in root folder of the workspace
```bash
catkin_make 
```
```bash
source devel/setup.bash 
```
Navigate to the Scripts folder inside the package and make the Python files executable by running the following commands:
```bash
chmod +x wall_line_following.py
chmod +x world_control.py
```

Press Enter and navigate to the launch folder inside the package
```bash
roslaunch wall_line_following.launch
```
