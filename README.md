# Autonomous Windblade Sanding 
**As of 12/8/2022**

## 12/8/2022 Branch Purpose 
- To document the performance of the workspace created by my team for a capstone during the ENGR 4200 Autonomous Systems class hosted by Louisiana State University (LSU).
- To differentiate the performance between the robot by the end of the class and what I've done to improve it independently for research thereafter. 
- To differentiate between before and after I knew how to put together a ROS package with dependencies. 

*NOTE*: The code for the workspace has **NOT** been provided (in this branch) because it was quite large and several files from existing repositories were changed during the project's 
development (see *Section: catkin_ws layout and modified repositories*). ***This should not be a problem now since the MAIN branch is now CLEAN***. I.e. it is 
self-contained and no other repositories' code was modified (although specific codependent packages may be necessary to execute the package being developed). 

## Contributors to Project 

**Contributors**
- Donovan Gegg (self)
- Brenda Hebert 
- Bailey Smoorenburg
- Hayden Gemeinhardt

![TeamPhoto](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/Team_photo.png)

## Why What We Did (and What I Continue to Research) MATTERS!
- Wind blades (and by extension, turbines) and sanded *by hand* during manufacturing. 
  - Meaning, [MANY workers](https://www.ge.com/renewableenergy/stories/lm-castellon-wind-turbine-blade-manufacturing#:~:text=workforce%20that's%20doubled.-,Each%20wind%20turbine%20blade%20takes%20two%20days%20and%20100%20employees,fiberglass%20fabric%20and%20balsa%20wood) needed to work a long time to make 1!
  - This leaves room for Health Hazards...
    - Related to [Styrene Exposure](https://www.researchgate.net/publication/352573239_Occupational_health_hazards_and_risks_in_the_wind_industry)
    - Related to [Fatigue](https://www.ccohs.ca/oshanswers/psychosocial/fatigue.html)
  - This *bottlenecks* wind turbine production.
- *SOLUTION*
  - Autonomous sanding robots to lighten worker's physical (via sanding )and mental (via finding defects) load.
  
## The Robot Setup 
- A Kobuki Base with Turtlebot platforms and 2 ASUS XTION Pro cameras.
- Laptop that came with the Turtlebot was not used due to the resource requirements of an unoptimized octomap 3D map. 

![Robot_Setup](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/Robot_Setup.png)

## What Was Scanned
- A scaled down windblade. Dimensions: 87.5 in. x 24.5 in.

![windblade](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/What_Was_Scanned.png)

## Workspace 
- The working area of the robot.

![workspace](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/Where_Was_Scanned.png)


## The Results (As seen in RVIZ)

### Filtered Pointclouds and LaserScans 
- Laserscan is FAKE. It was made from the filtered PointCloud data. 

![filted_PC_and_LS](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/Filtered_PointClouds_and_Laserscans.png)

### 3D Map Generated
- What was made using Octomap defaults from the filtered pointclouds.
- Notice the "large" amount of noise around the blade.

![3D_Map](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/3D_Map_Made.png)

### 2D Maps Generated 

- Top Camera's generated map:
![Top_Cam_Map](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/2D_Top_Map.png)

- Bottom Camera's generated map:
![Bottom_Cam_Map](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/2D_Bottom_Map.png)

- Overlayed Camera map:
![Overlayed_Map](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/Overlapped_Maps.png)

- Observations 
  - Top map much noisier. Needs filter. 
  - Top map thresholding not very good. Catches part of the floor depending on how the robot leans.
  - Demonstrates that multiple sensors help to filter out information that is NOT true.
  
## Localization (Based on AMCL)
- 2 pose arrays, 1 for each camera. For visualization. 

![localization](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/Final_Localization.png)
  
## Final Result
- Everything together.

![combined_project](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/Final_Result.png)

## Video Presentation
- Link to the Turtlebot's full performance, post Capstone.

[ENGR 4100 Robot Performance](https://www.youtube.com/watch?v=QBd6rT89Lpw)

# File Structure 
- Images are provided instead of the actual src folder to prove a point, that is, don't modify existing packages. Keep projects contained in a single package or a batch of packages. Copy code if needed to make changes. Otherwise, it gets messy. 

## Src Folder Structure 

![src_structure](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/original_src_file_structure.png)

## Pkg file structure 

![pkg_structure](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/original_pkg_file_structure.png)

## Modified Repositories
... Don't do this. Otherwise it'll try to push the change you've made to the pkg source. 

![modified_gits](https://github.com/HailtheWhale/Image_Repo/blob/main/Research/Autonomous_Windblade_Sanding/modified_github_repos.png)

# Main Areas of Improvement
- Future improvements to be made (as of 12/8/22):
  - Filter Noise from the depth images.
  - Reduce voxel resolution (Less blocky).
  - SLAM (Not using prebuilt/loaded map).
  - Improve pathing (Not jerky, like below).
  
  
  
 - Future improvements to be made (as of 12/8/22):
  - Make package self-contained (No modifications distributed among folders and other GitHub packages).
  - Make xacro file for Turtlebot more modular (Originally brute forced with copy pasting stuff in the ASUS camera xacro to get working. Not the way Xacro Macros are supposed to be used).
  - Ground the world frame anywhere that is not where robot base turns on (Imprecise. Not Accurate when using generated map for other robots).
  - Make pointcloud thresholding more intelligent (that is, not based on distance from the sensor).
  - Change the colormapping of the Octomap voxels (so that they mean something besides distance from the ground). 
  - Apply what was done to a mobile manipulator (like the Robotnik).
  - etc. 

