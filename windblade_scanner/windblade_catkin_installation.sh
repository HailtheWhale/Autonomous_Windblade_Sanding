echo 'This is to setup the Windblade Research Workspace Hosted here:
INSERT_LINK.'

echo 'WARNING: This installation 
assumes that you are in a catkin_ws src folder. Failure to set this up will 
result in problems. By proceeding, you are verifying that this has been properly set up
AND ONLY this package is in said workspace for this installation. Would like to attempt 
installation (Y or N)?  '

read install

if [[ $install == "Y" || $install == "y" ]]; then
    echo 'Setting up the Windblade Research Workspace...'
    echo 'Checking ROS Distribution...'
    distribution=$(rosversion -d)
    if [ $distribution == "melodic" ]; then
        echo 'Confirmed melodic is installed.' 
        echo "Attempting to download required github packages..."
        cd ..
        "Cloning ASUS Camera Packages..."
        git clone https://github.com/ros-drivers/openni2_camera
        echo "Cloning zed ROS wrapper..."
        git clone https://github.com/stereolabs/zed-ros-wrapper
        echo "Cloning ZED ROS interfaces..."
        git clone https://github.com/stereolabs/zed-ros-interfaces
        echo "Cloning Turtlebot packages..."
        git clone https://github.com/turtlebot/turtlebot
        echo "Cloning AruCo Detect..."
        git clone https://github.com/UbiquityRobotics/fiducials
        echo "Cloning Octomap Server..."
        git clone https://github.com/OctoMap/octomap_mapping
        echo "Cloning Octomap msgs..."
        git clone https://github.com/OctoMap/octomap_msgs
        echo "Cloning Octomap_ros..."
        git clone https://github.com/OctoMap/octomap_ros
        echo "Cloning Octomap Merger..."
        git clone https://github.com/dan-riley/octomap_merger
        echo "Cloning Localization packages..."
        git clone https://github.com/ros-planning/navigation.git
        echo "Cloning Vision msgs..."
        git clone https://github.com/ros-perception/vision_msgs.git
        echo "Cloning tf2 packages..."
        git clone https://github.com/ros/geometry2.git

        echo "Adding needed CATKIN_IGNORE's..."
        cd fiducials/stag_detect/
        touch CATKIN_IGNORE

        echo "Required packages downloaded! Please catkin_make or catkin build to finish installation."

    else
        echo "Check ROS distribution. Distribution found was $distribution"
    fi

elif [[ $install == "N" || $install == "n" ]]; then
    echo 'Cancelling installation.'
else
    echo 'Not a valid input. Run the script again.'
fi

