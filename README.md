# Contest 1 Code for MIE443

Group 22 Savo Bajic - 1003051485 Maximilian Glidden - 1002277396 Catherine Kucaba - 1003278026

Command lines required to initiate the code are given in sequential order below. NOTE: these need to be opened in seperate terminal windows. roslaunch mie443_contest1 turtlebot_world.launch world:=practice roslaunch mie443_contest1 gmapping.launch rosrun mie443_contest1 contest1

Once the robot has completed its exploration, run the following to save the map to the "Documents" folder rosrun map_server map_saver -f ~/Documents/team22map

There are no computer specific file locations that need to be modified if our diectory has been downloaded in its entirety. Our CMakeList.txt has some modifications to add our additional .cpp files, the following is our modified line 24: add_executable(contest1 src/contest1.cpp src/bumper.cpp src/explore.cpp src/laser.cpp src/movement.cpp src/scanning.cpp)

Where the output map files will be stored in the documents folder (~/Documents/) as "team22map.*"
