cmake_minimum_required(VERSION 2.8.3)

project(sampling_rover_with_arm_urdf)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	rospy
	std_msgs
	geometry_msgs
	urdf
	xacro
	message_generation
	sampling_rover_with_arm_urdf
)

catkin_package(
 CATKIN_DEPENDS
	geometry_msgs
	roscpp
	rospy
	std_msgs
)

catkin_install_python(PROGRAMS scripts/joint_translator.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/IKSolver.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf scripts)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
