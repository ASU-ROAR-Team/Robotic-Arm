cmake_minimum_required(VERSION 3.0.2)
project(my_gazebo_models) # Your new package name

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package()

# Install the 'pick_place' folder and its contents
# This will place it as: devel/share/my_gazebo_models/pick_place/pickupbox.sdf
install(DIRECTORY pick_place/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pick_place
  PATTERN ".git" EXCLUDE
  PATTERN ".svn" EXCLUDE
  PATTERN "CVS" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  # Add other patterns if you have different file types you want to exclude or include
)

# Install the 'sampling_rocks' folder and its contents
# This will place it as: devel/share/my_gazebo_models/sampling_rocks/rock1/model.sdf
install(DIRECTORY sampling_rocks/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sampling_rocks
  PATTERN ".git" EXCLUDE
  PATTERN ".svn" EXCLUDE
  PATTERN "CVS" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  # Add other patterns if you have different file types you want to exclude or include
)

# If you have any other top-level folders with models, you'd add more install(DIRECTORY ...) lines here.