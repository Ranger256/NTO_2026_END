SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. ROS sourced
source "$HOME/catkin_ws/devel/setup.bash"
source "$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$HOME/PX4-Autopilot" "$HOME/PX4-Autopilot/build/px4_sitl_default"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$HOME/PX4-Autopilot"

# 2. Create dir
rm -rf "Result" && mkdir "Result"
rm -rf "Result/models" && mkdir "Result/models"

# 3. Generate aruco
echo "Generate aruco"

cd "$SCRIPT_DIR/Dev/Aruco"

python3 "generate_config_aruco_yaml.py"
rosrun clover_simulation aruco_gen --single-model --dictionary=3 --model-path= model.config

rm -rf "model.config"

cd "$SCRIPT_DIR"

mv "$SCRIPT_DIR/Dev/Aruco/aruco_model_config" "$SCRIPT_DIR/Result/models/"

# 4. Clover
cp -r "$SCRIPT_DIR/Dev/Clover/clover" "$SCRIPT_DIR/Result/models"

cp -r "$SCRIPT_DIR/Dev/Tree/tree" "$SCRIPT_DIR/Result/models"

cp -r "$SCRIPT_DIR/Dev/Humans/Human1" "$SCRIPT_DIR/Result/models"
cp -r "$SCRIPT_DIR/Dev/Humans/Human2" "$SCRIPT_DIR/Result/models"
cp -r "$SCRIPT_DIR/Dev/Humans/Human3" "$SCRIPT_DIR/Result/models"

# 5. Create World
echo "Generate world"

cd "$SCRIPT_DIR/Dev/World"

python3 "generate_world.py"

cd "$SCRIPT_DIR"

mv "$SCRIPT_DIR/Dev/World/NTO.world" "Result/"
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$SCRIPT_DIR/Result/models"

cp -r "$SCRIPT_DIR/Dev/Scripts/world_launch.launch" "$SCRIPT_DIR/Result"

echo "World created"

#7.Start

#export PROJECT_ROOT=$SCRIPT_DIR/Result

#roslaunch "$SCRIPT_DIR/Result/world_launch.launch"

#gazebo --verbose $SCRIPT_DIR/Result/NTO.world &
#GAZEBO_PID=$!
