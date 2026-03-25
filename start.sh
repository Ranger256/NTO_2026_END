#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source "$HOME/catkin_ws/devel/setup.bash"
source "$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$HOME/PX4-Autopilot" "$HOME/PX4-Autopilot/build/px4_sitl_default"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$HOME/PX4-Autopilot"

# --- Временный файл только для мониторинга (не для подавления вывода) ---
LAUNCH_LOG=$(mktemp)

# --- Функция завершения ---
cleanup() {
    echo
    echo "→ Завершение симуляции..."
    pkill -f gzserver 2>/dev/null
    pkill -f gzclient 2>/dev/null
    pkill -f rosmaster 2>/dev/null
    pkill -f rosout 2>/dev/null
    pkill -f image_view 2>/dev/null
    rm -f "$LAUNCH_LOG"
    sleep 1
    echo "→ Готово."
    exit 0
}

trap cleanup EXIT  

echo "Запуск симуляции из '$SCRIPT_DIR/Result/world_launch.launch'..."

# Запускаем launch, дублируя вывод в консоль И в файл через tee
roslaunch "$SCRIPT_DIR/Result/world_launch.launch" 2>&1 | tee "$LAUNCH_LOG" &
LAUNCH_PID=$!

echo "Ожидание запуска Gazebo..."

while ! rostopic info /clock > /dev/null 2>&1; do
    sleep 0.5
done

echo "Gazebo запущен. Запускаем остальные узлы..."

echo "→ Симуляция работает. Нажмите ENTER для завершения."
read
