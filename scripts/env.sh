# 被 .bashrc 和 start.sh 复用source, 是"环境"的唯一定义处。
source /opt/ros/humble/setup.bash
if [ -f /car_ws/install/setup.bash ]; then
    source /car_ws/install/setup.bash
fi

ulimit -c 0