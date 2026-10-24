#!/usr/bin/env bash
# 一键停掉所有仿真 / 导航 / 遥控相关进程。
#
# ★ 为什么要分两步：gz sim server 会忽略 SIGTERM（渲染/EGL 收尾时卡住），
#   所以先发 SIGTERM 让能退的优雅退出，再对残留的用 SIGKILL(-9) 强杀。
# ★ 用 [x] 写法 + 以脚本文件运行，避免把执行这条命令的自己杀掉。
# ★ 为什么要把 Nav2 每个节点都点名：实测过——ros2 launch 收到 SIGTERM 时
#   会漏掉个别子进程。有一次 opennav_docking 活了下来，白占 15% CPU、
#   还继续订阅代价地图和 TF，下次启动就是两套系统抢资源。
#   所以宁可啰嗦，把 Nav2 全家桶都列上。
#
# 用法： ./停全部.sh

PATTERNS=(
  # —— 仿真、我们自己的节点 ——
  '[g]azebo_sim.launch.py' '[p]atrol_sim.launch.py' '[g]z sim'
  '[a]sync_slam_toolbox' '[p]arameter_bridge' '[a]uto_explore'
  '[r]obot_state_publisher' '[r]viz2' '[t]eleop_twist_keyboard'
  '[p]atrol_inspector' '[m]ap_server' '[a]mcl'

  # —— Nav2 全家桶（漏一个就变孤儿）——
  '[c]ontroller_server' '[p]lanner_server' '[b]t_navigator'
  '[w]aypoint_follower' '[b]ehavior_server' '[s]moother_server'
  '[v]elocity_smoother' '[c]ollision_monitor' '[r]oute_server'
  '[o]pennav_docking' '[l]ifecycle_manager'
  '[g]lobal_costmap' '[l]ocal_costmap' '[n]av2_container'

  # —— 兜底（真正管用的那一招）——
  # ROS 2 的可执行文件都在 /opt/ros/jazzy/lib/<包名>/ 或 /opt/ros/jazzy/bin/ 下面。
  # 直接按安装目录匹配，所有 ROS 进程一网打尽：不管它是被 launch 拉起的、
  # 被 ros2 run 起的，还是收 SIGTERM 时漏下来的孤儿。
  #   实测教训：只按 'launch_params_' / 'ros2 launch' 兜底是不够的，
  #   `ros2 run tf2_ros static_transform_publisher` 那种就漏了。
  # ⚠️ 副作用：同时开着的别的 ROS 项目也会被一起杀掉。
  #    不想这样就把下面这行注释掉，只留上面的点名清单。
  '/opt/ros/jazzy/(lib|bin)/'
)

echo "① SIGTERM（优雅退出，等 3 秒）…"
for pat in "${PATTERNS[@]}"; do pkill -f "$pat" 2>/dev/null || true; done
sleep 3

echo "② SIGKILL（强杀残留：gz sim server 必须走这一步）…"
for pat in "${PATTERNS[@]}"; do pkill -9 -f "$pat" 2>/dev/null || true; done
sleep 2

# ★ 残留检查不再靠"数名字"，而是直接列出所有还活着的 ROS 进程。
#   之前吃过亏：检查项里没有 opennav_docking，它明明是孤儿却检查不出来。
echo "--- 残留检查 ---"
LEFT="$(ps -eo pid,etime,pcpu,args | grep -E '/opt/ros/jazzy/(lib|bin)/' | grep -v grep)"
GZ="$(pgrep -f '[g]z sim' 2>/dev/null || true)"

if [ -z "$LEFT" ] && [ -z "$GZ" ]; then
  echo "  ✓ 干净：没有任何 ROS / Gazebo 进程残留"
else
  [ -n "$GZ" ] && echo "  ⚠️ 还有 gz sim 没退（它会在 GPU/EGL 收尾时卡住）"
  [ -n "$LEFT" ] && { echo "  ⚠️ 还有这些 ROS 进程没退："; echo "$LEFT"; }
  echo "  再跑一次本脚本；gz sim 始终杀不掉的话，注销一次会话（或重启）最干净。"
fi
