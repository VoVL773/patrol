#!/usr/bin/env bash
# 在 VS Code(snap) 终端里也能正常启动 ROS 仿真/GUI 的包装脚本。
# 用法： ./启动仿真.sh fishbot_description gazebo_sim.launch.py slam:=false
#       ./启动仿真.sh fishbot_navigation2 navigation2.launch.py
#       ./启动仿真.sh fishbot_navigation2 patrol_sim.launch.py
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ★ 关键：清掉 snap 版 VS Code 注入的 GTK/GIO 路径，否则 rviz2 / gz sim gui 会崩：
#   symbol lookup error: /snap/core20/.../libpthread.so.0: undefined symbol __libc_pthread_init
if [ -n "${GTK_EXE_PREFIX:-}" ] && [ "${GTK_EXE_PREFIX#/snap}" != "$GTK_EXE_PREFIX" ]; then
  echo "[启动仿真] 检测到 snap 终端环境，正在清理 GTK/GIO 变量…"
  unset GTK_EXE_PREFIX GTK_PATH GTK_IM_MODULE_FILE GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR LOCPATH
  export XDG_DATA_DIRS=/usr/share/ubuntu:/usr/share/gnome:/usr/local/share/:/usr/share/:/var/lib/snapd/desktop
fi

# ── 可选：如果 Gazebo 报 "libEGL warning ... driver (null)" 且 /scan 没数据 ──
#  说明离屏渲染被 Mesa 接管了（Mesa 不认 NVIDIA dGPU），强制只用 NVIDIA 的 EGL：
#  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
#  极端情况下可以退回软件渲染（慢，但一定有数据）：
#  export LIBGL_ALWAYS_SOFTWARE=1

source /opt/ros/jazzy/setup.bash
source "$HERE/install/setup.bash"

# 顺带提醒：上一次的仿真没关干净会导致"两套系统抢话题/地图重影"
if pgrep -f '[g]z sim server' >/dev/null; then
  echo "[启动仿真] ⚠️ 检测到已有 Gazebo 在跑！先执行 ./停全部.sh，否则会出现两套仿真、地图重影"
  exit 1
fi
exec ros2 launch "$@"
