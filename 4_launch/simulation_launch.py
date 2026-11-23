# Terminal 1: Path Planning Node
ros2 run kuka_riveting_system path_planning_node

# Terminal 2: Motion Control Node
ros2 run kuka_riveting_system motion_control_node

# Terminal 3: Supervisor (Orchestrierung)
ros2 run kuka_riveting_system riveting_supervisor_node

# Terminal 4: Trigger Planning Service
ros2 service call /path_planning_node/plan std_srvs/srv/Trigger

# Konfiguriere KUKA-Verbindung (launch file)
ros2 launch kuka_riveting_system hardware.launch.py \
  robot_ip:=192.168.1.100 \
  port:=54600
