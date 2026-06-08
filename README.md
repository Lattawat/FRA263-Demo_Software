# FRA263_BaseSystem_VerificationSystem_FRAB11

## Run — Verification System (visualizer)

**Terminal 1 — Teensy bridge**
```bash
cd ~/FRA263_BaseSystem_VerificationSystem_FRAB11/claude-visualizer-ws
source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=156
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

**Terminal 2 — pipeline**
```bash
cd ~/FRA263_BaseSystem_VerificationSystem_FRAB11/claude-visualizer-ws
source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=156
ros2 launch claude_visualizer bringup.launch.py
```

Open: [http://localhost:8000](http://localhost:8000)

---

## Run — Base System (Ubuntu)

**Terminal 1 — UI**
```bash
export ROS_DOMAIN_ID=156
pip install --user pymodbus==3.3.0 --force-reinstall
pip install websockets
pip install pylsl
cd ~/FRA263-Demo_Software/FRA263-264_BaseSystem
docker load -i frontend-image_v1_2.tar     # first time only
docker compose up -d
```

**To close UI**
```bash
cd ~/FRA263-Demo_Software/FRA263-264_BaseSystem
docker compose down      # stops + removes the UI container
```

**Terminal 2 — backend**
```bash
cd ~/FRA263-Demo_Software/BaseSystem_Frab11
pip install --user pymodbus websockets pyserial pylsl     # first time only
python3 server_111.py
```

Open: [http://localhost:3000](http://localhost:3000)
