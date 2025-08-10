# Emotion Active Perception (ROS2 Humble)

Emotion-guided active perception extension for PIC4rl_gym-style simulations and robots. Provides a Next-Best-View (NBV) micro-action, uncertainty-driven emotion signal, reliable artifact saving (PNG + JSON), duplicate-server protection, tests, CI, and a local artifact viewer.

## Features
- Action server `/move_and_capture` with modes: 0=passive, 1=active, 2=force_save
- Atomic PNG/JPG saving with ISO timestamps, unique filenames, SHA256, and rich metadata
- Duplicate-action-server detection, configurable startup (`allow_multiple_servers`, `startup_mode`)
- Optional retention policy (`retention_days`, `max_storage_mb`)
- JSON structured logs for key events
- Artifact viewer (Flask) for browsing images/metadata

## Quickstart
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select emotion_active_perception
source ~/ros2_ws/install/setup.bash
pkill -f ap_action_server || true
ros2 run emotion_active_perception ap_action_server
```

Send a forced-save goal:
```bash
ros2 action send_goal /move_and_capture emotion_active_perception/action/MoveAndCapture \
  "{request_id:'t_run', requested_mode:2, max_motion_m:0.2, max_pan_deg:10.0, timeout_s:2.0}"
```

Artifacts:
- Images: `~/ap_dataset/images/*.png`
- Metadata: `~/ap_dataset/meta/*.json`

View artifacts locally:
```bash
ros2 run emotion_active_perception artifact_viewer  # open http://127.0.0.1:5001
```

## Parameters
- `allow_multiple_servers` (bool, default false)
- `startup_mode` (exit|wait, default exit)
- `save_frames` (bool, default true)
- `output_dir` (string, default `~/ap_dataset`)
- `simulator_mode` (bool, default true)
- `actionability_threshold` (float, default 0.1)
- `max_steps` (int, default 5)
- `image_format` (png|jpg, default png)
- `file_prefix` (string, default "")
- `retention_days` (int, default 0)
- `max_storage_mb` (int, default 0)

## Development
```bash
pytest -q src/emotion_active_perception/tests
```

CI uses GitHub Actions (`.github/workflows/ci.yml`).

## License
MIT
