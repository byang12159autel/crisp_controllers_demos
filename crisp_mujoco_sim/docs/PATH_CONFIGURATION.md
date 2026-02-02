# MuJoCo Model Path Configuration

**Date:** 2026-02-02  
**Status:** ✅ Complete - Using Avant Robot Interface Assets

---

## Summary

Updated MuJoCo model path configuration to use scene files from `avant_robot_interface/assets` instead of package share directory. This resolves path mismatches between Docker and host systems.

---

## Changes Made

### 1. Launch File Updated ✅

**File:** `crisp_controllers_robot_demos/launch/franka.launch.py`

**Before:**
```python
"mujoco_model": os.path.join(
    get_package_share_directory("crisp_controllers_robot_demos"),
    "config",
    "fr3",
    "scene.xml",
),
```

**After:**
```python
"mujoco_model": "/home/ben/avant_robot_interface/assets/fr3/scene.xml",
```

### 2. Docker Compose Updated ✅

**File:** `docker-compose.yaml`

**Added volume mount to both `x-cameras-base` and `x-base` sections:**
```yaml
volumes:
  - $HOME/.Xauthority:/root/.Xauthority
  - /tmp/.X11-unix:/tmp/.X11-unix:rw
  - /dev:/dev
  - /home/ben/avant_robot_interface/assets:/home/ben/avant_robot_interface/assets:ro
```

---

## Path Resolution

### Host System (Python Simulator)
```
/home/ben/avant_robot_interface/assets/fr3/scene.xml
```
✅ Direct access - file exists

### Docker Container (System Interface)
```
/home/ben/avant_robot_interface/assets/fr3/scene.xml
```
✅ Mounted as read-only volume from host

---

## Benefits

✅ **Single Source of Truth** - One location for scene files  
✅ **No Path Mismatch** - Same path works on host and in Docker  
✅ **Read-Only Mount** - Docker can't accidentally modify assets  
✅ **Centralized Assets** - All robot models in one repository  
✅ **Version Control** - Assets tracked in avant_robot_interface repo  

---

## Testing Instructions

### Verify Path Configuration

**1. Check file exists on host:**
```bash
ls -la /home/ben/avant_robot_interface/assets/fr3/scene.xml
# Should show: -rw-rw-r-- 1 ben ben 940 Jan 27 10:16 ...
```

**2. Test Python simulator (host):**
```bash
cd ~/avant_robot_interface
source ~/crisp_controllers_demos/install/setup.bash

python examples/mujoco_sim_service_node.py --visualize
```

Expected: Simulator starts, waits for Init service call

**3. Test Docker mount:**
```bash
docker compose run --rm franka-overlay ls -la /home/ben/avant_robot_interface/assets/fr3/scene.xml
```

Expected: File is accessible inside container

### Full Integration Test

**Terminal 1: Start Python Simulator**
```bash
cd ~/avant_robot_interface
source ~/crisp_controllers_demos/install/setup.bash
python examples/mujoco_sim_service_node.py --visualize
```

**Terminal 2: Rebuild Docker and Launch Controller**
```bash
cd ~/crisp_controllers_demos

# Rebuild Docker image with new launch file
docker compose build

# Set network interface
export ROS_NETWORK_INTERFACE=wlp15s0

# Launch controller manager
./launch_single_franka_panes.sh sim
```

**Expected Behavior:**
1. Python simulator receives Init service call
2. Loads model from `/home/ben/avant_robot_interface/assets/fr3/scene.xml`
3. Returns initial state
4. System Interface begins calling Step service
5. Robot visualization appears in MuJoCo window

---

## Troubleshooting

### Issue: "No such file or directory"

**Cause:** Volume not mounted in Docker  
**Solution:** 
```bash
# Rebuild docker-compose services
docker compose down
docker compose build
```

### Issue: Path works in Docker but not on host

**Cause:** File doesn't exist on host  
**Solution:**
```bash
# Check if file exists
ls -la /home/ben/avant_robot_interface/assets/fr3/scene.xml

# If missing, check git status
cd ~/avant_robot_interface
git status
```

### Issue: "Permission denied" in Docker

**Cause:** Mount permissions  
**Solution:** Mount is already configured as read-only (`:ro`), which should work

---

## File Locations

### Scene Files (Assets)
```
/home/ben/avant_robot_interface/assets/
└── fr3/
    └── scene.xml          # Main scene file
```

### Configuration Files
```
crisp_controllers_demos/
├── crisp_controllers_robot_demos/
│   └── launch/
│       └── franka.launch.py        # Updated with new path
└── docker-compose.yaml             # Added volume mount
```

---

## Alternative Configurations

If you need to use a different scene file:

### Option 1: Different Robot
Update path in `franka.launch.py`:
```python
"mujoco_model": "/home/ben/avant_robot_interface/assets/iiwa/scene.xml",
```

### Option 2: Custom Scene
Place your scene file in assets:
```bash
cp my_custom_scene.xml /home/ben/avant_robot_interface/assets/fr3/custom.xml
```

Then update launch file:
```python
"mujoco_model": "/home/ben/avant_robot_interface/assets/fr3/custom.xml",
```

### Option 3: Environment Variable (Advanced)
Modify launch file to accept parameter:
```python
mujoco_model_env = os.environ.get('MUJOCO_MODEL_PATH')
if mujoco_model_env:
    mujoco_model_path = mujoco_model_env
else:
    mujoco_model_path = "/home/ben/avant_robot_interface/assets/fr3/scene.xml"
```

---

## Related Documentation

- **Phase 2 Implementation:** `crisp_mujoco_sim/docs/PHASE2_IMPLEMENTATION.md`
- **Architecture Diagrams:** `crisp_mujoco_sim/docs/architecture_diagrams.md`
- **Python Simulator Usage:** `~/avant_robot_interface/examples/README_MUJOCO_SIM.md`

---

## Rollback Instructions

To revert to package share directory paths:

**1. Revert launch file:**
```bash
cd ~/crisp_controllers_demos
git checkout crisp_controllers_robot_demos/launch/franka.launch.py
```

**2. Remove Docker volume mount:**
```bash
git checkout docker-compose.yaml
```

**3. Rebuild:**
```bash
docker compose build
```

---

**Configuration Status:** ✅ Complete  
**Ready for:** Phase 3 Integration Testing  
**Path:** `/home/ben/avant_robot_interface/assets/fr3/scene.xml`
