# Operations

Practical, robot-side knowledge for running this project on the physical
LanderPi: how to bring the stack up, how to shut it down safely, and how to
get out of the failure modes we actually hit.

None of this is needed to read the code or run the off-robot test suites —
see the [main README](../README.md) for that. It lives here because it is
support knowledge for one specific machine, and it would otherwise crowd out
the project description.

---

## Warnings

- **Do not charge the LanderPi while it is powered on and in use.** Charging under load can cause unstable power delivery to the Raspberry Pi 5 / STM32 and risk corrupting the SD card mid-write (this is also the fastest way to turn a disk-space issue into a corrupted filesystem). Power down, charge fully, then power back on.

## Shutting down safely

**Always shut down the robot properly before disconnecting power.** Do not unplug the battery or cut power while the Raspberry Pi is running or mid-boot.

### Why it matters

Cutting power abruptly (via a dead/low battery or manually disconnecting) can corrupt the SD card's filesystem and leave system services in a broken state. During testing, this caused the robot's WiFi to fail entirely: a stale `dnsmasq` (DHCP) process was left running and blocked NetworkManager from starting a working DHCP server, which in turn caused the robot to fall back to its own self-hosted AP mode instead of connecting to the intended network. This meant the robot could not be reached, and recovery required physical HDMI/keyboard access, manual service intervention via SSH, and direct editing of the robot's WiFi configuration. (**This entire process took over 4 hours**)

### The correct sequence

Before disconnecting power, always run:

```bash
sudo poweroff
```

Wait until the robot has fully powered off (LED activity stops) before removing the battery or power source. (~5-10s)

### If the robot is already stuck

If the robot's WiFi stops working after an improper shutdown, check for a conflicting `dnsmasq` process:

```bash
ps aux | grep dnsmasq
sudo systemctl stop dnsmasq
sudo systemctl disable dnsmasq
sudo systemctl restart NetworkManager
```

If the robot is stuck broadcasting its own hotspot (`HW-...`) instead of connecting to your network, locate and edit its WiFi config:

```bash
find / -name "wifi_conf.py" 2>/dev/null
```

Set the following, then restart the wifi service:

```python
HW_WIFI_MODE = 2
HW_WIFI_STA_SSID = "<your network name>"
HW_WIFI_STA_PASSWORD = "<your network password>"
```

```bash
sudo systemctl restart wifi.service
```

### Prevention

- Charge the battery before starting a session rather than letting it run to empty mid-operation.
- Never disconnect power while the Pi is actively running.
- If the battery's low-voltage alarm sounds, stop what you're doing and power down properly by following the steps above rather than letting it force a shutdown on its own.

## Remote-SSH hangs on "Downloading VS Code Server..."

**Symptom:** VS Code's Remote-SSH extension appears stuck on "Downloading VS Code Server" indefinitely when connecting to the LanderPi (`raspberrypi.local`), with no visible error.

**Root cause:** This is almost always the SD card running out of space, not a network problem. Every time the local `.vscode-server` installation gets corrupted or VS Code updates, it redownloads and unpacks a ~220 MB server tarball on the Pi. On a card already near capacity (easy to hit with Docker + ROS 2 Humble + build artifacts), the download completes to 100%, but unpacking fails with `StorageFull` — VS Code doesn't surface this clearly in the UI, so it just looks "stuck."

**How to confirm:**
1. Open **View → Output**, select **Remote - SSH** from the dropdown.
2. Look for a line like:
   ```
   Error installing server: ... kind: StorageFull, message: "No space left on device"
   ```

**How to fix (SSH into the Pi from a plain terminal, not VS Code):**
```bash
# 1. Confirm disk usage
df -h /

# 2. Check what Docker is using
docker system df

# 3. Safe cleanup — only removes stopped containers, unused networks, and dangling build cache
docker system prune

# 4. Clear any partial/broken VS Code Server install attempts
rm -rf /tmp/.tmp* ~/.vscode-server/cli/servers/*.staging

# 5. Confirm the space was freed
df -h /
```

**Caution:** Avoid `docker system prune -a --volumes` unless you've checked `docker system df` first — the `--volumes` flag deletes any Docker volume not attached to a running container, which can destroy data if trial recordings or bags are stored there instead of on the host filesystem directly.

**If Docker cleanup doesn't free enough space,** the ROS 2 Humble + Ubuntu 22.04 Docker image plus rosbag/colcon build artifacts can genuinely fill a small SD card. Check the biggest space users with:
```bash
du -sh /home/pi/* 2>/dev/null | sort -rh | head -15
```
and consider a larger SD card as the longer-term fix.

The two packages are independent: `proximity_alert` is what the A→B trials run on, `poc_fusion` is the depth/LiDAR fusion proof of concept. Neither imports the other.

`trials/granite.csv`, `trials/decision_log.csv`, and `trials/scan_trace.jsonl` are symlinks into the container's bind-mounted shared folder (see [the data index](../organized_data/DATA_INDEX.md) below) — they exist purely so all three logs show up directly in this repo's VS Code Explorer/file tree instead of requiring you to browse to `/home/pi/docker/tmp/trials/` separately. They live-update as the nodes write to them. If you log a new surface (e.g. `hpl.csv` for plastic laminate), symlink it the same way:

```bash
ln -sf /home/pi/docker/tmp/trials/hpl.csv trials/hpl.csv
```

`path_tracker.py` and `trial_logger.py` are independent ROS 2 nodes. `trial_logger.py` does not modify or depend on the internals of `path_tracker.py` — it only observes `/odom`, so either node can be developed, tested, or replaced without breaking the other.



---

## Bringing the stack up (quick start)

The robot's ROS 2 stack already runs in a Docker container named `MentorPi` on the Pi — you don't need to install ROS yourself. Do this from a VS Code integrated terminal (open the repo folder via the Remote-SSH extension if you're connecting from another machine, or directly if VS Code is running on the Pi itself):

1. **Copy the package into the container.** `docker cp` nests the source inside the destination if the destination already exists, silently leaving a stale duplicate that colcon keeps building instead of your edits — this bit us mid-session (see [Troubleshooting](#common-failures)). Always clear the destination first:

   ```bash
   docker exec -u ubuntu MentorPi rm -rf /home/ubuntu/ros2_ws/src/proximity_alert/proximity_alert
   docker cp proximity_alert/proximity_alert MentorPi:/home/ubuntu/ros2_ws/src/proximity_alert/proximity_alert
   ```

2. **Build it** as the `ubuntu` user via `zsh` (not `bash` + `/opt/ros/humble/setup.bash` — that fails in this container; `~/.zshrc` is what actually sets up the workspace environment correctly):

   ```bash
   docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc && cd ~/ros2_ws && colcon build --packages-select proximity_alert"
   ```

   If a previous build was ever run as `root` (e.g. via plain `docker exec` without `-u ubuntu`), leftover root-owned files under `build/proximity_alert` or `install/proximity_alert` will make this fail with `Permission denied`. Fix with:

   ```bash
   docker exec -u root MentorPi bash -c "chown -R ubuntu:ubuntu /home/ubuntu/ros2_ws/build/proximity_alert /home/ubuntu/ros2_ws/install/proximity_alert"
   ```

3. **Run each node in its own terminal**, as the `ubuntu` user via `zsh` (this loads `need_compile` and other env vars some of the robot's own launch files expect):

   ```bash
   # Terminal A — drive the robot
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert path_tracker --ros-args -p safety_distance:=0.20 -p target_distance:=2.0 -r scan:=/scan_raw -r /cmd_vel:=/cmd_vel_unsafe"
   ```

   `target_distance` is declared as a double parameter -- pass the decimal
   form (`2.0`, not `2`), or `--ros-args` raises
   `InvalidParameterTypeException` and the node never starts.

   **Tuning obstacle spacing?** The encounter-close reset is distance-based, not
   time-based — override it the same way, e.g.
   `-p clear_drive_distance:=0.4 -p encounter_close_confirm_scans:=2`. See
   `clear_drive_distance` in [`AvoidanceConfig`](../proximity_alert/proximity_alert/avoidance.py) for what
   each of the three related parameters does and why the defaults need
   on-hardware validation before you trust them against your actual obstacle
   spacing.

   Note the `-r /cmd_vel:=/cmd_vel_unsafe` remap — `path_tracker` no longer publishes directly to the motor-facing topic. **Terminal A' (motion watchdog) below is not optional** — without it, nothing is publishing on `/cmd_vel` at all and the robot won't move; see [Motion watchdog and emergency stop](../README.md#how-it-works) for why this exists.

   ```bash
   # Terminal A — motion watchdog (start this BEFORE or alongside Terminal A)
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert motion_watchdog"
   ```

   ```bash
   # Terminal B — log the trial
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert trial_logger --ros-args -p csv_path:=/home/ubuntu/shared/trials/granite.csv"
   ```

   **Running obstacle-avoidance trials instead of a clean surface run?** Point Terminal B at `avoidance_trials.csv` with `track_obstacle_outcome:=true` — this additionally prompts for `obstacle_count`, `layout_id`, `outcome` (success/failure), and `cause` once the trial stops (see [Data collected](../organized_data/DATA_INDEX.md)):

   ```bash
   # Terminal B (avoidance-trial variant)
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert trial_logger --ros-args -p csv_path:=/home/ubuntu/shared/trials/avoidance_trials.csv -p track_obstacle_outcome:=true"
   ```

   ```bash
   # Terminal C — log the avoidance decisions (separate CSV)
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert decision_logger --ros-args -p csv_path:=/home/ubuntu/shared/trials/decision_log.csv"
   ```

   ```bash
   # Terminal D — log every raw LiDAR scan (for diagnosing total detection misses)
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert scan_trace_logger --ros-args -p csv_path:=/home/ubuntu/shared/trials/scan_trace.jsonl"
   ```

   Pointing `csv_path` at `/home/ubuntu/shared/...` writes the file into the container's shared folder, which is bind-mounted to `~/docker/tmp` (i.e. `/home/pi/docker/tmp/trials/`) on the Pi — so all three logs appear in your local file manager and survive container restarts.

   Terminal A also plays the obstacle audio alert by default (no separate terminal needed — see `audio_alert_enabled` in [`AvoidanceConfig`](../proximity_alert/proximity_alert/avoidance.py)); pass `-p audio_alert_enabled:=false` there to disable it.

   **Running with `target_distance` set to test the return-to-line correction?** No node logs the lateral (cross-track) offset automatically — measure it against your taped A→B line once the robot stops, and record it by hand in `trials/lateral_offset_trials.csv` (`trials/lateral_offset_trials.csv`) (see [Lateral offset trials](#lateral-offset-trials-lateral_offset_trialscsv) below).

   **Just want the robot to drive, no watchdog/logging setup?** Skip Terminals A'/B/C/D and publish straight to the real `/cmd_vel` in one terminal:

   ```bash
   docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 run proximity_alert path_tracker --ros-args -p safety_distance:=0.20 -p target_distance:=1.5"
   ```

   Without `motion_watchdog` running there's no auto-stop if this process dies uncleanly (see [Motion watchdog and emergency stop](../README.md#how-it-works)), so know the emergency stop command before you run it:

   ```bash
   docker exec -u ubuntu MentorPi bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
   ```

After step 2, repeat only step 3 for future runs — you only need to rebuild when you change `path_tracker.py`/`trial_logger.py`/`decision_logger.py`/`scan_trace_logger.py`/`scan_trace_record.py`/`audio_trigger.py`/`motion_watchdog.py`/`motion_watchdog_logic.py` (repeat steps 1–2 each time).

## Setup

1. **Docker + ROS 2 Humble.** The project's Docker container (`MentorPi`, image `ros:humble`) is already running on the robot's Pi — confirm it can see the LanderPi's ROS 2 stack with `docker exec -u ubuntu MentorPi zsh -lc "source ~/.zshrc && ros2 topic list"` (`/scan_raw`, `/odom`, `/cmd_vel` should be visible). Always include `-u ubuntu` — running as `root` silently breaks FastRTPS's shared-memory transport between nodes (discovery still matches over UDP, but zero data ever delivers), which cost an entire debugging session before it was traced to this.
2. **Copy the package in** and **build** it — see [Running it in VS Code](#bringing-the-stack-up-quick-start) above for the exact commands.


---

## Common failures

- **`trial_logger` never detects a trial end.** Check that `path_tracker`'s obstacle stop is actually driving `linear.x` to zero (watch `ros2 topic echo /cmd_vel`) and that `/odom` twist values are reasonably close to zero when stationary — noisy odometry may need a higher `stop_velocity_threshold`.
- **Robot doesn't stop in time / stops too early.** Adjust `safety_distance` on `path_tracker`; the LiDAR's `range_min`/`range_max` limits also bound how close/far it can reliably see.
- **Robot makes contact with an obstacle that has a thin or overhanging profile (e.g. a pedestal desk, chair legs).** This is very likely the LiDAR's fixed-height blind spot, not a `safety_distance` or code issue — see the limitation note in [Project overview](../README.md#limitations). Reposition the obstacle so it has a consistent cross-section at the LiDAR's mounted height, don't just lower `safety_distance`.
- **No `/scan_raw` or `/odom` data.** Confirm the LanderPi's sensor drivers are running inside the `MentorPi` container (`docker exec MentorPi bash -lc "source /opt/ros/humble/setup.bash && ros2 node list"` should show `LD19`, `ekf_filter_node`, etc.) before starting either node. If the list comes back empty, the driver stack itself has died and needs restarting — **run this from a terminal on the Pi** (not the host's own shell — the Pi host has no `ros2` on `PATH` and does not have the container's workspace mounted in, so the driver stack can only be launched from inside the container):

  ```bash
  docker exec -it -u ubuntu MentorPi zsh -lc "source ~/.zshrc && source ~/ros2_ws/install/setup.bash && ros2 launch bringup bringup.launch.py"
  ```

  This is the exact command the stack is normally started with (confirmed against the live process inside the container — `ros2 launch bringup bringup.launch.py`, no supervisor watching it, so a hang or crash needs a manual relaunch). It brings up everything in one shot — `LD19`, `ekf_filter_node`, `arm_controller`, cameras, etc. — not just the LiDAR, so expect a brief interruption to those too. It does **not** need `-it`/foreground if you'd rather background it, but running it foreground the first time lets you see it actually come up before you move on.

  A host-side copy of this launch step exists at `~/robot_pi/tool/bringup.sh` on the Pi, but as of this writing it isn't wired to reach the container (it calls `ros2` directly, which isn't installed on the host, and `robot_pi/` isn't bind-mounted in) — use the `docker exec` form above instead.

- **`path_tracker` holds still and logs "No fresh scan within scan_timeout," repeatedly, and doesn't recover within a couple seconds.** This is the scan-freshness watchdog working as intended — the LiDAR isn't currently publishing. **Don't just keep restarting `path_tracker`** — restarting it does nothing if the problem is upstream. Check `ros2 topic hz /scan_raw` first: if it's flatlined, this LD19 has been observed to intermittently stop publishing mid-session, and the fix is restarting the driver stack (previous bullet), not `path_tracker` or `motion_watchdog`. A couple of these warnings right at startup (before the first scan/odom message has arrived) is normal DDS discovery delay, not this bug — only sustained warnings that don't clear are the real signal.
- **Edits to a `.py` file don't seem to take effect after rebuilding.** `docker cp` nests the source inside the destination directory if the destination already exists, rather than overwriting it — running the copy step twice without clearing the destination first silently produces a stale duplicate package tree that colcon keeps building from instead of your latest edit. This happened mid-session and cost real time to trace. Always `rm -rf` the destination package dir before `docker cp` (see [Running it in VS Code](#bringing-the-stack-up-quick-start)), and if in doubt, `find ~/ros2_ws/src/proximity_alert -name '<file>.py'` inside the container to check for more than one copy.
- **Rebuild fails with `Permission denied` on files under `build/` or `install/`.** A previous build ran as `root` (e.g. a bare `docker exec` without `-u ubuntu`) and left root-owned artifacts that the `ubuntu` user can't overwrite. `chown -R ubuntu:ubuntu` those two directories (command in [Running it in VS Code](#bringing-the-stack-up-quick-start)) and rebuild.

