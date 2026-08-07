"""Pure decision logic for Task 5 Step 4's camera_info watchdog.

`depth_preprocess_node` starts a one-shot timer at construction; when it
fires, this function decides whether the situation warrants a visible
warning. Kept pure/dependency-free so it is testable without rclpy -- the
node itself only wires up the timer, a `received` flag set by the
camera_info subscription callback, and the log call.

Why this exists at all: `depth_image_proc::PointCloudXyzNode` is a vendor
C++ node this project may not edit (additive-only constraint), and its
failure mode when camera_info never arrives is silent -- it simply emits no
points, so the fused costmap quietly runs LiDAR-only with no error. This
watchdog is the only place left to surface that condition.
"""


def should_warn(received: bool, elapsed_sec: float, timeout_sec: float) -> bool:
    """True iff camera_info has not arrived and the timeout has elapsed.

    `received=True` always suppresses the warning regardless of elapsed
    time (the node cancels its timer once a message arrives, but this pins
    the decision function's own behaviour for a callback that fires anyway
    on the same tick camera_info arrives).
    """
    if received:
        return False
    return elapsed_sec >= timeout_sec
