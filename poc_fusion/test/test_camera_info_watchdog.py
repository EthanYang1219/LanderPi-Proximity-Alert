from poc_fusion.lib.camera_info_watchdog import should_warn


# Step 4's watchdog: depth_preprocess_node starts a one-shot timer at
# construction and fires it once, some elapsed_sec later, to decide whether
# to log a visible warning that /ascamera/camera_publisher/depth0/camera_info
# has not been received yet. This is the pure decision the timer callback
# delegates to -- kept out of the node so it can be unit tested without
# rclpy. The node is responsible for cancelling the timer once a message
# arrives, so `received=True` covers "camera_info showed up before the timer
# fired" and the timeout_sec argument covers "the fixed 5s bar named in the
# brief, expressed as a real parameter rather than a hardcoded magic number".

def test_warns_when_not_received_after_timeout():
    assert should_warn(received=False, elapsed_sec=5.0, timeout_sec=5.0) is True


def test_does_not_warn_when_received_before_timeout():
    assert should_warn(received=True, elapsed_sec=5.0, timeout_sec=5.0) is False


def test_does_not_warn_before_timeout_elapses():
    # Guards against a timer misconfigured to fire early: even with
    # camera_info absent, no warning is due until timeout_sec has passed.
    assert should_warn(received=False, elapsed_sec=4.9, timeout_sec=5.0) is False


def test_does_not_warn_when_received_even_past_timeout():
    # Once camera_info has arrived, lateness is moot -- the node cancels the
    # timer, but this pins the decision function's own behaviour too, since
    # a callback that fires anyway (race at exactly 5s) must not warn.
    assert should_warn(received=True, elapsed_sec=9.0, timeout_sec=5.0) is False
