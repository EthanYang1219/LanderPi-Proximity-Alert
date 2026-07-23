#!/usr/bin/env python3
"""Companion node: plays a WAV dialogue clip through the robot's USB speaker
once per obstacle encounter, independent of path_tracker/trial_logger/
decision_logger/scan_trace_logger.

Prototype only -- observational, not a safety feature. Subscribes to the
existing /avoidance_decision topic and shells out to `aplay` (ALSA)
non-blocking, so a slow or failed playback can never stall this node, let
alone path_tracker's control loop, which this node never touches.
See docs/superpowers/specs/2026-07-24-obstacle-audio-alert-design.md.
"""
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from proximity_alert.audio_trigger import should_play
from proximity_alert.decision_record import DecisionRecord


class ObstacleAudio(Node):
    def __init__(self):
        super().__init__("obstacle_audio")
        self.declare_parameter("wav_path", "/home/ubuntu/shared/audio/obstacle_alert.wav")
        self.declare_parameter("alsa_device", "plughw:2,0")
        self.declare_parameter("decision_topic", "/avoidance_decision")
        self.wav_path = self.get_parameter("wav_path").value
        self.alsa_device = self.get_parameter("alsa_device").value
        decision_topic = self.get_parameter("decision_topic").value
        self._last_played_encounter_id = None

        self.create_subscription(String, decision_topic, self.on_decision, 50)
        self.get_logger().info(
            f"obstacle_audio up. Will play '{self.wav_path}' via '{self.alsa_device}' "
            f"on obstacle detection from '{decision_topic}'."
        )

    def on_decision(self, msg: String):
        # A companion node must never die from bad input it doesn't control:
        # a malformed message on the topic is logged and skipped, not fatal.
        try:
            decision = DecisionRecord.from_json(msg.data)
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f"Ignoring unparseable /avoidance_decision message: {e}")
            return
        if should_play(decision, self._last_played_encounter_id):
            # Mark as played BEFORE launching so a failed launch doesn't retry
            # within this encounter (retry within one encounter is suppressed
            # by the once-per-encounter gate anyway).
            self._last_played_encounter_id = decision.encounter_id
            # aplay is spawned non-blocking. Overlapping playback (two
            # encounters in quick succession) is left unguarded intentionally:
            # cosmetic doubled sound only, not worth added state for a prototype.
            # Finished aplay processes are also not reaped -- they linger as
            # <defunct> until this node exits. Negligible for a demo (a handful
            # of obstacles); a long unattended run would slowly leak PIDs. Both
            # are deliberate prototype trade-offs (see spec Open questions).
            # The try/except catches only aplay-not-installed (OSError); a
            # missing WAV doesn't raise here -- aplay handles that itself.
            try:
                subprocess.Popen(["aplay", "-D", self.alsa_device, self.wav_path])
            except OSError as e:
                self.get_logger().warn(
                    f"Could not launch aplay ({e}); is ALSA's aplay installed in this container?"
                )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAudio()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
