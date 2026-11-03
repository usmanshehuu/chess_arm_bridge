"""Publish a joint-space trajectory through Gazebo Transport's gz CLI."""

import shutil
import subprocess

from .planner import MovePlanError


class GazeboTransportBackend:
    """Adapter for Gazebo Sim's JointTrajectoryController system."""

    def __init__(self, config):
        execution = config.get("execution", {})
        self.enabled = bool(execution.get("enabled", False))
        self.gz_cli = execution.get("gz_cli", "gz")
        self.model_name = execution.get("model_name")
        self.topic = execution.get("joint_trajectory_topic")
        self.joint_names = list(execution.get("joint_names", []))
        self.gripper_names = list(execution.get("gripper_joint_names", []))
        self.gripper_open = list(execution.get("gripper_open_positions", []))
        self.gripper_closed = list(execution.get("gripper_closed_positions", []))
        self.seconds_per_waypoint = float(execution.get("seconds_per_waypoint", 1.0))
        self.gripper_dwell_seconds = float(execution.get("gripper_dwell_seconds", 0.5))
        self.ik = config.get("kinematics", {})
        if not self.topic and self.model_name:
            self.topic = f"/model/{self.model_name}/joint_trajectory"
        if not self.enabled:
            raise MovePlanError("Gazebo execution is disabled. Set execution.enabled=true only after configuring the SDF controller and calibration.")
        if not self.joint_names or len(self.joint_names) != 6 or len(set(self.joint_names)) != 6:
            raise MovePlanError("execution.joint_names must list the six arm joints in the same order as the DH rows")
        if not self.topic:
            raise MovePlanError("Set execution.joint_trajectory_topic or execution.model_name")
        if not self.gripper_names or len(self.gripper_names) != len(self.gripper_open) or len(self.gripper_names) != len(self.gripper_closed):
            raise MovePlanError("Configure matching gripper_joint_names, gripper_open_positions, and gripper_closed_positions")
        if self.seconds_per_waypoint <= 0:
            raise MovePlanError("seconds_per_waypoint must be positive")
        if self.gripper_dwell_seconds < 0:
            raise MovePlanError("gripper_dwell_seconds cannot be negative")
        if shutil.which(self.gz_cli) is None:
            raise MovePlanError(f"Gazebo CLI '{self.gz_cli}' was not found on PATH")

    def _trajectory_text(self, plan):
        names = self.joint_names + self.gripper_names
        lines = [f'joint_names: "{name}"' for name in names]
        seed = list(map(float, self.ik.get("seed_positions_rad", [])))
        if len(seed) != 6:
            raise MovePlanError("Set six kinematics.seed_positions_rad values for IK initialization")
        from .kinematics import SerialDhIk
        solver = SerialDhIk(self.ik)
        previous = seed
        elapsed = 0.0
        gripper_values = self.gripper_open
        for waypoint in plan["waypoints"]:
            pose = waypoint["robot_frame_xyz_m"]
            quat = waypoint["robot_frame_orientation_xyzw"]
            previous = solver.solve(pose, quat, previous)
            if waypoint["gripper"] == "close":
                gripper_values = self.gripper_closed
            elif waypoint["gripper"] == "open":
                gripper_values = self.gripper_open
            elapsed += self.seconds_per_waypoint
            sec = int(elapsed)
            nsec = int(round((elapsed - sec) * 1_000_000_000))
            positions = previous + list(map(float, gripper_values))
            lines.append("points {")
            lines.extend(f"  positions: {value:.9f}" for value in positions)
            lines.extend([f"  time_from_start {{ sec: {sec} nsec: {nsec} }}", "}"])
            if waypoint["gripper"] in ("close", "open") and self.gripper_dwell_seconds > 0:
                elapsed += self.gripper_dwell_seconds
                dwell_sec = int(elapsed)
                dwell_nsec = int(round((elapsed - dwell_sec) * 1_000_000_000))
                lines.append("points {")
                lines.extend(f"  positions: {value:.9f}" for value in positions)
                lines.extend([f"  time_from_start {{ sec: {dwell_sec} nsec: {dwell_nsec} }}", "}"])
        return "\n".join(lines)

    def execute(self, plan):
        message = self._trajectory_text(plan)
        # Check that the configured controller topic is advertised before publishing.
        topics = subprocess.run([self.gz_cli, "topic", "-l"], capture_output=True, text=True, timeout=5)
        if topics.returncode != 0:
            raise MovePlanError(f"Could not list Gazebo topics: {topics.stderr.strip()}")
        if self.topic not in topics.stdout.splitlines():
            raise MovePlanError(
                f"Configured trajectory topic {self.topic!r} is not advertised. "
                "Load Gazebo's JointTrajectoryController system on the arm and verify its topic."
            )
        result = subprocess.run(
            [self.gz_cli, "topic", "-t", self.topic, "-m", "gz.msgs.JointTrajectory", "-p", message],
            capture_output=True, text=True, timeout=10,
        )
        if result.returncode != 0:
            raise MovePlanError(f"Gazebo rejected the trajectory: {result.stderr.strip() or result.stdout.strip()}")
        return {"topic": self.topic, "joint_names": self.joint_names + self.gripper_names, "stdout": result.stdout.strip()}
