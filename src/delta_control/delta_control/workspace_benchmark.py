#!/usr/bin/env python3

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from delta_control.kinematics import DeltaIKError, DeltaKinematics2
from delta_control.workspace import (
    calculate_workspace_surface,
    compress_collinear_path,
    generate_limited_joint_segment,
)


def line_is_reachable(solver, start, end, spacing=0.002):
    distance = math.dist(start, end)
    count = max(2, int(math.ceil(distance / spacing)) + 1)
    preferred = None
    for index in range(count):
        ratio = index / (count - 1)
        point = tuple(
            start[axis] + ratio * (end[axis] - start[axis])
            for axis in range(3)
        )
        try:
            preferred = solver.inverse_kinematics(
                *point,
                check_limits=True,
                preferred=preferred,
            )
        except DeltaIKError:
            return False
    return True


def shortcut_connected_path(solver, scan_path, max_stride=12):
    points = [tuple(float(value) for value in point) for point in scan_path]
    shortcut = [points[0]]
    index = 0

    while index < len(points) - 1:
        upper = min(len(points) - 1, index + max(1, int(max_stride)))
        next_index = index + 1
        for candidate in range(upper, index, -1):
            if line_is_reachable(solver, points[index], points[candidate]):
                next_index = candidate
                break
        shortcut.append(points[next_index])
        index = next_index

    return shortcut


def build_benchmark_trajectory(
    solver,
    scan_path,
    speed,
    sample_rate,
    min_segment_duration,
):
    samples = []
    previous_q = (0.0, 0.0, 0.0)
    time_offset = 0.0

    segment_total = len(scan_path) - 1
    for segment_index, (start, end) in enumerate(
        zip(scan_path, scan_path[1:])
    ):
        p0 = tuple(float(value) for value in start)
        p1 = tuple(float(value) for value in end)
        distance = math.dist(p0, p1)
        if distance <= 1e-12:
            continue

        duration = max(distance / speed, min_segment_duration)
        if segment_index in (0, segment_total - 1):
            duration = max(duration, 0.5)
        count = max(5, int(math.ceil(duration * sample_rate)) + 1)
        segment = generate_limited_joint_segment(
            solver,
            p0=p0,
            p1=p1,
            duration=duration,
            n_waypoints=count,
            preferred_start=previous_q,
        )

        for point in segment[1:] if samples else segment:
            point = dict(point)
            point["t"] += time_offset
            point["index"] = len(samples)
            samples.append(point)

        previous_q = samples[-1]["q"]
        time_offset = samples[-1]["t"]

    if not samples:
        raise RuntimeError("The benchmark path did not produce a trajectory.")

    samples[0]["q"] = (0.0, 0.0, 0.0)
    samples[0]["q_dot"] = (0.0, 0.0, 0.0)
    samples[0]["q_ddot"] = (0.0, 0.0, 0.0)
    samples[-1]["q"] = (0.0, 0.0, 0.0)
    samples[-1]["q_dot"] = (0.0, 0.0, 0.0)
    samples[-1]["q_ddot"] = (0.0, 0.0, 0.0)
    return samples


def reference_position(samples, trajectory_time, hint):
    while (
        hint + 1 < len(samples)
        and samples[hint + 1]["t"] <= trajectory_time
    ):
        hint += 1

    if hint + 1 >= len(samples):
        return samples[-1]["p"], hint

    left = samples[hint]
    right = samples[hint + 1]
    dt = right["t"] - left["t"]
    ratio = 0.0 if dt <= 1e-12 else (trajectory_time - left["t"]) / dt
    ratio = max(0.0, min(1.0, ratio))
    position = tuple(
        left["p"][i] + ratio * (right["p"][i] - left["p"][i])
        for i in range(3)
    )
    return position, hint


class WorkspaceBenchmarkNode(Node):
    def __init__(self, samples, trajectory_id):
        super().__init__("delta_workspace_benchmark")
        self.samples = samples
        self.trajectory_id = trajectory_id
        self.records = []
        self.started = False
        self.finished = False
        self.reference_hint = 0
        self.publish_count = 0

        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/kinematic2/joint_trajectory_ros",
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/delta_robot/state",
            self._on_state,
            50,
        )
        self.timer = self.create_timer(0.1, self._publish)

    def _publish(self):
        if self.publish_count >= 5:
            self.timer.cancel()
            return

        values = [float(self.trajectory_id), float(len(self.samples))]
        for sample in self.samples:
            values.extend(
                [
                    float(sample["t"]),
                    *[float(value) for value in sample["q"]],
                    *[float(value) for value in sample["q_dot"]],
                    *[float(value) for value in sample["q_ddot"]],
                ]
            )

        message = Float64MultiArray()
        message.data = values
        self.publisher.publish(message)
        self.publish_count += 1

    def _on_state(self, message):
        if len(message.data) < 22:
            return
        values = [float(value) for value in message.data]
        if int(round(values[1])) != self.trajectory_id:
            return

        active = values[3] > 0.5
        trajectory_time = values[2]
        if active:
            self.started = True

        if not self.started:
            return

        p_ref, self.reference_hint = reference_position(
            self.samples,
            trajectory_time,
            self.reference_hint,
        )
        self.records.append(
            {
                "trajectory_time": trajectory_time,
                "active": active,
                "p_ref": p_ref,
                "p_actual": tuple(values[4:7]),
                "q_actual": tuple(values[7:10]),
                "qd_actual": tuple(values[10:13]),
                "q_ref": tuple(values[13:16]),
                "qd_ref": tuple(values[16:19]),
                "qdd_ref": tuple(values[19:22]),
                "torque": (
                    tuple(values[22:25])
                    if len(values) >= 25
                    else (0.0, 0.0, 0.0)
                ),
                "saturated": (
                    tuple(values[25:28])
                    if len(values) >= 28
                    else (0.0, 0.0, 0.0)
                ),
            }
        )

        if not active and trajectory_time >= self.samples[-1]["t"] - 1e-6:
            self.finished = True


def summarize(records):
    active = [record for record in records if record["active"]]
    if not active:
        raise RuntimeError("No active trajectory feedback was received.")

    q_errors = []
    qd_errors = []
    xyz_errors = []
    relative_velocity_errors = []
    worst_q = None
    worst_qd = None
    worst_xyz = None
    saturated_samples = [0, 0, 0]
    peak_torque = [0.0, 0.0, 0.0]

    for record in active:
        xyz_error = math.dist(record["p_ref"], record["p_actual"])
        xyz_errors.append(xyz_error)
        if worst_xyz is None or xyz_error > worst_xyz[0]:
            worst_xyz = (xyz_error, record)

        for joint in range(3):
            peak_torque[joint] = max(
                peak_torque[joint],
                abs(record["torque"][joint]),
            )
            if record["saturated"][joint] > 0.5:
                saturated_samples[joint] += 1

            q_error = abs(record["q_actual"][joint] - record["q_ref"][joint])
            qd_error = abs(record["qd_actual"][joint] - record["qd_ref"][joint])
            q_errors.append(q_error)
            qd_errors.append(qd_error)

            if worst_q is None or q_error > worst_q[0]:
                worst_q = (q_error, joint, record)
            if worst_qd is None or qd_error > worst_qd[0]:
                worst_qd = (qd_error, joint, record)

            if abs(record["qd_ref"][joint]) >= math.radians(20.0):
                relative_velocity_errors.append(
                    qd_error / abs(record["qd_ref"][joint])
                )

    def rmse(values):
        return math.sqrt(sum(value * value for value in values) / len(values))

    return {
        "sample_count": len(active),
        "duration_s": active[-1]["trajectory_time"],
        "q_rmse_deg": math.degrees(rmse(q_errors)),
        "q_max_deg": math.degrees(max(q_errors)),
        "qd_rmse_deg_s": math.degrees(rmse(qd_errors)),
        "qd_max_deg_s": math.degrees(max(qd_errors)),
        "qd_relative_mean_percent": (
            100.0 * sum(relative_velocity_errors) / len(relative_velocity_errors)
            if relative_velocity_errors
            else None
        ),
        "qd_relative_max_percent": (
            100.0 * max(relative_velocity_errors)
            if relative_velocity_errors
            else None
        ),
        "xyz_rmse_mm": 1000.0 * rmse(xyz_errors),
        "xyz_max_mm": 1000.0 * max(xyz_errors),
        "peak_torque_nm": peak_torque,
        "saturation_percent": [
            100.0 * count / len(active) for count in saturated_samples
        ],
        "worst_q": {
            "joint": worst_q[1] + 1,
            "error_deg": math.degrees(worst_q[0]),
            "time_s": worst_q[2]["trajectory_time"],
            "position_mm": [
                1000.0 * value for value in worst_q[2]["p_ref"]
            ],
        },
        "worst_qd": {
            "joint": worst_qd[1] + 1,
            "error_deg_s": math.degrees(worst_qd[0]),
            "reference_deg_s": math.degrees(
                worst_qd[2]["qd_ref"][worst_qd[1]]
            ),
            "actual_deg_s": math.degrees(
                worst_qd[2]["qd_actual"][worst_qd[1]]
            ),
            "reference_acceleration_deg_s2": math.degrees(
                worst_qd[2]["qdd_ref"][worst_qd[1]]
            ),
            "time_s": worst_qd[2]["trajectory_time"],
            "position_mm": [
                1000.0 * value for value in worst_qd[2]["p_ref"]
            ],
        },
        "worst_xyz": {
            "error_mm": 1000.0 * worst_xyz[0],
            "time_s": worst_xyz[1]["trajectory_time"],
            "position_mm": [
                1000.0 * value for value in worst_xyz[1]["p_ref"]
            ],
        },
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a connected workspace trajectory benchmark."
    )
    parser.add_argument("--divisions", type=int, default=5)
    parser.add_argument("--speed", type=float, default=0.08)
    parser.add_argument("--sample-rate", type=float, default=50.0)
    parser.add_argument("--min-segment-duration", type=float, default=0.08)
    parser.add_argument("--trajectory-id", type=int, default=9001)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--shortcut-stride", type=int, default=12)
    parser.add_argument(
        "--preserve-voxels",
        action="store_true",
        help="Only merge collinear runs so every voxel remains on the route.",
    )
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    solver = DeltaKinematics2()
    mesh = calculate_workspace_surface(
        solver,
        scan_divisions=args.divisions,
        full_scan=False,
    )
    if args.preserve_voxels:
        benchmark_path = compress_collinear_path(mesh.scan_path)
    else:
        benchmark_path = shortcut_connected_path(
            solver,
            mesh.scan_path,
            max_stride=args.shortcut_stride,
        )
    samples = build_benchmark_trajectory(
        solver,
        benchmark_path,
        speed=args.speed,
        sample_rate=args.sample_rate,
        min_segment_duration=args.min_segment_duration,
    )

    print(
        f"Benchmark path: {len(mesh.scan_path)} voxels -> "
        f"{len(benchmark_path)} checkpoints, "
        f"{len(samples)} samples, {samples[-1]['t']:.2f} s"
    )

    rclpy.init()
    node = WorkspaceBenchmarkNode(samples, args.trajectory_id)
    deadline = time.monotonic() + args.timeout
    try:
        while (
            rclpy.ok()
            and not node.finished
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not node.finished:
        raise RuntimeError("Workspace benchmark timed out.")

    result = summarize(node.records)
    result.update(
        {
            "divisions": args.divisions,
            "path_voxels": len(mesh.scan_path),
            "path_checkpoints": len(benchmark_path),
            "trajectory_samples": len(samples),
            "commanded_speed_m_s": args.speed,
        }
    )
    text = json.dumps(result, indent=2)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
