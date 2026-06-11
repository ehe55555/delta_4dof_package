#!/usr/bin/env python3
import bisect
import csv
import math
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from delta_control.kinematics import DeltaIKError, DeltaKinematics2
from delta_control.workspace import (
    calculate_workspace_surface,
    compress_collinear_path,
    generate_limited_joint_segment,
)


class DeltaControlNode(Node):
    def __init__(self):
        super().__init__("delta_control_gui")
        self.trajectory_pub = self.create_publisher(
            Float64MultiArray,
            "/kinematic2/joint_trajectory_ros",
            10,
        )
        self.joint_ref_pub = self.create_publisher(
            Float64MultiArray,
            "/kinematic2/joint_ref_ros",
            10,
        )
        self.feedback_xyz = None
        self.feedback_theta = None
        self.feedback_xyz_speed = 0.0
        self.feedback_theta_speed = (0.0, 0.0, 0.0)
        self.feedback_xyz_time = None
        self.feedback_theta_time = None
        self.feedback_state = None
        self.feedback_state_sequence = 0
        self.create_subscription(
            Float64MultiArray,
            "/delta_robot/state",
            self._on_state,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/delta_robot/feedback_xyz",
            self._on_xyz,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/delta_robot/feedback_theta",
            self._on_theta,
            10,
        )

    def _on_xyz(self, msg):
        if self.feedback_state is not None:
            return
        if len(msg.data) >= 3 and all(math.isfinite(v) for v in msg.data[:3]):
            now = time.perf_counter()
            xyz = tuple(float(v) for v in msg.data[:3])

            if self.feedback_xyz is not None and self.feedback_xyz_time is not None:
                dt = now - self.feedback_xyz_time
                if dt > 1e-9:
                    self.feedback_xyz_speed = math.dist(self.feedback_xyz, xyz) / dt

            self.feedback_xyz = xyz
            self.feedback_xyz_time = now

    def _on_theta(self, msg):
        if self.feedback_state is not None:
            return
        if len(msg.data) >= 3 and all(math.isfinite(v) for v in msg.data[:3]):
            now = time.perf_counter()
            theta = tuple(float(v) for v in msg.data[:3])

            if self.feedback_theta is not None and self.feedback_theta_time is not None:
                dt = now - self.feedback_theta_time
                if dt > 1e-9:
                    self.feedback_theta_speed = tuple(
                        math.atan2(
                            math.sin(theta[i] - self.feedback_theta[i]),
                            math.cos(theta[i] - self.feedback_theta[i]),
                        )
                        / dt
                        for i in range(3)
                    )

            self.feedback_theta = theta
            self.feedback_theta_time = now

    def _on_state(self, msg):
        if len(msg.data) < 22 or not all(math.isfinite(v) for v in msg.data[:22]):
            return

        values = [float(v) for v in msg.data[:22]]
        sim_time = values[0]
        xyz = tuple(values[4:7])
        theta = tuple(values[7:10])
        omega = tuple(values[10:13])

        if self.feedback_state is not None:
            dt = sim_time - self.feedback_state["sim_time"]
            if dt > 1e-9:
                self.feedback_xyz_speed = math.dist(
                    self.feedback_state["xyz"], xyz
                ) / dt

        self.feedback_xyz = xyz
        self.feedback_theta = theta
        self.feedback_theta_speed = omega
        self.feedback_state_sequence += 1
        self.feedback_state = {
            "sequence": self.feedback_state_sequence,
            "sim_time": sim_time,
            "trajectory_id": int(round(values[1])),
            "trajectory_time": values[2],
            "trajectory_active": values[3] > 0.5,
            "xyz": xyz,
            "theta": theta,
            "omega": omega,
            "q_ref": tuple(values[13:16]),
            "qd_ref": tuple(values[16:19]),
            "qdd_ref": tuple(values[19:22]),
        }

    def publish_trajectory(self, samples, trajectory_id):
        data = [float(trajectory_id), float(len(samples))]
        for sample in samples:
            data.extend(
                [
                    float(sample["t"]),
                    *[float(v) for v in sample["q"]],
                    *[float(v) for v in sample["q_dot"]],
                    *[float(v) for v in sample["q_ddot"]],
                ]
            )
        msg = Float64MultiArray()
        msg.data = data
        self.trajectory_pub.publish(msg)

    def publish_joint_reference(self, q, q_dot=(0.0, 0.0, 0.0)):
        msg = Float64MultiArray()
        msg.data = [*[float(v) for v in q], *[float(v) for v in q_dot]]
        self.joint_ref_pub.publish(msg)


class DeltaControlApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.solver = DeltaKinematics2()
        self.trajectory_id = 0
        self.last_samples = []
        self.sample_row_ids = []
        self.monitor_after_id = None
        self.monitor_start_time = None
        self.monitor_samples = []
        self.monitor_sample_times = []
        self.monitor_trajectory_id = None
        self.monitor_last_state_sequence = -1
        self.monitor_started = False
        self.monitor_records = []
        self.monitor_actual_distance = 0.0
        self.monitor_reference_distance = 0.0
        self.monitor_previous_actual = None
        self.monitor_previous_reference = None
        self.monitor_error_squared_sum = 0.0
        self.monitor_error_max = 0.0
        self.jog_after_id = None
        self.jog_delay_id = None
        self.held_jog_direction = None
        self.jog_stream_active = False
        self.jog_stream_point = None
        self.jog_stream_q = None
        self.jog_stream_model_q = None
        self.jog_stream_period = 0.05
        self.workspace_running = False
        self.workspace_results = queue.Queue()
        self.workspace_divisions = tk.IntVar(value=12)
        self.workspace_full_scan = tk.BooleanVar(value=True)
        self.last_target = (
            self.solver.g.home_x,
            self.solver.g.home_y,
            self.solver.g.home_z,
        )

        self.root.title("Delta 4DOF Control")
        self.root.geometry("1120x760")
        self.root.minsize(980, 660)

        self.step_mm = tk.DoubleVar(value=5.0)
        self.jog_time = tk.DoubleVar(value=0.6)
        self.x_mm = tk.DoubleVar(value=0.0)
        self.y_mm = tk.DoubleVar(value=0.0)
        self.z_mm = tk.DoubleVar(value=-361.867)
        self.duration = tk.DoubleVar(value=3.0)
        self.waypoints = tk.IntVar(value=61)
        self.status = tk.StringVar(value="San sang")
        self.feedback = tk.StringVar(value="Feedback: dang cho Gazebo...")
        self.summary = tk.StringVar(value="Chua co quy dao")

        self._configure_style()
        self._build_ui()
        self.root.bind("<ButtonRelease-1>", self._stop_hold_jog, add="+")
        self.root.after(5, self._spin_ros)
        self.root.after(100, self._refresh_feedback)

    def _configure_style(self):
        style = ttk.Style()
        style.configure("Title.TLabel", font=("DejaVu Sans", 17, "bold"))
        style.configure("Section.TLabelframe.Label", font=("DejaVu Sans", 11, "bold"))
        style.configure("Jog.TButton", font=("DejaVu Sans", 12, "bold"), padding=10)
        style.configure("Primary.TButton", font=("DejaVu Sans", 10, "bold"), padding=8)

    def _build_ui(self):
        shell = ttk.Frame(self.root, padding=14)
        shell.pack(fill="both", expand=True)

        header = ttk.Frame(shell)
        header.pack(fill="x", pady=(0, 10))
        ttk.Label(header, text="Delta 4DOF Control", style="Title.TLabel").pack(side="left")
        ttk.Label(header, textvariable=self.status).pack(side="right")

        panels = ttk.Panedwindow(shell, orient="horizontal")
        panels.pack(fill="x")

        jog_panel = ttk.LabelFrame(
            panels,
            text="Bang 1 - Dieu khien bang mui ten",
            style="Section.TLabelframe",
            padding=12,
        )
        target_panel = ttk.LabelFrame(
            panels,
            text="Bang 2 - Toa do va quy hoach quy dao",
            style="Section.TLabelframe",
            padding=12,
        )
        panels.add(jog_panel, weight=1)
        panels.add(target_panel, weight=1)

        self._build_jog_panel(jog_panel)
        self._build_target_panel(target_panel)

        feedback_frame = ttk.Frame(shell)
        feedback_frame.pack(fill="x", pady=(10, 6))
        ttk.Label(feedback_frame, textvariable=self.feedback, foreground="#135d9c").pack(
            side="left"
        )
        ttk.Label(feedback_frame, textvariable=self.summary).pack(side="right")

        table_frame = ttk.LabelFrame(
            shell,
            text="Quy dao da lap",
            style="Section.TLabelframe",
            padding=6,
        )
        table_frame.pack(fill="both", expand=True)

        columns = (
            "i",
            "t",
            "x_ref",
            "x_act",
            "y_ref",
            "y_act",
            "z_ref",
            "z_act",
            "v_ref",
            "v_act",
            "q1_ref",
            "q1_act",
            "q2_ref",
            "q2_act",
            "q3_ref",
            "q3_act",
            "qd1_ref",
            "qd1_act",
            "e_qd1",
            "qd2_ref",
            "qd2_act",
            "e_qd2",
            "qd3_ref",
            "qd3_act",
            "e_qd3",
            "e_qd_max",
            "e_xyz",
            "e_q",
        )

        self.table = ttk.Treeview(table_frame, columns=columns, show="headings", height=13)

        labels = {
            "i": "#",
            "t": "t",
            "x_ref": "X ref",
            "x_act": "X act",
            "y_ref": "Y ref",
            "y_act": "Y act",
            "z_ref": "Z ref",
            "z_act": "Z act",
            "v_ref": "v ref",
            "v_act": "v act",
            "q1_ref": "q1 ref",
            "q1_act": "q1 act",
            "q2_ref": "q2 ref",
            "q2_act": "q2 act",
            "q3_ref": "q3 ref",
            "q3_act": "q3 act",
            "qd1_ref": "qd1 ref",
            "qd1_act": "qd1 act",
            "e_qd1": "e qd1",
            "qd2_ref": "qd2 ref",
            "qd2_act": "qd2 act",
            "e_qd2": "e qd2",
            "qd3_ref": "qd3 ref",
            "qd3_act": "qd3 act",
            "e_qd3": "e qd3",
            "e_qd_max": "e qd max (deg/s)",
            "e_xyz": "e XYZ (mm)",
            "e_q": "e q max (deg)",
        }

        widths = {
            "i": 45,
            "t": 65,
            "x_ref": 75,
            "x_act": 75,
            "y_ref": 75,
            "y_act": 75,
            "z_ref": 75,
            "z_act": 75,
            "v_ref": 70,
            "v_act": 70,
            "q1_ref": 75,
            "q1_act": 75,
            "q2_ref": 75,
            "q2_act": 75,
            "q3_ref": 75,
            "q3_act": 75,
            "qd1_ref": 75,
            "qd1_act": 75,
            "e_qd1": 70,
            "qd2_ref": 75,
            "qd2_act": 75,
            "e_qd2": 70,
            "qd3_ref": 75,
            "qd3_act": 75,
            "e_qd3": 70,
            "e_qd_max": 120,
            "e_xyz": 90,
            "e_q": 100,
        }

        for column in columns:
            self.table.heading(column, text=labels[column])
            self.table.column(column, width=widths[column], anchor="center", stretch=False)

        scroll_y = ttk.Scrollbar(table_frame, orient="vertical", command=self.table.yview)
        scroll_x = ttk.Scrollbar(table_frame, orient="horizontal", command=self.table.xview)

        self.table.configure(
            yscrollcommand=scroll_y.set,
            xscrollcommand=scroll_x.set,
        )

        scroll_y.pack(side="right", fill="y")
        scroll_x.pack(side="bottom", fill="x")
        self.table.pack(side="left", fill="both", expand=True)
    def _build_jog_panel(self, parent):
        settings = ttk.Frame(parent)
        settings.pack(fill="x", pady=(0, 10))
        ttk.Label(settings, text="Buoc (mm)").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(
            settings,
            from_=0.1,
            to=100.0,
            increment=0.5,
            textvariable=self.step_mm,
            width=9,
        ).grid(row=0, column=1, padx=(6, 16))
        ttk.Label(settings, text="Thoi gian (s)").grid(row=0, column=2, sticky="w")
        ttk.Spinbox(
            settings,
            from_=0.1,
            to=10.0,
            increment=0.1,
            textvariable=self.jog_time,
            width=9,
        ).grid(row=0, column=3, padx=6)

        pad = ttk.Frame(parent)
        pad.pack(pady=4)
        self._create_hold_jog_button(pad, "\u2191  Y+", (0, 1, 0)).grid(
            row=0, column=1, padx=5, pady=5, sticky="ew"
        )
        self._create_hold_jog_button(pad, "\u2190  X-", (-1, 0, 0)).grid(
            row=1, column=0, padx=5, pady=5, sticky="ew"
        )
        ttk.Button(
            pad, text="HOME", style="Primary.TButton", command=self._go_home
        ).grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        self._create_hold_jog_button(pad, "X+  \u2192", (1, 0, 0)).grid(
            row=1, column=2, padx=5, pady=5, sticky="ew"
        )
        self._create_hold_jog_button(pad, "\u2193  Y-", (0, -1, 0)).grid(
            row=2, column=1, padx=5, pady=5, sticky="ew"
        )
        self._create_hold_jog_button(pad, "Z \u2191", (0, 0, 1)).grid(
            row=0, column=3, padx=(18, 5), pady=5, sticky="ew"
        )
        self._create_hold_jog_button(pad, "Z \u2193", (0, 0, -1)).grid(
            row=2, column=3, padx=(18, 5), pady=5, sticky="ew"
        )

        ttk.Label(
            parent,
            text="Bam mot lan de di mot buoc. Nhan giu de tiep tuc jog, tha chuot de dung.",
            wraplength=430,
        ).pack(anchor="w", pady=(10, 0))

    def _create_hold_jog_button(self, parent, text, direction):
        button = ttk.Button(parent, text=text, style="Jog.TButton")
        button.bind(
            "<ButtonPress-1>",
            lambda _event, value=direction: self._start_hold_jog(value),
        )
        return button

    def _build_target_panel(self, parent):
        grid = ttk.Frame(parent)
        grid.pack(fill="x")

        fields = (
            ("X (mm)", self.x_mm),
            ("Y (mm)", self.y_mm),
            ("Z (mm)", self.z_mm),
            ("Thoi gian T (s)", self.duration),
            ("So waypoint N", self.waypoints),
        )
        for row, (label, variable) in enumerate(fields):
            ttk.Label(grid, text=label).grid(row=row, column=0, padx=4, pady=5, sticky="w")
            ttk.Entry(grid, textvariable=variable, width=18).grid(
                row=row, column=1, padx=4, pady=5, sticky="ew"
            )
        grid.columnconfigure(1, weight=1)

        buttons = ttk.Frame(parent)
        buttons.pack(fill="x", pady=(12, 0))
        ttk.Button(
            buttons,
            text="Lap quy dao",
            command=self._plan_from_fields,
        ).pack(side="left", fill="x", expand=True, padx=(0, 4))
        ttk.Button(
            buttons,
            text="Chay quy dao",
            style="Primary.TButton",
            command=self._execute_from_fields,
        ).pack(side="left", fill="x", expand=True, padx=4)
        ttk.Button(
            buttons,
            text="Xuat CSV",
            command=self._export_csv,
        ).pack(side="left", fill="x", expand=True, padx=(4, 0))

        self.workspace_button = ttk.Button(
            parent,
            text="RUN WORKSPACE",
            style="Primary.TButton",
            command=self._run_workspace,
        )
        self.workspace_button.pack(fill="x", pady=(10, 0))
        workspace_settings = ttk.Frame(parent)
        workspace_settings.pack(fill="x", pady=(6, 0))
        ttk.Checkbutton(
            workspace_settings,
            text="Quet toan bo voxel",
            variable=self.workspace_full_scan,
            command=self._toggle_workspace_mode,
        ).pack(side="left")
        self.workspace_division_label = ttk.Label(
            workspace_settings,
            text="Chia workspace N",
        )
        self.workspace_division_label.pack(side="left", padx=(10, 0))
        self.workspace_division_spin = ttk.Spinbox(
            workspace_settings,
            from_=5,
            to=24,
            increment=1,
            textvariable=self.workspace_divisions,
            width=7,
        )
        self.workspace_division_spin.pack(side="right")
        self._toggle_workspace_mode()

    def _toggle_workspace_mode(self):
        if self.workspace_full_scan.get():
            self.workspace_division_spin.state(["disabled"])
            self.workspace_division_label.state(["disabled"])
        else:
            self.workspace_division_spin.state(["!disabled"])
            self.workspace_division_label.state(["!disabled"])

    def _current_point(self):
        if self.node.feedback_xyz is not None:
            return self.node.feedback_xyz
        return self.last_target

    def _target_from_fields(self):
        return (
            float(self.x_mm.get()) / 1000.0,
            float(self.y_mm.get()) / 1000.0,
            float(self.z_mm.get()) / 1000.0,
        )

    def _plan(self, target, duration, count):
        start = self._current_point()
        actual_q = self.node.feedback_theta
        samples = self.solver.generate_joint_trajectory(
            p0=start,
            p1=target,
            duration=float(duration),
            n_waypoints=int(count),
            preferred_start=actual_q,
        )
        if actual_q is not None:
            offsets = tuple(actual_q[i] - samples[0]["q"][i] for i in range(3))
            for sample in samples:
                tau = sample["tau"]
                tau2 = tau * tau
                tau3 = tau2 * tau
                tau4 = tau3 * tau
                tau5 = tau4 * tau
                blend = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
                blend_dot = (
                    30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4
                ) / float(duration)
                sample["q"] = tuple(
                    sample["q"][i] + (1.0 - blend) * offsets[i]
                    for i in range(3)
                )
                sample["q_dot"] = tuple(
                    sample["q_dot"][i] - blend_dot * offsets[i]
                    for i in range(3)
                )

            dt = float(duration) / float(len(samples) - 1)
            for k, sample in enumerate(samples):
                left = max(0, k - 1)
                right = min(len(samples) - 1, k + 1)
                span = (right - left) * dt
                sample["q_ddot"] = tuple(
                    (samples[right]["q_dot"][i] - samples[left]["q_dot"][i])
                    / span
                    for i in range(3)
                )
        self.last_samples = samples
        self.last_target = target
        self._show_samples(samples)

        distance = math.dist(start, target)
        max_qd = max(abs(v) for sample in samples for v in sample["q_dot"])
        max_qdd = max(abs(v) for sample in samples for v in sample["q_ddot"])
        self.summary.set(
            f"Quang duong: {distance * 1000.0:.2f} mm | "
            f"v TB: {distance / float(duration) * 1000.0:.2f} mm/s"
        )
        self.status.set(
            f"Da lap N={len(samples)}, max |qd|={max_qd:.3f} rad/s, "
            f"max |qdd|={max_qdd:.3f} rad/s2"
        )
        return samples

    def _format_sample_row(
        self,
        sample,
        actual_xyz=None,
        actual_theta=None,
        actual_v_xyz=None,
        actual_q_dot=None,
    ):
        p_ref = sample["p"]
        q_ref = sample["q"]
        q_ref_deg = [math.degrees(value) for value in q_ref]

        p_dot_ref = sample.get("p_dot", (0.0, 0.0, 0.0))
        q_dot_ref = sample.get("q_dot", (0.0, 0.0, 0.0))

        v_ref = math.sqrt(sum(value * value for value in p_dot_ref)) * 1000.0
        qd_ref_deg = tuple(math.degrees(value) for value in q_dot_ref)

        if actual_xyz is None:
            actual_xyz = sample.get("actual_xyz")

        if actual_theta is None:
            actual_theta = sample.get("actual_theta")

        if actual_v_xyz is None:
            actual_v_xyz = sample.get("actual_v_xyz")

        if actual_q_dot is None:
            actual_q_dot = sample.get("actual_q_dot")

        x_act = y_act = z_act = ""
        q1_act = q2_act = q3_act = ""
        v_act = ""
        qd_act_deg = ("", "", "")
        err_qd_deg = ("", "", "")
        err_qd_max = ""
        err_xyz = ""
        err_q = ""

        if actual_xyz is not None:
            x_act = f"{actual_xyz[0] * 1000.0:.2f}"
            y_act = f"{actual_xyz[1] * 1000.0:.2f}"
            z_act = f"{actual_xyz[2] * 1000.0:.2f}"

            err_xyz_value = math.sqrt(
                sum((actual_xyz[i] - p_ref[i]) ** 2 for i in range(3))
            ) * 1000.0
            err_xyz = f"{err_xyz_value:.2f}"

        if actual_theta is not None:
            q_act_deg = [math.degrees(value) for value in actual_theta]
            q1_act = f"{q_act_deg[0]:.2f}"
            q2_act = f"{q_act_deg[1]:.2f}"
            q3_act = f"{q_act_deg[2]:.2f}"

            err_q_value = max(
                abs(
                    math.degrees(
                        self.solver.normalize_angle(actual_theta[i] - q_ref[i])
                    )
                )
                for i in range(3)
            )
            err_q = f"{err_q_value:.2f}"

        if actual_v_xyz is not None:
            v_act = f"{actual_v_xyz * 1000.0:.2f}"

        if actual_q_dot is not None:
            qd_act_values = tuple(
                math.degrees(value) for value in actual_q_dot
            )
            err_qd_values = tuple(
                abs(qd_act_values[i] - qd_ref_deg[i])
                for i in range(3)
            )
            qd_act_deg = tuple(f"{value:.2f}" for value in qd_act_values)
            err_qd_deg = tuple(f"{value:.2f}" for value in err_qd_values)
            err_qd_max = f"{max(err_qd_values):.2f}"

        return (
            sample["index"],
            f"{sample['t']:.3f}",
            f"{p_ref[0] * 1000.0:.2f}",
            x_act,
            f"{p_ref[1] * 1000.0:.2f}",
            y_act,
            f"{p_ref[2] * 1000.0:.2f}",
            z_act,
            f"{v_ref:.2f}",
            v_act,
            f"{q_ref_deg[0]:.2f}",
            q1_act,
            f"{q_ref_deg[1]:.2f}",
            q2_act,
            f"{q_ref_deg[2]:.2f}",
            q3_act,
            f"{qd_ref_deg[0]:.2f}",
            qd_act_deg[0],
            err_qd_deg[0],
            f"{qd_ref_deg[1]:.2f}",
            qd_act_deg[1],
            err_qd_deg[1],
            f"{qd_ref_deg[2]:.2f}",
            qd_act_deg[2],
            err_qd_deg[2],
            err_qd_max,
            err_xyz,
            err_q,
        )

    def _show_samples(self, samples):
        for item in self.table.get_children():
            self.table.delete(item)

        self.sample_row_ids = []

        for sample in samples:
            row_id = f"sample_{sample['index']}"
            self.sample_row_ids.append(row_id)

            self.table.insert(
                "",
                "end",
                iid=row_id,
                values=self._format_sample_row(sample),
            )

    def _start_execution_monitor(self, samples, trajectory_id):
        if self.monitor_after_id is not None:
            self.root.after_cancel(self.monitor_after_id)
            self.monitor_after_id = None

        self.monitor_samples = list(samples)
        self.monitor_sample_times = [float(sample["t"]) for sample in samples]
        self.monitor_start_time = time.perf_counter()
        self.monitor_trajectory_id = trajectory_id
        self.monitor_last_state_sequence = -1
        self.monitor_started = False
        self.monitor_records = []
        self.monitor_actual_distance = 0.0
        self.monitor_reference_distance = 0.0
        self.monitor_previous_actual = None
        self.monitor_previous_reference = None
        self.monitor_error_squared_sum = 0.0
        self.monitor_error_max = 0.0

        self._monitor_execution_actual()

    def _reference_at_time(self, trajectory_time, state):
        index = bisect.bisect_right(self.monitor_sample_times, trajectory_time)
        if index <= 0:
            left = right = 0
            ratio = 0.0
        elif index >= len(self.monitor_samples):
            left = right = len(self.monitor_samples) - 1
            ratio = 0.0
        else:
            left = index - 1
            right = index
            dt = self.monitor_sample_times[right] - self.monitor_sample_times[left]
            ratio = 0.0 if dt <= 1e-12 else (
                trajectory_time - self.monitor_sample_times[left]
            ) / dt

        left_sample = self.monitor_samples[left]
        right_sample = self.monitor_samples[right]
        p_ref = tuple(
            left_sample["p"][i]
            + ratio * (right_sample["p"][i] - left_sample["p"][i])
            for i in range(3)
        )
        p_dot_ref = tuple(
            left_sample.get("p_dot", (0.0, 0.0, 0.0))[i]
            + ratio
            * (
                right_sample.get("p_dot", (0.0, 0.0, 0.0))[i]
                - left_sample.get("p_dot", (0.0, 0.0, 0.0))[i]
            )
            for i in range(3)
        )
        nearest = left if ratio < 0.5 else right
        return {
            "index": self.monitor_samples[nearest]["index"],
            "t": trajectory_time,
            "p": p_ref,
            "p_dot": p_dot_ref,
            "q": state["q_ref"],
            "q_dot": state["qd_ref"],
            "q_ddot": state["qdd_ref"],
        }

    def _monitor_execution_actual(self):
        if not self.monitor_samples or self.monitor_start_time is None:
            return

        state = self.node.feedback_state
        wall_elapsed = time.perf_counter() - self.monitor_start_time

        if (
            state is not None
            and state["sequence"] != self.monitor_last_state_sequence
            and state["trajectory_id"] == self.monitor_trajectory_id
        ):
            self.monitor_last_state_sequence = state["sequence"]

            if state["trajectory_active"] or state["trajectory_time"] > 0.0:
                self.monitor_started = True
                reference = self._reference_at_time(
                    state["trajectory_time"], state
                )
                actual_xyz = state["xyz"]
                actual_theta = state["theta"]
                actual_q_dot = state["omega"]
                qd_error_values = tuple(
                    abs(actual_q_dot[i] - state["qd_ref"][i])
                    for i in range(3)
                )
                qd_error_joint = max(
                    range(3),
                    key=lambda index: qd_error_values[index],
                )

                if self.monitor_previous_actual is not None:
                    self.monitor_actual_distance += math.dist(
                        self.monitor_previous_actual, actual_xyz
                    )
                if self.monitor_previous_reference is not None:
                    self.monitor_reference_distance += math.dist(
                        self.monitor_previous_reference, reference["p"]
                    )

                self.monitor_previous_actual = actual_xyz
                self.monitor_previous_reference = reference["p"]
                error_xyz = math.dist(actual_xyz, reference["p"])
                self.monitor_error_squared_sum += error_xyz * error_xyz
                self.monitor_error_max = max(self.monitor_error_max, error_xyz)

                record = {
                    "sim_time": state["sim_time"],
                    "trajectory_time": state["trajectory_time"],
                    "p_ref": reference["p"],
                    "p_actual": actual_xyz,
                    "q_ref": state["q_ref"],
                    "q_actual": actual_theta,
                    "qd_ref": state["qd_ref"],
                    "qd_actual": actual_q_dot,
                    "error_qd_joint": qd_error_joint + 1,
                    "error_qd": qd_error_values[qd_error_joint],
                    "error_xyz": error_xyz,
                }
                self.monitor_records.append(record)

                row_id = f"sample_{reference['index']}"
                if self.table.exists(row_id):
                    self.table.item(
                        row_id,
                        values=self._format_sample_row(
                            reference,
                            actual_xyz=actual_xyz,
                            actual_theta=actual_theta,
                            actual_v_xyz=self.node.feedback_xyz_speed,
                            actual_q_dot=actual_q_dot,
                        ),
                    )
                    self.table.see(row_id)

                if self.monitor_started and not state["trajectory_active"]:
                    self._finish_execution_monitor()
                    return

        timeout = self.monitor_sample_times[-1] + 10.0
        if wall_elapsed <= timeout:
            self.monitor_after_id = self.root.after(
                20,
                self._monitor_execution_actual,
            )
        else:
            self._finish_execution_monitor()

    def _finish_execution_monitor(self):
        self.monitor_after_id = None
        count = len(self.monitor_records)
        if count == 0:
            self.status.set("Khong nhan duoc state dong bo tu Gazebo")
            return

        self._fill_table_from_synchronized_records()
        rmse = math.sqrt(self.monitor_error_squared_sum / count)
        path_error = self.monitor_actual_distance - self.monitor_reference_distance
        self.summary.set(
            f"L ref: {self.monitor_reference_distance * 1000.0:.2f} mm | "
            f"L act: {self.monitor_actual_distance * 1000.0:.2f} mm | "
            f"dL: {path_error * 1000.0:+.2f} mm"
        )
        self.status.set(
            f"RMSE XYZ: {rmse * 1000.0:.2f} mm | "
            f"max: {self.monitor_error_max * 1000.0:.2f} mm"
        )

    def _fill_table_from_synchronized_records(self):
        if not self.monitor_records:
            return

        records = sorted(
            self.monitor_records,
            key=lambda record: record["trajectory_time"],
        )
        record_times = [record["trajectory_time"] for record in records]
        sample_by_index = {
            int(sample["index"]): sample for sample in self.monitor_samples
        }

        for row_id in self.sample_row_ids:
            try:
                sample_index = int(row_id.removeprefix("sample_"))
                sample = sample_by_index[sample_index]
            except (KeyError, ValueError):
                continue

            index = bisect.bisect_right(record_times, float(sample["t"]))
            if index <= 0:
                left = right = records[0]
                ratio = 0.0
            elif index >= len(records):
                left = right = records[-1]
                ratio = 0.0
            else:
                left = records[index - 1]
                right = records[index]
                dt = right["trajectory_time"] - left["trajectory_time"]
                ratio = 0.0 if dt <= 1e-12 else (
                    float(sample["t"]) - left["trajectory_time"]
                ) / dt

            actual_xyz = tuple(
                left["p_actual"][axis]
                + ratio
                * (right["p_actual"][axis] - left["p_actual"][axis])
                for axis in range(3)
            )
            actual_theta = tuple(
                left["q_actual"][axis]
                + ratio
                * self.solver.normalize_angle(
                    right["q_actual"][axis] - left["q_actual"][axis]
                )
                for axis in range(3)
            )
            actual_q_dot = tuple(
                left["qd_actual"][axis]
                + ratio
                * (right["qd_actual"][axis] - left["qd_actual"][axis])
                for axis in range(3)
            )

            dt = right["trajectory_time"] - left["trajectory_time"]
            actual_v_xyz = 0.0 if dt <= 1e-12 else (
                math.dist(left["p_actual"], right["p_actual"]) / dt
            )

            if self.table.exists(row_id):
                self.table.item(
                    row_id,
                    values=self._format_sample_row(
                        sample,
                        actual_xyz=actual_xyz,
                        actual_theta=actual_theta,
                        actual_v_xyz=actual_v_xyz,
                        actual_q_dot=actual_q_dot,
                    ),
                )

    def _plan_from_fields(self):
        try:
            self._plan(
                self._target_from_fields(),
                float(self.duration.get()),
                int(self.waypoints.get()),
            )
        except (ValueError, DeltaIKError) as error:
            messagebox.showerror("Khong lap duoc quy dao", str(error))

    def _execute_from_fields(self):
        try:
            samples = self._plan(
                self._target_from_fields(),
                float(self.duration.get()),
                int(self.waypoints.get()),
            )
            self._publish(samples)
        except (ValueError, DeltaIKError) as error:
            messagebox.showerror("Khong chay duoc quy dao", str(error))

    def _publish(self, samples):
        self.trajectory_id += 1
        self.node.publish_trajectory(samples, self.trajectory_id)
        self._start_execution_monitor(samples, self.trajectory_id)
        self.status.set(f"Da gui trajectory #{self.trajectory_id} den Gazebo")
    def _start_hold_jog(self, direction):
        self._cancel_hold_jog()
        self.held_jog_direction = direction
        self.jog_delay_id = self.root.after(250, self._begin_hold_jog)

    def _begin_hold_jog(self):
        self.jog_delay_id = None
        if self.held_jog_direction is None:
            return
        try:
            point = self._current_point()
            actual_q = self.node.feedback_theta
            model_q = self.solver.inverse_kinematics(
                *point,
                preferred=actual_q,
            )
            q = actual_q if actual_q is not None else model_q
            self.jog_stream_point = point
            self.jog_stream_q = q
            self.jog_stream_model_q = model_q
            self.jog_stream_active = True
            self._stream_hold_jog()
        except (ValueError, DeltaIKError) as error:
            self._cancel_hold_jog()
            messagebox.showerror("Jog ngoai workspace", str(error))

    def _stream_hold_jog(self):
        self.jog_after_id = None
        if not self.jog_stream_active or self.held_jog_direction is None:
            return
        try:
            duration = float(self.jog_time.get())
            step = float(self.step_mm.get()) / 1000.0
            if duration <= 0.0 or step <= 0.0:
                raise ValueError("Buoc va thoi gian jog phai lon hon 0.")

            direction = self.held_jog_direction
            speed = step / duration
            p_dot = tuple(axis * speed for axis in direction)
            point = self.jog_stream_point
            target = tuple(
                point[i] + p_dot[i] * self.jog_stream_period for i in range(3)
            )
            model_q = self.solver.inverse_kinematics(
                *target,
                preferred=self.jog_stream_model_q,
            )
            q = tuple(
                self.jog_stream_q[i]
                + self.solver.normalize_angle(
                    model_q[i] - self.jog_stream_model_q[i]
                )
                for i in range(3)
            )
            q_dot = tuple(
                (q[i] - self.jog_stream_q[i]) / self.jog_stream_period
                for i in range(3)
            )
            self.node.publish_joint_reference(q, q_dot)

            self.jog_stream_point = target
            self.jog_stream_q = q
            self.jog_stream_model_q = model_q
            self.last_target = target
            self.x_mm.set(round(target[0] * 1000.0, 3))
            self.y_mm.set(round(target[1] * 1000.0, 3))
            self.z_mm.set(round(target[2] * 1000.0, 3))
            self.status.set(f"Jog lien tuc: {speed * 1000.0:.2f} mm/s")
            self.jog_after_id = self.root.after(
                int(self.jog_stream_period * 1000.0),
                self._stream_hold_jog,
            )
        except (ValueError, DeltaIKError, ZeroDivisionError) as error:
            self._cancel_hold_jog(send_stop=True)
            messagebox.showerror("Jog ngoai workspace", str(error))

    def _stop_hold_jog(self, _event=None):
        direction = self.held_jog_direction
        was_click = self.jog_delay_id is not None and not self.jog_stream_active
        if was_click:
            self.root.after_cancel(self.jog_delay_id)
            self.jog_delay_id = None

        if self.jog_stream_active:
            self._cancel_hold_jog(send_stop=True)
        else:
            self._cancel_hold_jog()
            if was_click and direction is not None:
                self._jog(*direction)

    def _cancel_hold_jog(self, send_stop=False):
        if send_stop and self.jog_stream_q is not None:
            self.node.publish_joint_reference(self.jog_stream_q)
        self.held_jog_direction = None
        self.jog_stream_active = False
        if self.jog_delay_id is not None:
            self.root.after_cancel(self.jog_delay_id)
            self.jog_delay_id = None
        if self.jog_after_id is not None:
            self.root.after_cancel(self.jog_after_id)
            self.jog_after_id = None
        self.jog_stream_point = None
        self.jog_stream_q = None
        self.jog_stream_model_q = None

    def _jog(self, dx, dy, dz):
        try:
            start = self._current_point()
            step = float(self.step_mm.get()) / 1000.0
            target = (
                start[0] + dx * step,
                start[1] + dy * step,
                start[2] + dz * step,
            )
            self.x_mm.set(round(target[0] * 1000.0, 3))
            self.y_mm.set(round(target[1] * 1000.0, 3))
            self.z_mm.set(round(target[2] * 1000.0, 3))
            samples = self._plan(target, float(self.jog_time.get()), 31)
            self._publish(samples)
        except (ValueError, DeltaIKError) as error:
            self._cancel_hold_jog()
            messagebox.showerror("Jog ngoai workspace", str(error))

    def _go_home(self):
        try:
            target = (
                self.solver.g.home_x,
                self.solver.g.home_y,
                self.solver.g.home_z,
            )
            self.x_mm.set(target[0] * 1000.0)
            self.y_mm.set(target[1] * 1000.0)
            self.z_mm.set(target[2] * 1000.0)
            samples = self._plan(
                target,
                float(self.duration.get()),
                int(self.waypoints.get()),
            )
            samples[-1]["q"] = (0.0, 0.0, 0.0)
            samples[-1]["q_dot"] = (0.0, 0.0, 0.0)
            samples[-1]["q_ddot"] = (0.0, 0.0, 0.0)
            self._show_samples(samples)
            self._publish(samples)
        except (ValueError, DeltaIKError) as error:
            messagebox.showerror("Khong ve duoc HOME", str(error))

    def _run_workspace(self):
        if self.workspace_running:
            return
        self.workspace_running = True
        self.workspace_button.state(["disabled"])
        try:
            divisions = int(self.workspace_divisions.get())
        except (ValueError, tk.TclError):
            divisions = 12
        full_scan = bool(self.workspace_full_scan.get())
        if full_scan:
            self.status.set("Dang tao duong quet qua toan bo voxel workspace...")
        else:
            self.status.set(
                f"Dang chia workspace {divisions} x {divisions} x {divisions}..."
            )
        threading.Thread(
            target=self._calculate_workspace,
            args=(divisions, full_scan),
            daemon=True,
        ).start()
        self.root.after(50, self._poll_workspace)

    def _calculate_workspace(self, divisions, full_scan):
        try:
            mesh = calculate_workspace_surface(
                self.solver,
                scan_divisions=divisions,
                full_scan=full_scan,
            )
            self.workspace_results.put(("ok", mesh))
        except Exception as error:
            self.workspace_results.put(("error", str(error)))

    def _poll_workspace(self):
        try:
            result, value = self.workspace_results.get_nowait()
        except queue.Empty:
            if self.workspace_running:
                self.root.after(50, self._poll_workspace)
            return

        if result == "ok":
            self._show_workspace(value)
        else:
            self._workspace_failed(value)

    def _workspace_failed(self, error):
        self.workspace_running = False
        self.workspace_button.state(["!disabled"])
        self.status.set("Tinh workspace that bai")
        messagebox.showerror("Khong ve duoc workspace", error)

    def _show_workspace(self, mesh):
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure

        window = tk.Toplevel(self.root)
        window.title("Delta 4DOF Workspace")
        window.geometry("940x760")

        figure = Figure(figsize=(9, 7), dpi=100)
        axis = figure.add_subplot(111, projection="3d")
        axis.scatter(
            mesh.volume_x,
            mesh.volume_y,
            mesh.volume_z,
            c=mesh.volume_z,
            cmap="viridis",
            s=7,
            alpha=0.055,
            linewidths=0,
            depthshade=False,
            label="The tich workspace",
        )
        points = axis.scatter(
            mesh.x,
            mesh.y,
            mesh.z,
            c=mesh.z,
            cmap="viridis",
            s=2.5,
            alpha=0.34,
            linewidths=0,
            depthshade=False,
            label="Bien workspace",
        )
        path_mm = mesh.scan_path * 1000.0
        axis.plot(
            path_mm[:, 0],
            path_mm[:, 1],
            path_mm[:, 2],
            color="#d62728",
            linewidth=1.4,
            label="Quy dao Gazebo",
        )
        axis.scatter(
            [self.solver.g.home_x * 1000.0],
            [self.solver.g.home_y * 1000.0],
            [self.solver.g.home_z * 1000.0],
            color="#d62728",
            s=65,
            label="HOME (q1=q2=q3=0)",
        )
        axis.set_xlabel("X (mm)")
        axis.set_ylabel("Y (mm)")
        axis.set_zlabel("Z (mm)")
        axis.set_title(
            f"Workspace: quet {mesh.scan_voxel_count:,}/"
            f"{mesh.volume_voxel_count:,} voxel - luoi {mesh.grid_step_mm * 2.0:.0f} mm"
        )
        axis.set_box_aspect((1.0, 1.0, 0.55))
        axis.view_init(elev=24, azim=-55)
        axis.legend(loc="upper right")
        figure.colorbar(points, ax=axis, shrink=0.72, pad=0.08, label="Z (mm)")

        bounds = mesh.bounds_mm
        figure.text(
            0.02,
            0.02,
            (
                f"Bien hien thi: X [{bounds[0]:.0f}, {bounds[1]:.0f}] mm | "
                f"Y [{bounds[2]:.0f}, {bounds[3]:.0f}] mm | "
                f"Z [{bounds[4]:.0f}, {bounds[5]:.0f}] mm\n"
                "Bien phuong trinh (0.01 mm): "
                "X +/-333.65 | Y +/-315.12 | Z [-482.84, -199.10] mm"
            ),
            fontsize=9,
        )
        figure.tight_layout(rect=(0, 0.07, 1, 1))

        canvas = FigureCanvasTkAgg(figure, master=window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)

        try:
            duration = self._execute_workspace_scan(
                mesh.scan_path,
                mesh.grid_step_mm * 2.0 / 1000.0,
            )
        except (ValueError, DeltaIKError) as error:
            self._workspace_failed(str(error))
            return

        if duration >= 120.0:
            duration_text = f"{duration / 60.0:.1f} phut"
        else:
            duration_text = f"{duration:.1f} s"
        self.status.set(
            f"Dang quet {mesh.scan_voxel_count:,} voxel trong {duration_text}"
        )
        self.root.after(
            max(1, int(duration * 1000.0)),
            self._workspace_finished,
        )

    def _workspace_finished(self):
        self.workspace_running = False
        self.workspace_button.state(["!disabled"])
        self.status.set("Da quet xong workspace va ve HOME")

    def _execute_workspace_scan(self, scan_path, scan_grid_step):
        start = self._current_point()
        actual_q = self.node.feedback_theta
        home = tuple(float(v) for v in scan_path[0])
        transition = self.solver.generate_joint_trajectory(
            p0=start,
            p1=home,
            duration=3.0,
            n_waypoints=61,
            preferred_start=actual_q,
        )
        if actual_q is not None:
            offsets = tuple(
                actual_q[i] - transition[0]["q"][i] for i in range(3)
            )
            for sample in transition:
                tau = sample["tau"]
                blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
                sample["q"] = tuple(
                    sample["q"][i] + (1.0 - blend) * offsets[i]
                    for i in range(3)
                )

        samples = list(transition)
        samples[-1]["q"] = (0.0, 0.0, 0.0)
        time_value = samples[-1]["t"]
        previous_p = home
        speed = 0.06
        max_spacing = max(0.002, float(scan_grid_step))
        trajectory_path = compress_collinear_path(scan_path)

        endpoint_count = len(trajectory_path) - 1
        for endpoint_index, endpoint in enumerate(trajectory_path[1:]):
            endpoint = tuple(float(v) for v in endpoint)
            distance = math.dist(previous_p, endpoint)
            segment_time = max(distance / speed, 0.05)
            if endpoint_index in (0, endpoint_count - 1):
                segment_time = max(segment_time, 0.5)
            segment_count = max(
                5,
                int(math.ceil(distance / max_spacing - 1e-9)) + 1,
            )
            segment = generate_limited_joint_segment(
                self.solver,
                p0=previous_p,
                p1=endpoint,
                duration=segment_time,
                n_waypoints=segment_count,
            )
            for point in segment[1:]:
                point = dict(point)
                point["index"] = len(samples)
                point["t"] += time_value
                samples.append(point)
            time_value = samples[-1]["t"]
            previous_p = endpoint

        samples[0]["q_dot"] = (0.0, 0.0, 0.0)
        samples[0]["q_ddot"] = (0.0, 0.0, 0.0)
        samples[-1]["q"] = (0.0, 0.0, 0.0)
        samples[-1]["q_dot"] = (0.0, 0.0, 0.0)
        samples[-1]["q_ddot"] = (0.0, 0.0, 0.0)
        self.last_samples = samples
        self.last_target = home
        self._publish(samples)
        return time_value

    def _export_csv(self):
        if not self.last_samples and not self.monitor_records:
            messagebox.showwarning("Chua co du lieu", "Hay lap quy dao truoc.")
            return
        output_dir = Path.home() / "Downloads"
        output_dir.mkdir(parents=True, exist_ok=True)
        path = output_dir / f"delta_trajectory_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            if self.monitor_records:
                writer.writerow(
                    [
                        "sim_time",
                        "trajectory_time",
                        "x_ref",
                        "y_ref",
                        "z_ref",
                        "x_actual",
                        "y_actual",
                        "z_actual",
                        "q1_ref",
                        "q2_ref",
                        "q3_ref",
                        "q1_actual",
                        "q2_actual",
                        "q3_actual",
                        "qd1_ref",
                        "qd2_ref",
                        "qd3_ref",
                        "qd1_actual",
                        "qd2_actual",
                        "qd3_actual",
                        "error_qd1_rad_s",
                        "error_qd2_rad_s",
                        "error_qd3_rad_s",
                        "error_qd_joint",
                        "error_qd_max_rad_s",
                        "error_xyz",
                    ]
                )
                for record in self.monitor_records:
                    writer.writerow(
                        [
                            record["sim_time"],
                            record["trajectory_time"],
                            *record["p_ref"],
                            *record["p_actual"],
                            *record["q_ref"],
                            *record["q_actual"],
                            *record["qd_ref"],
                            *record["qd_actual"],
                            *[
                                abs(
                                    record["qd_actual"][i]
                                    - record["qd_ref"][i]
                                )
                                for i in range(3)
                            ],
                            record["error_qd_joint"],
                            record["error_qd"],
                            record["error_xyz"],
                        ]
                    )
            else:
                writer.writerow(
                    [
                        "index",
                        "t",
                        "x",
                        "y",
                        "z",
                        "q1",
                        "q2",
                        "q3",
                        "qd1",
                        "qd2",
                        "qd3",
                    ]
                )
                for sample in self.last_samples:
                    writer.writerow(
                        [
                            sample["index"],
                            sample["t"],
                            *sample["p"],
                            *sample["q"],
                            *sample["q_dot"],
                        ]
                    )
        self.status.set(f"Da xuat CSV: {path}")

    def _spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self.root.after(5, self._spin_ros)

    def _refresh_feedback(self):
        xyz = self.node.feedback_xyz
        theta = self.node.feedback_theta

        if xyz is not None:
            v_xyz = self.node.feedback_xyz_speed * 1000.0

            text = (
                f"Feedback XYZ: {xyz[0] * 1000.0:.2f}, "
                f"{xyz[1] * 1000.0:.2f}, {xyz[2] * 1000.0:.2f} mm"
                f" | v: {v_xyz:.2f} mm/s"
            )

            if theta is not None:
                q_text = ", ".join(f"{math.degrees(v):.2f}" for v in theta)
                qd_text = ", ".join(
                    f"{math.degrees(v):.2f}"
                    for v in self.node.feedback_theta_speed
                )
                text += f" | q: {q_text} deg | qd: {qd_text} deg/s"

            self.feedback.set(text)

        self.root.after(100, self._refresh_feedback)

def main(args=None):
    rclpy.init(args=args)
    node = DeltaControlNode()
    root = tk.Tk()
    app = DeltaControlApp(root, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
