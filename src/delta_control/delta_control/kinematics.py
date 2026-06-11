#!/usr/bin/env python3

import math
from dataclasses import dataclass


class DeltaIKError(Exception):
    pass


@dataclass
class DeltaGeometry2:
    """
    Geometry same as current SDF model.
    Unit:
      length: meter
      angle: rad
    """

    rb: float = 0.130
    rp: float = 0.0238676
    l1: float = 0.080
    l2: float = 0.39485

    home_x: float = 0.0
    home_y: float = 0.0
    home_z: float = -0.361867
    motor_plane_z: float = -0.020

    motor_lower: float = -math.pi
    motor_upper: float = math.pi

    motor_sign_1: float = 1.0
    motor_sign_2: float = 1.0
    motor_sign_3: float = 1.0

    motor_offset_1: float = 0.0813412
    motor_offset_2: float = 0.0813412
    motor_offset_3: float = 0.0813412

    @property
    def d_base(self) -> float:
        return self.rb - self.rp


class DeltaKinematics2:
    def __init__(self, geometry: DeltaGeometry2 | None = None):
        self.g = geometry if geometry is not None else DeltaGeometry2()

        self.phis = (
            0.0,
            -2.0 * math.pi / 3.0,
            2.0 * math.pi / 3.0,
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def rad_to_deg(angle: float) -> float:
        return angle * 180.0 / math.pi

    def _choose_solution_near(
        self,
        theta_a: float,
        theta_b: float,
        preferred: float,
    ) -> float:
        theta_a = self.normalize_angle(theta_a)
        theta_b = self.normalize_angle(theta_b)

        err_a = abs(self.normalize_angle(theta_a - preferred))
        err_b = abs(self.normalize_angle(theta_b - preferred))

        return theta_a if err_a <= err_b else theta_b

    def solve_one_branch(
        self,
        x: float,
        y: float,
        z: float,
        phi: float,
        preferred_theta: float = 0.0,
    ) -> float:
        g = self.g

        ux = math.cos(phi)
        uy = math.sin(phi)

        vx = -math.sin(phi)
        vy = math.cos(phi)

        s = ux * x + uy * y - g.d_base
        h = vx * x + vy * y

        A = -s
        B = z
        C = (
            g.l2 * g.l2
            - s * s
            - h * h
            - z * z
            - g.l1 * g.l1
        ) / (2.0 * g.l1)

        R = math.sqrt(A * A + B * B)

        if R < 1e-9:
            raise DeltaIKError("Singular branch: R is too small")

        ratio = C / R

        if ratio > 1.0 + 1e-9 or ratio < -1.0 - 1e-9:
            raise DeltaIKError(
                f"Target outside workspace: C/R={ratio:.9f}, "
                f"phi={self.rad_to_deg(phi):.3f} deg"
            )

        ratio = max(-1.0, min(1.0, ratio))

        beta = math.atan2(B, A)
        gamma = math.acos(ratio)

        theta_a = beta + gamma
        theta_b = beta - gamma

        return self._choose_solution_near(theta_a, theta_b, preferred_theta)

    def inverse_kinematics_raw(
        self,
        x: float,
        y: float,
        z: float,
        preferred: tuple[float, float, float] | None = None,
    ) -> tuple[float, float, float]:
        if preferred is None:
            preferred = (0.0, 0.0, 0.0)
        else:
            preferred = (
                (preferred[0] - self.g.motor_offset_1) / self.g.motor_sign_1,
                (preferred[1] - self.g.motor_offset_2) / self.g.motor_sign_2,
                (preferred[2] - self.g.motor_offset_3) / self.g.motor_sign_3,
            )

        z_from_motor_plane = z - self.g.motor_plane_z

        theta1 = self.solve_one_branch(
            x, y, z_from_motor_plane, self.phis[0], preferred_theta=preferred[0]
        )
        theta2 = self.solve_one_branch(
            x, y, z_from_motor_plane, self.phis[1], preferred_theta=preferred[1]
        )
        theta3 = self.solve_one_branch(
            x, y, z_from_motor_plane, self.phis[2], preferred_theta=preferred[2]
        )

        return theta1, theta2, theta3

    def apply_calibration(
        self,
        raw_thetas: tuple[float, float, float],
        normalize: bool = True,
    ) -> tuple[float, float, float]:
        t1, t2, t3 = raw_thetas
        g = self.g

        cmd1 = g.motor_sign_1 * t1 + g.motor_offset_1
        cmd2 = g.motor_sign_2 * t2 + g.motor_offset_2
        cmd3 = g.motor_sign_3 * t3 + g.motor_offset_3

        if normalize:
            cmd1 = self.normalize_angle(cmd1)
            cmd2 = self.normalize_angle(cmd2)
            cmd3 = self.normalize_angle(cmd3)

        return cmd1, cmd2, cmd3

    def is_within_motor_limits(
        self,
        x: float,
        y: float,
        z: float,
    ) -> bool:
        try:
            raw = self.inverse_kinematics_raw(x, y, z)
        except DeltaIKError:
            return False
        commands = self.apply_calibration(raw, normalize=False)
        return all(
            self.g.motor_lower <= value <= self.g.motor_upper
            for value in commands
        )

    def check_joint_limits(self, thetas: tuple[float, float, float]) -> None:
        for i, theta in enumerate(thetas, start=1):
            if theta < self.g.motor_lower or theta > self.g.motor_upper:
                raise DeltaIKError(
                    f"motor_joint_{i} out of limit: "
                    f"{theta:.6f} rad = {self.rad_to_deg(theta):.3f} deg"
                )

    def inverse_kinematics(
        self,
        x: float,
        y: float,
        z: float,
        check_limits: bool = True,
        preferred: tuple[float, float, float] | None = None,
    ) -> tuple[float, float, float]:
        raw = self.inverse_kinematics_raw(x, y, z, preferred=preferred)
        cmd = self.apply_calibration(raw)

        if check_limits:
            self.check_joint_limits(cmd)

        return cmd

    def _mat3_det(self, m):
        return (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )

    def _mat3_inv(self, m):
        det = self._mat3_det(m)

        if abs(det) < 1e-12:
            raise DeltaIKError("Jacobian matrix A is singular")

        inv_det = 1.0 / det

        return [
            [
                (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det,
                (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det,
                (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det,
            ],
            [
                (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det,
                (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det,
                (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det,
            ],
            [
                (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det,
                (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det,
                (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det,
            ],
        ]

    def _mat3_vec_mul(self, m, v):
        return (
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
        )

    def constraint_jacobians(
        self,
        p: tuple[float, float, float],
        q_ref: tuple[float, float, float],
    ):
        """
        Delta robot velocity constraint:

            A(P, q) * P_dot + B(P, q) * q_dot = 0

        So:

            q_dot = -inv(B) * A * P_dot

        A: Cartesian-side Jacobian, 3x3
        B: joint-side Jacobian, 3x3 diagonal
        """

        x, y, z_world = p
        z = z_world - self.g.motor_plane_z

        A = []
        B = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ]

        raw_q = (
            (q_ref[0] - self.g.motor_offset_1) / self.g.motor_sign_1,
            (q_ref[1] - self.g.motor_offset_2) / self.g.motor_sign_2,
            (q_ref[2] - self.g.motor_offset_3) / self.g.motor_sign_3,
        )

        for i, (theta, phi) in enumerate(zip(raw_q, self.phis)):
            ux = math.cos(phi)
            uy = math.sin(phi)

            vx = -math.sin(phi)
            vy = math.cos(phi)

            s = ux * x + uy * y - self.g.d_base
            h = vx * x + vy * y

            # Constraint of each lower arm:
            # (s - l1*cos(theta))^2 + h^2 + (z + l1*sin(theta))^2 = l2^2
            a = s - self.g.l1 * math.cos(theta)
            b = z + self.g.l1 * math.sin(theta)

            # Cartesian Jacobian row:
            # df/dP = [a*ux + h*vx, a*uy + h*vy, b]
            #
            # The common factor 2 is omitted because it appears on both
            # A and B and cancels in -inv(B)*A.
            A_row = [
                a * ux + h * vx,
                a * uy + h * vy,
                b,
            ]

            # Joint Jacobian term:
            # df/dtheta = l1 * (s*sin(theta) + z*cos(theta))
            #
            # The common factor 2 is also omitted.
            B_ii = self.g.l1 * (
                s * math.sin(theta) + z * math.cos(theta)
            )

            if abs(B_ii) < 1e-12:
                raise DeltaIKError(
                    f"Joint-side Jacobian B is singular at branch {i + 1}"
                )

            A.append(A_row)
            B[i][i] = B_ii

        return A, B

    def inverse_jacobian(
        self,
        p: tuple[float, float, float],
        q_ref: tuple[float, float, float],
    ):
        """
        Return J_inv such that:

            q_dot = J_inv * P_dot

        From:
            A * P_dot + B * q_dot = 0

        Then:
            J_inv = -inv(B) * A
        """

        A, B = self.constraint_jacobians(p, q_ref)

        J_inv = []

        for i in range(3):
            B_ii = B[i][i]

            row = [
                -A[i][0] / B_ii,
                -A[i][1] / B_ii,
                -A[i][2] / B_ii,
            ]

            J_inv.append(row)

        return J_inv

    def forward_jacobian(
        self,
        p: tuple[float, float, float],
        q_ref: tuple[float, float, float],
    ):
        """
        Return J such that:

            P_dot = J * q_dot

        From:
            A * P_dot + B * q_dot = 0

        Then:
            J = -inv(A) * B
        """

        A, B = self.constraint_jacobians(p, q_ref)
        A_inv = self._mat3_inv(A)

        J = []

        for r in range(3):
            row = []

            for c in range(3):
                value = 0.0

                for k in range(3):
                    value += -A_inv[r][k] * B[k][c]

                row.append(value)

            J.append(row)

        return J

    def cartesian_to_joint_velocity(
        self,
        p: tuple[float, float, float],
        p_dot: tuple[float, float, float],
        q_ref: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """
        Convert Cartesian velocity to joint velocity by inverse Jacobian:

            q_dot = J_inv(P, q) * P_dot
        """

        J_inv = self.inverse_jacobian(p, q_ref)
        return self._mat3_vec_mul(J_inv, p_dot)

    def trajectory_point(
        self,
        p0: tuple[float, float, float],
        p1: tuple[float, float, float],
        duration: float,
        tau: float,
    ):
        """
        Quintic smoothstep time scaling:
        P = P0 + s(tau)(P1 - P0)

        s      = 10*tau^3 - 15*tau^4 + 6*tau^5
        s_dot  = (30*tau^2 - 60*tau^3 + 30*tau^4) / T
        s_ddot = (60*tau - 180*tau^2 + 120*tau^3) / T^2

        Properties:
        s(0) = 0, s(1) = 1
        s_dot(0) = 0, s_dot(1) = 0
        s_ddot(0) = 0, s_ddot(1) = 0

        The Cartesian path stays straight while adjacent jog segments meet
        without an acceleration step at their endpoints.
        """

        if duration <= 0.0:
            raise DeltaIKError("Duration must be greater than 0")

        tau = max(0.0, min(1.0, tau))

        tau2 = tau * tau
        tau3 = tau2 * tau
        tau4 = tau3 * tau
        tau5 = tau4 * tau

        s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5

        s_dot = (
            30.0 * tau2
            - 60.0 * tau3
            + 30.0 * tau4
        ) / duration

        s_ddot = (
            60.0 * tau
            - 180.0 * tau2
            + 120.0 * tau3
        ) / (duration * duration)

        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        dz = p1[2] - p0[2]

        p = (
            p0[0] + s * dx,
            p0[1] + s * dy,
            p0[2] + s * dz,
        )

        p_dot = (
            s_dot * dx,
            s_dot * dy,
            s_dot * dz,
        )

        p_ddot = (
            s_ddot * dx,
            s_ddot * dy,
            s_ddot * dz,
        )

        return p, p_dot, p_ddot

    def generate_joint_trajectory(
        self,
        p0: tuple[float, float, float],
        p1: tuple[float, float, float],
        duration: float,
        n_waypoints: int,
        preferred_start: tuple[float, float, float] | None = None,
    ) -> list[dict]:
        if duration <= 0.0:
            raise DeltaIKError("Duration must be greater than 0")

        if n_waypoints < 2:
            raise DeltaIKError("Number of waypoints must be at least 2")

        dt = duration / float(n_waypoints - 1)

        samples = []
        preferred_q = preferred_start

        for k in range(n_waypoints):
            tau = float(k) / float(n_waypoints - 1)
            t = tau * duration

            p, p_dot, p_ddot = self.trajectory_point(p0, p1, duration, tau)

            q = self.inverse_kinematics(
                p[0],
                p[1],
                p[2],
                check_limits=True,
                preferred=preferred_q,
            )

            q_dot = self.cartesian_to_joint_velocity(p, p_dot, q)

            samples.append(
                {
                    "index": k,
                    "t": t,
                    "tau": tau,
                    "p": p,
                    "p_dot": p_dot,
                    "p_ddot": p_ddot,
                    "q": q,
                    "q_dot": q_dot,
                    "q_ddot": (0.0, 0.0, 0.0),
                }
            )

            preferred_q = q

        # q_ddot by finite difference from q_dot.
        #
        # Note:
        #   This is still an approximation.
        #   Later, if you want cleaner acceleration, you can compute q_ddot
        #   using Jacobian derivative or numerical differentiation with smoothing.
        for k in range(n_waypoints):
            if n_waypoints == 2:
                qdd = tuple(
                    (samples[1]["q_dot"][i] - samples[0]["q_dot"][i]) / dt
                    for i in range(3)
                )
            elif k == 0:
                qdd = tuple(
                    (samples[1]["q_dot"][i] - samples[0]["q_dot"][i]) / dt
                    for i in range(3)
                )
            elif k == n_waypoints - 1:
                qdd = tuple(
                    (samples[k]["q_dot"][i] - samples[k - 1]["q_dot"][i]) / dt
                    for i in range(3)
                )
            else:
                qdd = tuple(
                    (samples[k + 1]["q_dot"][i] - samples[k - 1]["q_dot"][i])
                    / (2.0 * dt)
                    for i in range(3)
                )

            samples[k]["q_ddot"] = qdd

        return samples
