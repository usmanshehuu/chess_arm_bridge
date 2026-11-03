"""Configurable numerical inverse kinematics using standard DH parameters.

This is a generic adapter, not a calibration of any particular arm. DH values,
joint limits, frame offsets, and tool pose must match the robot model.
"""

import math

from .planner import MovePlanError


def _identity4():
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for i in range(len(a))]


def _dh(a, alpha, d, theta):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, 0.0],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, 0.0],
        [-sp, cp*sr, cp*cr, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_from_xyzw(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        raise MovePlanError("Target orientation quaternion has zero length")
    x, y, z, w = x/n, y/n, z/n, w/n
    return [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]


def _solve(a, b):
    """Solve a small dense linear system with pivoted Gauss-Jordan elimination."""
    n = len(b)
    m = [list(a[i]) + [b[i]] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(m[row][col]))
        if abs(m[pivot][col]) < 1e-12:
            raise MovePlanError("IK Jacobian is singular; target may be unreachable")
        m[col], m[pivot] = m[pivot], m[col]
        scale = m[col][col]
        m[col] = [v / scale for v in m[col]]
        for row in range(n):
            if row == col:
                continue
            scale = m[row][col]
            m[row] = [m[row][j] - scale * m[col][j] for j in range(n + 1)]
    return [m[i][-1] for i in range(n)]


class SerialDhIk:
    """Damped-least-squares IK for a serial revolute chain described by DH rows."""

    def __init__(self, config):
        self.rows = config.get("dh_standard", [])
        self.lower = config.get("lower_limits_rad", [])
        self.upper = config.get("upper_limits_rad", [])
        self.seed = config.get("seed_positions_rad", [])
        self.tcp_xyz = list(map(float, config.get("flange_to_tcp_xyz_m", [0.0, 0.0, 0.0])))
        self.tcp_rpy = list(map(float, config.get("flange_to_tcp_rpy_rad", [0.0, 0.0, 0.0])))
        self.max_iterations = int(config.get("max_iterations", 250))
        self.position_tolerance = float(config.get("position_tolerance_m", 0.003))
        self.orientation_tolerance = float(config.get("orientation_tolerance_rad", 0.05))
        self.damping = float(config.get("damping", 0.08))
        self.max_joint_step = float(config.get("max_joint_step_rad", 0.12))
        count = len(self.rows)
        if count != 6 or len(self.lower) != count or len(self.upper) != count or len(self.seed) != count:
            raise MovePlanError("kinematics requires six DH rows and six lower/upper/seed joint values")
        if any(float(lo) >= float(hi) for lo, hi in zip(self.lower, self.upper)):
            raise MovePlanError("Each lower joint limit must be below its upper limit")
        self.rows = [[float(v) for v in row] for row in self.rows]
        if any(len(row) != 4 for row in self.rows):
            raise MovePlanError("Each DH row must be [a_m, alpha_rad, d_m, theta_offset_rad]")
        if len(self.tcp_xyz) != 3 or len(self.tcp_rpy) != 3:
            raise MovePlanError("TCP offset and RPY must each have three values")
        self.lower = list(map(float, self.lower))
        self.upper = list(map(float, self.upper))
        self.seed = list(map(float, self.seed))

    def forward(self, q):
        t = _identity4()
        for (a, alpha, d, offset), angle in zip(self.rows, q):
            t = _matmul(t, _dh(a, alpha, d, angle + offset))
        tcp = _rpy_matrix(*self.tcp_rpy)
        tcp[0][3], tcp[1][3], tcp[2][3] = self.tcp_xyz
        return _matmul(t, tcp)

    @staticmethod
    def _error(t, goal_p, goal_r):
        current_r = [row[:3] for row in t[:3]]
        # Spatial rotation error R_goal * R_current^T.
        rel = [[sum(goal_r[i][k] * current_r[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
        cos_angle = max(-1.0, min(1.0, (rel[0][0] + rel[1][1] + rel[2][2] - 1.0) / 2.0))
        angle = math.acos(cos_angle)
        if angle < 1e-9:
            rot = [0.0, 0.0, 0.0]
        else:
            denom = 2.0 * math.sin(angle)
            axis = [(rel[2][1] - rel[1][2]) / denom, (rel[0][2] - rel[2][0]) / denom, (rel[1][0] - rel[0][1]) / denom]
            rot = [angle * v for v in axis]
        pos = [goal_p[i] - t[i][3] for i in range(3)]
        return pos + rot, math.sqrt(sum(v*v for v in pos)), math.sqrt(sum(v*v for v in rot))

    def solve(self, position, orientation_xyzw, seed=None):
        goal_p = list(map(float, position))
        goal_r = _rotation_from_xyzw(orientation_xyzw)
        q = list(self.seed if seed is None else seed)
        eps = 1e-5
        for _ in range(self.max_iterations):
            t = self.forward(q)
            error, p_error, r_error = self._error(t, goal_p, goal_r)
            if p_error <= self.position_tolerance and r_error <= self.orientation_tolerance:
                return q
            columns = []
            for i in range(6):
                q_perturbed = q[:]
                q_perturbed[i] += eps
                t2 = self.forward(q_perturbed)
                # Estimate the spatial Jacobian from small forward-kinematics changes.
                p_delta = [(t2[k][3] - t[k][3]) / eps for k in range(3)]
                r2 = [row[:3] for row in t2[:3]]
                current_r = [row[:3] for row in t[:3]]
                rel = [[sum(r2[i][k] * current_r[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
                c = max(-1.0, min(1.0, (rel[0][0] + rel[1][1] + rel[2][2] - 1.0) / 2.0))
                a = math.acos(c)
                if a < 1e-8:
                    r_delta = [0.0, 0.0, 0.0]
                else:
                    den = 2 * math.sin(a)
                    r_delta = [a*(rel[2][1]-rel[1][2])/den/eps, a*(rel[0][2]-rel[2][0])/den/eps, a*(rel[1][0]-rel[0][1])/den/eps]
                columns.append(p_delta + r_delta)
            jac = [[columns[col][row] for col in range(6)] for row in range(6)]
            # Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 error.
            jjt = [[sum(jac[i][k] * jac[j][k] for k in range(6)) + (self.damping**2 if i == j else 0.0) for j in range(6)] for i in range(6)]
            try:
                weighted = _solve(jjt, error)
            except MovePlanError:
                break
            dq = [sum(jac[row][col] * weighted[row] for row in range(6)) for col in range(6)]
            scale = max(1.0, max(abs(v) for v in dq) / self.max_joint_step)
            q = [min(self.upper[i], max(self.lower[i], q[i] + dq[i] / scale)) for i in range(6)]
        raise MovePlanError("IK did not converge. Check DH convention, calibration, joint limits, seed, and target reachability")
