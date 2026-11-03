"""Chess move validation and board-to-robot waypoint generation.

The module deliberately has no ROS dependency. It produces pose waypoints;
an application-specific motion backend must execute them.
"""

from dataclasses import dataclass
import math
from typing import Any

try:
    import chess
except ImportError as exc:  # pragma: no cover - environment dependent
    chess = None
    _CHESS_IMPORT_ERROR = exc
else:
    _CHESS_IMPORT_ERROR = None


class MovePlanError(ValueError):
    """Invalid or unsupported chess move / planning configuration."""


@dataclass(frozen=True)
class Pose:
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


def _qmul(a: tuple[float, float, float, float], b: tuple[float, float, float, float]):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _q_from_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _rotate(q: tuple[float, float, float, float], v: tuple[float, float, float]):
    # q * (v, 0) * conjugate(q)
    t = _qmul(q, (v[0], v[1], v[2], 0.0))
    r = _qmul(t, (-q[0], -q[1], -q[2], q[3]))
    return r[:3]


class ChessMovePlanner:
    """Create safe vertical pick-and-place poses for ordinary moves."""

    def __init__(self, config: dict[str, Any]):
        board = config.get("board", {})
        robot = config.get("robot", {})
        self.square_size = float(board["square_size_m"])
        self.board_height = float(board.get("surface_height_m", 0.0))
        self.origin = tuple(map(float, board.get("a1_center_xyz_m", [0, 0, 0])))
        self.board_q = _q_from_rpy(*map(float, board.get("board_rpy_rad", [0, 0, 0])))
        self.grasp_height = float(robot["grasp_height_above_board_m"])
        self.clearance = float(robot["travel_clearance_above_board_m"])
        self.tool_q = _q_from_rpy(*map(float, robot.get("tool_rpy_in_board_rad", [0, 0, 0])))
        if self.square_size <= 0 or self.grasp_height < 0 or self.clearance <= self.grasp_height:
            raise MovePlanError("square_size must be positive and travel clearance must exceed nonnegative grasp height")
        if len(self.origin) != 3:
            raise MovePlanError("board.a1_center_xyz_m must contain three values")

    def _square_position(self, square: int, z: float):
        file_index = square % 8
        rank_index = square // 8
        local = (file_index * self.square_size, rank_index * self.square_size, self.board_height + z)
        rotated = _rotate(self.board_q, local)
        return tuple(self.origin[i] + rotated[i] for i in range(3))

    def _pose(self, square: int, z: float):
        return Pose(self._square_position(square, z), _qmul(self.board_q, self.tool_q))

    def create_plan(self, fen: str, uci_move: str) -> dict[str, Any]:
        if chess is None:
            raise MovePlanError(
                "python-chess is required for legal move validation. Install it with: "
                "python3 -m pip install 'python-chess>=1.999'"
            ) from _CHESS_IMPORT_ERROR
        try:
            board = chess.Board(fen)
        except (ValueError, TypeError) as exc:
            raise MovePlanError(f"Invalid FEN: {exc}") from exc
        try:
            move = chess.Move.from_uci(uci_move)
        except ValueError as exc:
            raise MovePlanError(f"Invalid UCI move '{uci_move}'") from exc
        if move not in board.legal_moves:
            raise MovePlanError(f"Move '{uci_move}' is not legal in the supplied FEN")
        if board.is_castling(move):
            raise MovePlanError("Castling is recognized but unsupported: it also requires moving the rook")
        if board.is_en_passant(move):
            raise MovePlanError("En passant is recognized but unsupported: it requires removing a pawn from another square")
        if board.is_capture(move):
            raise MovePlanError("Captures are recognized but unsupported until captured-piece tray handling is configured")
        if move.promotion:
            raise MovePlanError("Promotion is recognized but unsupported until piece replacement is configured")

        src, dst = move.from_square, move.to_square
        z_grasp = self.grasp_height
        z_clear = self.clearance
        points = [
            ("approach_source", src, z_clear, "open"),
            ("grasp_source", src, z_grasp, "close"),
            ("lift_source", src, z_clear, "hold"),
            ("approach_destination", dst, z_clear, "hold"),
            ("release_destination", dst, z_grasp, "open"),
            ("retreat_destination", dst, z_clear, "open"),
        ]
        result_points = []
        for name, square, z, gripper in points:
            square_name = chess.square_name(square)
            board_position = [
                (square % 8) * self.square_size,
                (square // 8) * self.square_size,
                self.board_height + z,
            ]
            pose = self._pose(square, z)
            result_points.append({
                "name": name,
                "square": square_name,
                "board_frame_xyz_m": board_position,
                "robot_frame_xyz_m": list(pose.position),
                "robot_frame_orientation_xyzw": list(pose.orientation_xyzw),
                "gripper": gripper,
            })
        return {"move_uci": uci_move, "fen_before": fen, "waypoints": result_points}
