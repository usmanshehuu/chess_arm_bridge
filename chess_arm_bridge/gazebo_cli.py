"""Standalone CLI: validate Stockfish move, create plan, optionally publish to Gazebo."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

from .gazebo_transport import GazeboTransportBackend
from .planner import ChessMovePlanner, MovePlanError
from .stockfish import UciChessEngine


DEFAULT_FEN = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"


def main():
    parser = argparse.ArgumentParser(description="Plan a chess pick-and-place and optionally send a Gazebo joint trajectory")
    parser.add_argument("--config", required=True)
    parser.add_argument("--fen", default=DEFAULT_FEN)
    parser.add_argument("--move", help="Use a supplied UCI move instead of querying Stockfish")
    parser.add_argument("--engine", "--stockfish", dest="engine_path", help="Path/name of UCI chess engine (overrides config)")
    parser.add_argument("--depth", type=int, default=12, help="Engine search depth when choosing the move")
    parser.add_argument("--movetime-ms", type=int, help="Use a fixed engine search time instead of depth")
    parser.add_argument("--execute", action="store_true", help="Publish to the configured Gazebo Sim trajectory controller")
    args = parser.parse_args()
    try:
        config = json.loads(Path(args.config).read_text())
        engine_result = None
        move = args.move
        if move is None:
            engine_config = config.get("uci_engine", config.get("stockfish", {}))
            engine_path = args.engine_path or engine_config.get("engine_path", "sunfish-uci")
            engine_result = UciChessEngine(
                engine_path, timeout_seconds=engine_config.get("timeout_seconds", 30)
            ).best_move(args.fen, depth=args.depth, movetime_ms=args.movetime_ms)
            move = engine_result["move_uci"]
        plan = ChessMovePlanner(config).create_plan(args.fen, move)
        result = {"mode": "EXECUTION REQUEST" if args.execute else "DRY RUN — no Gazebo command sent", **plan}
        if engine_result is not None:
            result["stockfish"] = {
                "bestmove_response": engine_result["response"],
                "depth": args.depth,
                "movetime_ms": args.movetime_ms,
            }
        if args.execute:
            result["gazebo"] = GazeboTransportBackend(config).execute(plan)
        print(json.dumps(result, indent=2))
        if not args.execute:
            print("\nDry run only. This command does not move the arm.")
        else:
            print("\nTrajectory published. Gazebo completion and grasp success are not monitored by this prototype.")
        return 0
    except (OSError, json.JSONDecodeError, KeyError, TypeError, MovePlanError, subprocess.TimeoutExpired) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
