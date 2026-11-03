"""Command-line dry-run interface."""

import argparse
import json
from pathlib import Path
import sys

from .planner import ChessMovePlanner, MovePlanError


DEFAULT_FEN = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"


def main():
    parser = argparse.ArgumentParser(description="Generate a dry-run pick-and-place plan from a legal UCI chess move")
    parser.add_argument("--config", required=True, help="Path to JSON robot/board calibration config")
    parser.add_argument("--fen", default=DEFAULT_FEN, help="Current position in FEN notation")
    parser.add_argument("--move", required=True, help="Stockfish move in UCI notation, e.g. e2e4")
    parser.add_argument("--dry-run", action="store_true", help="Print waypoints only; never command an arm")
    args = parser.parse_args()
    if not args.dry_run:
        parser.error("Only --dry-run is available until a verified robot motion backend is configured")
    try:
        config = json.loads(Path(args.config).read_text())
        plan = ChessMovePlanner(config).create_plan(args.fen, args.move)
    except (OSError, json.JSONDecodeError, KeyError, TypeError, MovePlanError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"mode": "DRY RUN — no robot commands sent", **plan}, indent=2))
    print("\nNOTE: Robot-frame output assumes the configured board transform and tool orientation are calibrated.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
