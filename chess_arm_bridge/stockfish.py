"""Small UCI client for an installed Stockfish executable."""

import queue
import shutil
import subprocess
import threading

from .planner import MovePlanError


class StockfishUci:
    def __init__(self, engine_path="stockfish", timeout_seconds=30):
        self.engine_path = engine_path
        self.timeout_seconds = float(timeout_seconds)

    def best_move(self, fen, depth=12, movetime_ms=None):
        executable = shutil.which(self.engine_path)
        if executable is None:
            raise MovePlanError(
                f"Stockfish executable not found: {self.engine_path!r}. "
                "Download the official binary and pass its path with --stockfish."
            )
        if depth < 1:
            raise MovePlanError("Stockfish search depth must be at least 1")
        if movetime_ms is not None and movetime_ms < 1:
            raise MovePlanError("Stockfish movetime must be at least 1 ms")

        lines = queue.Queue()
        process = subprocess.Popen(
            [executable], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, bufsize=1,
        )

        def read_stdout():
            for line in process.stdout:
                lines.put(line.rstrip("\r\n"))
            lines.put(None)

        threading.Thread(target=read_stdout, daemon=True).start()

        def send(command):
            process.stdin.write(command + "\n")
            process.stdin.flush()

        def wait_for(prefix):
            while True:
                try:
                    line = lines.get(timeout=self.timeout_seconds)
                except queue.Empty as exc:
                    raise MovePlanError(f"Timed out waiting for Stockfish response {prefix!r}") from exc
                if line is None:
                    raise MovePlanError("Stockfish exited before returning a best move")
                if line.startswith(prefix):
                    return line

        try:
            send("uci")
            wait_for("uciok")
            send("isready")
            wait_for("readyok")
            send(f"position fen {fen}")
            send(f"go movetime {int(movetime_ms)}" if movetime_ms is not None else f"go depth {int(depth)}")
            response = wait_for("bestmove ")
            parts = response.split()
            if len(parts) < 2 or parts[1] in ("(none)", "0000"):
                raise MovePlanError("Stockfish returned no legal move for this position")
            return {"move_uci": parts[1], "response": response}
        finally:
            if process.poll() is None:
                try:
                    send("quit")
                    process.wait(timeout=2)
                except (OSError, subprocess.TimeoutExpired):
                    process.kill()
                    process.wait()
            if process.stdout:
                process.stdout.close()
            if process.stdin:
                process.stdin.close()


# Kept as a generic name because this UCI client also works with Sunfish and
# other engines that implement the standard UCI protocol.
UciChessEngine = StockfishUci
