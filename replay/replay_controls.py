"""Thread-safe replay clock and loopback-only control panel."""

from bisect import bisect_right
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from threading import Lock, Thread
from time import monotonic


class Playback:
    def __init__(self, times, speed=1.0, clock=monotonic):
        self.times = times
        self.speed = speed
        self.clock = clock
        self.lock = Lock()
        self.anchor = 0.0
        self.started = self.clock()
        self.playing = True
        self.period = times[-1] - times[0] + 3.0

    def _offset(self, now):
        if self.playing:
            return (self.anchor + (now - self.started) * self.speed) % self.period
        return self.anchor

    def _frame(self, offset):
        playhead = min(self.times[0] + offset, self.times[-1])
        return playhead, bisect_right(self.times, playhead) - 1, self.playing

    def sample(self):
        with self.lock:
            return self._frame(self._offset(self.clock()))

    def command(self, action, frame=None):
        with self.lock:
            now = self.clock()
            offset = self._offset(now)
            if action in ("play", "pause"):
                self.anchor = offset
                self.started = now
                self.playing = action == "play"
                return
            if action in ("previous", "next"):
                index = self._frame(offset)[1]
                delta = -1 if action == "previous" else 1
                frame = min(max(index + delta, 0), len(self.times) - 1)
            elif action != "seek" or type(frame) is not int or not 0 <= frame < len(self.times):
                raise ValueError("Invalid replay command or frame")
            self.anchor = self.times[frame] - self.times[0]
            self.playing = False


@contextmanager
def panel_server(port, playback, case, seed):
    times, frames, _, metrics, events = case
    page = Path(__file__).with_name("replay_panel.html").read_bytes()
    event_times = [time for time in times if events.get(time)]
    metadata = {
        "seed": seed,
        "times": times,
        "events": [
            {"frame": index, "time": time, **event}
            for index, time in enumerate(times)
            for event in events.get(time, ())
        ],
    }

    def current():
        playhead, index, playing = playback.sample()
        time = times[index]
        distance, closest = metrics[time]
        previous = bisect_right(event_times, time) - 1
        last_event = event_times[previous] if previous >= 0 else None
        return {
            "time": playhead,
            "frame": index,
            "sample_time": time,
            "playing": playing,
            "distance": distance,
            "closest": closest,
            "roles": {uid: pose[3] for uid, pose in frames[time].items()},
            "last_event_time": last_event,
            "last_events": events[last_event] if last_event is not None else [],
        }

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def reply(self, body, media="application/json", status=200):
            self.send_response(status)
            self.send_header("Content-Type", media + "; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, payload, status=200):
            self.reply(json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8"),
                       status=status)

        def local_host(self):
            host = self.headers.get("Host", "")
            port = self.server.server_port
            return host if host in (f"127.0.0.1:{port}", f"localhost:{port}") else ""

        def do_GET(self):
            if not self.local_host():
                return self.send_json({"error": "Use a local address"}, 403)
            if self.path == "/":
                self.reply(page, "text/html")
            elif self.path == "/api/case":
                self.send_json(metadata)
            elif self.path == "/api/state":
                self.send_json(current())
            else:
                self.send_json({"error": "Not found"}, 404)

        def do_POST(self):
            host = self.local_host()
            if (not host or self.headers.get("Origin") != f"http://{host}"
                    or self.headers.get_content_type() != "application/json"):
                return self.send_json({"error": "Only local panel requests are accepted"}, 403)
            if self.path != "/api/control":
                return self.send_json({"error": "Not found"}, 404)
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 512:
                    raise ValueError("Invalid request length")
                payload = json.loads(self.rfile.read(size))
                if not isinstance(payload, dict):
                    raise ValueError("Invalid request")
                playback.command(payload.get("action"), payload.get("frame"))
            except (ValueError, UnicodeError):
                return self.send_json({"error": "Invalid playback command"}, 400)
            self.send_json(current())

    with ThreadingHTTPServer(("127.0.0.1", port), Handler) as server:
        worker = Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            yield f"http://localhost:{server.server_port}/"
        finally:
            server.shutdown()
            worker.join()
