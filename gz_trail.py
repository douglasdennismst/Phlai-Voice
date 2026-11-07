"""
Optional Gazebo "smoke trail": leaves glowing dots along the drone's REAL path (from Gazebo's
pose topic) while the pen is down. Dots are Gazebo visual markers, so they have no physics and
only show in the Gazebo GUI window. They fade after `lifetime` seconds of sim time.

Needs Gazebo's Python bindings, run with the SYSTEM Python (/usr/bin/python3), not the .venv:
the .venv has a pip protobuf (pulled in by faster-whisper) that breaks gz.msgs10.
    sudo apt install python3-gz-transport13 python3-gz-msgs10
Used by drone_adapter.py, which turns it on automatically when the bindings import.

Design: the pose subscription lives in this process, but marker requests are sent from a
separate helper process. In one process, the busy pose subscription (dozens of messages a
second) stalls the blocking marker requests, so only the first dot or two of each maneuver
ever got drawn. Dots are also batched (one /marker_array request per ~0.1 s) instead of one
request per dot.
"""
import multiprocessing as mp
import queue
import re
import threading
import time

TRAIL_VERSION = "gz_trail v3: separate marker process + batching"


# ======================= helper process: sends markers =======================

def _marker_worker(dots, logs, ns, size, color, lifetime):
    """Runs in its own process. Reads ("dot", x, y, z) / ("clear",) / None from `dots`."""
    try:
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.marker_pb2 import Marker
        from gz.msgs10.marker_v_pb2 import Marker_V
        from gz.transport13 import Node
    except Exception as exc:
        logs.put(f"trail: marker helper can't import Gazebo bindings ({type(exc).__name__}: {exc})")
        return

    node = Node()
    state = {"next_id": 1, "sent": 0, "fail": 0, "array_ok": None, "reply": None}

    # Which marker services does Gazebo offer?
    marker_srv, array_srv = "/marker", "/marker_array"
    for _ in range(15):
        try:
            found = [s for s in node.service_list() if "marker" in s.lower()]
        except Exception as exc:
            logs.put(f"trail: could not list services ({type(exc).__name__}: {exc})")
            break
        if found:
            marker_srv = next((s for s in found if s == "/marker"),
                              next((s for s in found if s.endswith("/marker")), marker_srv))
            array_srv = next((s for s in found if s == "/marker_array"),
                             next((s for s in found if s.endswith("/marker_array")), array_srv))
            logs.put(f"trail: marker services {found}; using {marker_srv} / {array_srv}")
            break
        time.sleep(1.0)
    else:
        logs.put("trail: Gazebo offers NO marker service - is the Gazebo GUI window open?")

    def make_dot(x, y, z):
        m = Marker()
        m.ns = ns
        m.id = state["next_id"]
        state["next_id"] += 1
        m.action = Marker.ADD_MODIFY
        m.type = Marker.SPHERE
        m.scale.x = m.scale.y = m.scale.z = size
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        r, g, b = color
        for c in (m.material.ambient, m.material.diffuse, m.material.emissive):
            c.r, c.g, c.b, c.a = r, g, b, 1.0
        if lifetime and lifetime > 0:
            m.lifetime.sec = int(lifetime)
            m.lifetime.nsec = int((lifetime % 1) * 1e9)
        return m

    def note_fail(why):
        state["fail"] += 1
        if state["fail"] in (1, 25, 250):
            logs.put(f"trail: {why} ({state['fail']} failed so far)")

    def send_one(marker, quiet=False):
        for reply in ([state["reply"]] if state["reply"] else [Boolean, Empty]):
            try:
                ok, _ = node.request(marker_srv, marker, Marker, reply, 2000)
            except Exception as exc:
                logs.put(f"trail: {marker_srv} request raised {type(exc).__name__}: {exc}")
                ok = False
            if ok:
                state["reply"] = reply
                return True
        if not quiet:
            note_fail(f"{marker_srv} did not answer")
        return False

    def send_batch(markers):
        if state["array_ok"] is not False:
            mv = Marker_V()
            mv.marker.extend(markers)
            try:
                ok, _ = node.request(array_srv, mv, Marker_V, Boolean, 2000)
            except Exception as exc:
                logs.put(f"trail: {array_srv} request raised {type(exc).__name__}: {exc}")
                ok = False
            if ok:
                state["array_ok"] = True
                return True
            if state["array_ok"] is None:     # never worked: stop trying it
                state["array_ok"] = False
                logs.put(f"trail: {array_srv} not usable, sending dots one at a time")
        ok = True
        for m in markers:
            ok = send_one(m) and ok
        return ok

    while True:
        item = dots.get()
        if item is None:
            return
        batch = [item]
        deadline = time.monotonic() + 0.1          # gather ~0.1 s worth of dots
        while time.monotonic() < deadline:
            try:
                nxt = dots.get(timeout=max(0.0, deadline - time.monotonic()))
            except queue.Empty:
                break
            if nxt is None:
                return
            batch.append(nxt)

        markers = []
        for it in batch:
            if it[0] == "clear":
                if markers:                        # flush dots queued before the clear
                    send_batch(markers)
                    markers = []
                m = Marker()
                m.ns = ns
                m.action = Marker.DELETE_ALL
                send_one(m, quiet=True)            # refused when nothing is drawn yet: fine
            else:
                markers.append(make_dot(*it[1:]))
        if markers and send_batch(markers):
            first = state["sent"] == 0
            state["sent"] += len(markers)
            if first:
                logs.put("trail: first dots accepted by Gazebo")


# ======================= main process: follows the drone =======================

class GzTrail:
    def __init__(self, log=print, model_name="x3", world=None, lifetime=120.0,
                 spacing=0.05, size=0.08, color=(0.1, 0.9, 1.0), namespace="phlai_trail"):
        # Imports are inside so a missing binding raises ImportError to the caller.
        from gz.msgs10.marker_pb2 import Marker          # noqa: F401  (checks the bindings)
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
        import google.protobuf
        pb = getattr(google.protobuf, "__version__", "?")
        if not pb.startswith(("3.", "4.")):
            log(f"trail: WARNING protobuf {pb} from {google.protobuf.__file__} - the Gazebo "
                "bindings expect Ubuntu's protobuf; run this with /usr/bin/python3, not the .venv")

        self._Pose_V = Pose_V
        self.node = Node()
        self.log = log
        self.model_name = model_name
        self._model_key = str(model_name).lower()      # match x3 / X3 either way
        self.world = world
        self.spacing = spacing
        self._worker_args = (namespace, size, color, lifetime)

        self._pen = False
        self._last = None
        self._seen_model = False
        self._reported = set()
        self._stop = threading.Event()
        self._ctx = mp.get_context("spawn")             # clean child: no copied ROS/gz state
        self._dots = self._ctx.Queue()
        self._logs = self._ctx.Queue()
        self._proc = None

    # ---------- public ----------
    def start(self):
        self._proc = self._ctx.Process(target=_marker_worker, daemon=True,
                                       args=(self._dots, self._logs) + self._worker_args)
        self._proc.start()
        self.log(f"trail: {TRAIL_VERSION} (marker helper pid {self._proc.pid})")
        threading.Thread(target=self._log_loop, daemon=True).start()
        threading.Thread(target=self._connect_loop, daemon=True).start()

    def stop(self):
        self._stop.set()
        try:
            self._dots.put(None)
        except Exception:
            pass
        if self._proc is not None:
            self._proc.join(timeout=1.0)
            if self._proc.is_alive():
                self._proc.terminate()

    def set_pen(self, down: bool):
        if down and not self._pen:
            self._last = None       # start a fresh stroke at the drone's current position
        self._pen = bool(down)

    def clear(self):
        self._last = None
        self._dots.put(("clear",))

    # ---------- helpers ----------
    def _log_loop(self):
        while not self._stop.is_set():
            try:
                self.log(self._logs.get(timeout=0.5))
            except queue.Empty:
                if self._proc is not None and not self._proc.is_alive() and not self._stop.is_set():
                    self.log(f"trail: marker helper exited (code {self._proc.exitcode})")
                    return
            except Exception:
                return

    def _report(self, kind, text):
        if kind not in self._reported:
            self._reported.add(kind)
            self.log(text)

    # ---------- pose subscription ----------
    def _find_topic(self):
        if self.world:
            return f"/world/{self.world}/dynamic_pose/info"
        pattern = re.compile(r"^/world/[^/]+/dynamic_pose/info$")
        try:
            for topic in self.node.topic_list():
                if pattern.match(topic):
                    return topic
        except Exception:
            pass
        return None

    def _connect_loop(self):
        while not self._stop.is_set():
            topic = self._find_topic()
            if topic and self.node.subscribe(self._Pose_V, topic, self._on_pose_v):
                self.log(f"trail: following '{self.model_name}' on {topic}")
                return
            time.sleep(1.0)

    def _on_pose_v(self, msg):
        # Keep this cheap: it runs for every pose message. Exceptions would be swallowed.
        try:
            if not self._pen:
                return
            for p in msg.pose:
                if p.name.lower() == self._model_key:
                    x, y, z = p.position.x, p.position.y, p.position.z
                    if self._last is not None:
                        dx, dy, dz = x - self._last[0], y - self._last[1], z - self._last[2]
                        if (dx * dx + dy * dy + dz * dz) ** 0.5 < self.spacing:
                            return
                    self._seen_model = True
                    self._last = (x, y, z)
                    self._dots.put(("dot", x, y, z))
                    return
            if not self._seen_model:
                names = [p.name for p in msg.pose][:8]
                self._report("model", f"trail: no model named '{self.model_name}' in pose "
                             f"messages (saw e.g. {names}); set -p trail_model:=<name>")
        except Exception as exc:
            self._report("pose", f"trail: pose callback error {type(exc).__name__}: {exc}")
