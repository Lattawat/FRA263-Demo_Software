# Per-Group Web Port Configuration — ARCHIVED approach

**Status:** removed. Ports are now **fixed** at `9090` (WebSocket) / `8000` (HTTP), because groups run on different machines (distinct IPs), so `localhost`/IP already separates them.

**Why this file exists:** it captures the full *derived-port* scheme that used to exist, so it can be re-implemented later by reading this doc. It was part of the old `pair_id` model (which also used `ROS_DOMAIN_ID`); isolation is now by ROS **namespace** (`group_number` → `/G<N>/`) — see §13 of *System Design.md*. Only the **port derivation** is described here.

---

## What the old scheme did

Each group `N` got a **unique WebSocket + HTTP port** so multiple verifiers could co-locate on one host without binding conflicts:

| group | ws_port | http_port |
|---|---|---|
| 0 | 9090 | 8000 |
| N≥1 | `9000 + N` | `8000 + N` |

e.g. group 11 → WS `9011`, HTTP `8011`.

## The three pieces (to restore)

### 1. Derivation helper (`bringup.launch.py`)
The launch `_derive_*` helper returned the ports alongside the LSL session:
```python
def _derive_pair(pair_id: int):        # old signature
    if pair_id:
        return 9000 + pair_id, 8000 + pair_id, str(pair_id)   # (ws_port, http_port, session)
    return 9090, 8000, ""
```
The current helper is `_derive_group(n) -> (namespace, session)` and does **not** return ports. To restore, have it also compute `ws_port = 9000+N`, `http_port = 8000+N` (default `9090/8000` for group 0).

### 2. Pass the ports to `web_visualizer` (in the OpaqueFunction)
The node was built with the derived ports as param overrides:
```python
web_visualizer_node = Node(
    package="claude_visualizer", executable="web_visualizer.py", name="web_visualizer",
    parameters=[
        params_file,
        {"ws_port": ws_port, "http_port": http_port, "session": session},   # ← restore ws_port/http_port
    ],
)
```
Currently only `{"session": session}` is passed (ports come from `params.yaml`). To restore: add the two port keys back to this dict.

### 3. Browser side — **already in place, no change needed to restore**
The port-discovery mechanism was **kept** even though the port is now fixed, so restoring derived ports needs *nothing* on the browser side:
- `web_visualizer.py` `_SilentHTTPHandler.do_GET` serves `GET /config.json → {"ws_port": <the node's ws_port>}`, and `_run_http_server` attaches `self._http_server.ws_port = self._ws_port`.
- `app.js` `resolveWsUrl()` does `fetch("/config.json")` and opens `ws://${location.hostname}:${cfg.ws_port}`.

Because the browser reads the port from `/config.json` (not a hardcoded value), it automatically follows whatever `ws_port` the node is running on. So the frontend works for fixed *or* derived ports with no edits.

## How to restore (checklist)
1. In `bringup.launch.py`, make the derivation return ports again: `ws_port = 9000+N`, `http_port = 8000+N` (`9090/8000` for group 0).
2. In the OpaqueFunction's `web_visualizer_node`, add `"ws_port": ws_port, "http_port": http_port` back to the params dict.
3. Nothing else — `/config.json` + `resolveWsUrl` (browser) and `params.yaml` defaults are unchanged.

## Caveat that motivated removal
- Different groups run on **different machines** → same fixed port on each host is fine and simpler.
- If you ever co-locate multiple groups on **one** host, restore this scheme (or override `ws_port`/`http_port` per launch) so they don't collide on `9090`/`8000`.
- Under Docker with published ports, the port must be published **1:1** (`"9011:9011"`) because `/config.json` reports the container-internal port to the browser.
