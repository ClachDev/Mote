# M3 verification ledger

What was measured for the fleet dashboard and the dispatch API, how, and what is
still unverified. The interfaces it verifies are
[`fleet-api.md`](fleet-api.md) and, unchanged, [`control-plane.md`](control-plane.md).

## 1. The broker question M1 left open — **decided and measured**

M1 found that conda-forge's mosquitto is built without libwebsockets, which the
dashboard's read path depends on ([`m1-verification.md` §4](m1-verification.md)).
Of the three options recorded there, M3 takes the first: **a container
mosquitto**, which is also what the design already has the fleet server being.
EMQX is not needed at this size, and a WS↔MQTT relay in the fleet API would put
a service back in the middle of the read path that Q5 argues to keep clear.

```console
$ pixi run fleet-broker-ws        # mote_fleet/server/broker.sh --docker
broker: eclipse-mosquitto:2 (docker)   state: ~/.mote-fleet   config: …/mosquitto.conf
1785092291: Opening ipv4 listen socket on port 11883.
1785092291: Opening ipv4 listen socket on port 19001.
```

(The ports are shifted by 10000 in this run and everywhere below: the
workstation it was measured on already runs Home Assistant's own MQTT broker on
1883. The shipped config uses 1883/9001.)

Two things fell out of running it:

- **`eclipse-mosquitto:2` is now mosquitto 2.1.2, which implements WebSockets
  itself** rather than linking libwebsockets — `ldd | grep websockets` finds
  nothing and `Websockets support available.` is printed anyway. So "does this
  build have websockets" cannot be answered by inspecting the binary alone;
  `broker.sh` uses the link check to decide whether to *strip* the listener from
  a conda build, which is the safe direction (a build that does support it and
  is not detected still runs, just without WS).
- **`log_type` replaces mosquitto's default set rather than adding to it.** The
  M1 config named `warning` and `notice`, which silently dropped `error` — which
  is exactly how the port clash above was found: a *second* broker was already
  holding 1883 (the M1 conda one, serving a live robot), and the new one exited
  with **no message at all**, so a failed bind looked like a websockets problem
  for twenty minutes. The config no longer sets `log_type`; the default set
  includes errors and the listener lines, which are the direct answer to both
  "why did it stop?" and "is the WS listener up?".

`pixi run fleet-broker` (the conda one) still runs, prints that it has no
websockets, names the task that does, and serves robots and `fleetctl`
unchanged.

## 2. The operator view, in a real browser — **9/9 checks, off the ROS graph**

The dashboard was driven by headless Chrome over the DevTools protocol against a
live stack: the container broker, `fleet-server` (from the ROS-free `fleet`
environment), two enrolled robots, and an operator token.

```console
$ node mote_fleet/test/browser_check.mjs http://127.0.0.1:8088 <token>
ok   the browser connected to the broker over WebSockets  — broker connected
ok   the roster came from retained MQTT state  — mote-01,mote-02
ok   health states are rendered  — ok,degraded
ok   a basemap was resolved for the selected robot  — office_world/ground
ok   the map canvas has pixels on it  — 467726 painted pixels
ok   the health roll-up lists subsystems  — 4 rows
ok   dispatch went through the fleet API  — dispatched f6f07ef1809a4f18
ok   the robot answered on task/status  — accepted,dispatched,rejected
ok   no uncaught page errors  — 0

9/9 checks passed
```

![The fleet dashboard](../images/fleet-ui.webp)

Nothing was polled: the roster, the health roll-up, both robot positions and the
task-status log are all retained MQTT state that arrived on the WebSocket within
a second of the page loading. Both colour schemes were rendered
(`Emulation.setEmulatedMedia`) and checked by eye.

The robots behind it are the wire, not the hardware — a script publishing the
real `protocol.py` payloads and answering `task/command`. The **real** agent and
behaviour tree are covered by the end-to-end test in §4; what this run is for is
the half only a browser can answer.

## 3. Dispatch is mediated — **confirmed, including the refusals**

`fleetctl dispatch` now POSTs to the fleet API instead of publishing. The status
half still reads straight from the broker, so the transitions look exactly as
they did in M1:

```console
$ pixi run fleetctl -- dispatch mote-01 goto pickup --wait 30
an operator token is required: --token, or MOTE_FLEET_TOKEN in the environment.
Mint one on the fleet box with 'fleetctl operator new --name <you>'.

$ export MOTE_FLEET_TOKEN=$(pixi run fleetctl -- operator new --name michael)
$ pixi run fleetctl -- dispatch mote-01 goto pickup --wait 30
-> mote-01: goto pickup  (id 9aec3ce2dab7458f)
2026-07-26T19:03:54.144Z  dispatched
2026-07-26T19:03:54.144Z  accepted
2026-07-26T19:04:00.150Z  succeeded          # exit 0

$ pixi run fleetctl -- dispatch mote-01 wibble --wait 15
-> mote-01: wibble  (id 8365e501be564767)
2026-07-26T19:04:01.644Z  dispatched
2026-07-26T19:04:01.644Z  rejected  (unknown command 'wibble')   # exit 1

$ pixi run fleetctl -- dispatch mote-99 goto home
http://…/v1/robots/mote-99/dispatch: 404 Not Found — no such robot

$ pixi run fleetctl -- audit --limit 6
WHEN                  WHO            ROBOT      RESULT       COMMAND
2026-07-26T19:00:27Z  michael        mote-01    published    goto dropoff
2026-07-26T19:02:42Z  michael        mote-01    published    goto pickup
2026-07-26T19:02:49Z  michael        mote-01    published    wibble
```

A bug this transcript found and closed: `dispatch` used to filter incoming
statuses against a correlation id it did not have yet — the robot's `dispatched`
and `accepted` can arrive before the HTTP response has been parsed, and they
were being discarded. It now collects every status and filters when it knows
what to filter for.

## 4. The whole loop, automated — **132 tests, 0 failures**

```console
$ pixi run -e dev test-fleet
132 passed in 41.79s

$ pixi run -e fleet test-fleet          # no ROS on a fleet box
110 passed, 3 skipped in 19.71s
```

New coverage, in the tiers `mote_fleet/README.md` describes:

- **contract** — the dispatch route's authorize → audit → publish order, its
  status codes, the refused-and-recorded case, a revoked token, a broker that is
  down (503, audit row `error`), the map metadata and both path-traversal
  attempts, and that the ES modules are served as JavaScript.
- **browser logic under node** (`ui_test.mjs`, run by `test_ui.py`, skipped
  where there is no node) — the MQTT packet codec against hand-built wire bytes,
  including a packet split across two WebSocket frames and two packets in one;
  and the Q5 world→pixel transform against a real floor's `map.yaml`, including
  that image y runs top-down and that the inverse round-trips.
- **end to end** (`test_e2e_fleet.py`) — a new case that dispatches through the
  real fleet server, with its own paho client, to the real agent and the real
  `mote_tasks` tree over a real broker: 401 with no token and no command on the
  wire, then 202, an audit row reading `published`, the correct Nav2 goal, and
  `dispatched → accepted → succeeded` carrying the id the API returned.

## 5. DDS participants — **unchanged**

M0 asks that `dds-check` be re-run whenever a milestone adds processes. M3 adds
none on the robot: everything here runs on the fleet box or in a browser. The
budget is still M1's ~23 of 33 with `foxglove_bridge` (M2) unclaimed.

## 6. Against the real robot — **confirmed, over the tailnet**

Run on `mote-01` (a Raspberry Pi on the tailnet, reached direct rather than via
a DERP relay) with the dashboard and the fleet server on the workstation. The
robot was running `pixi run robot` — bringup plus Nav2 — and `mote-agent.service`
under systemd, which is the first time that unit has been started by systemd
rather than by hand (an M1 gap, `m1-verification.md` §5).

**The broker swap is undisruptive, and the agent heals itself.** The M1 conda
broker was replaced in place by the container one on the same port, over the
same `$MOTE_FLEET_HOME`, while the robot was connected:

```
20:21:56  [mote_agent] disconnected from broker; paho will retry
20:22:27  [mote_agent] connected to broker as mote-01
```

Nothing on the robot was touched or restarted, and nothing about the mission
noticed: the agent is a bridge, not part of the control loop.

**A live health monitor's payload, at last** — the other M1 gap. Every health
state seen before this was `unknown` or a fixture; this is the real
`/diagnostics_agg` roll-up forwarded verbatim, and it is what the dashboard's
subsystem list renders:

```json
{"state":"ok","summary":"OK","subsystems":[
  {"name":"scan","state":"ok"},{"name":"scan_filtered","state":"ok"},
  {"name":"joint_states","state":"ok"},{"name":"camera","state":"ok"},
  {"name":"odometry","state":"ok"},{"name":"localization","state":"ok"},
  {"name":"system","state":"ok"},{"name":"self_check","state":"ok","message":"ready"}],
 "site":"home","floor":"ground","version":"b950358","uptime_s":21230.2,"battery":null}
```

**The basemap is a real robot's map, and the transform lands.** The `home/ground`
bundle rsynced off the robot serves as `234x166` px at `0.05 m/px` with origin
`[-5.98, -4.84]`; the reported pose `(0.432, 0.210)` puts the marker at pixel
`(128, 65)` — mid-room, where the robot was.

**Dispatch, twice, and the failure is the informative one.** The first attempt
went out to a robot whose *task layer was not running* — `pixi run tasks` is
deliberately not part of `pixi run robot`, so a nav mission has no `task_server`
unless one is started. The command reached the ROS graph and nothing answered:

```
20:34:35  [mote_agent] dispatching 'goto office' (id 407eb9b4393c4064)
20:34:56  [mote_agent] command 407eb9b4393c4064: no verdict from the task server within 20s
```

which is exactly the state machine's documented behaviour for that case
(`control-plane.md`) — observed on hardware for the first time, having only been
unit-tested. `ros2 topic info -v /task/command` confirmed the diagnosis: one
publisher (`mote_agent`), one subscriber, and it was the **bag recorder**.

With `pixi run tasks` started, the same command ran, and the dashboard's status
feed shows both attempts as the operator saw them:

```
19:42:32  succeeded  goto office                                              fleet
19:41:33  accepted   goto office                                              fleet
19:41:33  dispatched goto office                                              fleet
19:34:56  failed     goto office — no verdict from the task server within 20s  fleet
19:34:35  dispatched goto office                                              fleet
```

with both attempts in the audit log under the operator who sent them:

```
2026-07-26T19:34:35Z  michael  mote-01  goto office  407eb9b4393c4064  published
2026-07-26T19:41:33Z  michael  mote-01  goto office  7d679cf267f14058  published
```

So the whole write path — dashboard → fleet API (authorized, audited) → broker →
tailnet → agent → `/task/command` → behaviour tree → Nav2 → wheels — and the
whole read path back, are confirmed against hardware. The correlation id
survives every hop in both directions.

## 7. Not verified here

- **A browser on a different physical network.** §6 crossed the tailnet for the
  robot↔fleet-box hop, but the two were also on one LAN and the browser was on
  the fleet box itself. What remains is the last hop: browse to
  `http://<fleet-box>:8080/` from a tethered laptop and dispatch. The thing to
  watch there is the **WS listener's bind address** — the same lever as the MQTT
  listener, commented in `mosquitto.conf`.
- **The Foxglove deep link.** The button is rendered from the configured
  template and opens `foxglove://…`, which needs both the Foxglove desktop app
  on the operator's machine and a `foxglove_bridge` on the robot. Neither exists
  until M2, so nothing has been observed on the other end of it. `--foxglove-url
  ""` hides the button meanwhile.
- **More than two robots.** The roster, the map and the per-floor filter were
  exercised with two (scripted), and with one real robot. Marker clustering and
  basemap tiling, which `fleet.md` Q5 describes for large sites, are not built —
  at this fleet size they would be unmeasured complexity.
- **A robot dropping off while the dashboard watches.** The Last Will is tested
  against a real broker (`m1-verification.md` §2) and the UI renders `offline`
  from the same retained payload, but the two have not been observed together.
