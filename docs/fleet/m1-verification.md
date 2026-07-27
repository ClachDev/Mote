# M1 verification ledger

What was measured for [M1](README.md), how, and what is still unverified. The
interface it verifies is [`control-plane.md`](control-plane.md).

## 1. Enroll → dispatch → status, over MQTT — **confirmed**

The milestone's acceptance test, run end to end with the shipped commands on a
workstation (broker, fleet server, `enroll`, `pixi run agent`, `fleetctl`). The
only stand-in is Nav2 itself: a mock `navigate_to_pose` action server plus the
real `mote_tasks` `TaskServer`, so the behaviour tree, the command grammar, the
zone lookup and the status strings are all the production ones.

Enrollment, from a robot with no identity at all:

```console
$ pixi run enroll -- --server http://127.0.0.1:8099 --token j_wWBxg4… --name Scout --site home
enrolled as mote-01 (new)
  identity: …/mote/robot.yaml
  fleet:    …/mote/fleet.yaml
  broker:   127.0.0.1:1883
next: start the agent with 'pixi run agent'

$ cat $MOTE_HOME/robot.yaml            $ cat $MOTE_HOME/fleet.yaml
schema: 1                              schema: 1
id: mote-01                            server: http://127.0.0.1:8099
name: Scout                            broker:
site: home                               host: 127.0.0.1
                                         port: 1883

$ pixi run fleetctl -- --server http://127.0.0.1:8099 robots
ID           NAME             SITE       ENROLLED              FINGERPRINT
mote-01      Scout            home       2026-07-26T16:12:46Z  machine_id:d25bff05e0f35ff718a875be6889edcc
```

The retained state, seen by an operator who connects *after* the robot did —
nothing was replayed for them, the broker simply had it:

```console
$ pixi run fleetctl -- watch
watching + on localhost:1883 (ctrl-c to stop)
mote-01    presence     online
mote-01    health       unknown: no diagnostics from the health monitor
```

(`unknown` is correct and is the point: no health monitor was running in this
walkthrough, and the agent says so rather than reporting a robot it cannot see
as healthy.)

A **fetch**, dispatched from a process sharing no ROS graph with the robot:

```console
$ pixi run fleetctl -- dispatch mote-01 fetch lab kitchen --wait 60
-> mote-01: fetch lab kitchen  (id 3e99cf44d1294ab5)
2026-07-26T16:15:35.961Z  dispatched
2026-07-26T16:15:35.963Z  accepted
2026-07-26T16:15:38.305Z  succeeded
$ echo $?
0
```

and the robot really ran the mission — drive to the object zone, pick stub, drive
to the drop zone, place stub:

```console
[mock_nav] nav goal -> (4.00, -2.00)     # lab
[mock_nav] nav goal -> (-1.50, 0.50)     # kitchen
```

A rejection carries its reason back, and the exit code mirrors the outcome:

```console
$ pixi run fleetctl -- dispatch mote-01 goto nowhere --wait 20
-> mote-01: goto nowhere  (id 88dc9cb623b34410)
2026-07-26T16:15:39.911Z  dispatched
2026-07-26T16:15:39.913Z  rejected  (unknown zone 'nowhere', have ['kitchen', 'lab'])
$ echo $?
1
```

The same loop is an automated test — `mote_fleet/test/test_e2e_fleet.py`, which
additionally asserts the exact status sequence, the correlation ids, the
single-in-flight rejection, and that a `fetch` produces two Nav2 goals in order.
It runs wherever a broker and ROS coexist:

```console
$ pixi run -e dev test-fleet
102 passed in 24.51s
```

## 2. The Last Will fires on a hard drop — **confirmed against a real broker**

The claim worth testing against mosquitto rather than against the agent's own
belief: kill the connection *without* a DISCONNECT (paho's loop stopped, socket
closed — what a power cut looks like to a broker) and the broker publishes the
will itself.

```
presence  online                     <- agent connected
presence  OFFLINE (last will)        <- broker published it, ~1 keepalive later
```

A subscriber connecting afterwards is told immediately, because the will is
retained (`test_a_dead_agent_is_reported_offline_by_the_broker`). With
`keepalive` at its 20 s default a robot that loses power is marked offline
within roughly 30 s; the test pins keepalive to 2 s to keep itself quick.

## 3. DDS participant cost of the agent — **1 slot, as projected**

M0 asked that `dds-check` be re-run whenever a milestone adds processes, having
measured 17/33 for the sim nav mission and projected ~24 with M1 and M2 added.
The agent is a single-node process and costs exactly one slot:

```console
$ pixi run dds-check                       # nothing running
0/33 participant slots used, 33 free (MaxAutoParticipantIndex=32)

$ pixi run dds-check                       # with mote-agent up
  index   0  port 26910  pid 3511525  python3.12 agent
1/33 participant slots used, 32 free (MaxAutoParticipantIndex=32)
```

So the projection holds: the robot's nav mission plus perception plus this agent
is ~23, leaving M2's `foxglove_bridge` inside the budget. Nothing here changes
the M2 decision.

## 4. conda-forge's mosquitto has **no websockets support** — a real M3 constraint

Found by the broker refusing to start:

```console
$ pixi run fleet-broker          # then the conda binary; since Ms, `--local`
1785082301: Error: Websockets support not available.
1785082301: Error found at …/mote_fleet/server/mosquitto.conf:27.
```

`mosquitto 2.0.20` from conda-forge is built without libwebsockets. That matters
because [`fleet.md` Q5](../design/fleet.md) has the M3 dashboard subscribing to
`mote/v1/+/health` **directly from the browser over MQTT-over-WebSockets** — no
polling, no service in the middle — and a browser cannot speak raw MQTT.

Nothing in M1 needs it, so the WS listener ships commented out in
`mosquitto.conf` with the reason inline, and M1 is unaffected. M3 has to pick
one of:

- a distro or container mosquitto built against libwebsockets (Debian's is, and
  the fleet server is a container in the design anyway);
- **EMQX**, which is the Regime-B broker the design already names — same
  protocol, same topic tree, same agent code, and its WS listener is standard;
- a small WS↔MQTT relay in the fleet API, which is the option that adds a
  service to the middle and should therefore lose.

Filed as a follow-up so M3 does not rediscover it.

Second, smaller gotcha from the same package: conda-forge puts the **broker** in
`$PREFIX/sbin` and only the clients (`mosquitto_pub`/`_sub`) in `bin`, and pixi
only puts `bin` on `PATH` — so a plain `mosquitto` is "command not found" in an
environment that definitely has it. `broker.sh` and the test's skip check both
look in `$CONDA_PREFIX/sbin` first.

## 5. Not verified here — needs the hardware

- **Off-LAN.** The acceptance criterion says "off-LAN, over MQTT". Everything
  above ran on one machine. What makes it off-LAN is the M0 tailnet, not this
  code — the same MQTT connection over a WireGuard interface is the same
  connection — but that final hop is unrun, exactly as
  [`m0-verification.md` §5](m0-verification.md) leaves the clean-Pi provisioning
  unrun. To close it: enroll the Pi against a fleet server on the tailnet
  (`--server http://<fleet-box>:8080`), `systemctl enable --now mote-agent`, and
  dispatch from a tethered laptop.
- **`mote-agent.service` under systemd.** The unit is installed by
  `pixi run setup` and modelled on `mote-health.service`, but it has not been
  started by systemd on the Pi — only by `pixi run agent` on the workstation.
- **Enrollment during cloud-init.** The M0 provisioning template still writes an
  operator-set identity; wiring `enroll` into first boot is a template change
  that should happen the next time a Pi is actually provisioned, so the change
  and its verification land together.
- **A real health monitor's payload.** The walkthrough had none running, so
  `state: unknown` is the only health state exercised end to end against a live
  broker. The mapping from `/diagnostics_agg` levels is unit-tested
  (`test_agent.py`), not observed on a robot.
