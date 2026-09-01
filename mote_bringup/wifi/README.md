# Wifi on the robot

Two things live here: the power-save drop-in that keeps SDIO from stalling an
ssh session, and the roaming configuration that lets the robot change access
point as it drives between rooms.

```bash
pixi run wifi-check       # what takes the roam decision right now (read-only)
pixi run wifi-powersave   # NetworkManager power-save drop-in
pixi run wifi-roaming     # let the firmware roam; reverts an earlier iwd switch
pixi run wifi-roamlog     # log BSSID/signal/loss during a walk
pixi run setup            # all of the above, plus udev and the systemd units
```

## The problem: nothing was taking a roam decision

The site this was measured at has an access point in nearly every room, all on
one SSID, because signal is poor throughout. The robot would associate once and
never move: measured on 2026-07-29 at about -80 dBm on a 5 GHz access point
while a same-SSID 2.4 GHz one 25 dB stronger was in view. Carried between rooms it dropped off the network rather than roaming,
which is what truncated mapping sessions and stranded teleop.

Raspberry Pi OS ships `options brcmfmac roamoff=1 feature_disable=0x282000` in
`/usr/lib/modprobe.d/rpi-brcmfmac.conf` — a vendor path, easily missed when
looking only at `/etc/modprobe.d` and the kernel command line. So the Broadcom
firmware's roaming engine was already off, and had been since the Pi was
imaged. Every observation of "sticky firmware roaming" on this robot was an
observation of no roaming at all.

## Userspace cannot take the decision over on this card

That looks like an opening — take the decision in userspace, where thresholds
are configurable — and it is not. This robot's wifi is a BCM4345/6 on
`brcmfmac`, a **fullmac** driver, and both backends decline the job for
reasons that are in their source rather than in their configuration.

**iwd** (3.8, the version Raspberry Pi OS carries) refuses twice over.
`netdev_cqm_rssi_update()` in `src/netdev.c` returns before it sends
`NL80211_CMD_SET_CQM`:

```c
	/*
	 * Fullmac cards handle roaming in firmware, there is no need to set
	 * CQM thresholds
	 */
	if (nhs->type == CONNECTION_TYPE_FULLMAC)
		return 0;
```

so no RSSI threshold is ever armed, so `station_low_rssi()` is never called and
no roam scan is ever started. The connection *is* fullmac by iwd's own test
(`netdev_handshake_state_setup_connection_type`): a WPA2-PSK connection is
softmac only if the wiphy supports the `authenticate`/`associate` commands, and
offloaded only if it advertises `4WAY_HANDSHAKE_STA_PSK`. `iw phy phy0 info` on
this robot lists `connect`/`disconnect` and neither of the others, and its whole
extended-feature set is `CQM_RSSI_LIST` and `DFS_OFFLOAD`. And the polling
fallback does not cover it: `netdev_rssi_polling_update()` returns immediately
when the wiphy advertises `CQM_RSSI_LIST`, which this one does. Turn firmware
roaming back on and iwd stands aside on purpose, in `station_cannot_roam()`:

```c
	/*
	 * Disable roaming with hardware that can roam automatically. Note this
	 * is now required for recent kernels which have CQM event support on
	 * this type of hardware (e.g. brcmfmac).
	 */
	if (wiphy_supports_firmware_roam(station->wiphy))
		return true;
```

There is no configuration between those two branches. `RoamThreshold` and
`RoamThreshold5G` are read and never reach a decision on this card.

**wpa_supplicant** scans while associated only under a per-network `bgscan=`,
and NetworkManager exposes no property for it — there is no supported way to
set it on an NM-managed interface. That leaves NetworkManager's own periodic
scan, measured on the robot, connected and passing traffic, by sampling the age
of the kernel's scan cache every 20 s:

| elapsed | associated signal | age of newest scan result | BSS in cache |
|--------:|------------------:|--------------------------:|-------------:|
|    0 s  |            -48 dBm |                  16.8 s   |            5 |
|   60 s  |            -49 dBm |                  76.8 s   |            1 |
|  180 s  |            -48 dBm |                 196.9 s   |            1 |
|  300 s  |            -48 dBm |                 296.9 s   |            1 |
|  320 s  |            -48 dBm |                  13.3 s   |            3 |
|  420 s  |            -48 dBm |                 113.4 s   |            1 |

Thirty samples over ten continuous minutes contain exactly one scan, about
304 s after the previous one. A roam decision can be taken at best once every
five minutes, which is thirty times too slow for a robot that crosses a room in
ten seconds, and for most of that window the kernel has expired every BSS but
the associated one, so there is nothing to roam *to*.

The iwd switch was measured, on 2026-09-01, and it made this strictly worse. A
2 min 19 s walk logged one BSSID throughout, 70 s at or below -85 dBm with the
tx bitrate collapsed from 433 to 13 Mbps, and `visible_same_ssid = 1` on all 133
rows. Before the walk the kernel's scan cache held exactly one BSS, last updated
at the boot six days earlier: with iwd in charge, NetworkManager's five-minute
scan stops too, and nothing scans at all. It also takes the diagnostics with it
— `nmcli dev wifi list --rescan yes` under iwd triggers no scan and answers from
iwd's own network list, which reports invented BSSIDs (`00:01:02:00:00:0a`) and
one frequency for every AP.

## The fix: give the decision back to the firmware

`options brcmfmac roamoff=0` — the driver's own default, which the vendor file
overrides. With it the firmware roams, and both backends recognise that and
leave it alone.

The thresholds come from `brcmfmac/cfg80211.h` and are set at every connect:

| | value | source |
|---|---:|---|
| roam trigger | **-75 dBm** | `WL_ROAM_TRIGGER_LEVEL` |
| roam delta — how much better a candidate must be | **20 dB** | `WL_ROAM_DELTA` |
| beacon timeout | 2 s (4 s with roaming off) | `BRCMF_DEFAULT_BCN_TIMEOUT_ROAM_ON` |

**None of them is tunable.** They are compiled into the driver and reach the
firmware over `BRCMF_C_SET_ROAM_TRIGGER`/`BRCMF_C_SET_ROAM_DELTA`, which no
userspace interface exposes. That is the cost of this route, and it is worth
stating plainly: if -75 dBm is the wrong trigger for this flat, the answer is
not a config file. It is the fallback at the bottom of this page.

Against the failed walk the trigger is at least in the right place — the link
sat below -85 dBm for 70 s, so a -75 dBm trigger fires with 10 dB to spare — and
the firmware scans on its own schedule inside the chip, so the "defer scanning
under continuous traffic" problem that a host bgscan has is the firmware's to
solve rather than the host's.

### The file name is load-bearing

`modprobe` concatenates the `options` lines from every config file, **sorted by
base name across all of its directories**, and the kernel applies duplicate
parameters left to right, so the last one wins. Priority between
`/etc/modprobe.d` and `/usr/lib/modprobe.d` applies to files of the *same* name,
not to the merged option list. On mote-01, before this change:

```
$ modprobe -c | grep brcmfmac
options brcmfmac roamoff=1                          # /etc/modprobe.d/99-mote-brcmfmac.conf
options brcmfmac roamoff=1 feature_disable=0x282000 # /usr/lib/modprobe.d/rpi-brcmfmac.conf
```

The `/etc` file is read **first** and then overridden — the opposite of what
"`/etc` wins" suggests, and invisible while both files agree. So the file
installs as `zz-mote-brcmfmac.conf`, which sorts after `rpi-brcmfmac.conf`, and
`install.sh` reads `modprobe -c` back and warns if its line is not the last one.
It names only `roamoff`, so the vendor's `feature_disable` carries through.

| file | installed to | does |
|---|---|---|
| `wifi-powersave.conf` | `/etc/NetworkManager/conf.d/` | `wifi.powersave=2` — power save stalls SDIO and hangs ssh |
| `brcmfmac-roam.conf` | `/etc/modprobe.d/zz-mote-brcmfmac.conf` | `roamoff=0`, after the vendor's line |

## Installing

```bash
pixi run wifi-roaming
sudo reboot                 # module parameters are read when the module loads
pixi run wifi-check
```

On a freshly provisioned Pi that is the whole of it: one file, no service
touched, nothing restarted. On a robot still carrying the iwd backend an earlier
version of this branch installed, the same command also hands the backend back
to wpa_supplicant, which restarts NetworkManager. Wifi is the robot's only link
— ethernet is unplugged and tailscale rides the wifi — so that step runs
detached, outliving the ssh session that starts it, and guards itself: if the
robot is not back on the network within 120 s it puts iwd back and says so.
Reconnect and read the verdict:

```bash
sudo tail -20 /var/log/mote-wifi-install.log
```

`--guard-timeout N` changes the window; `--no-guard` runs it in the foreground
with no rollback, which is only sensible at the robot's own console. Once the
guard has seen the network come back it removes iwd's drop-in, its config and
the copy of the wifi key that was made for its store — not before, because a
rollback needs all three.

## Verifying

`pixi run wifi-check` answers the one question that settles it: does
`iw phy phy0 info` say `Device supports roaming.`? brcmfmac sets
`WIPHY_FLAG_SUPPORTS_FW_ROAM` only when `roamoff` is 0, so that line is present
exactly when something is taking the decision. It reads without root, unlike
`/sys/module/brcmfmac/parameters/roamoff` (mode 0400), which the check also
reports under `sudo` — as a reboot-pending indicator, since the two disagree
between installing and rebooting.

The acceptance is a walk, and `pixi run wifi-roamlog` makes it a file rather
than an impression. It writes one row a second to
`~/.mote/wifi/roam-<stamp>.csv` — BSSID, frequency, signal, tx bitrate, RTT to
the default gateway, and the best *other* same-SSID AP in view — and prints each
change of BSSID as it happens, so the person carrying the robot hears the roam.

```bash
ssh mote-01 tmux new -s walk
pixi run wifi-roamlog            # then carry the robot room to room and back
```

That last column is what makes a walk with no roam in it worth anything. The
firmware scans inside the chip and tells the host nothing, so without it a log
that stays on one AP cannot distinguish "there was nothing better" from "there
was, and it would not move". So the logger scans for itself, through whichever
daemon owns the radio — neither needs root, and it has to be the right one:
under the iwd backend `nmcli dev wifi list --rescan yes` leaves the kernel's BSS
cache at one entry, because NetworkManager answers it out of iwd's network list
instead of scanning.

A scan is not free, and it sweeps both bands: measured on mote-01, about 4 s
over which the round trip goes from 3 ms to 90-114 ms and the tx bitrate from
433 to 24 Mbps. Narrowing it to the channels the network is on would make it cheap,
which is what the design wanted — but `iw dev wlan0 scan freq ...` needs root
and neither `nmcli` nor `iwctl` takes a frequency list, so a walk cannot have
it. Instead the scan runs only every 15 s and only while the link is at or below
-70 dBm, so the strong stretches are measured undisturbed and the off-channel
time lands where a roam was due anyway, and the `scanned` column marks the tick
that paid for it so the rows after it are attributable rather than a mystery.
`--scan-every 0` turns scanning off, `--scan-below` moves the gate.

The `ipv4` column is there for the same reason and was added because the first
successful walk needed the DHCP journal to explain itself: a roam that changes
the robot's address is a roam that breaks everything above IP while the link
reads perfectly, so the address is logged and a change is called out on stderr
like a roam.

Do the walk twice: once idle, once under load, since roaming under traffic is
what failed before. What the log should show is the BSSID changing within a few
seconds of the signal crossing -75 dBm, one address throughout, and no run of
empty `ping_ms` longer than a second or two.

### The loaded walk

The load stands in for a camera stream rather than being one. Foxglove plus the
camera is the real thing, but it produces no number — you cannot tell from it
whether the stream slowed or stalled, and it cannot be repeated at the same rate
twice. `iperf3` can, and `roamlog`'s `rx_kbps`/`tx_kbps` measure whatever it
sends. It is a test dependency, not the robot's: install it by hand rather than
adding it to `pixi.toml`, which would put it on every robot.

```bash
sudo apt install -y iperf3      # on the robot and on the machine it streams to

iperf3 -s                       # on that machine

# on the robot, in one tmux pane, against that machine's LAN address:
while :; do iperf3 -c <host> -t 120 -i 1 -b 8M; sleep 2; done

# and in another:
pixi run wifi-roamlog
```

`-b 8M` rate-limits the stream instead of letting it take the whole link. That
is the point: an unlimited `iperf3` measures capacity, which is a different
experiment and a misleading one here, because saturating the radio changes the
roaming behaviour being measured. 8 Mbit/s of TCP is a generous stand-in for a
compressed camera stream, and TCP is the right transport because the real stream
is TCP too — it backs off and retransmits, so a stall shows up as `tx_kbps`
going to nothing while the connection stays open.

The retry loop is there because `iperf3` exits when its connection dies. A roam
should not kill it — the address no longer changes, so the connection stalls and
retransmits rather than resetting — and if it does die, the loop keeps the load
running for the rest of the walk instead of leaving it silently idle.

## What the walk measured

Applied to mote-01 and walked on 2026-09-01, idle, room to room and back over
2 min 1 s.

**The firmware roams, at the threshold it says it does.** Four roams across
three access points in 121 s, every one of them fired between -75 and -83 dBm,
each landing on an access point 25 to 35 dB stronger than the one it left. A
roam costs about 3 s of round trips and an ssh session over the tailnet survives
it. That is the acceptance, and -75 dBm is a usable trigger for a building with
an access point per room.

### One SSID can span two subnets, and then a roam moves the robot

Not every access point answering to one SSID is necessarily on one network. At
the site this was measured at, one of them ran its own DHCP server and its own
NAT on a second subnet. It carried the same SSID and the same key — the
four-way handshake completed, `PTK=CCMP GTK=CCMP` — so the roam onto it
succeeded at every layer that wifi is responsible for, and then:

```
14:47:09  dhcp4 (wlan0): new lease, address=<second subnet>
14:48:04  dhcp4 (wlan0): new lease, address=<the robot's usual address>
```

The robot spent 54 s of the walk on that access point at **-35 dBm** and
72 Mbps with every ping to its gateway lost — not because the link was bad but
because its address had changed under it. That is 40 of the walk's 41 lost
pings, and none of them are wifi's fault.

**What breaks is address-bound access, not everything.** Anything reaching the
robot at its LAN address is gone for as long as it is over there. The tailnet
survives: tailscaled saw the default route change, rebound, had DERP back in 1 s
and its endpoints resettled in **4 s**. But it resettles *worse* — `NetInfo`
flipped to `varies=true`, the symmetric-NAT case, because that access point's
NAT sits on top of the site gateway's, and that pushes traffic off a direct path
and onto a relay. For a camera stream and teleop that is a degradation rather
than a blip.

This is worth knowing before deploying a robot into any building, because it
looks exactly like a roaming fault and is not one, and because roaming *working*
is what exposes it: a robot that never leaves its first access point never meets
the second subnet. The signature is in the log — the `ipv4` column changing
across a roam — and the fix is on the network, not the robot. Two steps, and the
first is easy to miss: check the offending access point's uplink is on the same
network as everything else before putting it in bridge or access-point mode,
since bridging it only ever joins it to whatever network it is already on.

`802-11-wireless.band=a` was considered as a robot-side workaround and rejected.
It would keep NetworkManager off whichever band such an access point happens to
be on, but it gives up that band's reach everywhere else — which is what having
an access point per room exists to provide — and it is unverified whether a
*firmware* roam honours the host's frequency list at all.

**Still to do**: the second walk, under load with Foxglove connected and the
camera streaming. Roaming under traffic is what failed before, and the idle walk
does not stand in for it.

## If -75 dBm turns out to be wrong

Nothing in this directory can move it, so the next step is not a threshold but a
mechanism: a small watchdog on the robot that reads the signal at 1 Hz and, below
a threshold of its own, tells NetworkManager to activate a better BSSID
(`nmcli device wifi connect <ssid> bssid <bssid>`). It costs a few seconds of
link where the firmware's roam costs a fraction of one, so it is worth building
only once a walk has shown the firmware will not move — and the log from that
walk, with its `best_other_dbm` column, is exactly the evidence for setting its
threshold.

The other direction, taking `wlan0` out of NetworkManager entirely and running
`wpa_supplicant` with `bgscan="simple:30:-65:300"`, gets host-driven roaming
with tunable thresholds and a proper scan cadence. It also means owning DHCP,
DNS and connectivity checking for the robot's only link, and fighting netplan,
which regenerates NM profiles. Not before the cheaper two have been measured.
