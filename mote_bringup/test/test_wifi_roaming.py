"""The wifi roaming config says what the installer and the README claim it says.

Every failure mode here is silent. `roamoff=1` leaves nothing on this card
roaming at all -- neither the firmware nor any userspace backend -- and the
robot goes on associating happily to one access point until it is carried out of
range. Worse, a `roamoff=0` in a file modprobe reads *before* the vendor's is
overridden without a word, which is how the first version of this config came to
be wrong. And the installer copies files by name: rename one and the copy fails
on a robot, not here.

None of this proves the robot roams. That is a walk with `wifi-roamlog`, and
`mote_bringup/wifi/README.md` says how.
"""

import configparser
import re
import tomllib
from pathlib import Path

import pytest

WIFI = Path(__file__).resolve().parents[1] / "wifi"
REPO = Path(__file__).resolve().parents[2]
INSTALL = WIFI / "install.sh"
CHECK = WIFI / "check.sh"
ROAMLOG = WIFI / "roamlog.sh"
README = WIFI / "README.md"

# The file whose `options brcmfmac` line ours has to be read after.
VENDOR_CONF = "rpi-brcmfmac.conf"


@pytest.fixture(scope="module")
def install_sh():
    return INSTALL.read_text()


def ini(name):
    """Read a drop-in. NM config is INI; modprobe.d is not."""
    parser = configparser.ConfigParser()
    parser.read(WIFI / name)
    return parser


def shell_var(text, name):
    match = re.search(rf"^{name}=(\S+)", text, re.M)
    assert match, f"install.sh sets no {name}"
    return match.group(1)


# --- the files the installer names must exist -------------------------------


@pytest.mark.parametrize("name", ["wifi-powersave.conf", "brcmfmac-roam.conf"])
def test_installer_sources_exist(install_sh, name):
    assert name in install_sh, f"{name} is not installed by install.sh"
    assert (WIFI / name).is_file()


def test_pixi_tasks_point_at_files_that_exist():
    tasks = tomllib.loads((REPO / "pixi.toml").read_text())["tasks"]
    referenced = set()
    for task in tasks.values():
        command = task if isinstance(task, str) else task.get("cmd", "")
        referenced.update(re.findall(r"mote_bringup/wifi/[\w.-]+", str(command)))

    assert referenced, "no pixi task runs anything in mote_bringup/wifi"
    for path in sorted(referenced):
        assert (REPO / path).is_file(), f"pixi.toml runs {path}, which does not exist"


# --- the settings that fail silently ----------------------------------------


def test_powersave_is_disabled():
    assert ini("wifi-powersave.conf")["connection-wifi"]["wifi.powersave"] == "2"


def test_firmware_roaming_is_enabled():
    options = (WIFI / "brcmfmac-roam.conf").read_text()
    assert re.search(r"^options\s+brcmfmac\b.*\broamoff=0\b", options, re.M), (
        "roamoff must be 0, or nothing on this card takes the roam decision"
    )


def test_the_installed_name_sorts_after_the_vendors(install_sh):
    # modprobe concatenates every `options` line, sorted by base name across all
    # of its directories, and the kernel takes the last value for a duplicated
    # parameter. So a file named `99-...` loses to the vendor's `rpi-...` no
    # matter which directory it is in -- which is not what "/etc wins" suggests,
    # and is how the first version of this shipped a setting that did nothing.
    dest = shell_var(install_sh, "MODPROBE_DEST")
    assert Path(dest).name > VENDOR_CONF, (
        f"{dest} sorts before {VENDOR_CONF}, so the vendor's roamoff wins"
    )


def test_the_installer_removes_the_name_that_sorted_before(install_sh):
    # Left in place it says roamoff=1 in a file `modprobe -c` still prints,
    # which reads like the effective setting and is not.
    stale = shell_var(install_sh, "STALE_MODPROBE")
    assert Path(stale).name < VENDOR_CONF, "the stale name is not one that loses"
    assert 'rm -f "$STALE_MODPROBE"' in install_sh


def test_the_installer_reports_what_modprobe_will_actually_do(install_sh):
    # The ordering rule above is checkable on the robot in one command, so the
    # installer checks it rather than trusting the file name it just wrote.
    assert "modprobe -c" in install_sh
    assert "tail -1" in install_sh


def test_no_iwd_configuration_is_left_behind(install_sh):
    # iwd cannot roam on this card (README.md), so nothing here may install it.
    for name in ("iwd-main.conf", "wifi-backend-iwd.conf"):
        assert not (WIFI / name).exists(), f"{name} is dead configuration"
    assert "wifi.backend=iwd" not in install_sh


# --- the guard is the reason the revert is safe to run over ssh -------------


def test_the_revert_rolls_itself_back(install_sh):
    revert = install_sh.split("guarded_revert()", 1)[1]
    assert "enable --now wpa_supplicant" in revert, (
        "the revert must restore wpa_supplicant"
    )
    assert "enable --now iwd" in revert, "the rollback must put iwd back"
    assert "FAILED:" in revert and "OK:" in revert, "the guard must record a verdict"


def test_the_rollbacks_own_inputs_survive_until_it_is_not_needed(install_sh):
    # Deleting iwd's drop-in, its config and its copy of the wifi key is what
    # makes the revert clean -- but a rollback needs all three, so none of it can
    # go until the guard has seen the network come back.
    revert = install_sh.split("guarded_revert()", 1)[1]
    cleanup = revert.index("rm -f /var/lib/iwd/*.psk")
    verdict = revert.index('log "OK:')
    assert verdict < cleanup, "the key copy is removed before the guard has a verdict"


def test_the_revert_outlives_the_ssh_session_that_starts_it(install_sh):
    # NetworkManager's restart drops the link the operator is connected over.
    assert "setsid" in install_sh and "nohup" in install_sh


def test_the_log_follow_skips_earlier_runs(install_sh):
    # The log accumulates runs. Following from line 1 replays the previous
    # run's verdict and reports a rolled-back revert as this run's result.
    assert "LOG_START" in install_sh
    assert "tail -f -n +1" not in install_sh


def test_a_robot_that_never_had_iwd_is_not_restarted(install_sh):
    # `pixi run setup` runs this on a freshly provisioned Pi, where the only
    # change is one modprobe file. Restarting NetworkManager there would drop
    # the link for nothing.
    assert "has_iwd_backend" in install_sh
    assert "if ! has_iwd_backend; then" in install_sh


# --- the check and the log --------------------------------------------------


def test_the_check_reads_the_fact_that_settles_it():
    # brcmfmac advertises NL80211_ATTR_ROAM_SUPPORT only with roamoff=0, and
    # `iw phy` prints it without root -- unlike /sys/module, which is 0400. A
    # check that can only answer as root is a check nobody runs.
    check = CHECK.read_text()
    assert "Device supports roaming." in check
    assert "sudo -n" in check, "the live module parameter must not be required"


def test_the_log_records_what_else_was_in_range():
    # A walk that logs no roam is only evidence if it also says whether there
    # was anything to roam to. The firmware scans without telling the host, so
    # this has to scan for itself.
    roamlog = ROAMLOG.read_text()
    header = re.search(r'^echo "(time,.*)" > "\$OUT"', roamlog, re.M)
    assert header, "roamlog.sh writes no CSV header"
    assert "best_other_bssid" in header.group(1)
    assert "best_other_dbm" in header.group(1)
    # And which ticks paid for it: a scan sweeps both bands and costs about 4 s
    # of 90-114 ms round trips, so unmarked it would read as the link degrading
    # -- in the stretch of the walk where that is exactly the question.
    assert "scanned" in header.group(1)
    # And the address, because a roam that changes it breaks everything above IP
    # while the link reads perfectly: the 2026-09-01 walk lost 54 s at -35 dBm
    # that way, and needed the DHCP journal to explain it.
    assert "ipv4" in header.group(1)


def test_the_log_only_scans_when_a_roam_is_due():
    # Scanning costs off-channel time on the link being measured. Doing it only
    # below the threshold keeps the strong-signal stretches undisturbed.
    roamlog = ROAMLOG.read_text()
    assert "SCAN_BELOW" in roamlog and "SCAN_EVERY" in roamlog
    assert re.search(r"\$\{sig%\.\*\}\" -le \"\$SCAN_BELOW", roamlog), (
        "the scan must be gated on the current signal"
    )


def test_the_log_scans_through_whichever_daemon_owns_the_radio():
    # `iw scan` needs CAP_NET_ADMIN and the walk is run by the login user, so
    # the scan goes through a daemon -- and it has to be the right one. Measured
    # on mote-01: `nmcli ... --rescan yes` under the iwd backend leaves the
    # kernel BSS cache at one entry, so a walk on a robot not yet reverted would
    # log an empty candidate column and prove nothing.
    roamlog = ROAMLOG.read_text()
    trigger = roamlog.split("trigger_scan() {", 1)[1].split("\n}", 1)[0]
    assert "iwctl station" in trigger
    assert "nmcli dev wifi list --rescan yes" in trigger
    assert 'iw dev "$IFACE" scan' not in roamlog.replace("scan dump", "")


# --- the README carries the numbers ----------------------------------------


def test_readme_quotes_the_firmware_thresholds():
    # -75 dBm and 20 dB are brcmfmac's WL_ROAM_TRIGGER_LEVEL and WL_ROAM_DELTA.
    # They are the thresholds in force and nothing here can change them, which
    # is the fact a reader most needs and cannot get from any config file.
    readme = README.read_text()
    assert "-75" in readme and "20 dB" in readme
