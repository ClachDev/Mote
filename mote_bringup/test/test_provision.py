"""The rendered cloud-init user-data has to be valid on a card that boots once,
unattended, with no one watching — so render it here and read it back."""

import argparse

import pytest
import yaml

from mote_bringup import provision

SECRETS = {
    # A wifi passphrase and an auth key are arbitrary bytes; sed/regex-style
    # substitution mangles &, \ and | in exactly these.
    "psk": r"p&ss\w|ord",
    "authkey": "tskey-auth-kM1&z\\x|Q",
}


@pytest.fixture
def args(tmp_path):
    key = tmp_path / "id_ed25519.pub"
    key.write_text("ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAI+fake operator@workstation\n")
    return argparse.Namespace(
        id="mote-02",
        name="Scout",
        site="hq",
        ssh_key=str(key),
        ts_authkey=SECRETS["authkey"],
        ts_authkey_file=None,
        wifi_ssid="Home Net",
        wifi_psk=SECRETS["psk"],
        wifi_country="GB",
        user="michael",
        repo="https://github.com/ClachDev/Mote.git",
        timezone="Europe/London",
    )


def test_renders_valid_cloud_config(args):
    text = provision.build(args)
    assert text.startswith("#cloud-config")
    data = yaml.safe_load(text)
    assert data["hostname"] == "mote-02"
    assert data["users"][0]["name"] == "michael"
    assert data["timezone"] == "Europe/London"


def test_identity_is_written_before_anything_uses_it(args):
    data = yaml.safe_load(provision.build(args))
    staged = next(
        f for f in data["write_files"] if f["path"] == "/var/lib/mote/robot.yaml"
    )
    assert yaml.safe_load(staged["content"]) == {
        "schema": 1,
        "id": "mote-02",
        "name": "Scout",
        "site": "hq",
    }
    commands = [c[-1] for c in data["runcmd"]]
    installs_identity = next(
        i for i, c in enumerate(commands) if "/.mote/robot.yaml" in c
    )
    joins_tailnet = next(i for i, c in enumerate(commands) if "tailscale up" in c)
    assert installs_identity < joins_tailnet


def test_the_environment_is_installed_from_the_lockfile(args):
    """`--locked` aborts if pixi.lock is out of date with pixi.toml rather than
    solving something new, so a robot provisions the dependency set that was
    tested. Without it, a card imaged months later quietly gets a different
    one."""
    commands = [c[-1] for c in yaml.safe_load(provision.build(args))["runcmd"]]
    install = next(i for i, c in enumerate(commands) if "pixi install" in c)
    build = next(i for i, c in enumerate(commands) if "pixi run build" in c)
    assert "--locked" in commands[install]
    assert install < build


def test_secrets_survive_substitution_verbatim(args):
    data = yaml.safe_load(provision.build(args))
    keyfile = next(
        f for f in data["write_files"] if f["path"].endswith("tailscale.authkey")
    )
    assert keyfile["content"] == SECRETS["authkey"]
    assert keyfile["permissions"] == "0600"
    wifi = next(f for f in data["write_files"] if f["path"].endswith(".nmconnection"))
    assert f"psk={SECRETS['psk']}" in wifi["content"]
    assert "ssid=Home Net" in wifi["content"]


def test_the_auth_key_is_destroyed_after_use(args):
    commands = [c[-1] for c in yaml.safe_load(provision.build(args))["runcmd"]]
    joins = next(i for i, c in enumerate(commands) if "tailscale up" in c)
    shreds = next(i for i, c in enumerate(commands) if "shred" in c)
    assert joins < shreds


def test_no_wifi_drops_the_whole_wifi_block(args):
    args.wifi_ssid = None
    args.wifi_psk = None
    text = provision.build(args)
    assert "nmconnection" not in text
    assert "nmcli" not in text
    yaml.safe_load(text)


def test_a_site_is_optional(args):
    args.site = None
    data = yaml.safe_load(provision.build(args))
    staged = next(
        f for f in data["write_files"] if f["path"] == "/var/lib/mote/robot.yaml"
    )
    assert "site" not in yaml.safe_load(staged["content"])


def test_bad_id_is_refused(args):
    args.id = "Mote 02"
    with pytest.raises(ValueError):
        provision.build(args)


def test_missing_auth_key_is_refused(args):
    args.ts_authkey = ""
    with pytest.raises(ValueError):
        provision.build(args)


def test_wifi_ssid_without_passphrase_is_refused(args):
    args.wifi_psk = None
    with pytest.raises(ValueError):
        provision.build(args)


@pytest.mark.parametrize("country", [None, "", "gb", "GBR", "1B"])
def test_wifi_without_a_valid_country_is_refused(args, country):
    """The WLAN radio stays rfkill-blocked until the regulatory domain is set, so
    a card rendered without one boots to a robot with no network."""
    args.wifi_country = country
    with pytest.raises(ValueError, match="wifi-country"):
        provision.build(args)


def test_the_country_is_set_before_the_connection_comes_up(args):
    commands = [c[-1] for c in yaml.safe_load(provision.build(args))["runcmd"]]
    wifi = next(c for c in commands if "nmcli connection up" in c)
    assert wifi.index("do_wifi_country GB") < wifi.index("nmcli connection up")


def test_no_wifi_needs_no_country(args):
    args.wifi_ssid = None
    args.wifi_psk = None
    args.wifi_country = None
    assert "do_wifi_country" not in provision.build(args)


def test_quoted_placeholders_really_sit_inside_quoted_scalars():
    """provision.QUOTED escapes those values for a double-quoted YAML scalar, so
    the template must actually place them in one — otherwise the escaping is
    wrong in the other direction."""
    template = provision.template_path().read_text()
    for name in provision.QUOTED:
        occurrences = [line for line in template.splitlines() if f"@{name}@" in line]
        assert occurrences, f"@{name}@ is escaped but never used"
        for line in occurrences:
            assert f'"@{name}@"' in line, line


def test_a_newline_in_a_secret_is_refused(args):
    args.wifi_psk = "line1\nline2"
    with pytest.raises(ValueError, match="control character"):
        provision.build(args)


def test_bad_user_is_refused(args):
    args.user = "Root User"
    with pytest.raises(ValueError):
        provision.build(args)


def test_unsubstituted_placeholder_is_caught():
    with pytest.raises(ValueError, match="placeholders"):
        provision.check("#cloud-config\nhostname: @ROBOT_ID@\n")
