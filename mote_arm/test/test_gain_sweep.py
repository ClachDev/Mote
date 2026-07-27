"""The gain sweep's trial loop against a simulated droopy servo.

The sweep writes EEPROM and moves a real arm, so the properties that keep it
safe to run are pinned here rather than discovered at the bench: it leaves the
servo's original gains and torque state behind whatever happens, it never
commands past a soft limit, and it refuses to produce trials that are not
comparable. The fake servo droops like the real one (settling short by
``torque/kp``), which also checks the loop actually applies each gain before
measuring under it.
"""

import argparse

import pytest
from mote_arm import arm_gains, config
from mote_arm.step_response import RAD_PER_COUNT

# kp * error, in rad, from the hardware measurement this task started from:
# kp=16 stopped 0.071 rad short, kp=32 stopped 0.033 rad short.
DROOP_TORQUE = 1.14


def make_config(**overrides):
    cfg = {
        "arm": {
            "port": "/dev/fake",
            "baud_rate": 1000000,
            "moving_speed": 500,
            "moving_acc": 20,
            "gains": {"kp": 32, "kd": 32, "ki": 0},
            "joints": [
                {
                    "name": "elbow_flex",
                    "id": 3,
                    "min": -3.291,
                    "max": 0.103,
                    "home": 2931,
                },
            ],
        }
    }
    cfg["arm"].update(overrides)
    return config.ArmConfig.from_dict(cfg)


class FakeServo:
    """A position servo that settles short of its goal by torque/kp."""

    def __init__(self, gains=(16, 32, 0), position=2931, temps=None):
        self.gains = gains
        self.position = position
        self.torque = False
        self.goals = []
        self.gain_writes = []
        self.temps = list(temps or [])
        self.temperature = 30
        self.readable = True

    # --- bus surface used by the sweep -------------------------------------
    def read_gains(self, _id):
        return self.gains if self.readable else None

    def write_gains(self, _id, kp, kd, ki):
        self.gain_writes.append((kp, kd, ki))
        self.gains = (kp, kd, ki)
        return True

    def read_position(self, _id):
        return self.position if self.readable else None

    def read_position_load(self, _id):
        return (self.position, 190) if self.readable else None

    def read_health(self, _id):
        if self.temps:
            self.temperature = self.temps.pop(0)
        return _Health(self.temperature)

    def set_torque(self, _id, enable):
        self.torque = enable

    def write_goal(self, _id, counts, _speed, _acc):
        self.goals.append(counts)
        kp = self.gains[0]
        target = counts
        droop_counts = (DROOP_TORQUE / kp) / RAD_PER_COUNT
        direction = 1 if target > self.position else -1
        # Settle short of the goal, in the direction it travelled.
        self.position = round(target - direction * droop_counts)


class _Health:
    def __init__(self, temperature):
        self.temperature = temperature
        self.voltage = 5.1


def sweep_args(**overrides):
    args = argparse.Namespace(
        joint="elbow_flex",
        step=-0.2,
        kp="16,32",
        ki="0",
        kd="",
        hold=0.05,
        rate=200.0,
        max_temp=55,
        out=None,
        yes=True,
    )
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


@pytest.fixture(autouse=True)
def no_sleeping(monkeypatch):
    monkeypatch.setattr(arm_gains.time, "sleep", lambda _s: None)


@pytest.fixture
def out_file(tmp_path):
    return str(tmp_path / "sweep.json")


def test_sweep_measures_error_falling_as_kp_rises(out_file, capsys):
    servo = FakeServo(gains=(16, 32, 0))
    arm_gains._cmd_sweep(make_config(), servo, sweep_args(out=out_file))

    printed = capsys.readouterr().out
    assert "proportional droop" in printed
    assert servo.gain_writes[:2] == [(16, 32, 0), (32, 32, 0)]


def test_sweep_restores_the_original_gains_and_leaves_the_joint_limp(out_file):
    servo = FakeServo(gains=(16, 32, 0))
    arm_gains._cmd_sweep(make_config(), servo, sweep_args(kp="16,32,64", out=out_file))

    assert servo.gains == (16, 32, 0)
    assert servo.torque is False


def test_overheating_stops_the_sweep_but_still_restores(out_file):
    servo = FakeServo(gains=(16, 32, 0), temps=[30, 70])
    with pytest.raises(SystemExit) as exc:
        arm_gains._cmd_sweep(
            make_config(), servo, sweep_args(kp="16,32,64,128", out=out_file)
        )

    assert "70C" in str(exc.value)
    assert servo.gains == (16, 32, 0)
    assert servo.torque is False


def test_a_step_that_clamps_against_a_soft_limit_is_refused():
    # Sitting at the top of its travel, a -0.2 step would clamp to almost
    # nothing and the trials would not be comparable.
    cfg = make_config(
        joints=[
            {"name": "elbow_flex", "id": 3, "min": -0.05, "max": 0.103, "home": 2931}
        ]
    )
    servo = FakeServo()
    with pytest.raises(SystemExit) as exc:
        arm_gains._cmd_sweep(cfg, servo, sweep_args())

    assert "soft limits" in str(exc.value)
    assert servo.goals == []


def test_a_silent_servo_is_reported_not_driven():
    servo = FakeServo()
    servo.readable = False
    with pytest.raises(SystemExit) as exc:
        arm_gains._cmd_sweep(make_config(), servo, sweep_args())

    assert "did not answer" in str(exc.value)
    assert servo.goals == []


def test_no_goal_ever_leaves_the_soft_limits(out_file):
    cfg = make_config()
    joint = cfg.joint("elbow_flex")
    servo = FakeServo(gains=(16, 32, 0))
    arm_gains._cmd_sweep(cfg, servo, sweep_args(kp="16,32,64,128", out=out_file))

    for counts in servo.goals:
        rad = joint.counts_to_rad(counts)
        assert joint.min_rad - 1e-9 <= rad <= joint.max_rad + 1e-9


def test_unknown_joint_names_the_real_ones():
    with pytest.raises(SystemExit) as exc:
        arm_gains._cmd_sweep(make_config(), FakeServo(), sweep_args(joint="elbow"))

    assert "elbow_flex" in str(exc.value)


def test_gain_values_outside_the_servo_range_are_rejected():
    with pytest.raises(SystemExit) as exc:
        arm_gains._cmd_sweep(make_config(), FakeServo(), sweep_args(kp="16,300"))

    assert "0-254" in str(exc.value)


def test_trace_is_written_for_every_trial(out_file):
    import json

    arm_gains._cmd_sweep(
        make_config(), FakeServo(gains=(16, 32, 0)), sweep_args(out=out_file)
    )
    data = json.loads(open(out_file).read())

    assert data["joint"] == "elbow_flex"
    assert [t["kp"] for t in data["trials"]] == [16, 32]
    assert all(t["trace"] for t in data["trials"])
    assert (
        data["trials"][0]["metrics"]["abs_error"]
        > (data["trials"][1]["metrics"]["abs_error"])
    )
