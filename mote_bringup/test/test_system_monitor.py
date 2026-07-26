"""Throttle-flag decoding and fan discovery in system_monitor."""

from mote_bringup.system_monitor import (
    find_pwmfan,
    parse_throttled,
    read_fan_rpm,
    throttle_values,
    throttle_warning,
)


def _hwmon(root, index, name, **files):
    hwmon = root / f"hwmon{index}"
    hwmon.mkdir()
    hwmon.joinpath("name").write_text(f"{name}\n")
    for key, value in files.items():
        hwmon.joinpath(key).write_text(value)
    return hwmon


def test_parse_vcgencmd_output():
    assert parse_throttled("throttled=0xe0000\n") == 0xE0000
    assert parse_throttled("throttled=0x0") == 0


def test_observed_incident_flags_are_latched_only():
    values = throttle_values(0xE0000)
    assert values["throttled_flags"] == "0xe0000"
    assert values["freq_capped_ever"] == "True"
    assert values["throttled_ever"] == "True"
    assert values["soft_temp_limit_ever"] == "True"
    assert values["undervoltage_ever"] == "False"
    assert all(values[f"{k}_now"] == "False" for k in ("undervoltage", "throttled"))
    assert throttle_warning(0xE0000) is None


def test_soft_temperature_limit_alone_warns():
    assert throttle_warning(1 << 3) == "power: soft temperature limit"
    assert throttle_values(1 << 3)["soft_temp_limit_now"] == "True"


def test_undervoltage_and_throttled_keep_original_text():
    assert throttle_warning(0x5) == "power: under-voltage/throttled"


def test_clean_flags_have_no_warning():
    assert throttle_warning(0) is None


def test_find_pwmfan_by_name_not_index(tmp_path):
    _hwmon(tmp_path, 0, "cpu_thermal")
    _hwmon(tmp_path, 1, "rp1_adc")
    fan = _hwmon(tmp_path, 2, "pwmfan", fan1_input="3300\n")
    _hwmon(tmp_path, 3, "rpi_volt")
    assert find_pwmfan(tmp_path) == fan
    assert read_fan_rpm(fan) == 3300


def test_no_fan_fitted(tmp_path):
    _hwmon(tmp_path, 0, "cpu_thermal")
    assert find_pwmfan(tmp_path) is None


def test_missing_hwmon_root(tmp_path):
    assert find_pwmfan(tmp_path / "absent") is None


def test_fan_read_error_is_none(tmp_path):
    assert read_fan_rpm(_hwmon(tmp_path, 0, "pwmfan")) is None
