import copy
from dataclasses import replace
import ipaddress
import json
from pathlib import Path

import pytest

from arena_runner import LIVE_ACK, validate_live_gate
from tracker_config import ConfigError, load_config, parse_config


ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "config" / "tracker.example.json"


def data():
    return json.loads(EXAMPLE.read_text(encoding="utf-8"))


def test_example_is_valid_and_deliberately_non_runnable():
    config = load_config(EXAMPLE)
    assert config.live_motion_enabled is False
    assert config.enabled_robots == ()
    documentation_net = ipaddress.ip_network("192.0.2.0/24")
    assert all(
        ipaddress.ip_address(host) in documentation_net
        for robot in config.robots
        for host in robot.panel_hosts
    )


@pytest.mark.parametrize(
    "mutation,match",
    [
        (lambda value: value.update(schema_version=2), "schema_version"),
        (lambda value: value["robots"][0].update(imu_yaw_sign=0), "imu_yaw_sign"),
        (lambda value: value["navigation"].update(max_forward_mps=0.41), "max_forward"),
        (lambda value: value["robots"][0].update(panel_hosts=["not-an-ip"]), "IP address"),
        (lambda value: value["robots"][0].update(waypoints_m=[[0.1, 0.1]]), "safe interior"),
    ],
)
def test_unsafe_or_malformed_configuration_fails_closed(mutation, match):
    value = copy.deepcopy(data())
    mutation(value)
    with pytest.raises(ConfigError, match=match):
        parse_config(value)


def live_config(host, tmp_path):
    token_file = tmp_path / "control-token.local.txt"
    token_file.write_text("t" * 32 + "\n", encoding="utf-8")
    config = load_config(EXAMPLE)
    robot = replace(
        config.robots[0],
        enabled=True,
        panel_hosts=(host,),
        auth_token_file=token_file.name,
    )
    return replace(
        config,
        path=tmp_path / "tracker.local.json",
        live_motion_enabled=True,
        uwb_port="COM9",
        robots=(robot,),
    )


@pytest.mark.parametrize(
    "host",
    ["192.0.2.10", "198.51.100.10", "203.0.113.10", "127.0.0.1", "8.8.8.8", "::1"],
)
def test_live_gate_rejects_documentation_and_non_rfc1918_hosts(host, tmp_path):
    with pytest.raises(ConfigError, match="RFC1918"):
        validate_live_gate(live_config(host, tmp_path), LIVE_ACK)


@pytest.mark.parametrize("host", ["10.1.2.3", "172.16.2.3", "192.168.50.10"])
def test_live_gate_accepts_rfc1918_ipv4_after_all_other_gates(host, tmp_path):
    validate_live_gate(live_config(host, tmp_path), LIVE_ACK)


def test_live_gate_requires_exact_acknowledgement(tmp_path):
    with pytest.raises(ConfigError, match="acknowledge"):
        validate_live_gate(live_config("10.1.2.3", tmp_path), "almost")
