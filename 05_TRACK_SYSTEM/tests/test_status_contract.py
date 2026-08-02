import pytest

from booster_motion import _parse_status_payload


def payload(age):
    return {
        "mode": "walk",
        "mode_age_s": age,
        "gait": "unknown",
        "last_rc": 0,
    }


def test_status_requires_fresh_source_mode_age():
    assert _parse_status_payload(payload(0.2))[0] == "walk"
    with pytest.raises(ValueError, match="mode_age"):
        _parse_status_payload(payload(1.3))
    missing = payload(0.2)
    missing.pop("mode_age_s")
    with pytest.raises(ValueError, match="mode_age"):
        _parse_status_payload(missing)
