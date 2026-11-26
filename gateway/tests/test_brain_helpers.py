"""Pure helpers of the world-model brain: signal tiers, spread, chains."""
from datetime import timezone

import pytest

import mosaic_brain as b


# --- robust_spread: one deep fade must not make a static device "moving" ---

def test_spread_small_sample_falls_back_to_raw():
    assert b.robust_spread([-60, -62, -70]) == 10


def test_spread_ignores_single_deep_fade():
    vals = [-60, -61, -62, -63, -64, -60, -61, -62, -63, -64, -106]
    spread = b.robust_spread(vals)
    assert spread <= b.STATIONARY_MAX_SPREAD
    assert b._movement_class(spread) == "STATIC"


def test_movement_classes():
    assert b._movement_class(b.STATIONARY_MAX_SPREAD) == "STATIC"
    assert b._movement_class(b.MOVING_MIN_SPREAD) == "MOVING"
    assert b._movement_class(20) == "AMBI"


# --- tiers / zones share boundaries ---

@pytest.mark.parametrize("rssi,tier,zone", [
    (-30, "STRONG", "Z1 apartment"),
    (-70, "STRONG", "Z1 apartment"),
    (-71, "MID", "Z2 hallway"),
    (-85, "MID", "Z2 hallway"),
    (-86, "EDGE", "Z3 deep bleed"),
])
def test_tier_and_zone(rssi, tier, zone):
    assert b.tier_of(rssi) == tier
    assert b.zone_of(rssi) == zone


# --- timestamps: legacy naive rows and new aware rows must compare ---

def test_parse_ts_normalizes_to_aware_utc():
    naive = b._parse_ts("2026-08-10T22:37:33")
    aware = b._parse_ts("2026-08-10T22:37:33+00:00")
    zulu = b._parse_ts("2026-08-10T22:37:33Z")
    assert naive == aware == zulu
    assert naive.tzinfo == timezone.utc


@pytest.mark.parametrize("bad", [None, "", "not-a-date"])
def test_parse_ts_bad_input(bad):
    assert b._parse_ts(bad) is None


# --- collapse_chains: rotating MACs bind into one entity ---

def rot(a, c, weight=0.9, avg=-60, ca=None, cb=None):
    return {"kind": "ROTATION?", "from": a, "to": c, "weight": weight,
            "from_tier": "STRONG", "to_tier": "STRONG",
            "from_avg": avg, "to_avg": avg,
            "from_company": ca, "to_company": cb}


def test_transitive_rotation_is_one_chain():
    rows = [rot("A", "B"), rot("B", "C"), rot("C", "D")]
    chains = b.collapse_chains(rows)
    assert chains == [["A", "B", "C", "D"]]


def test_weak_or_different_company_binds_are_rejected():
    rows = [
        rot("A", "B", weight=0.5),               # below min_weight
        rot("C", "D", ca=76, cb=6),              # two known, different companies
        {**rot("E", "F"), "to_tier": "MID"},     # not STRONG on both sides
    ]
    assert b.collapse_chains(rows) == []


def test_stream_continuation_binds_sequential_macs():
    macs = {
        "A": {"mac": "A", "first_ts": "2026-08-10T10:00:00+00:00",
              "last_ts": "2026-08-10T10:14:00+00:00", "avg_rssi": -60,
              "company_id": 76, "device_class": "unknown"},
        "B": {"mac": "B", "first_ts": "2026-08-10T10:15:00+00:00",
              "last_ts": "2026-08-10T10:29:00+00:00", "avg_rssi": -61,
              "company_id": 76, "device_class": "unknown"},
        # far away in signal level — a different device
        "X": {"mac": "X", "first_ts": "2026-08-10T10:15:30+00:00",
              "last_ts": "2026-08-10T10:20:00+00:00", "avg_rssi": -80,
              "company_id": 76, "device_class": "unknown"},
    }
    assert b.collapse_chains([], macs=macs) == [["A", "B"]]


@pytest.mark.parametrize("n", range(11, 40))
def test_spread_trims_one_deep_fade(n):
    # n=10 is excluded on purpose: nearest-rank p10 of 10 samples IS the minimum.
    vals = [-60 - (i % 5) for i in range(n - 1)] + [-106]
    assert b.robust_spread(vals) <= 4


def test_mcp_and_brain_spread_agree():
    import mosaic_mcp
    for n in range(1, 60):
        vals = [-50 - (i * 7) % 23 for i in range(n)]
        assert mosaic_mcp.robust_spread(vals) == b.robust_spread(vals)
