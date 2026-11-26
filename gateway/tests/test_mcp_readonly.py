"""The MCP `query` tool must never be able to modify the world model."""
import json
import sqlite3

import pytest

import mosaic_mcp


@pytest.fixture
def db(tmp_path):
    path = tmp_path / "orb.db"
    c = sqlite3.connect(path)
    c.execute("CREATE TABLE sightings (mac TEXT, rssi INTEGER)")
    c.executemany("INSERT INTO sightings VALUES (?, ?)",
                  [("aa:bb:cc:00:00:%02x" % i, -50 - i) for i in range(20)])
    c.commit()
    c.close()
    return str(path)


def count(path):
    c = sqlite3.connect(path)
    try:
        return c.execute("SELECT COUNT(*) FROM sightings").fetchone()[0]
    finally:
        c.close()


def test_select_works(db):
    out = json.loads(mosaic_mcp.run_readonly_query(
        "SELECT mac, rssi FROM sightings ORDER BY rssi DESC", limit=3, db=db))
    assert len(out) == 3
    assert out[0]["rssi"] == -50


def test_plain_delete_rejected(db):
    out = json.loads(mosaic_mcp.run_readonly_query("DELETE FROM sightings", db=db))
    assert "error" in out
    assert count(db) == 20


@pytest.mark.parametrize("sql", [
    # The bypass: passes the SELECT/WITH prefix check, but is a write.
    "WITH x AS (SELECT 1) DELETE FROM sightings",
    "WITH x AS (SELECT 1) UPDATE sightings SET rssi = 0",
    "WITH x AS (SELECT 1) INSERT INTO sightings VALUES ('evil', 0)",
    # Statement stacking is rejected by sqlite3.execute, but check anyway.
    "SELECT 1; DROP TABLE sightings",
])
def test_write_disguised_as_read_is_blocked(db, sql):
    out = json.loads(mosaic_mcp.run_readonly_query(sql, db=db))
    assert "error" in out
    assert count(db) == 20


def test_limit_is_clamped(db):
    out = json.loads(mosaic_mcp.run_readonly_query(
        "SELECT * FROM sightings", limit=10**9, db=db))
    assert len(out) == 20  # all rows, but the cap would apply on bigger DBs
    out = json.loads(mosaic_mcp.run_readonly_query(
        "SELECT * FROM sightings", limit=0, db=db))
    assert len(out) == 1


def test_missing_db_is_an_error_not_a_new_file(tmp_path):
    path = tmp_path / "nope.db"
    out = json.loads(mosaic_mcp.run_readonly_query("SELECT 1", db=str(path)))
    assert "error" in out
    assert not path.exists()  # read-only open must not create the DB
