# ESP32-Mosaic gateway — systemd user service (deploy)

Run the gateway as a **systemd user service** so it survives terminal sessions,
restarts itself on crash, and starts automatically at login. The gateway is the
brain of the mosaic — if it dies, the nodes' evidence goes nowhere. Treat it
like a service, not a shell background job.

## Install

```sh
cp orb-gateway.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now orb-gateway.service
```

Check it:

```sh
systemctl --user status orb-gateway.service
curl -s http://localhost:9000/status
```

Logs: `journalctl --user -u orb-gateway -f` (a copy is also appended to
`~/.orb/gateway.log`).

## Notes

- The unit uses `%h` (the user's home) so it works from any checkout location,
  e.g. `~/workspace/esp32-mosaic/gateway`. Adjust `WorkingDirectory` /
  `ExecStart` if your repo lives elsewhere.
- `Restart=always` + `RestartSec=5` means a crash is healed within seconds —
  no human in the loop.
- The gateway reads `config.yaml` from its working directory if present
  (`data_dir: ~/.orb` by default). See `../gateway/config.example.yaml`.
- Requires a user systemd bus with lingering enabled for the account
  (`loginctl enable-linger <user>`), otherwise the service stops at logout.

## Why

Previous failure mode: the gateway was launched from an interactive session and
silently died with it (a `mcp_stdio_watchdog` kept the MCP server alive, but
nothing watched the gateway). Result: ~2h of missed sightings before anyone
noticed. systemd user services are the standard, auditable answer.
