# Configuration

`tracker.example.json` is the shareable schema-1 example. It is intentionally
non-runnable for motion:

- `live_motion_enabled` is `false`;
- both robot entries are disabled;
- addresses are from the IANA documentation range; and
- tag IDs, calibration, geometry, and waypoints are examples only.

The strict loader rejects unknown or missing fields, non-finite numbers,
unsupported variants, duplicate robot names/tags, unsafe caps, invalid arena
geometry, and waypoints outside the safe interior
([pc/tracker_config.py](../pc/tracker_config.py#L279)).

## Offline preparation

From the package root:

```powershell
Copy-Item config\tracker.example.json config\tracker.local.json
py -3.10 pc\arena_runner.py --config config\tracker.local.json
```

The local filename is ignored by Git. Keep motion disabled while replacing the
example values. The command above omits `--live`, so it performs validation
only and opens no serial or network connection.

## Schema summary

| Section | Required values |
|---|---|
| Root | `schema_version`, `live_motion_enabled` |
| `uwb` | port, baud, `allow_unverified_lo`, ordered anchors, height correction, freshness, minimum anchors, residual limit |
| `arena` | bounds, inward safe margin, circular static keepouts |
| `navigation` | control rate, dwell, arrival radius, speed/turn caps, head-scan yaw and interval |
| `robots[]` | name, enabled flag, variant, panel hosts/port, local bearer-token file, tag, radius, heading freshness/sign/offset, tag-to-center offset, fixed waypoints |

See [CALIBRATION.md](../CALIBRATION.md) before assigning measured values.

## Live-gate facts

The code contains a live integration path, but this configuration guide does
not authorize it. The live gate requires a true local enable, at least one
enabled robot, an exact acknowledgement phrase, an explicit UWB serial port,
site-local RFC1918 IPv4 panel addresses, and readable local bearer-token files
([pc/arena_runner.py](../pc/arena_runner.py#L432)). Those software conditions
are safeguards, not permission or proof that the robot is ready.

`auth_token_file` is resolved relative to the tracker configuration unless it
is absolute. Put the real secret in an ignored local file, not in JSON. Both
ends require a non-symlink regular file no larger than 4096 bytes containing
exactly one ASCII token (an optional final line ending is allowed) matching
`[A-Za-z0-9._~+/=-]{32,512}`. Provision the identical value through an approved
process and restrict its file permissions.

The robot HTTP bridge verifies a bearer token but uses plaintext HTTP, not TLS.
Keep it on an isolated, access-controlled robot network; anyone who can observe
that traffic may be able to recover the token. Do not put passwords, tokens,
SSH keys, captures, logs, or other credentials in committed files.
