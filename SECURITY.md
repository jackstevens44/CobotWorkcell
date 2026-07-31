# Security policy

## Supported versions

Until the first public release, security fixes are made on the default branch.

## Reporting a vulnerability

Do not publish API keys, private workcell data, device identifiers, or a weaponizable robot-control vulnerability in a public issue.

Use GitHub's private vulnerability reporting feature when enabled. If it is unavailable, contact the repository owner through a private GitHub channel and include:

- affected commit or version;
- impact;
- reproducible steps using simulation or fake hardware;
- suggested mitigation;
- whether credentials may have been exposed.

Do not demonstrate a vulnerability by commanding a physical robot.

## Deployment warning

The development HTTP server has no authentication and is intended for loopback use. Keep it bound to `127.0.0.1`. Do not expose it to the internet or an untrusted network.
