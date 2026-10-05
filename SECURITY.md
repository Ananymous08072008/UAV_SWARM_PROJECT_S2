# Security policy

This is a simulation platform; it does not control real aircraft. The mission
studio (`python -m dashboard.server`) binds to `127.0.0.1` by default. Only use
`--host 0.0.0.0` on networks you trust, and never expose it to the internet
without authentication in front of it.

## Reporting a vulnerability

Please do not open a public issue. Use GitHub's private reporting:
**Security tab -> Report a vulnerability** on this repository.

## Secrets

Never commit credentials, tokens, `.env` files or keys. `.gitignore` already
excludes `.env*`, `*.pem` and `*.key`.
