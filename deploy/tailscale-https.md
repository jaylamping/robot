# HTTPS for Navi with Tailscale

Navi serves the Link UI over **HTTPS** and reads TLS material from `certs/robot.pem` and `certs/robot-key.pem` when both files exist (see `navi --help` for `--tls-cert` / `--tls-key`).

Tailscale can provision **publicly trusted** Let’s Encrypt certificates for each machine’s MagicDNS name (for example `robot.tail1234.ts.net`). Use that so browsers show a normal lock on your tailnet URL.

**WebTransport** (port 4433 by default) still uses the app’s built-in certificate and `/api/cert-hash` pinning; it does not need to match the HTTPS PEMs. You only need Tailscale certs for the main HTTPS UI.

## 1. Admin console (once per tailnet)

1. Open [DNS](https://login.tailscale.com/admin/dns).
2. Turn on **MagicDNS** if it isn’t already.
3. Under **HTTPS Certificates**, choose **Enable HTTPS** and accept the notice (machine names appear on the public Certificate Transparency log; see [Enabling HTTPS](https://tailscale.com/kb/1153/enabling-https)).

Rename the machine in the admin console first if its display name is sensitive.

## 2. On the Pi (or any host running `navi`)

Install Tailscale and log in (`tailscale up`). Ensure the machine is online on the tailnet.

From the repo root (for example `/home/joey/mr_robot`):

```bash
chmod +x deploy/renew-tailscale-cert.sh
./deploy/renew-tailscale-cert.sh
```

The script resolves this node’s MagicDNS name (`tailscale status --json`), then runs `tailscale cert` and writes:

- `certs/robot.pem`
- `certs/robot-key.pem`

You need `jq` (`sudo apt install jq`) and a working `sudo` (passwordless sudo for `joey` is already typical on this project).

To pin the name explicitly (optional):

```bash
./deploy/renew-tailscale-cert.sh robot.tail1234.ts.net
```

Or manually:

```bash
sudo tailscale cert \
  --cert-file /home/joey/mr_robot/certs/robot.pem \
  --key-file /home/joey/mr_robot/certs/robot-key.pem \
  "$(tailscale status --json | jq -r '.Self.DNSName' | sed 's/\.$//')"
```

Restart navi after new certs:

```bash
sudo systemctl restart link.service
```

## 3. Open the site

From another device on the same tailnet, use the **HTTPS** URL with your machine’s name and the port `navi` uses (default **8080**):

`https://robot.<your-tailnet>.ts.net:8080`

Use the exact name shown in the Tailscale admin **Machines** page or `tailscale status` (not bare `http://robot` unless you know your resolver setup).

## 4. Renewals

Let’s Encrypt certs expire about every **90 days**. Tailscale does **not** auto-renew files written with `tailscale cert`; re-run the script periodically.

Example: **weekly** cron as root (adjust the path):

```cron
0 4 * * 0 /home/joey/mr_robot/deploy/renew-tailscale-cert.sh && systemctl restart link.service
```

Or a systemd timer that runs the script then `systemctl restart link.service`. Avoid restarting on every successful deploy if you only refresh certs when needed.

## 5. Optional: `navi` flags

If cert paths differ from the defaults:

```bash
/home/joey/mr_robot/target/release/navi \
  --config /home/joey/mr_robot/config/robot.yaml \
  --tls-cert /path/to/fullchain.pem \
  --tls-key /path/to/key.pem
```

## 6. Don’t commit private keys

Keep `certs/robot-key.pem` out of git if it’s a real key (add `certs/*.pem` to `.gitignore` or store certs only on the robot). Replace any sample PEMs in the repo before relying on them in production.
