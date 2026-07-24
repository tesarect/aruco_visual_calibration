#!/bin/bash
# Install + start Loki, Promtail, and Grafana on this rosject. Loki/Promtail
# stay INTERNAL ONLY (127.0.0.1, no browser access needed — Grafana is the
# only UI a human queries Loki through). Grafana itself IS exposed, via
# webpage_ws/app/scripts/proxy_server.mjs's /grafana route — see that file's
# header comment for the proxy side of this. Re-provisioned every fresh
# rosject session, same as install_jenkins.sh/setup.sh/setup_real.sh.
#
# Why binary tarballs, not apt: apt-get install on this rosject can exit
# non-zero on ANY package due to an unrelated broken postinst
# (zenoh-bridge-ros2dds, needs systemd/dbus — see install_jenkins.sh's own
# comment on this). Grafana/Loki/Promtail's official .deb packages carry
# their own systemd-unit postinst hooks too, which would hit the exact same
# wall in this environment. Plain binary tarballs sidestep both problems
# entirely and match how Jenkins itself is run here (WAR file + setsid, no
# systemd/apt service).
#
# Usage:
#   ./install_grafana.sh

set -e

# ========= Versions =========
LOKI_VERSION="3.2.1"
PROMTAIL_VERSION="3.2.1"
GRAFANA_VERSION="11.4.0"

# ========= Layout =========
# Runtime data/config lives OUTSIDE resources/grafana/ (git-tracked install
# scripts only) — mirrors install_jenkins.sh's JENKINS_HOME split from
# resources/jenkins/.
GRAFANA_BASE="$HOME/ros2_ws/src/visual_calibration/grafana_stack"
LOKI_HOME="$GRAFANA_BASE/loki"
PROMTAIL_HOME="$GRAFANA_BASE/promtail"
GRAFANA_HOME="$GRAFANA_BASE/grafana"
mkdir -p "$LOKI_HOME" "$PROMTAIL_HOME" "$GRAFANA_HOME"

BIN_DIR="$GRAFANA_BASE/bin"
mkdir -p "$BIN_DIR"

# ========= Ports =========
# Loki: internal-only (127.0.0.1), no proxy route — not meant for direct
# human browsing (Grafana is the UI in front of it). Grafana: also bound
# 127.0.0.1 (never directly exposed), but reachable externally through
# webpage_ws's proxy at PUBLIC_PORT 7000 (see GRAFANA_PREFIX resolution
# below and proxy_server.mjs's /grafana route).
LOKI_PORT="${LOKI_PORT:-3100}"
GRAFANA_PORT="${GRAFANA_PORT:-3000}"

# ========= Download helper =========
# Tolerates a failed/interrupted prior download the same way
# install_jenkins.sh checks jenkins-plugin-cli.jar: verify the file is
# actually the right archive type before trusting it's present.
download_if_needed() {
  local url="$1"
  local dest="$2"
  local check_cmd="$3" # command that exits 0 if $dest is a valid archive

  if [ -f "$dest" ] && ! eval "$check_cmd '$dest'" >/dev/null 2>&1; then
    echo "Existing $(basename "$dest") is corrupt/invalid — removing and re-downloading."
    rm -f "$dest"
  fi
  if [ ! -f "$dest" ]; then
    echo "⬇️  Downloading $(basename "$dest")..."
    wget -O "$dest" "$url"
  else
    echo "$(basename "$dest") already present. Skipping download."
  fi
}

# ========= Loki =========
LOKI_ZIP="$HOME/loki-linux-amd64.zip"
download_if_needed \
  "https://github.com/grafana/loki/releases/download/v${LOKI_VERSION}/loki-linux-amd64.zip" \
  "$LOKI_ZIP" "unzip -l"

if [ ! -x "$BIN_DIR/loki" ]; then
  echo "📦 Extracting Loki..."
  unzip -o "$LOKI_ZIP" -d "$BIN_DIR" >/dev/null
  mv "$BIN_DIR/loki-linux-amd64" "$BIN_DIR/loki"
  chmod +x "$BIN_DIR/loki"
fi

# Minimal single-process Loki config, filesystem storage under LOKI_HOME —
# no S3/GCS, no auth (internal-only, single-user rosject, same trust model
# as Jenkins' fixed admin/admin — see install_jenkins.sh's comment on that).
LOKI_CONFIG="$LOKI_HOME/loki-config.yaml"
cat > "$LOKI_CONFIG" <<EOF
auth_enabled: false

server:
  http_listen_address: 127.0.0.1
  http_listen_port: ${LOKI_PORT}
  grpc_listen_address: 127.0.0.1

common:
  path_prefix: ${LOKI_HOME}/data
  storage:
    filesystem:
      chunks_directory: ${LOKI_HOME}/data/chunks
      rules_directory: ${LOKI_HOME}/data/rules
  replication_factor: 1
  # instance_addr: without this, Loki auto-detects its own self-advertise
  # IP from the machine's real network interfaces (found this rosject's
  # eth0 IP, e.g. 172.18.0.6) instead of using loopback — even with
  # -target=all and both listeners above already bound to 127.0.0.1. Every
  # internal component (querier, scheduler, ingester) then tries to dial
  # itself at that eth0 address on the ring/gRPC port, which nothing
  # actually listens on there, and every internal RPC fails with
  # "connection refused" — confirmed live (Promtail pushes were being
  # silently dropped, returning HTTP 500 from Loki, not the earlier
  # "just noisy startup warnings" assumption). Forcing instance_addr to
  # 127.0.0.1 makes self-advertise match the listeners' actual bind
  # address.
  instance_addr: 127.0.0.1
  ring:
    kvstore:
      store: inmemory

schema_config:
  configs:
    - from: 2020-10-24
      store: tsdb
      object_store: filesystem
      schema: v13
      index:
        prefix: index_
        period: 24h

limits_config:
  allow_structured_metadata: false
EOF

# ========= Promtail =========
# Ships Jenkins' build-stage log (stage_build.sh's $LOG_DIR/build_colcon.log,
# i.e. $WORKSPACE/logs/build_colcon.log under Jenkins' per-build workspace —
# see pipeline_common.sh) into Loki. Chosen over a bespoke log-forwarder
# script: Promtail is the standard companion to Loki (same project, same
# label conventions Grafana's LogQL expects out of the box), and its file
# glob + position-tracking (promtail-positions.yaml) already solves
# "resume from where it left off across restarts" and "pick up NEW Jenkins
# build workspaces automatically" for free — a hand-rolled tail loop would
# just be re-implementing both, worse.
PROMTAIL_ZIP="$HOME/promtail-linux-amd64.zip"
download_if_needed \
  "https://github.com/grafana/loki/releases/download/v${PROMTAIL_VERSION}/promtail-linux-amd64.zip" \
  "$PROMTAIL_ZIP" "unzip -l"

if [ ! -x "$BIN_DIR/promtail" ]; then
  echo "📦 Extracting Promtail..."
  unzip -o "$PROMTAIL_ZIP" -d "$BIN_DIR" >/dev/null
  mv "$BIN_DIR/promtail-linux-amd64" "$BIN_DIR/promtail"
  chmod +x "$BIN_DIR/promtail"
fi

# JENKINS_HOME's workspace root — matches pipeline_common.sh's
# WORKSPACE-derived $LOG_DIR (Jenkins sets $WORKSPACE per-job to
# JENKINS_HOME/workspace/<job-name>). Globbing */logs/build_colcon.log
# under workspace/ picks up build_colcon.log for ANY job name, not just
# today's, without hardcoding the pipeline's job name here.
JENKINS_HOME_DIR="$HOME/ros2_ws/src/visual_calibration/jenkins"
PROMTAIL_CONFIG="$PROMTAIL_HOME/promtail-config.yaml"
cat > "$PROMTAIL_CONFIG" <<EOF
server:
  http_listen_address: 127.0.0.1
  http_listen_port: 9080
  grpc_listen_port: 0

positions:
  filename: ${PROMTAIL_HOME}/promtail-positions.yaml

clients:
  - url: http://127.0.0.1:${LOKI_PORT}/loki/api/v1/push

scrape_configs:
  - job_name: jenkins_build_colcon
    static_configs:
      - targets:
          - localhost
        labels:
          job: jenkins_build_colcon
          __path__: ${JENKINS_HOME_DIR}/workspace/*/logs/build_colcon.log
EOF

# ========= Resolve the real public path prefix =========
# Same root cause + same fix as install_jenkins.sh's own JENKINS_PREFIX/
# JENKINS_PUBLIC_URL resolution (read that script's comment for the full
# story) — verified against Grafana's own docs before assuming this applies
# here too, rather than assuming a lighter-weight header-based fix would
# work (Jenkins already burned time on that assumption once):
#   Grafana's `root_url` / `serve_from_sub_path` config is read ONCE at
#   process boot (grafana.ini, below) and every redirect/asset URL Grafana
#   generates for the rest of that process's life is built from it — same
#   "fixed at boot, not per-request" shape as Jenkins' servlet context
#   path. Grafana does NOT dynamically honor X-Forwarded-Prefix (or any
#   other proxy header) to extend its own root path per-request; the
#   subpath must already be baked into `root_url` at startup, with
#   `serve_from_sub_path = true` telling Grafana to actually serve/redirect
#   under that subpath (rather than just using root_url for OAuth-callback
#   purposes, which is its only effect when serve_from_sub_path is false —
#   the default, kept off for compatibility). So: same class of fix as
#   Jenkins' --prefix, just via Grafana's own ini keys instead of a CLI
#   flag.
#
# SLOT_PREFIX (env var, set by the rosject platform) + the EC2
# instance-metadata endpoint — identical resolution to install_jenkins.sh's
# JENKINS_PREFIX/JENKINS_PUBLIC_URL and
# webpage_ws/app/scripts/write_rosject_config.sh's webpageAddress. Falls
# back to plain "/grafana" (serve_from_sub_path left at its default false,
# since there's no real external subpath to serve from in that case) if
# unresolvable — matches Jenkins' degrade-no-worse-than-before behavior.
GRAFANA_PREFIX="/grafana"
GRAFANA_PUBLIC_URL=""
GRAFANA_ROOT_URL=""
GRAFANA_SERVE_FROM_SUB_PATH="false"
if [ -n "${SLOT_PREFIX:-}" ]; then
  INSTANCE_ID="$(curl -s --max-time 2 http://169.254.169.254/latest/meta-data/instance-id 2>/dev/null)"
  if [ -n "$INSTANCE_ID" ]; then
    GRAFANA_PREFIX="/${SLOT_PREFIX}/webpage/grafana"
    GRAFANA_PUBLIC_URL="https://${INSTANCE_ID}.robotigniteacademy.com${GRAFANA_PREFIX}/"
    GRAFANA_ROOT_URL="$GRAFANA_PUBLIC_URL"
    GRAFANA_SERVE_FROM_SUB_PATH="true"
  fi
fi
if [ -z "$GRAFANA_PUBLIC_URL" ]; then
  echo "WARNING: could not resolve \$SLOT_PREFIX/instance-id — launching Grafana with"
  echo "root_url unset (serve_from_sub_path=false). It will only be correctly reachable"
  echo "via direct http://127.0.0.1:${GRAFANA_PORT} access, NOT through webpage_ws's"
  echo "/grafana proxy route from a real rosject session. Re-run once \$SLOT_PREFIX is"
  echo "available."
fi

# ========= Grafana Live (WebSocket) origin allowlist =========
# Separate from root_url/serve_from_sub_path above — confirmed against
# Grafana 11.4.0's own source (pkg/services/live/*, getCheckOriginFunc):
# Grafana Live (the WebSocket transport behind Explore's "Live" tail toggle,
# and live dashboard panel updates) validates the browser's Origin header
# on the WebSocket Upgrade request THROUGH A DIFFERENT CODE PATH than the
# plain-HTTP redirect/asset-URL logic root_url feeds. It is NOT simply "one
# more consumer of root_url" — same lesson this project already paid for
# twice today (Jenkins' X-Forwarded-Prefix not mattering to its own
# redirect logic; Loki's -target=all alone not being enough without
# instance_addr). Concretely: getCheckOriginFunc first accepts if the
# Origin header's host matches the request's own Host header (should
# already hold here, since neither the platform nginx nor this project's
# proxy_server.mjs rewrite Host/changeOrigin) — but falls back to matching
# Origin against root_url's scheme+host ONLY (path is never compared, so
# root_url's /grafana subpath is irrelevant to this check either way) if
# that first check ever misses, e.g. due to some intermediate hop
# normalizing/altering Host. Rather than depend on that implicit fallback
# "should be sufficient for most scenarios" (Grafana's own docs' wording,
# not a guarantee), set [live] allowed_origins explicitly and directly from
# the SAME already-resolved GRAFANA_PUBLIC_URL used for root_url above —
# no separate origin value computed. allowed_origins wants scheme+host
# (with optional wildcard), not a full URL with path, so strip the path
# here purely for a clean value; it would be ignored either way.
GRAFANA_ALLOWED_ORIGINS=""
if [ -n "$GRAFANA_PUBLIC_URL" ]; then
  GRAFANA_ALLOWED_ORIGINS="$(printf '%s\n' "$GRAFANA_PUBLIC_URL" | sed -E 's#^(https?://[^/]+).*#\1#')"
fi

# csrf_trusted_origins wants a bare host[:port] (no scheme, no path) —
# see the [security] csrf_trusted_origins comment below for why this is a
# separate setting from GRAFANA_ALLOWED_ORIGINS above, not a duplicate.
GRAFANA_CSRF_TRUSTED_ORIGIN=""
if [ -n "$GRAFANA_PUBLIC_URL" ]; then
  GRAFANA_CSRF_TRUSTED_ORIGIN="$(printf '%s\n' "$GRAFANA_PUBLIC_URL" | sed -E 's#^https?://([^/]+).*#\1#')"
fi

# ========= Grafana =========
GRAFANA_TARBALL="$HOME/grafana-${GRAFANA_VERSION}.linux-amd64.tar.gz"
download_if_needed \
  "https://dl.grafana.com/oss/release/grafana-${GRAFANA_VERSION}.linux-amd64.tar.gz" \
  "$GRAFANA_TARBALL" "tar -tzf"

# The official tarball extracts to a "v"-prefixed directory
# (grafana-v11.4.0), unlike Loki/Promtail's own release layout — confirmed
# live (find ~/.../grafana_stack/bin/ showed grafana-v11.4.0 on disk) after
# the non-"v" path here caused `nohup: failed to run command ...: No such
# file or directory` and Grafana silently never started.
GRAFANA_DIST="$BIN_DIR/grafana-v${GRAFANA_VERSION}"
if [ ! -d "$GRAFANA_DIST" ]; then
  echo "📦 Extracting Grafana..."
  tar -xzf "$GRAFANA_TARBALL" -C "$BIN_DIR"
fi

# Grafana config: bind 127.0.0.1 only (never directly exposed — reachable
# externally only through webpage_ws's proxy). root_url/serve_from_sub_path
# set from the resolution above so Grafana's own redirects/asset URLs are
# correct from boot (see that block's comment for why this can't be a
# proxy-header trick). Fixed admin/admin, same trust model/justification as
# Jenkins (rosject-local, no public IP, single-user, wiped every fresh
# session).
GRAFANA_INI="$GRAFANA_HOME/grafana.ini"
cat > "$GRAFANA_INI" <<EOF
[server]
http_addr = 127.0.0.1
http_port = ${GRAFANA_PORT}
root_url = ${GRAFANA_ROOT_URL}
serve_from_sub_path = ${GRAFANA_SERVE_FROM_SUB_PATH}

[live]
# See the GRAFANA_ALLOWED_ORIGINS resolution above for why this is set
# explicitly rather than left to root_url's implicit origin-matching
# fallback. Empty when GRAFANA_PUBLIC_URL couldn't be resolved (local-only
# run) — Live then falls back to matching the plain Origin/Host equality
# check, which is fine for direct 127.0.0.1 access.
allowed_origins = ${GRAFANA_ALLOWED_ORIGINS}

[security]
# A non-default username AND password for the initial admin account —
# Grafana only forces its "update your password" nag screen when the
# CURRENT password is literally the string "admin" (confirmed live: this
# was the actual root cause of a long login/password-change 401 loop this
# session, not a deeper CSRF/proxy-topology issue as first suspected).
# Using a distinct username here too (not just a non-default password)
# sidesteps that check entirely at account-creation time, rather than
# relying on the password alone never regressing back to the literal
# default. These [security] admin_user/admin_password keys set the
# INITIAL admin account's credentials directly (full org Admin rights,
# same as Grafana's own default admin — no separate role/permission
# config needed) — only take effect on Grafana's first-ever boot for a
# given grafana_stack/grafana/data dir, same one-time-only semantics
# Jenkins' admin/admin seeding already has via init.groovy.d.
admin_user = rosject_admin
admin_password = rosject_admin_pw
# cookie_samesite=none: root cause of the Live-tail failure, confirmed via
# direct testing (NOT the [live] allowed_origins/origin-check theory tried
# first — that was a red herring the browser's own error text pointed at
# incorrectly). A curl WebSocket handshake against /api/live/ws — both
# straight to Grafana and through webpage_ws's proxy — returned a clean
# 401 Unauthorized, and Grafana's own log confirmed
# "[auth.unauthorized] cannot authenticate request", not any CSRF/origin
# rejection. Inspecting the actual Set-Cookie Grafana issues on login
# showed grafana_session with SameSite=Lax and no Secure flag. Lax cookies
# are inconsistently attached by browsers to a JS-initiated WebSocket
# handshake (new WebSocket(...), which is how Grafana's frontend opens
# Live) — unlike a normal top-level navigation or fetch()/XHR, which Lax
# does cover (explaining why regular page loads and non-live Explore
# queries worked fine while only Live failed). SameSite=None is the
# documented fix for exactly this case, and per browser spec (and
# Grafana's own handling) requires Secure to also be set — Grafana sets
# Secure automatically once cookie_samesite is none, no separate
# cookie_secure line needed.
cookie_samesite = none
# csrf_trusted_origins: confirmed live — after the cookie_samesite fix,
# login itself works (verified: POST .../login returns 200/"Logged in"),
# but the forced "change default password" follow-up screen's own
# PUT .../api/user/password request 401s. That endpoint is a normal
# session-authenticated, state-changing API call, which Grafana's CSRF
# middleware gates on Origin/Referer matching a trusted origin list —
# a SEPARATE check from both [live] allowed_origins (Live-websocket-only,
# already set above) and cookie_samesite (session-cookie-only, just
# fixed above). Neither of those two covers this. Format here is bare
# host[:port], no scheme (different from [live] allowed_origins, which
# wants scheme+host) — confirmed against Grafana's own sample.ini.
csrf_trusted_origins = ${GRAFANA_CSRF_TRUSTED_ORIGIN}

# [auth.anonymous] was tried (enabled=true, org_role=Admin) as a way to
# skip the login screen entirely, but REVERTED — confirmed live it breaks
# Grafana Live specifically: WebSocket auth for /api/live/ws needs a real
# logged-in user's session token, which an anonymous session doesn't have
# (regular page loads/queries worked fine anonymously; Live consistently
# failed with "user token not found", status=400 — a different, more
# specific error than the earlier CSRF-related 401s). The actual root
# cause of the earlier login/password-change loop turned out to be
# simpler than a proxy-topology issue: Grafana forces its "update your
# password" nag ONLY when the password is literally the string "admin";
# admin_password below being exactly that was the real trigger, not CSRF/
# cookie config (those fixes were still independently needed for OTHER
# symptoms — Live's cookie_samesite/csrf_trusted_origins above are correct
# and stay). Since real login now works cleanly with a non-default
# password, anonymous access is unneeded and was net-negative (traded
# away Live for a login screen that wasn't actually broken).

[analytics]
reporting_enabled = false
check_for_updates = false

[paths]
data = ${GRAFANA_HOME}/data
logs = ${GRAFANA_HOME}/logs
plugins = ${GRAFANA_HOME}/plugins
provisioning = ${GRAFANA_HOME}/provisioning
EOF

# Provisioned Loki datasource — so Loki is queryable in Grafana's Explore
# view immediately after first boot, no manual "Add data source" click
# needed (re-written every run, matching install_jenkins.sh's
# re-provisioned-every-session convention).
PROVISIONING_DS_DIR="$GRAFANA_HOME/provisioning/datasources"
mkdir -p "$PROVISIONING_DS_DIR"
cat > "$PROVISIONING_DS_DIR/loki.yaml" <<EOF
apiVersion: 1
datasources:
  - name: Loki
    type: loki
    access: proxy
    url: http://127.0.0.1:${LOKI_PORT}
    isDefault: true
    editable: true
EOF
mkdir -p "$GRAFANA_HOME/provisioning/dashboards" "$GRAFANA_HOME/provisioning/alerting" \
  "$GRAFANA_HOME/provisioning/notifiers" "$GRAFANA_HOME/provisioning/plugins"

# ========= Start (or report already-running) =========
LOKI_LOG="$LOKI_HOME/loki.log"
LOKI_PID_FILE="$LOKI_HOME/loki.pid"
PROMTAIL_LOG="$PROMTAIL_HOME/promtail.log"
PROMTAIL_PID_FILE="$PROMTAIL_HOME/promtail.pid"
GRAFANA_LOG="$GRAFANA_HOME/grafana.log"
GRAFANA_PID_FILE="$GRAFANA_HOME/grafana.pid"

# setsid (not just nohup) for all three, same reasoning as install_jenkins.sh:
# meant to survive `tmux kill-session` on whatever pane launched this, not
# just SIGHUP — see that script's comment for the full nohup-vs-setsid case.

if pgrep -f "$BIN_DIR/loki " >/dev/null 2>&1 || pgrep -f "$BIN_DIR/loki$" >/dev/null 2>&1; then
  echo "Loki already running."
else
  echo "🚀 Starting Loki..."
  # -target=all: forces genuine single-binary mode (everything — ingester,
  # querier, scheduler, distributor — runs in-process, no real network
  # calls between them). Without this flag, this Loki build still spun up
  # per-component gRPC listeners and tried to dial itself over the
  # rosject's bridge network IP (172.18.0.6:9095) for inter-component
  # calls — confirmed live: Promtail's pushes were failing with
  # "server returned HTTP status 500 ... connection refused" against that
  # exact address, meaning Loki was silently DROPPING every log line
  # Promtail sent, not just printing harmless warnings as first assumed.
  setsid nohup "$BIN_DIR/loki" -target=all -config.file="$LOKI_CONFIG" \
    >"$LOKI_LOG" 2>&1 &
  echo $! > "$LOKI_PID_FILE"
  sleep 2
fi

if pgrep -f "$BIN_DIR/promtail " >/dev/null 2>&1 || pgrep -f "$BIN_DIR/promtail$" >/dev/null 2>&1; then
  echo "Promtail already running."
else
  echo "🚀 Starting Promtail..."
  setsid nohup "$BIN_DIR/promtail" -config.file="$PROMTAIL_CONFIG" \
    >"$PROMTAIL_LOG" 2>&1 &
  echo $! > "$PROMTAIL_PID_FILE"
  sleep 1
fi

if pgrep -f "$GRAFANA_DIST/bin/grafana " >/dev/null 2>&1 || pgrep -f "$GRAFANA_DIST/bin/grafana$" >/dev/null 2>&1; then
  echo "Grafana already running."
else
  echo "🚀 Starting Grafana..."
  setsid nohup "$GRAFANA_DIST/bin/grafana" server \
    --config="$GRAFANA_INI" \
    --homepath="$GRAFANA_DIST" \
    >"$GRAFANA_LOG" 2>&1 &
  echo $! > "$GRAFANA_PID_FILE"
  sleep 3
fi

echo ""
echo "✅ Grafana stack up"
echo "  Loki:     http://127.0.0.1:${LOKI_PORT}  (internal-only, log: $LOKI_LOG)"
echo "  Promtail: shipping ${JENKINS_HOME_DIR}/workspace/*/logs/build_colcon.log -> Loki (log: $PROMTAIL_LOG)"
echo "  Grafana:  http://127.0.0.1:${GRAFANA_PORT}${GRAFANA_PREFIX}/  (internal, log: $GRAFANA_LOG, login rosject_admin/rosject_admin_pw)"
if [ -n "$GRAFANA_PUBLIC_URL" ]; then
  echo "  Public URL (once webpage_ws's proxy is running): $GRAFANA_PUBLIC_URL"
else
  echo "  Public URL: unresolved this run — see warning above, re-run once \$SLOT_PREFIX is set."
fi
echo ""
echo "Verify from inside the rosject (e.g. this same shell, or the tmux debug pane):"
echo "  curl -s http://127.0.0.1:${LOKI_PORT}/ready"
echo "  curl -s http://127.0.0.1:${GRAFANA_PORT}${GRAFANA_PREFIX}/api/health"
echo "Browser access: through webpage_ws's proxy at the Public URL above (proxy_server.mjs's /grafana route)."
