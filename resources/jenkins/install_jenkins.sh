#!/bin/bash
# Install Java 21 + Jenkins WAR and start Jenkins on this rosject.
# Re-provisioned every fresh rosject session, same as setup.sh/setup_real.sh.
#
# Usage:
#   ./install_jenkins.sh

set -e

# ========= ROS apt key refresh (only if expired) =========
# Ubuntu's ros-archive-keyring.gpg periodically expires (EXPKEYSIG
# F42ED6FBAB17C654), which blocks apt-get update/install entirely until
# refreshed. Detect and fix in place before installing anything else.
UPDATE_OUT="$(sudo apt-get update 2>&1)" || true
echo "$UPDATE_OUT"

if echo "$UPDATE_OUT" | grep -q "EXPKEYSIG F42ED6FBAB17C654"; then
  echo "🔧 ROS apt key expired; refreshing..."
  CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"
  ARCH="$(dpkg --print-architecture)"
  KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"

  sudo rm -f "$KEYRING"
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | sudo tee "$KEYRING" >/dev/null
  sudo chmod 644 "$KEYRING"

  ROS1_LINE="deb [arch=${ARCH} signed-by=${KEYRING}] http://packages.ros.org/ros/ubuntu ${CODENAME} main"
  ROS2_LINE="deb [arch=${ARCH} signed-by=${KEYRING}] http://packages.ros.org/ros2/ubuntu ${CODENAME} main"

  if [ -f /etc/apt/sources.list.d/ros-latest.list ]; then
    echo "$ROS1_LINE" | sudo tee /etc/apt/sources.list.d/ros-latest.list >/dev/null
    [ -f /etc/apt/sources.list.d/ros1.list ] && \
      sudo mv /etc/apt/sources.list.d/ros1.list /etc/apt/sources.list.d/ros1.list.disabled
  else
    echo "$ROS1_LINE" | sudo tee /etc/apt/sources.list.d/ros1.list >/dev/null
  fi

  echo "$ROS2_LINE" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

  for f in /etc/apt/sources.list.d/*.list; do
    [ "$f" = "/etc/apt/sources.list.d/ros-latest.list" ] && continue
    [ "$f" = "/etc/apt/sources.list.d/ros1.list" ] && continue
    if grep -q "http://packages.ros.org/ros/ubuntu" "$f"; then
      echo "Disabling duplicate ROS1 source: $f"
      sudo mv "$f" "${f}.disabled"
    fi
  done

  sudo rm -rf /var/lib/apt/lists/*
  sudo apt-get update
fi

# ========= Java 21 =========
# Jenkins 2.504.3 requires Java 17+ (21 here); this rosject's system java
# is 11 (used elsewhere, e.g. by ROS tooling) so install 21 alongside it
# rather than switching the system-wide `java` alternative.
# Tolerate failure here: this rosject has an unrelated broken package
# (zenoh-bridge-ros2dds, whose postinst needs systemd/dbus and can't run
# in this environment) that makes apt-get exit non-zero on ANY install,
# even when the package we actually asked for configured fine. The real
# check is whether the java 21 binary exists afterward, not apt's exit code.
echo "☕ Installing Java 21..."
sudo apt-get install -y openjdk-21-jre || true

JAVA21_BIN="/usr/lib/jvm/java-21-openjdk-amd64/bin/java"
if [ ! -x "$JAVA21_BIN" ]; then
  echo "Java 21 binary not found at $JAVA21_BIN after install — check 'update-alternatives --list java'."
  exit 1
fi

# ========= Jenkins WAR =========
export JENKINS_HOME="$HOME/ros2_ws/src/visual_calibration/jenkins"
mkdir -p "$JENKINS_HOME"

JENKINS_FILE="$HOME/jenkins.war"
JENKINS_VERSION="2.504.3"
JENKINS_URL_WAR="https://updates.jenkins.io/download/war/${JENKINS_VERSION}/jenkins.war"

if [ ! -f "$JENKINS_FILE" ]; then
  echo "⬇️  Downloading Jenkins WAR ${JENKINS_VERSION}..."
  wget -O "$JENKINS_FILE" "$JENKINS_URL_WAR"
else
  echo "jenkins.war already present. Skipping download."
fi

if pgrep -f "java .*jenkins\.war" >/dev/null 2>&1; then
  echo "Jenkins is already running. Exiting."
  exit 0
fi

# ========= Resolve the real public path prefix =========
# Root cause of a redirect/asset-404 loop hit repeatedly while testing
# this behind webpage_ws's proxy: Jenkins builds ALL of its own internal
# redirects (login, etc.) and asset URLs relative to its OWN servlet
# context path — whatever --prefix was launched with, below — NOT from
# JenkinsLocationConfiguration (that field is closer to informational,
# used for e.g. email links) and NOT from X-Forwarded-Prefix (Jenkins/
# Jetty's context path is fixed at boot, it does not dynamically extend
# from a per-request header). Both of those were tried first and neither
# fixed it — confirmed live (curl + browser): with --prefix=/jenkins,
# Jenkins always redirects to "/jenkins/login...", a ROOT-RELATIVE path,
# which the browser resolves against the domain root — losing
# "/<SLOT_PREFIX>/webpage/" entirely and 404ing, since the platform's own
# nginx only forwards requests that carry that exact prefix.
#
# The actual fix: launch Jenkins with --prefix set to the FULL external
# path, so ITS OWN context path already includes "/<SLOT_PREFIX>/webpage",
# and every redirect/asset URL it generates is correct from the start —
# same idea as the Vite dashboard's base:"./" fix, just via Jenkins' own
# native mechanism instead (Jenkins has no "use relative URLs" setting,
# so the context path itself has to be the true external one).
#
# SLOT_PREFIX (env var, set by the rosject platform) + the EC2
# instance-metadata endpoint — same resolution
# webpage_ws/app/scripts/write_rosject_config.sh uses for webpage_address.
# Falls back to plain "/jenkins" if unresolvable (e.g. not actually in a
# rosject) — matches this script's pre-fix behavior in that case, so it
# degrades no worse than before, it just won't be externally correct
# outside a real rosject session.
JENKINS_PREFIX="/jenkins"
JENKINS_PUBLIC_URL=""
if [ -n "${SLOT_PREFIX:-}" ]; then
  INSTANCE_ID="$(curl -s --max-time 2 http://169.254.169.254/latest/meta-data/instance-id 2>/dev/null)"
  if [ -n "$INSTANCE_ID" ]; then
    JENKINS_PREFIX="/${SLOT_PREFIX}/webpage/jenkins"
    JENKINS_PUBLIC_URL="https://${INSTANCE_ID}.robotigniteacademy.com${JENKINS_PREFIX}/"
  fi
fi
if [ -z "$JENKINS_PUBLIC_URL" ]; then
  echo "WARNING: could not resolve \$SLOT_PREFIX/instance-id — launching Jenkins with"
  echo "plain --prefix=/jenkins. Its redirects/links will be WRONG when reached through"
  echo "webpage_ws's proxy from a real rosject session (only correct for direct"
  echo "localhost:8080 access). Re-run once \$SLOT_PREFIX is available."
fi

# ========= Skip setup wizard =========
mkdir -p "$JENKINS_HOME/init.groovy.d"
touch "$JENKINS_HOME/jenkins.install.UpgradeWizard.state" \
      "$JENKINS_HOME/jenkins.install.InstallUtil.lastExecVersion" 2>/dev/null || true
echo "$JENKINS_VERSION" > "$JENKINS_HOME/jenkins.install.UpgradeWizard.state"
echo "$JENKINS_VERSION" > "$JENKINS_HOME/jenkins.install.InstallUtil.lastExecVersion"

# ========= Pre-seed admin user =========
# Fixed admin/admin credentials: acceptable here only because this Jenkins
# is rosject-local (no public IP), single-user, manually-triggered, and
# wiped every fresh session — not an internet-facing instance. Re-running
# this script against an already-provisioned JENKINS_HOME is a no-op below
# (skip if the user's already been created earlier this session).
SECURITY_GROOVY="$JENKINS_HOME/init.groovy.d/basic-security.groovy"
if [ ! -f "$JENKINS_HOME/config.xml" ]; then
  echo "🔐 Seeding admin/admin user via init.groovy.d (skips setup wizard login)..."
  cat > "$SECURITY_GROOVY" <<'EOF'
import jenkins.model.*
import hudson.security.*

def instance = Jenkins.get()

def hudsonRealm = new HudsonPrivateSecurityRealm(false)
hudsonRealm.createAccount("admin", "admin")
instance.setSecurityRealm(hudsonRealm)

def strategy = new FullControlOnceLoggedInAuthorizationStrategy()
strategy.setAllowAnonymousRead(false)
instance.setAuthorizationStrategy(strategy)

instance.save()
EOF
else
  echo "Admin user already seeded from earlier this session. Skipping."
fi

# ========= Auto-install plugin list =========
# workflow-aggregator pulls in Pipeline (declarative + scripted), and with it
# the archiveArtifacts/junit steps — this is the exact gap left open in the
# prior CI-ros2-fastbot/CI-ros1 repos. git plugin intentionally omitted:
# this project deploys config/scripts by hand-copy, not git.
#
# blueocean added per user request: modern staged-pipeline visualization UI
# with a per-stage live log panel — exactly what's wanted for watching a
# staged, logged deployment run during a live presentation. Note: Blue
# Ocean's own per-stage log panel already covers "watch logs stream in a
# nice UI" — no separate terminal/shell-in-browser plugin is installed or
# needed here (that would be interactive shell access, a different
# feature, out of scope for this pipeline).
#
# Listing the specific blueocean-* sub-plugins actually needed (pipeline
# visualization + dashboard + REST backing them), NOT the "blueocean"
# umbrella meta-plugin — that umbrella pulls in every SCM-specific Blue
# Ocean integration (blueocean-bitbucket-pipeline, blueocean-github-pipeline,
# etc.), none of which apply here (this project deploys by hand-copy, not
# git/Bitbucket/GitHub webhooks). Confirmed live: blueocean-bitbucket-
# pipeline failed dependency resolution on this Jenkins version and
# cascaded into blueocean-pipeline-editor and blueocean itself failing to
# load — dropping the umbrella and the unneeded SCM plugins avoids pulling
# in that broken dependency at all.
PLUGIN_DIR="$JENKINS_HOME/plugins"
PLUGINS_TXT="$JENKINS_HOME/plugins.txt"
cat > "$PLUGINS_TXT" <<'EOF'
workflow-aggregator
pipeline-stage-view
blueocean-pipeline-editor
blueocean-dashboard
blueocean-rest
blueocean-web
timestamper
ws-cleanup
EOF

if [ ! -f "$PLUGIN_DIR/.installed-from-plugins-txt" ]; then
  echo "🔌 Installing plugin list via jenkins-plugin-cli..."
  JENKINS_PLUGIN_CLI="$HOME/jenkins-plugin-cli.jar"
  # A failed/interrupted prior download (e.g. a 404 error page saved as the
  # jar) leaves a file that satisfies -f but isn't a real jar. `unzip -l`
  # (jars are zip files) catches that; a plain existence check doesn't.
  if [ -f "$JENKINS_PLUGIN_CLI" ] && ! unzip -l "$JENKINS_PLUGIN_CLI" >/dev/null 2>&1; then
    echo "Existing jenkins-plugin-cli.jar is corrupt/invalid — removing and re-downloading."
    rm -f "$JENKINS_PLUGIN_CLI"
  fi
  if [ ! -f "$JENKINS_PLUGIN_CLI" ]; then
    echo "⬇️  Downloading jenkins-plugin-cli..."
    # The GitHub "latest" redirect points at a release tag, but the jar
    # filename itself embeds that same version (e.g. jenkins-plugin-manager-
    # 2.15.0.jar) — a hardcoded old filename 404s the moment upstream cuts a
    # new release. Resolve the real asset URL from the API instead of
    # guessing the filename.
    PLUGIN_CLI_API_URL=https://api.github.com/repos/jenkinsci/plugin-installation-manager-tool/releases/latest
    PLUGIN_CLI_URL="$(curl -sL "$PLUGIN_CLI_API_URL" \
      | grep -o '"browser_download_url": *"[^"]*jenkins-plugin-manager-[0-9.]*\.jar"' \
      | head -1 | sed 's/.*"\(https[^"]*\)"/\1/')"
    if [ -z "$PLUGIN_CLI_URL" ]; then
      echo "Could not resolve latest jenkins-plugin-manager jar URL from GitHub API." >&2
      exit 1
    fi
    wget -O "$JENKINS_PLUGIN_CLI" "$PLUGIN_CLI_URL"
  fi
  mkdir -p "$PLUGIN_DIR"
  "$JAVA21_BIN" -jar "$JENKINS_PLUGIN_CLI" \
    --war "$JENKINS_FILE" \
    --plugin-file "$PLUGINS_TXT" \
    --plugin-download-directory "$PLUGIN_DIR"
  touch "$PLUGIN_DIR/.installed-from-plugins-txt"
else
  echo "Plugins already installed from earlier this session. Skipping."
fi

echo "🚀 Starting Jenkins..."
LOG_FILE="$JENKINS_HOME/jenkins.log"
PID_FILE="$JENKINS_HOME/jenkins.pid"

# ========= Port =========
# JENKINS_PORT: internal-only — NOT reachable from outside the rosject on
# its own. The only externally-reachable port on this rosject is 7000
# (platform-owned nginx, fixed, see webpage_ws/README.md); webpage_ws's
# scripts/proxy_server.mjs binds 7000 and forwards paths under
# JENKINS_PREFIX (resolved above) to this port, everything else to the
# dashboard. 8080 is Jenkins' own default and was already never exposed by
# the platform proxy, but we bind it explicitly here (rather than relying
# on that default) so the internal-only intent is documented at the one
# place that controls it.
JENKINS_PORT="${JENKINS_PORT:-8080}"

# setsid (not just nohup) — this is meant to survive `tmux kill-session`
# on the tmuxwebstacksim pane that launched it (Jenkins is meant to keep
# running independently of that dev session, by design). `nohup` alone
# only makes the process ignore SIGHUP; it does not detach it from the
# calling shell's process group, and tmux kill-session can signal the
# whole group, not just send SIGHUP to the leader — so nohup-only
# survival isn't guaranteed across tmux versions/config. setsid starts
# Jenkins as the leader of a brand-new session with no controlling
# terminal at all, which tmux teardown cannot reach.
setsid nohup "$JAVA21_BIN" -jar "$JENKINS_FILE" \
  -Djenkins.install.runSetupWizard=false \
  --httpPort="$JENKINS_PORT" \
  --prefix="$JENKINS_PREFIX" \
  >"$LOG_FILE" 2>&1 &
echo $! > "$PID_FILE"

JENKINS_PID="$(cat "$PID_FILE")"
sleep 5

echo ""
echo "✅ Jenkins started (PID: $JENKINS_PID)"
echo "Local URL (internal only): http://localhost:${JENKINS_PORT}${JENKINS_PREFIX}/"
if [ -n "$JENKINS_PUBLIC_URL" ]; then
  echo "Public URL (once webpage_ws's proxy is running): $JENKINS_PUBLIC_URL"
else
  echo "Public URL: unresolved this run — see warning above, re-run once \$SLOT_PREFIX is set."
fi

echo "Log file:  $LOG_FILE"
echo "Login: admin / admin (fixed dev credentials, see comment above)"
