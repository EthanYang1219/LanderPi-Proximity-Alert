#!/usr/bin/env bash
#
# Deploys the host poc_fusion/ package into the MentorPi container's ROS 2
# workspace source tree.
#
# ONE-DIRECTIONAL: host -> container, ONLY. This script never copies
# anything from the container back to the host. /home/ubuntu/ros2_ws is
# Hiwonder's own git repo, and this deploy DELETES destination files that
# no longer exist in the host source (see the prune step below) -- so
# editing files directly inside the container and expecting them to
# survive is wrong and the failure is silent: the next run of this script
# just removes them. Author and commit only at the host path
# (/home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion/); this script is
# the only sanctioned way source reaches the container.
#
# Usage: scripts/deploy_poc_fusion.sh

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST_SRC="$REPO_ROOT/poc_fusion"
CONTAINER=MentorPi
DEST=/home/ubuntu/ros2_ws/src/poc_fusion

if [ ! -d "$HOST_SRC" ]; then
  echo "ERROR: host source not found at $HOST_SRC" >&2
  exit 1
fi

echo "Deploying $HOST_SRC -> $CONTAINER:$DEST"

docker exec -u ubuntu "$CONTAINER" mkdir -p "$DEST"

# Copy host tree into the container. The trailing "/." on the source makes
# docker cp copy the directory's *contents* into DEST rather than nesting
# a poc_fusion/ directory inside it.
docker cp "$HOST_SRC/." "$CONTAINER:$DEST"

# Prune destination files that no longer exist in the host source, so a
# renamed or removed module does not linger and get imported. build/,
# install/, log/, and __pycache__ are colcon/Python artifacts that can
# legitimately exist under DEST from a prior in-container build or test
# run and must never be touched by this script.
HOST_LIST="$(mktemp)"
DEST_LIST="$(mktemp)"
trap 'rm -f "$HOST_LIST" "$DEST_LIST"' EXIT

find "$HOST_SRC" \
  \( -name build -o -name install -o -name log -o -name __pycache__ \) -prune \
  -o -type f -print \
  | sed "s#^$HOST_SRC/##" | sort > "$HOST_LIST"

docker exec -u ubuntu "$CONTAINER" bash -lc "
  find '$DEST' \
    \( -name build -o -name install -o -name log -o -name __pycache__ \) -prune \
    -o -type f -print \
  | sed 's#^$DEST/##'
" | sort > "$DEST_LIST"

# Files present in the destination but absent from the host source: delete.
STALE="$(comm -13 "$HOST_LIST" "$DEST_LIST")"
if [ -n "$STALE" ]; then
  echo "Removing stale destination files (absent from host source):"
  echo "$STALE" | sed 's/^/  /'
  while IFS= read -r rel; do
    [ -z "$rel" ] && continue
    docker exec -u ubuntu "$CONTAINER" rm -f "$DEST/$rel"
  done <<< "$STALE"
  # Clean up any directories left empty by the deletions above (but never
  # touch build/, install/, log/, or __pycache__ -- they are pruned out of
  # both listings above and find's -empty here only matches truly empty
  # dirs, so a non-empty preserved dir is untouched regardless).
  docker exec -u ubuntu "$CONTAINER" bash -lc "
    find '$DEST' -depth -type d -empty \
      -not -name build -not -name install -not -name log -not -name __pycache__ \
      -delete
  "
else
  echo "No stale destination files to remove."
fi

# docker cp lands root-owned; a subsequent colcon build as ubuntu then
# fails with an error that does not name the real cause.
docker exec -u root "$CONTAINER" chown -R ubuntu:ubuntu "$DEST"

echo "Deploy complete."
