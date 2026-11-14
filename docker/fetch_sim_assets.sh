#!/usr/bin/env bash
# Third-party simulation assets, downloaded once at image build time and
# verified by SHA256 (never fetched at simulation start). Shared by
# Dockerfile.ci and Dockerfile.dev.
#
# Assets:
#   actor   animated walking person for the warehouse world
#           Gazebo Fuel model Mingfei/actor, version 1, by Mingfei
#           License: Creative Commons Attribution 4.0 International
#           https://fuel.gazebosim.org/1.0/Mingfei/models/actor
#           Only meshes/walk.dae and model.config are kept.
#
# Installed under ${ASSET_ROOT}/models (default /opt/wmm_assets/models), which
# warehouse_bringup's sim launch adds to GZ_SIM_RESOURCE_PATH.
set -euo pipefail

ASSET_ROOT="${ASSET_ROOT:-/opt/wmm_assets}"
ACTOR_URL="https://fuel.gazebosim.org/1.0/Mingfei/models/actor/1/actor.zip"
# Verified against two independent downloads on 2026-11-14.
ACTOR_SHA256="edd48c4e609c32e88ab63dc770c1fd0a6c7847d28340a3ce88b65f01ae04e5b4"

work="$(mktemp -d)"
trap 'rm -rf "${work}"' EXIT

python3 - "${ACTOR_URL}" "${work}/actor.zip" <<'EOF'
import sys
import urllib.request

url, out = sys.argv[1], sys.argv[2]
with urllib.request.urlopen(url, timeout=120) as response, open(out, "wb") as f:
    f.write(response.read())
EOF
echo "${ACTOR_SHA256}  ${work}/actor.zip" | sha256sum --check --strict

target="${ASSET_ROOT}/models/actor"
mkdir -p "${target}/meshes"
python3 - "${work}/actor.zip" "${target}" <<'EOF'
import sys
import zipfile
from pathlib import Path

archive, target = sys.argv[1], Path(sys.argv[2])
with zipfile.ZipFile(archive) as z:
    for name in ("model.config", "meshes/walk.dae"):
        (target / name).write_bytes(z.read(name))
EOF
cat > "${ASSET_ROOT}/ATTRIBUTION.txt" <<'EOF'
models/actor: Gazebo Fuel model "actor" by Mingfei, version 1,
https://fuel.gazebosim.org/1.0/Mingfei/models/actor
License: Creative Commons Attribution 4.0 International (CC BY 4.0),
http://creativecommons.org/licenses/by/4.0/
Subset used: meshes/walk.dae, model.config (unmodified).
EOF
chmod -R a+rX "${ASSET_ROOT}"
ls -la "${target}" "${target}/meshes"
