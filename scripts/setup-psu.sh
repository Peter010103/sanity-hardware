#!/bin/sh
# Fetch the pinned GPL dependency and apply the included model/protocol patch.
set -eu
repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
dep_dir="$repo_dir/deps/tenma-serial"
revision=d1b73c25b61d308349acdfd8695041d0ea05ec09
if [ -e "$dep_dir" ]; then
    echo "Dependency directory exists; inspect it before installing again: $dep_dir" >&2
    exit 1
fi
mkdir -p "$repo_dir/deps"
git clone https://github.com/kxtells/tenma-serial.git "$dep_dir"
git -C "$dep_dir" checkout --detach "$revision"
git -C "$dep_dir" apply --check "$repo_dir/patches/tenma-72-13350.patch"
git -C "$dep_dir" apply "$repo_dir/patches/tenma-72-13350.patch"
echo "Pinned and patched tenma-serial in $dep_dir (GPL-3.0-or-later)."
