#!/bin/sh
# Build the 3D single-precision Windows x86_64 library, the export build (as the release
# script builds it), and copy it into a Godot project's addon when one is given:
#
#     scripts/build-win-3d.sh [path/to/project/addons/godot-rapier3d]
#
# The editor build (with class docs, `register-docs`) goes to bin/editor/ next to it.
set -eu
FEATURES="single-dim3,serde-serialize,parallel,api-4-7"
DLL=libgodot_rapier.windows.x86_64-pc-windows-msvc.dll
cargo build --release --features="$FEATURES,register-docs"
mkdir -p target/win3d/editor
cp target/release/godot_rapier.dll "target/win3d/editor/$DLL"
cargo build --release --features="$FEATURES"
cp target/release/godot_rapier.dll "target/win3d/$DLL"
if [ $# -ge 1 ]; then
    mkdir -p "$1/bin"
    cp "target/win3d/$DLL" "$1/bin/$DLL"
    if [ -d "$1/bin/editor" ]; then
        cp "target/win3d/editor/$DLL" "$1/bin/editor/$DLL"
    fi
    echo "installed into $1/bin"
fi
