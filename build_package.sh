#!/bin/bash
set -e

# Build distributable ForRocket packages (Linux + Windows) bundled with a
# complete, ready-to-run example set. Produces one zip per platform.

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "${SCRIPT_DIR}"

# Version is derived from the single source of truth in src/ForRocket.cpp.
FORROCKET_VER_DOT=$(sed -n 's/.*program_ver = "\(.*\)".*/\1/p' src/ForRocket.cpp)
if [ -z "${FORROCKET_VER_DOT}" ]; then
    echo "ERROR: program_ver not found in src/ForRocket.cpp" >&2
    exit 1
fi
FORROCKET_VER_US=${FORROCKET_VER_DOT//./_}
CURRENT_DATE=$(date '+%Y%m%d%H%M%S')

echo "Packaging ForRocket v${FORROCKET_VER_DOT}"

BUILD_WIN=build-win-pkg
BUILD_LINUX=build-linux-pkg

# Example files required to run the default sample case (sample_config_solver.json).
RUN_FILES=(
    sample_config_solver.json
    sample_config_list_stage1.json
    sample_param_rocket.json
    sample_param_engine.json
    sample_sequence_of_event.json
    sample_CA.csv
    sample_thrust.csv
    sample_wind.csv
)
# Optional reference CSVs (used when the corresponding "Enable ... File" is set true).
EXTRA_FILES=(
    sample_CNa.csv
    sample_Xcp.csv
    sample_MOI.csv
    sample_Ixy.csv
    sample_Ixz.csv
    sample_Iyz.csv
)

# --- Build both targets ---
rm -rf "${BUILD_WIN}" "${BUILD_LINUX}"

cmake -B "${BUILD_WIN}" \
    -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64-x86_64.cmake \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_WIN}" --parallel

cmake -B "${BUILD_LINUX}" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_LINUX}" --parallel

# --- Assemble a platform package: $1 = os label, $2 = binary path, $3 = run cmd ---
make_package() {
    local os_label="$1"
    local binary_path="$2"
    local run_cmd="$3"
    local pkg_dir="ForRocket_v${FORROCKET_VER_DOT}_${os_label}_${CURRENT_DATE}"

    rm -rf "${pkg_dir}"
    mkdir "${pkg_dir}"

    cp "${binary_path}" "${pkg_dir}/"
    for f in "${RUN_FILES[@]}" "${EXTRA_FILES[@]}"; do
        cp "examples/${f}" "${pkg_dir}/"
    done

    # Smoke test: run the bundled sample inside the package dir before zipping
    # (Linux binary only — the Windows exe cannot run on this host).
    if [ "${os_label}" = "linux" ]; then
        (
            cd "${pkg_dir}"
            ./ForRocket sample_config_solver.json -q
            test -f sample_stage1_flight_log.csv
        )
        rm -f "${pkg_dir}"/*_flight_log.csv "${pkg_dir}"/*_log.csv
        echo "Smoke test passed: ${os_label}"
    fi

    cat > "${pkg_dir}/README.txt" <<EOF
ForRocket v${FORROCKET_VER_DOT} (${os_label})

Run the bundled sample case:
    ${run_cmd}

Output: sample_stage1_flight_log.csv is written to the current directory.

Files:
  sample_config_solver.json        entry point (run this)
  sample_config_list_stage1.json   stage-1 file list
  sample_param_rocket.json         airframe / aero parameters
  sample_param_engine.json         engine / nozzle parameters
  sample_sequence_of_event.json    flight sequence + solver tolerance
  sample_CA.csv / sample_thrust.csv / sample_wind.csv   table inputs
  sample_CNa/Xcp/MOI/Ixy/Ixz/Iyz.csv   optional table inputs (enable in JSON)
EOF

    zip -r "ForRocket_v${FORROCKET_VER_US}_${os_label}_${CURRENT_DATE}.zip" "${pkg_dir}/"
    rm -rf "${pkg_dir}"
}

make_package "win"   "${BUILD_WIN}/ForRocket.exe" "ForRocket.exe sample_config_solver.json"
make_package "linux" "${BUILD_LINUX}/ForRocket"   "./ForRocket sample_config_solver.json"

# --- Cleanup ---
rm -rf "${BUILD_WIN}" "${BUILD_LINUX}"

echo "Done:"
ls -1 ForRocket_v${FORROCKET_VER_US}_*_${CURRENT_DATE}.zip
