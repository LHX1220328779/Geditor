#!/bin/bash

set -Eeuo pipefail

if [ "$#" -ge 1 ]; then
    make_args=("$@")
else
    make_args=(-j8)
fi
echo "编译使用参数：${make_args[*]}"

CMD_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$CMD_DIR"

# Anaconda/Miniconda packages must never participate in this build. In
# particular, mixing their Boost/Protobuf headers with Ubuntu libraries causes
# link-time ABI errors. Remove Conda entries from every compiler/CMake search
# path, not only PATH, so an activated environment cannot leak into the build.
strip_conda_entries() {
    local variable_name="$1"
    local original_value="${!variable_name-}"
    local filtered_value=""
    local entry
    local entries=()

    IFS=: read -r -a entries <<< "$original_value"
    for entry in "${entries[@]}"; do
        case "${entry,,}" in
            *anaconda*|*miniconda*|*conda*)
                ;;
            *)
                if [ -z "$filtered_value" ]; then
                    filtered_value="$entry"
                else
                    filtered_value="$filtered_value:$entry"
                fi
                ;;
        esac
    done
    printf -v "$variable_name" '%s' "$filtered_value"
    export "$variable_name"
}

for search_path in \
    PATH CMAKE_PREFIX_PATH CPATH CPLUS_INCLUDE_PATH C_INCLUDE_PATH \
    LIBRARY_PATH LD_LIBRARY_PATH PKG_CONFIG_PATH PYTHONPATH; do
    strip_conda_entries "$search_path"
done
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CE_M

audit_cache() {
    local found_conda=1
    if command -v rg >/dev/null 2>&1; then
        if rg -n -i \
            'anaconda|miniconda|/[^:;[:space:]]*conda[^:;[:space:]]*' \
            CMakeCache.txt; then
            found_conda=0
        fi
    else
        if grep -Eni \
            'anaconda|miniconda|/[^:;[:space:]]*conda[^:;[:space:]]*' \
            CMakeCache.txt; then
            found_conda=0
        fi
    fi
    if [ "$found_conda" -eq 0 ]; then
        echo "错误：CMakeCache.txt 中仍包含 Conda/Anaconda 路径，已终止构建。" >&2
        exit 1
    fi
}

configure() {
    # Repair stale dependency locations in existing build directories too.
    cmake .. \
        -U 'Qt5*_DIR' \
        -U 'Protobuf_*' \
        -U 'Boost*' \
        -U 'boost*' \
        -DCMAKE_BUILD_TYPE=Release \
        "$@"
    audit_cache
}

# minilzo
mkdir -p thirdparty/minilzo/build
cd thirdparty/minilzo/build
configure
make "${make_args[@]}"

# sqlite_wrap
cd ../../sqlite_wrap
mkdir -p build
cd build
configure
make "${make_args[@]}"

cd ../../..

# main project
mkdir -p build
cd build
configure -DENABLE_TESTS=OFF -DBoost_NO_BOOST_CMAKE=ON
make "${make_args[@]}"
