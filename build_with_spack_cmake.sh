#!/bin/bash

set -e

# Gets the directory location of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

function prereqs() {
    echo "Before running this script, you must follow the directions here:"
    echo "https://github.com/nicspack/nicspack/"
}

# Tests for spack existence
if ! which spack; then
    prereqs
    exit -1
fi

# Parses command line arguments
SPACK_TARGET=$(spack spec zlib | grep arch | tr '-' ' ' | awk '{print $NF}')
CLEAN=
function help() {
    echo "usage: $0 <options>"
    echo "  -h,--help              - show this message and exit"
    echo "  -t=,--target=<TARGET>  - set the spack target architecture"
    echo "  -c,--clean             - clean before building"
}
for i in "$@"; do
    case $i in
        -h|--help)
            help
            exit 0
            ;;
        -t=*|--target=*)
            SPACK_TARGET="${i#*=}"
            shift
            ;;
        -c|--clean)
            CLEAN="YES"
            shift
            ;;
        *)
            echo "unknown argument: $i"
            help
            exit -1
    esac
done
echo "SPACK_TARGET=${SPACK_TARGET}"
echo "CLEAN=${CLEAN}"

# Ensure dependencies are installed
BRANCH=cmake
BUILD_SPEC="libdes@${BRANCH} target=${SPACK_TARGET}"
echo "BUILD_SPEC=${BUILD_SPEC}"
# Ensures all dependencies are installed
if ! spack install --only dependencies ${BUILD_SPEC}; then
    prereqs
    exit -1
fi

# Saves the full spec to a tmp file for later parsing
spack spec ${BUILD_SPEC} > /tmp/libdes_spec_${BRANCH}

# Extracts the exact spec of a dependency then returns the install directory
function location() {
    local spec=$(grep "\^${1}@" /tmp/libdes_spec_${BRANCH} | sed 's/\s.*^//g')
    spack location -i ${spec}
}

function add_to_install_rpath() {
    if [[ ";$INSTALL_RPATH;" != *";$1;"* ]]; then
        INSTALL_RPATH="$1;${INSTALL_RPATH}"
    fi
}

function add_to_prefix_path() {
    if [[ ";$PREFIX_PATH;" != *";$1;"* ]]; then
        PREFIX_PATH="$1;${PREFIX_PATH}"
    fi
}

BUILD_DIR=${SCRIPT_DIR}/cmake-build
INSTALL_DIR=${SCRIPT_DIR}/cmake-local

INSTALL_RPATH="${INSTALL_DIR}/lib;${INSTALL_DIR}/lib64;"
add_to_install_rpath $(location numactl)/lib
add_to_install_rpath $(location zlib)/lib
add_to_install_rpath $(location libprim)/lib
add_to_install_rpath $(location librnd)/lib
echo "INSTALL_RPATH:"
echo "$INSTALL_RPATH"
echo ""

PREFIX_PATH=""
add_to_prefix_path $(location numactl)
add_to_prefix_path $(location zlib)
add_to_prefix_path $(location libprim)
add_to_prefix_path $(location librnd)
echo "PREFIX_PATH:"
echo "$PREFIX_PATH"
echo ""

CMAKE_DIR=$(spack location -i cmake)
CMAKE=${CMAKE_DIR}/bin/cmake

if [[ ! -z "${CLEAN}" ]]; then
    echo "cleaning build and install dirs"
    rm -rf ${BUILD_DIR} ${INSTALL_DIR}
fi

mkdir -p ${BUILD_DIR}
cd ${BUILD_DIR}
${CMAKE} \
    -G 'Unix Makefiles' \
    -DCMAKE_INSTALL_PREFIX:STRING=${INSTALL_DIR} \
    -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo \
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF \
    -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH:BOOL=OFF \
    -DCMAKE_INSTALL_RPATH:STRING=${INSTALL_RPATH} \
    -DCMAKE_PREFIX_PATH:STRING=${PREFIX_PATH} \
    ..
make -j $(nproc) all
make install

echo ""
echo "Build successful :)"
echo ""

