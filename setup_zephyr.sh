#!/usr/bin/env bash
#
# Setup Zephyr workspace for simple_umbreon_zephyr firmware.
#
# Usage:
#   ./setup_zephyr.sh                       # full setup (default ~/zephyrproject-v4.3)
#   ./setup_zephyr.sh ~/my/workspace        # custom workspace path
#   ./setup_zephyr.sh --version v4.3.0      # specific Zephyr version
#   ./setup_zephyr.sh --sdk-only            # only install Zephyr SDK
#
set -euo pipefail

# ── Configuration ────────────────────────────────────────────────────────────
ZEPHYR_VERSION="v4.4.0"
ZEPHYR_SDK_VERSION="1.0.0"
ZEPHYR_SDK_INSTALL_DIR="${HOME}/zephyr-sdk-${ZEPHYR_SDK_VERSION}"
ZEPHYR_DIR="${HOME}/zephyrproject-v4.4"

ZEPHYR_PATCH_URL="https://github.com/zephyrproject-rtos/zephyr/commit/5d36e85b99a.patch"
HAL_PATCH_URL="https://github.com/zephyrproject-rtos/hal_rpi_pico/commit/5d7744c.patch"

SDK_ONLY=false
SKIP_SDK=false

# ── Parse arguments ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --sdk-only)    SDK_ONLY=true; shift ;;
        --skip-sdk)    SKIP_SDK=true; shift ;;
        --version)     ZEPHYR_VERSION="$2"; shift 2 ;;
        --help|-h)
            cat <<HELP
Usage: $0 [OPTIONS] [ZEPHYR_DIR]

Sets up a Zephyr v4.3 environment for Pico 2 (RP2350A).

Options:
  --version VER  Zephyr version tag (default: ${ZEPHYR_VERSION})
  --sdk-only     Only install Zephyr SDK
  --skip-sdk     Skip SDK installation (if already installed)
  ZEPHYR_DIR     Workspace path (default: ${ZEPHYR_DIR})
HELP
            exit 0
            ;;
        *)  ZEPHYR_DIR="$1"; shift ;;
    esac
done

ZEPHYR_DIR="${ZEPHYR_DIR/#\~/$HOME}"

# ── Helpers ──────────────────────────────────────────────────────────────────
info()  { echo -e "\033[1;32m[INFO]\033[0m $*"; }
warn()  { echo -e "\033[1;33m[WARN]\033[0m $*"; }
error() { echo -e "\033[1;31m[ERROR]\033[0m $*"; exit 1; }

needs_patches() {
    local ver="${ZEPHYR_VERSION#v}"
    local major="${ver%%.*}"
    local minor="${ver#*.}"; minor="${minor%%.*}"
    [ "$major" -lt 4 ] || { [ "$major" -eq 4 ] && [ "$minor" -lt 3 ]; }
}

# ── Step 1: System dependencies ─────────────────────────────────────────────
install_system_deps() {
    info "Checking system dependencies..."
    local missing=()
    for cmd in git cmake ninja python3 pip3 wget dtc ccache; do
        command -v "$cmd" >/dev/null 2>&1 || missing+=("$cmd")
    done

    if [ ${#missing[@]} -gt 0 ]; then
        info "Installing missing packages: ${missing[*]}"
        if command -v apt-get >/dev/null 2>&1; then
            sudo apt-get update -qq
            sudo apt-get install -y -qq \
                git cmake ninja-build python3 python3-pip python3-venv \
                wget device-tree-compiler ccache dfu-util openocd picocom
        elif command -v dnf >/dev/null 2>&1; then
            sudo dnf install -y \
                git cmake ninja-build python3 python3-pip \
                wget dtc ccache dfu-util openocd picocom
        elif command -v pacman >/dev/null 2>&1; then
            sudo pacman -S --noconfirm \
                git cmake ninja python python-pip \
                wget dtc ccache dfu-util openocd picocom
        else
            error "Cannot install packages automatically. Please install: ${missing[*]}"
        fi
    else
        info "All system dependencies present"
    fi
}

# ── Step 2: Zephyr SDK ──────────────────────────────────────────────────────
install_sdk() {
    if [ -d "${ZEPHYR_SDK_INSTALL_DIR}" ]; then
        info "Zephyr SDK ${ZEPHYR_SDK_VERSION} already installed at ${ZEPHYR_SDK_INSTALL_DIR}"
        return
    fi

    info "Installing Zephyr SDK ${ZEPHYR_SDK_VERSION}..."
    local sdk_url="https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-$(uname -m)_minimal.tar.xz"
    local tmp_dir
    tmp_dir=$(mktemp -d)

    wget -q --show-progress -O "${tmp_dir}/sdk.tar.xz" "${sdk_url}" \
        || error "Failed to download SDK from ${sdk_url}"
    tar xf "${tmp_dir}/sdk.tar.xz" -C "${HOME}/"
    rm -rf "${tmp_dir}"

    cd "${ZEPHYR_SDK_INSTALL_DIR}"
    ./setup.sh -t arm-zephyr-eabi -c
    info "Zephyr SDK installed at ${ZEPHYR_SDK_INSTALL_DIR}"
}

# ── Step 3: West workspace ──────────────────────────────────────────────────
init_workspace() {
    if [ -d "${ZEPHYR_DIR}/.west" ]; then
        info "Workspace already exists at ${ZEPHYR_DIR}, skipping init"
    else
        info "Initializing Zephyr ${ZEPHYR_VERSION} workspace at ${ZEPHYR_DIR}..."
        pip3 install --quiet --user west 2>/dev/null || true
        west init -m https://github.com/zephyrproject-rtos/zephyr \
            --mr "${ZEPHYR_VERSION}" "${ZEPHYR_DIR}"
        info "Running west update (this may take 5-15 minutes)..."
        cd "${ZEPHYR_DIR}"
        west update --narrow -o=--depth=1
    fi
}

# ── Step 4: Python venv ─────────────────────────────────────────────────────
setup_venv() {
    if [ -d "${ZEPHYR_DIR}/.venv" ]; then
        info "Python venv already exists, skipping creation"
    else
        info "Creating Python virtual environment..."
        python3 -m venv "${ZEPHYR_DIR}/.venv"
    fi

    info "Installing Python dependencies..."
    # shellcheck disable=SC1091
    source "${ZEPHYR_DIR}/.venv/bin/activate"
    pip install --quiet west
    pip install --quiet -r "${ZEPHYR_DIR}/zephyr/scripts/requirements.txt"
}

# ── Step 5: RP2350 flash patches (only for Zephyr < v4.3) ───────────────────
apply_patches() {
    if ! needs_patches; then
        info "Zephyr ${ZEPHYR_VERSION} includes RP2350 flash support, no patches needed"
        return
    fi

    info "Zephyr ${ZEPHYR_VERSION} requires RP2350 flash patches"

    apply_patch() {
        local repo_dir="$1"
        local patch_url="$2"
        local label="$3"

        cd "${repo_dir}"
        if curl -sfL "${patch_url}" | git apply --check 2>/dev/null; then
            info "Applying ${label}..."
            curl -sfL "${patch_url}" | git apply -v
        else
            info "${label} — already applied, skipping"
        fi
    }

    apply_patch "${ZEPHYR_DIR}/zephyr" \
        "${ZEPHYR_PATCH_URL}" \
        "Zephyr RP2350 QMI flash controller (5d36e85b99a)"

    apply_patch "${ZEPHYR_DIR}/modules/hal/rpi_pico" \
        "${HAL_PATCH_URL}" \
        "HAL flash_write_partial (5d7744c)"
}

# ── Main ─────────────────────────────────────────────────────────────────────

if [ "$SDK_ONLY" = true ]; then
    install_system_deps
    install_sdk
    echo ""
    info "SDK-only setup complete!"
    exit 0
fi

install_system_deps

if [ "$SKIP_SDK" = false ]; then
    install_sdk
fi

init_workspace
setup_venv
apply_patches

# ── Summary ──────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo ""
info "=========================================="
info "  Setup complete!"
info "=========================================="
info "Workspace : ${ZEPHYR_DIR}"
info "Zephyr    : ${ZEPHYR_VERSION}"
info "SDK       : ${ZEPHYR_SDK_INSTALL_DIR}"
echo ""
echo "Build firmware:"
echo "  make build"
echo ""
echo "Flash (hold BOOTSEL, connect USB):"
echo "  make flash"
echo ""
echo "Monitor serial console:"
echo "  make monitor"
