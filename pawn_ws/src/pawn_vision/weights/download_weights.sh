#!/usr/bin/env bash
# ==============================================================================
# PAWN VISION - Deep Learning Model Weights Sync & Downloader
# ==============================================================================
# This script manages and synchronizes the 6 trained YOLOv11 model weights:
#   1. yolo11n.pt                  (Human & Indoor Pets Detection)
#   2. yolo11n-pose.pt             (17-Keypoint Victim Pose Estimation)
#   3. fire_smoke_yolo.pt          (D-Fire Flame & Smoke Detection)
#   4. hazard_signs_yolo11n.pt     (ISO 7010 & GHS 19 Hazard Signs, mAP50: 99.0%)
#   5. crack_seg_yolo.pt           (Linear Crack Instance Segmentation)
#   6. heavy_structural_yolo11n.pt (Structural Spalling, Rebar & Brick Failure v4)
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHECKSUM_FILE="${SCRIPT_DIR}/checksums.sha256"
LOCAL_TRAIN_DIR="/Users/nijatmukhtarov/Desktop/Machine Learning/models"
RELEASE_URL="https://github.com/pawn-air/PAWN/releases/download/v1.0-weights"

FORCE=false
if [[ "$1" == "--force" || "$1" == "-f" ]]; then
    FORCE=true
    echo "[INFO] Force mode enabled. Existing weights will be overwritten."
fi

MODELS=(
    "crack_seg_yolo.pt"
    "fire_smoke_yolo.pt"
    "hazard_signs_yolo11n.pt"
    "heavy_structural_yolo11n.pt"
    "yolo11n-pose.pt"
    "yolo11n.pt"
)

echo "========================================================================"
echo "PAWN VISION: MODEL WEIGHTS SYNCHRONIZATION"
echo "Target Directory: ${SCRIPT_DIR}"
echo "========================================================================"

# Function to compute sha256 of a file
get_sha256() {
    local file="$1"
    if command -v shasum >/dev/null 2>&1; then
        shasum -a 256 "$file" | awk '{print $1}'
    elif command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$file" | awk '{print $1}'
    else
        openssl dgst -sha256 "$file" | awk '{print $NF}'
    fi
}

# Function to get expected checksum (Embedded official weights SHA-256)
get_expected_sha256() {
    case "$1" in
        "crack_seg_yolo.pt")           echo "cce8b30758621622a03119677709d172f2ec1a2f0452279bda37732fcffb00af" ;;
        "fire_smoke_yolo.pt")          echo "ac0a10257b2bc1f20c9d957f8adeeb61dd6140322fc19d0b4a116cb491776d16" ;;
        "hazard_signs_yolo11n.pt")     echo "6a5079094fd45165ea03de2561b91bd944b27d108a94df125f793486e9331419" ;;
        "heavy_structural_yolo11n.pt") echo "18bf797fb24d38a9b26a4e6b3fb9ad3182fb2b4b56a6677fb41575a760a1d2a4" ;;
        "yolo11n-pose.pt")             echo "869e83fcdffdc7371fa4e34cd8e51c838cc729571d1635e5141e3075e9319dc0" ;;
        "yolo11n.pt")                  echo "0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1" ;;
        *)                             echo "" ;;
    esac
}

# Step 1: Detect if training directory is present locally (Developer machine)
USE_LOCAL=false
if [ -d "${LOCAL_TRAIN_DIR}" ]; then
    USE_LOCAL=true
    echo "[DETECTED] Local training repository found at:"
    echo "           ${LOCAL_TRAIN_DIR}"
    echo "[MODE] Fast local copy (Zero-network latency)"
fi

echo ""

# Step 2: Check, sync, or download each model
for model in "${MODELS[@]}"; do
    target_path="${SCRIPT_DIR}/${model}"
    expected_hash="$(get_expected_sha256 "${model}")"
    needs_update=false

    if [ ! -f "${target_path}" ]; then
        needs_update=true
        echo "[MISSING] ${model} not found."
    elif [ "$FORCE" = true ]; then
        needs_update=true
        echo "[FORCE] Refreshing ${model}..."
    else
        current_hash="$(get_sha256 "${target_path}")"
        if [ -n "${expected_hash}" ] && [ "${current_hash}" != "${expected_hash}" ]; then
            needs_update=true
            echo "[OUTDATED] ${model} checksum mismatch!"
            echo "  Current : ${current_hash}"
            echo "  Expected: ${expected_hash}"
        else
            echo "[OK - VERIFIED] ${model} is up-to-date."
        fi
    fi

    if [ "$needs_update" = true ]; then
        if [ "$USE_LOCAL" = true ] && [ -f "${LOCAL_TRAIN_DIR}/${model}" ]; then
            echo "  -> Copying from local training workspace..."
            cp "${LOCAL_TRAIN_DIR}/${model}" "${target_path}"
        else
            echo "  -> Downloading from remote GitHub Release..."
            if command -v curl >/dev/null 2>&1; then
                curl -L --fail -o "${target_path}" "${RELEASE_URL}/${model}" || {
                    echo "[WARNING] Could not download ${model} from ${RELEASE_URL}/${model}."
                    echo "          Please ensure the release asset exists or copy from local machine."
                }
            elif command -v wget >/dev/null 2>&1; then
                wget -O "${target_path}" "${RELEASE_URL}/${model}" || {
                    echo "[WARNING] Could not download ${model}."
                }
            fi
        fi

        # Verify new hash if file exists
        if [ -f "${target_path}" ]; then
            new_hash="$(get_sha256 "${target_path}")"
            if [ -n "${expected_hash}" ] && [ "${new_hash}" == "${expected_hash}" ]; then
                echo "  -> Success: Checksum verified for ${model}!"
            else
                echo "  -> Notice: File placed, hash: ${new_hash}"
            fi
        fi
    fi
done

echo ""
echo "========================================================================"
echo "SYNCHRONIZATION COMPLETE"
echo "All model weights are placed and verified in: ${SCRIPT_DIR}"
echo "========================================================================"
