## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: bash run_parallel_test.sh localization radar

# USER INPUT: SELECT THE SEQUENCES YOU WISH TO TEST IN PARALLEL FOR EITHER MODE
if [ "$1" = "odometry" ]; then
    # Odometry sequences, SET THESE YOURSELF
    SEQUENCES=(
    "canoe-2025-08-20-16-07" # d1 short
    "canoe-2025-08-20-18-07" # d1 long
    "canoe-2025-08-21-11-31" # d2 short
    "canoe-2025-08-21-14-08" # d2 long
    "canoe-2025-08-28-14-13" # reservoir doppler 1
    "canoe-2025-08-28-14-50" # reservoir doppler 2
    "canoe-2025-08-28-16-15" # reservoir 3
    )
else
    # Odometry reference for localization, SET THIS YOURSELF
    # REFERENCE="canoe-2025-08-20-16-07" # d1 short
    # REFERENCE="canoe-2025-08-20-18-07" # d1 long
    # REFERENCE="canoe-2025-08-21-11-31" # d2 short
    # REFERENCE="canoe-2025-08-21-14-08" # d2 long
    # REFERENCE="canoe-2025-08-28-14-13" # reservoir doppler 1
    REFERENCE="canoe-2025-08-28-14-50" # reservoir doppler 2
    # REFERENCE="canoe-2025-08-28-16-15" # reservoir 3
fi

# Get arguments
MODE=$1         # [odometry, localization]
SENSOR=$2       # [radar, lidar, radar_lidar]

# Set results subfolder, VTRRESULT is set in setup_container.sh
export VTRRRESULT=${VTRRESULT}/${SENSOR}

EVAL_SCRIPT="${ROOTDIR}/runtime/run_eval.sh"

# Evaluate results from tests
if [ "$1" = "odometry" ]; then
    for seq in ${SEQUENCES[@]}; do
    echo "Executing command: bash $EVAL_SCRIPT $1 $2 $seq"
    bash $EVAL_SCRIPT $1 $2 $seq
    done
else
    echo "Executing command: bash $EVAL_SCRIPT $1 $2 $REFERENCE"
    bash $EVAL_SCRIPT $1 $2 $REFERENCE
fi
