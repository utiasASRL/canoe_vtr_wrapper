## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: run_parallel_test.sh localization radar

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
    REFERENCE="canoe-2025-08-28-14-50" # doppler reservoir 2

    # Localization sequences, SET THESE YOURSELF
    SEQUENCES=(
    # "canoe-2025-08-20-16-07" # d1 short
    # "canoe-2025-08-20-18-07" # d1 long
    # "canoe-2025-08-21-11-31" # d2 short
    # "canoe-2025-08-21-14-08" # d2 long
    "canoe-2025-08-28-14-13" # reservoir doppler 1
    # "canoe-2025-08-28-14-50" # reservoir doppler 2
    "canoe-2025-08-28-16-15" # reservoir 3
    )
fi

# Get arguments
MODE=$1         # [odometry, localization]
SENSOR=$2       # [radar, lidar, radar_lidar]

# Load in param file based on sensor
PARAM_FILE=${ROOTDIR}/runtime/config/${SENSOR}_config.yaml

# Set results subfolder, VTRRESULT is set in setup_container.sh
export VTRRRESULT=${VTRRESULT}/${SENSOR}
mkdir -p ${VTRRRESULT}

# maximum number of jobs running in parallel
GROUPSIZE=8

SCRIPT="${VTRRROOT}/runtime/run_test.sh"
# EVAL_SCRIPT="${VTRRROOT}/runtime/run_eval.sh"
EVAL_SCRIPT="${VTRRROOT}/runtime/run_multi_eval.sh"

declare -A pids

# Run tests in parallel
for seq in ${SEQUENCES[@]}; do
    if [ "$1" = "odometry" ]; then
        echo "Executing command: bash $SCRIPT $1 $2 $seq &>/dev/null &"
        ### command to execute
        bash $SCRIPT $1 $2 $seq &>/dev/null &
    else
        echo "Executing command: bash $SCRIPT $1 $2 $REFERENCE $seq &>/dev/null &"
        ### command to execute
        bash $SCRIPT $1 $2 $REFERENCE $seq &>/dev/null &
    fi

    pids[${seq}]=$!
    # wait for any pid to finish if reached group size
    if [[ ${#pids[@]} -ge ${GROUPSIZE} ]]; then
        wait -n
        pid_finished=$?
        seq_finished=${pids[${pid_finished}]}
        echo "Process ${seq_finished} finished with return code ${?}"
        unset pids[$?]
    fi
done

# Wait for all processes to finish
for key in ${!pids[@]}; do
  wait ${pids[${key}]}
  echo "Process ${key} finished with return code ${?}"
  unset pids[${key}]
done

# Evaluate results from tests
if [ "$1" = "odometry" ]; then
    bash $EVAL_SCRIPT $1 $2 --seq "${SEQUENCES[@]}"
    # for seq in ${SEQUENCES[@]}; do
    # echo "Executing command: bash $EVAL_SCRIPT $1 $2 $seq"
    # bash $EVAL_SCRIPT $1 $2 $seq
    # done
else
    echo "Executing command: bash $EVAL_SCRIPT $1 $2 $REFERENCE"
    bash $EVAL_SCRIPT $1 $2 $REFERENCE
fi