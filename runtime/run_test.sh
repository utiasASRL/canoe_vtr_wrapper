## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: bash run_test.sh localization radar canoe-2025-08-20-16-07 canoe-2025-08-21-11-31

# Get arguments
MODE=$1         # [odometry, localization]
SENSOR=$2       # [radar, lidar, radar_lidar]
ODO_INPUT=$3    # CANOE sequence
LOC_INPUT=$4    # CANOE sequence, not used if mode=odometry

# Check validity of inputs
if [ "$MODE" != "odometry" ] && [ "$MODE" != "localization" ]; then
    echo "Error: MODE must be either 'odometry' or 'localization'"
    exit 1
fi
if [ "$SENSOR" != "radar" ] && [ "$SENSOR" != "lidar" ] && [ "$SENSOR" != "radar_lidar" ] && [ "$SENSOR" != "aeva" ]; then
    echo "Error: SENSOR must be either 'radar', 'lidar', 'radar_lidar', or 'aeva'"
    exit 1
fi
if [ -z "$ODO_INPUT" ]; then
    echo "Error: ODO_INPUT (canoe sequence) must be provided"
    exit 1
fi
if [ "$MODE" = "localization" ] && [ -z "$LOC_INPUT" ]; then
    echo "Error: LOC_INPUT (canoe sequence) must be provided if mode is 'localization'"
    exit 1
fi

# Set results subfolder, VTRRESULT is set in setup_container.sh
export VTRRRESULT=${VTRRESULT}/${SENSOR}
mkdir -p ${VTRRRESULT}

# Load in param file based on sensor
PARAM_FILE=${VTRRROOT}/runtime/config/${SENSOR}_config.yaml
echo "Using parameter file ${PARAM_FILE}"

# Save param file
SAVE_CONFIG=${SENSOR}_${MODE}_config.yaml
mkdir -p ${VTRRRESULT}/${ODO_INPUT}
cp ${PARAM_FILE} ${VTRRRESULT}/${ODO_INPUT}/${SAVE_CONFIG}

# Call corresponding script from canoe_vtr_wrapper
if [ "$1" = "odometry" ]; then
    bash ${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_odometry.sh ${ODO_INPUT} ${PARAM_FILE}
else
    bash ${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_localization.sh ${ODO_INPUT} ${LOC_INPUT} ${PARAM_FILE}
fi
