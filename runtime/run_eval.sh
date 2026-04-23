## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: bash run_eval.sh localization radar canoe-2025-08-20-16-07

# Get arguments
MODE=$1         # [odometry, localization]
SENSOR=$2       # [radar, lidar, radar_lidar, aeva]
ODO_INPUT=$3    # CANOE sequence

# Set results subfolder, VTRRESULT is set in setup_container.sh
export VTRRRESULT=${VTRRESULT}/${SENSOR}

# Load in param file based on sensor
PARAM_FILE=${ROOTDIR}/runtime/config/${SENSOR}_config.yaml

# Call corresponding script from canoe_vtr_wrapper
bash ${VTRRROOT}/src/vtr_testing_${SENSOR}/script/test_${MODE}_eval.sh ${ODO_INPUT} ${PARAM_FILE}