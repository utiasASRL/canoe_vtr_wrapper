## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT
## example usage: test_odometry.sh canoe-2025-08-20-16-07

#set -e

# Get arguments
ODO_INPUT=$1

# Log
echo "Evaluating odometry of sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/src/install/setup.bash
source ${VTRROOT}/venv/bin/activate

#   - dump odometry result to canoe expected format (txt file)
python ${VTRRROOT}/src/vtr_testing_lidar/script/canoe_generate_odometry_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT} --velocity
#   - evaluate the result using the evaluation script
python -m pycanoe.eval.odometry --gt ${VTRRDATA} --pred ${VTRRRESULT}/${ODO_INPUT}/odometry_result --velocity ${VTRRRESULT}/${ODO_INPUT}/odometry_vel_result --align-timestamps
