## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT
## example usage: test_localization_eval.sh canoe-2025-08-20-16-07

# Get arguments
ODO_INPUT=$1

# Log
echo "Evaluating localization to reference sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/src/install/setup.bash
source ${VTRROOT}/venv/bin/activate

#   - dump localization result to canoe expected format (txt file)
python ${VTRRROOT}/src/vtr_testing_radar/script/canoe_generate_localization_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT}
#   - evaluate the result using the evaluation script
python -m pycanoe.eval.localization --gt ${VTRRDATA} --pred ${VTRRRESULT}/${ODO_INPUT}/localization_result --ref_seq ${ODO_INPUT} --ref_sensor radar --test_sensor radar --dim 2  --plot ${VTRRRESULT}/${ODO_INPUT}/localization_result/radar-radar
