# Assumes that ROOTDIR is set and pointing to canoe_vtr_wrapper root directory

# Rebuild vtr3
cd $ROOTDIR
source scripts/build_vtr3.sh

# Rebuild canoe_vtr_wrapper
cd $ROOTDIR
source scripts/build_canoe_vtr_wrapper.sh