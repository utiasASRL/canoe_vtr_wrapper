# Assumes that ROOTDIR is set and pointing to root directory
cd $ROOTDIR
virtualenv venv
touch venv/COLCON_IGNORE
source venv/bin/activate

pip install -r $ROOTDIR/requirements.txt

cd $ROOTDIR
