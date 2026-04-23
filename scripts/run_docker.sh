# Set ROOTDIR to the root directory of the project
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
export ROOTDIR=$(dirname "$SCRIPT_DIR")

container_state=$(docker inspect -f '{{.State.Running}}' canoe_wrapper_$(whoami) 2>/dev/null)

if [ "$container_state" = "true" ]
then
	echo 'Container already running, joining it now.'
	docker exec -it canoe_wrapper_$(whoami) /entrypoint.sh
else
	echo 'New container run initialized.'
	docker run -it --rm --name canoe_wrapper_$(whoami) \
	--privileged \
	--network=host \
	--ipc=host \
	-e DISPLAY=$DISPLAY \
	-e ROOTDIR=$ROOTDIR \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v ${HOME}/.Xauthority:${HOME}/.Xauthority:rw \
	-v $ROOTDIR:$ROOTDIR:rw \
	-w $ROOTDIR canoe_wrapper_$(whoami)
fi
cd $ROOTDIR