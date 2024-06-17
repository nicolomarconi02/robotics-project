#!/usr/bin/bash

# relative paths to work with
LOCOSIM_WORLDS="$LOCOSIM_DIR/ros_impedance_controller/worlds"
PROJECT="$HOME/ros_ws/src/robotics-project"

# models paths
MODELS_SOURCE="$PROJECT/models"
MODELS_DESTINATION="$LOCOSIM_WORLDS/models"

# world path
WORLD_NAME="robotics_project"
LOCAL_WORLD="$PROJECT/src/world/$WORLD_NAME.world"
LOCOSIM_WORLD="$LOCOSIM_WORLDS/$WORLD_NAME.world"


# Saving the blocks inside locosim
for block in $(ls $MODELS_SOURCE)
do
    if [ ! -d $MODELS_DESTINATION/$block ]
        then 
            cp -r $MODELS_SOURCE/$block $MODELS_DESTINATION/$block
    fi
done

if [ ! -f $LOCOSIM_WORLD ]
    then 
        cp ${LOCAL_WORLD} ${LOCOSIM_WORLD}
fi