#!/usr/bin/bash

# relative paths to work with
LOCOSIM_WORLDS="$LOCOSIM_DIR/ros_impedance_controller/worlds"
PROJECT="$HOME/ros_ws/src/robotics-project"

# models paths
MODELS_SOURCE="$PROJECT/models"
MODELS_DESTINATION="$LOCOSIM_WORLDS/models"

# world path
WORLD_NAME="robotics_project"
LOCAL_WORLD="$PROJECT/src/world/tmp.world"
LOCOSIM_WORLD="$LOCOSIM_WORLDS/$WORLD_NAME.world"

# admitted Gazebo colors (in fact defined as materials) that we assign to each block
colors=('Red' 'Green' 'Blue' 'Yellow' 'Purple' 'Orange' 'Grey' 'White' 'Indigo' 'RedBright' 'Turquoise')
colors=( $(shuf -e "${colors[@]}") )

# ------------------- Managing the blocks' instances ------------------- 
# moving tmp.world inside locosim
mv ${LOCAL_WORLD} ${LOCOSIM_WORLD}

# -------------------- Managing the blocks' models -------------------- 
i=0
# Saving the models inside locosim, before each save we assign a random color to each one
for block in $(ls $MODELS_SOURCE)
do
    # substitute the default color (Grey) with a random one
    sed -i -e "s/Grey/${colors[$i]}/g" $MODELS_SOURCE/$block/model.sdf
    # insert the block inside the robot's folder
    cp -r $MODELS_SOURCE/$block $MODELS_DESTINATION
    # reset the color to the default one
    sed -i -e "s/${colors[$i]}/Grey/g" $MODELS_SOURCE/$block/model.sdf
    # iterate $i inside $colors
    i=$(((i+1)%${#colors[@]}))
done