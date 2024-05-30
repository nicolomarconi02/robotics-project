SHELL:=/bin/bash

WORKSPACE_PATH=~/ros_ws
PROJECT_NAME=robotics_project
BUILD_COMMAND=catkin_make
INCLUDE_SERVICE_PATH_ORIGIN=${WORKSPACE_PATH}/devel/include/${PROJECT_NAME}/
INCLUDE_SERVICE_PATH_DESTINATION=generated/${PROJECT_NAME}/
LOCOSIM_PATH=${WORKSPACE_PATH}/src/locosim

TOTAL_SRV_MSG_FILES=$(shell echo $$(( "$(shell ls srv | wc -l) * 3 + $(shell ls msg | wc -l)" )))
GENERATED_ROBOTICS_PROJECT_FILES := $(shell echo $$(( "$(shell ls ${INCLUDE_SERVICE_PATH_DESTINATION} | wc -l)" )))

build_package:
	@${BUILD_COMMAND} -C ${WORKSPACE_PATH} --pkg ${PROJECT_NAME}
	@make manage_services

build_project:
	@${BUILD_COMMAND} -C ${WORKSPACE_PATH}
	@make manage_services

manage_services:
	@if [ $(TOTAL_SRV_MSG_FILES) -ne $(GENERATED_ROBOTICS_PROJECT_FILES) ]; then make import_services; fi;

import_services:
	@find ${INCLUDE_SERVICE_PATH_ORIGIN} -name "MovementHandler*.h" -exec cp '{}' ${INCLUDE_SERVICE_PATH_DESTINATION} \;
	@find ${INCLUDE_SERVICE_PATH_ORIGIN} -name "Path.h" -exec cp '{}' ${INCLUDE_SERVICE_PATH_DESTINATION} \;

run_movement_handler:
	@( cd ${WORKSPACE_PATH} && source devel/setup.bash && rosrun ${PROJECT_NAME} movement_handler )

run_vision:
	@( cd ${WORKSPACE_PATH} && source devel/setup.bash && rosrun ${PROJECT_NAME} vision.py )

run_client:
	@( cd ${WORKSPACE_PATH} && source devel/setup.bash && rosrun ${PROJECT_NAME} main )

run_robot:
	@( cd ${WORKSPACE_PATH} && source devel/setup.bash && python3 -i ${LOCOSIM_PATH}/robot_control/base_controllers/ur5_generic.py )

see_graph:
	@rosrun rqt_graph rqt_graph


.PHONY:
	build_package
	build_project
	manage_services
	import_services
	run_movement_handler
	run_client
	run_robot