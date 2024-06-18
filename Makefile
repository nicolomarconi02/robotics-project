SHELL:=/bin/bash

WORKSPACE_PATH=~/ros_ws
PROJECT_NAME=robotics_project
BUILD_COMMAND=catkin_make
INCLUDE_SERVICE_PATH_ORIGIN=${WORKSPACE_PATH}/devel/include/${PROJECT_NAME}/
INCLUDE_SERVICE_PATH_DESTINATION=generated/${PROJECT_NAME}/
LOCOSIM_PATH=${WORKSPACE_PATH}/src/locosim

TOTAL_SRV_MSG_FILES=$(shell echo $$(( "$(shell ls srv | wc -l) * 3 + $(shell ls msg | wc -l)" )))
GENERATED_ROBOTICS_PROJECT_FILES := $(shell echo $$(( "$(shell ls ${INCLUDE_SERVICE_PATH_DESTINATION} | wc -l)" )))

SERVICES=$(shell ls srv -1 | rev | cut -f 2- -d "." | rev)
MSGS=$(shell ls msg -1 | rev | cut -f 2- -d "." | rev)

SOURCE=source ${WORKSPACE_PATH}/devel/setup.bash

build-pkg:
	@${BUILD_COMMAND} -C ${WORKSPACE_PATH} --pkg ${PROJECT_NAME}
	@make manage_services

build-proj:
	@${BUILD_COMMAND} -C ${WORKSPACE_PATH}
	@make manage_services

manage_services:
	@if [ ! -d ${INCLUDE_SERVICE_PATH_DESTINATION} ]; then mkdir -p ${INCLUDE_SERVICE_PATH_DESTINATION}; fi;
	@if [ $(TOTAL_SRV_MSG_FILES) -ne $(GENERATED_ROBOTICS_PROJECT_FILES) ]; then make import_services; fi;

import_services:
	@if [ ! -d ${INCLUDE_SERVICE_PATH_DESTINATION} ]; then mkdir -p ${INCLUDE_SERVICE_PATH_DESTINATION}; fi;
	@for file in ${SERVICES}; do \
		find ${INCLUDE_SERVICE_PATH_ORIGIN} -name "${file}*.h" -exec cp '{}' ${INCLUDE_SERVICE_PATH_DESTINATION} \; \
		; done
	
	@for file in ${MSGS}; do \
		find ${INCLUDE_SERVICE_PATH_ORIGIN} -name "${file}.h" -exec cp '{}' ${INCLUDE_SERVICE_PATH_DESTINATION} \; \
		; done

run-movement:
	@${SOURCE} && rosrun ${PROJECT_NAME} movement_handler

run-vision: camera-rolls
	@${SOURCE} && rosrun ${PROJECT_NAME} vision.py

run-client:
	@${SOURCE} && rosrun ${PROJECT_NAME} main

run-robot: world-setup
	@${SOURCE} && rosrun ${PROJECT_NAME} ur5_generic.py

graph:
	@rosrun rqt_graph rqt_graph

world-setup: position-blocks
	@bash src/world/setup.bash

position-blocks:
	@python3 src/world/position-blocks.py

camera-rolls:
	@mkdir camera-rolls

.PHONY:
	build-pkg
	build-proj
	manage_services
	import_services
	world-setup
	position-blocks
	run-movement
	run-client
	run-robot