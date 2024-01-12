#!/bin/bash

if [ -z "$GPSS_ROOT" ]; then
    echo "Error! You need to source meta/set_paths.bash first."
    exit 2;
fi

cp output/scheduler_input.json "${GPSS_ROOT}/src/gpss_scenario_generated/config/"
cp output/scheduler_result.json "${GPSS_ROOT}/src/gpss_scenario_generated/config/"
