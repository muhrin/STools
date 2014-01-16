#!/bin/bash

set -x

source castep_manip.sh

if [ "$#" -ne "2" ]
then
  echo "Usage: $0 [settings_file] [structure_file]"
  exit 1
fi

declare -r QSUB_SETTINGS="./castep_qsub.settings"

## SETTINGS ##
if [ ! -e "$QSUB_SETTINGS" ]
then
  echo "$QSUB_SETTINGS not found."
  exit 1
fi
# The folder from which this script is called should have a file called
# castep_qsub.settings
#
# It should contain the following: 
# declare -r SEED=graphene (the castep seed)
# declare -r REMOTE_HOST=muhrincp@login.hector.ac.uk (the remote host string)
# declare -r REMOTE_WORK_DIR=work/runs/remote (the remote host working directory)
# declare -r JOB_FILE=graphene.job (the submission script)
source $QSUB_SETTINGS

# INPUTS ##
declare -r SETTINGS=$1
declare -r STRUCTURE=$2

filename=$(basename "$STRUCTURE")
extension="${filename##*.}"
filename="${filename%.*}"

declare -r PARAM_FILE=${filename}.param
declare -r CELL_FILE=${filename}.cell

# Check that we have a cell file
if [ "$extension" != "cell" ]
then
  sconvert $STRUCTURE $CELL_FILE
fi

# Copy over any stub cell contents
if [ -e "${SEED}.cell" ]
then
  cat ${SEED}.cell >> $CELL_FILE
fi

cp ${SEED}.param $PARAM_FILE

# Set up the param file using the input settings
iter=$(castep_param_get_value $SETTINGS maxIter)
if [ -n "$iter" ]
then
  castep_param_set_value $PARAM_FILE GEOM_MAX_ITER $iter
fi

qsub_castep.sh $REMOTE_HOST $filename $JOB_FILE $REMOTE_WORK_DIR > /dev/null 2>&1
if [ "$?" -ne "0" ]
then
  exit 1
fi

## Did we get the castep file?
declare -r CASTEP=${filename}.castep
if [ -e "$CASTEP" ]
then
  sconvert ${filename}.castep\#last $STRUCTURE
  echo -e "\n\n#  OUTCOME" >> $SETTINGS
  echo "successful: true" >> $SETTINGS
  # Get information from the castep file
  sinfo graphene.castep\#last -f -n -i 'finalEnthalpy: $h$\nfinalInternalEnergy: $u$ \nfinalPressure: $p$ \n' >> $SETTINGS
  iters=`grep -E "finished iteration .*" $CASTEP | tail -n 1 | sed -r 's/finished iteration[[:blank:]]([[:digit:]]+)[[:blank:]]+.*/\1/'`
  if [ -n "$iters" ]
  then
    echo "finalIters: $iters" >> $SETTINGS
  fi
else
  echo "successful: false" >> $SETTINGS
  exit 1
fi

exit 0
