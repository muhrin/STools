#!/bin/bash

#set -x

if [ "$#" -ne 4 ]
then
  echo "Usage: $0 [remote_host] [castep_seed] [job_file] [remote_dir]"
  exit 1
fi

declare -r REMOTE_HOST=$1
declare -r SEED=$2
declare -r JOB_FILE=$3
declare -r REMOTE_DIR=$4

# Interval between checking if job is finished
declare -r JOB_POLL_INTERVAL=60

declare -r CELL_FILE=${SEED}.cell
declare -r PARAM_FILE=${SEED}.param
declare -r CELL_OUT_FILE=${SEED}-out.cell

declare -r RAND_FOLDER=`strings /dev/urandom | tr -dc [:alnum:] | head -c8`
declare -r WORK_DIR=$REMOTE_DIR/$RAND_FOLDER

ssh $REMOTE_HOST mkdir -p $WORK_DIR
scp $CELL_FILE $PARAM_FILE $JOB_FILE $REMOTE_HOST:$WORK_DIR

declare -r JOBID=`ssh $REMOTE_HOST "cd $WORK_DIR; qsub $JOB_FILE"`

# Loop until the job finishes
finished=false
while ! $finished
do
  sleep $JOB_POLL_INTERVAL

  (ssh $REMOTE_HOST qstat $JOBID) < /dev/null 2>&1 | grep -q "Unknown Job Id"
  if [ "$?" -eq 0 ]
  then
    finished=true
  fi
done

scp $REMOTE_HOST:$WORK_DIR/$SEED\* . > /dev/null

# Delete the working directory
ssh $REMOTE_HOST "if [ -d \"$WORK_DIR\" ]; then rm -r $WORK_DIR; fi"


