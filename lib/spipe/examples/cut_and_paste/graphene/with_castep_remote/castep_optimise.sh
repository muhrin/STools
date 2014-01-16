#!/bin/bash

#set -x

export SEED=C
export REMOTE_HOST=muhrincp@login.hector.ac.uk
export REMOTE_WORK_DIR=work/runs/remote
export JOB_FILE=graphene.job

castep_qsub_optimiser.sh $1 $2

exit $?

