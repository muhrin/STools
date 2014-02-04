#!/bin/bash

export SEED=graphene
export REMOTE_HOST=muhrincp@login.hector.ac.uk
export REMOTE_WORK_DIR=work/runs/remote
export JOB_FILE=graphene.job

castep_qsub_optimiser $1 $2

exit $?
