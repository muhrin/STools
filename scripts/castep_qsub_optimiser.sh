#!/bin/bash


declare -r SETTINGS=$1
declare -r STRUCTURE=$2

filename=$(basename "$STRUCTURE")
extension="${filename##*.}"


