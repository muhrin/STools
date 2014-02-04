#!/bin/bash

# Functions to manipulate castep files

#set -x

# Get a parameter value from a .param file, arguments:
# 1 - the .param file
# 2 - the parameter name
castep_param_get_value () {
  grep -qiE $2[[:blank:]+:] $1
  if [ "$?" -eq "0" ]
  then
    grep -iE $2[[:blank:]+:] $1 | sed -r "s/$2[[:blank:]]*:[[:blank:]]*(.*)/\1/I"
  fi
}

# Set a parmeter to a value in the .param file, arguments:
# 1 - the .param file
# 2 - the parameter name
# 3 - the parameter value
castep_param_set_value () {
  grep -q -i $2 $1
  # Does the parameter already exists?
  if [ "$?" -eq "0" ]
  then
    sed -ri "s/($2[[:blank:]]*:[[:blank:]]*).*/\1$3/I" $1
  else
    echo "$2 : $3" >> $1
  fi
}
