#!/bin/sh

usage(){
    echo "usage:"
    echo "you need to be in the \"script\" folder"
    echo "./exec.sh simu|moni|super"
}  
curr_path=`pwd`
curr_dir=`basename $curr_path`

if [ $# -ne 1 -o $curr_dir != "scripts" ]; then
    usage
else
    if [ $1 = "super" ]; then
        cd ../software/raspberry/superviseur-robot/dist/Debug__PC_/GNU-Linux/
        sudo ./superviseur-robot
    elif [ $1 = "simu" ]; then
        pwd
        cd ../software/simulateur/dist/Debug/GNU-Linux/simulateur
        sudo ./simulateur
    elif [ $1 = "moni" ]; then
        cd ../software/monitor/monitor/monitor
        sudo ./monitor
    else
        usage
    fi
fi
