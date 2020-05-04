#!/bin/sh

usage(){
    echo "usage:"
    echo "you need to be in the \"script\" folder"
    echo "./exec.sh [-g] simu|moni|super"
    echo "-g : launch with gdb"
}  
curr_path=`pwd`
curr_dir=`basename $curr_path`

if [ $curr_dir != "scripts" -o $# -eq 0 ]; then
    usage
else
    if [ $# -eq 1 ]; then
        if [ $1 = "super" ]; then
            cd ../software/raspberry/superviseur-robot/dist/Debug__PC_/GNU-Linux/
            sudo ./superviseur-robot
        elif [ $1 = "simu" ]; then
            cd ../software/simulateur/dist/Debug/GNU-Linux/
            sudo ./simulateur
        elif [ $1 = "moni" ]; then
            cd ../software/monitor/monitor/
            sudo ./monitor
        else
            usage
        fi
    elif [ $# -eq 2 ]; then 
        if [ $1 = "-g" ]; then
            if [ $2 = "super" ]; then
            cd ../software/raspberry/superviseur-robot/dist/Debug__PC_/GNU-Linux/
            sudo  gdb superviseur-robot
        elif [ $2 = "simu" ]; then
            cd ../software/simulateur/dist/Debug/GNU-Linux/
            sudo gdb simulateur
        elif [ $2 = "moni" ]; then
            cd ../software/monitor/monitor/
            sudo gdb monitor
        else
            usage
        fi
        fi
    else
        usage
    fi
        
fi
