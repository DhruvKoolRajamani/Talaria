#!/bin/bash

TALARIA_WS="/media/raunak/OS/ADI_challenge/Talaria"
UPLOAD_FLAG="False"

run_setup() {

    if [[ "$1" = "-h" ]] || [[ "$1" = "--help" ]]; then
        echo "Usage: ./setup.sh [option 1] [option 2] [option 3]">&2
        echo
        echo "[option 1]                                                            "
        echo "     mbed    Runs the Talaria environment mbed with rosserial         "
        echo "  arduino    Runs the Talaria environment arduino with rosserial      "
        echo "  nucleo    Runs the Talaria environment arduino with rosserial      "
        echo 
        echo "[option 2]                                                            "
        echo "   serial    Runs the Talaria environment in the serial monitor       "
        echo 
        echo "[option 2]                                                            "
        echo "   upload    Runs the Talaria environment and uploads the environment "
        echo "             to the microcontroller                                   "
        echo
        return
    fi

    cd $TALARIA_WS
    clean

    local FRAMEWORK_NAME=$1
    local ENV_TYPE=$2
    local ARD_FRAMEWORK="arduino"
    local MBED_FRAMEWORK="mbed"
    local NUCLEO_FRAMEWORK="nucleo"
    local ENV_SERIAL="serial"
    local ENV_NUMBER=0

    if [[ -z "$3" ]]; then
        if [[ "$3" = "upload" ]]; then
            UPLOAD_FLAG="True"
        else
            UPLOAD_FLAG="False"
        fi
    fi

    if [[ $ENV_TYPE = $ENV_SERIAL ]]; then
        echo
        echo "Running serial client"
        echo

        if [[ "$FRAMEWORK_NAME" = "$MBED_FRAMEWORK" ]]; then
            PLATFORMIO_DEFAULT_ENVS="Talaria-hand-serial"
            ENV_NUMBER=4
        elif [[ "$FRAMEWORK_NAME" = "$ARD_FRAMEWORK" ]]; then
            PLATFORMIO_DEFAULT_ENVS="Talaria-hand-uno-serial"
            ENV_NUMBER=5
        elif [[ "$FRAMEWORK_NAME" = "$NUCLEO_FRAMEWORK" ]]; then
            PLATFORMIO_DEFAULT_ENVS="Talaria-hand-nucleo-serial"
            ENV_NUMBER=6
        else
            echo
            echo "Incorrect arguments provided, exiting"
            echo
            return
        fi
    else
        if [[ "$FRAMEWORK_NAME" = "$MBED_FRAMEWORK" ]]; then
            PLATFORMIO_DEFAULT_ENVS="Talaria-hand"
            ENV_NUMBER=1
        elif [[ "$FRAMEWORK_NAME" = "$ARD_FRAMEWORK" ]]; then
            PLATFORMIO_DEFAULT_ENVS="Talaria-hand-uno"
            ENV_NUMBER=2
        elif [[ "$FRAMEWORK_NAME" = "$NUCLEO_FRAMEWORK" ]]; then
            PLATFORMIO_DEFAULT_ENVS="Talaria-hand-nucleo"
            ENV_NUMBER=3
        else
            echo
            echo "Incorrect arguments provided, exiting"
            echo
            return
        fi
    fi

    if [[ -z "$ENV_TYPE" ]]; then
        echo
        echo "Running Rosserial"
        echo

        DIR_NAME=ros_lib
        local FLAG="False"

        pushd lib
        if [[ -d $DIR_NAME ]]; then
            echo "$DIR_NAME exists"
            echo "Checking if $DIR_NAME is a git repo..."
            
            pushd $DIR_NAME
            if [[ -d ".git" ]]; then
                echo "$DIR_NAME is a git repo"
                if ! git diff-index --quiet HEAD --; then
                    echo
                    echo ERROR: Please commit or stash your changes in $PWD 1>&2
                    echo
                    FLAG="False"
                    return
                else
                    echo
                    echo "checking out ros_lib to rosserial-$FRAMEWORK_NAME"
                    echo
                    git fetch
                    git checkout rosserial-$FRAMEWORK_NAME
                    git pull
                fi
                FLAG="False"
                popd
            else
                echo "$DIR_NAME is not a git repo"
                echo "Deleting $DIR_NAME and cloning updated repo"
                popd
                rm -rf $DIR_NAME
                FLAG="True"
            fi
            popd
        else
            popd
            FLAG="True"
        fi


        if [[ "$FLAG" = "True" ]]; then
            echo "Cloning repo"
            pio lib -d lib install -f https://github.com/DhruvKoolRajamani/ros_lib.git#rosserial-$FRAMEWORK_NAME
        fi
    fi
    
    build $ENV_NUMBER

}

# run_setup $1 $2

# Build
build() {

    echo "Building"
    echo

    EXTRA_ARGS=""

    if [[ "$UPLOAD_FLAG" = "True" ]]; then
        EXTRA_ARGS="-t upload"
    else
        EXTRA_ARGS=""
    fi
    
    if [[ $1 -eq 1 ]]; then
        pio init --ide vscode -b lpc1768 --env-prefix Talaria-hand
        pio run -e Talaria-hand $EXTRA_ARGS
    elif [[ $1 -eq 2 ]]; then
        pio init --ide vscode -b uno --env-prefix Talaria-hand-uno
        pio run -e Talaria-hand-uno $EXTRA_ARGS
    elif [[ $1 -eq 3 ]]; then
        pio init --ide vscode -b nucleo_f413zh --env-prefix Talaria-hand-nucleo
        pio run -e Talaria-hand-nucleo $EXTRA_ARGS
    elif [[ $1 -eq 4 ]]; then
        pio init --ide vscode -b lpc1768 --env-prefix Talaria-hand-serial
        pio run -e Talaria-hand-serial $EXTRA_ARGS
    elif [[ $1 -eq 5 ]]; then
        pio init --ide vscode -b uno --env-prefix Talaria-hand-uno-serial
        pio run -e Talaria-hand-uno-serial $EXTRA_ARGS
    elif [[ $1 -eq 6 ]]; then
        pio init --ide vscode -b nucleo_f413zh --env-prefix Talaria-hand-nucleo-serial
        pio run -e Talaria-hand-nucleo-serial $EXTRA_ARGS
    else
        echo "Exiting"
        return
    fi

}

clean() {

    cd $TALARIA_WS
    
    rm -rf .pio
    rm -rf platformio/*

    pushd lib
    rm -rf ros_lib
    popd
    
}
