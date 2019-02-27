#!/bin/bash

information_communication='Invalid'
robot_communication='Invalid'

#enter participant ID
echo "Enter Participant ID:"
read PARTICIPANT_ID
BAG_PATH=~/Bags/$PARTICIPANT_ID
mkdir -p $BAG_PATH

echo "Enter Participant Name:"
read PARTICIPANT_NAME


#select the conditions
echo "Select Robot Communication Method"
PS3='Please Enter Response: '
options=("r - representing [Greeen]" "i - inter-robot [Yellow]" "s - social [Blue]")
select opt in "${options[@]}"
do
    case $opt in
        "r - representing [Greeen]")
            robot_communication='r'
            break
            ;;
        "i - inter-robot [Yellow]")
            robot_communication="i"
            break
            ;;
        "s - social [Blue]")
            robot_communication="s"
            break
            ;;
        *) echo invalid option;;
    esac
done

#select the conditions
echo "Select Information Communication Method"
PS3='Please Enter Response: '
options=("s - silent" "e - explicit" "v - verbal")
select opt in "${options[@]}"
do
    case $opt in
        "s - silent")
            information_communication="s"
            break
            ;;
        "e - explicit")
            information_communication='e'
            break
            ;;
        "v - verbal")
            information_communication="v"
            break
            ;;
        *) echo invalid option;;
    esac
done


echo "Comfirming Settings"
COMMAND="roslaunch hri19_multirobot_transition_study run.launch interpretor_filepath:=condition-$robot_communication-$information_communication.yaml participant_name:=$PARTICIPANT_NAME participant_id:=$PARTICIPANT_ID bag_path:=$BAG_PATH"
echo -e "\x1b[0;31m $COMMAND \x1b[0m"
echo "Do you wish to run it?"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) break;;
        No ) exit;;
    esac
done

echo 'start running ...'

$COMMAND