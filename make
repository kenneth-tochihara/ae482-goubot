#!/bin/bash
cwd=$(pwd)

found='false'
catkin_folder=''
for entry in `ls ~`; do
    if [[ $entry == *"catkin"* ]]
    then
        found='true'
        catkin_folder="$entry"
        echo "$catkin_folder"
    fi
done

if $found
then
    rm -r ~/"$catkin_folder"/src/lab2andDriver/
    cp -R ~/ae482_goubot/lab2andDriver ~/"$catkin_folder"/src
    echo "updating directory"
else
    mkdir ~/catkin
    mkdir ~/catkin/src
    echo ~/catkin/src
    cd ~/catkin/src
    catkin_init_workspace
    cd "$cwd"
    cp -R lab2andDriver ~/catkin/src
    cd ~/catkin
    catkin_make
    echo "new directory being made"
fi
