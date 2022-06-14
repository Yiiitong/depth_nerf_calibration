#!/bin/bash

#!/bin/bash

ROSVERSION=`rosversion -d`
PACKAGES="swig"
echo "Installing $PACKAGES"
read -p "Are you sure? " -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
    sudo apt-get install $PACKAGES
fi
