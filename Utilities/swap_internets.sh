#!/bin/bash

#This script will swap out one internet configuration for another

path=$1 #This is the path to a directory with an internet configuration in it

rm /etc/netplan/50-cloud-init.yaml
cp $path/50-cloud-init.yaml /etc/netplan/