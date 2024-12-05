#!/bin/bash

#This script will delete the oldest files in a directory (path) and keep the newest (max) files
#command line argument 1 specifies the absolute path to the directory
#command line argument 2 specifies how many files to keep in the directory

path=$1
max=$2

while (($(ls $path | wc -l) > $max))
	do
		#echo "there are more than $max files in $path"
		fileToDelete=$(ls -t $path | tail -1)
		#echo "I am going to delete $fileToDelete"
		pathToFileForDeletion=$path/$fileToDelete
		echo "Deleting $pathToFileForDeletion"
		rm $pathToFileForDeletion
done