#!/bin/sh 

for file in `ls`;
do
    tar cvf "$file.tar.gz" $file
done
