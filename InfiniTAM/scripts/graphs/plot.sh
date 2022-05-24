#!/bin/sh

if [ -z $1 ] || [ -z $2 ];
then
    echo "Usage: plot.sh <graph type> <data file>"
    echo "<graph type>: one of 'bricks' or 'voxels' or 'controller' or 'execution'"
    echo "<data file>: path to csv file"
    exit 0
fi

gnuplot -c $1.plt $2
