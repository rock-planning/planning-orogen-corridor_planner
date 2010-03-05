#! /bin/sh

HERE=`pwd`
cd $1
$HERE/run $3 terrain_classes.txt terrain.tif 76 71 865 285 $2

