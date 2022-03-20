seq=$1

sequence=('depth_rtabmap' 'depth_elas' 'depth_3rdpt')

echo "Select a sequence from the following candidates:"
for element in ${sequence[@]}
do
	echo $element
done
echo ""

if [ -z "$seq" ]
then
	echo "[Error] !!! \$seq is empty, please set the sequence name !!!"
fi

../../build/Apps/InfiniTAM/InfiniTAM ../calibration/EuRoC.txt /home/bytian/Documents/Datasets/EuRoC/$seq.txt /home/bytian/Documents/Datasets/EuRoC/$seq.txt
