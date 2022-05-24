sequences=('large_loop_1' 'mannequin_1' 'mannequin_face_2' 'plant_5' 'plant_scene_1' 'repetitive' 'sofa_1' 'table_3')

seq=$1
file=$2

if [ -z $seq ] || [ -z $file ];
then
	echo "Usage: ./ETH3D.sh <sequence> <pose_file>"
	echo "<sequence>: One of the following sequences:"

	for element in ${sequences[@]}
	do
		echo -e "\t$element"
	done
	echo ""

	echo "<pose_file>: Associated pose file"
	exit 1
fi

export useICP=false
export approxDepthCheck=false
export usePrevList=true
export freqMode=constant
export frequency=30.0

# Sequence fully associated with specified pose 
../../build/Apps/InfiniTAM_cli/InfiniTAM_cli ../calibration/ETH3D.txt /home/bytian/Documents/Datasets/ETH3D/training/$seq/associated/$file.txt
