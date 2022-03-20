seq=$1

sequence=('large_loop_1' 'mannequin_1' 'mannequin_face_2' 'plant_5' 'plant_scene_1' 'repetitive' 'sofa_1' 'table_3')

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

# Fully associated with GT
# ../../build/Apps/InfiniTAM/InfiniTAM ../calibration/ETH3D.txt /home/bytian/Documents/Datasets/ETH3D/training/$seq/data.txt /home/bytian/Documents/Datasets/ETH3D/training/$seq/data.txt

# RGB-D associated
../../build/Apps/InfiniTAM/InfiniTAM ../calibration/ETH3D.txt /home/bytian/Documents/Datasets/ETH3D/training/$seq/associated/rgbd.txt
