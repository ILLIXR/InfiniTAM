seq=$1
traj=$2

# sequence=('large_loop_1' 'mannequin_1' 'mannequin_face_2' 'plant_5' 'plant_scene_1' 'repetitive' 'sofa_1' 'table_3')
# 
# echo "Select a sequence from the following candidates:"
# for element in ${sequence[@]}
# do
# 	echo $element
# done
# echo ""
# 
# if [ -z "$seq" ]
# then
# 	echo "[Error] !!! \$seq is empty, please set the sequence name !!!"
# fi


# Using converted: PPM PGM
# ../../build/Apps/InfiniTAM_cli/InfiniTAM_cli ../calibration/VCU_RVI.txt /home/bytian/Documents/Datasets/VCU_RVI/Converted/$seq/$traj.txt

# Using parsed: PNG
../../build/Apps/InfiniTAM_cli/InfiniTAM_cli ../calibration/VCU_RVI.txt /home/bytian/Documents/Datasets/VCU_RVI/Parsed/$seq/associated/$traj.txt
