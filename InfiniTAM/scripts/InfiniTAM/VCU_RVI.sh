seq=$1

# Fully associated with GT
# ../../build/Apps/InfiniTAM/InfiniTAM ../calibration/VCU_RVI.txt /home/bytian/Documents/Datasets/VCU_RVI/Converted/$seq/groundtruth.txt

# Partially associated RGBD
../../build/Apps/InfiniTAM/InfiniTAM ../calibration/VCU_RVI.txt /home/bytian/Documents/Datasets/VCU_RVI/Parsed/$seq/associated/RGBD.txt
