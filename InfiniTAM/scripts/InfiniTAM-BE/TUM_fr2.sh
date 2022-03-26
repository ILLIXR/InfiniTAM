seq=$1
traj=$2

# TUM RGBD with mask
# ../../build/Apps/InfiniTAM_cli/InfiniTAM_cli ../calibration/TUM_fr2.txt ~/Documents/Datasets/TUM-RGBD/$seq/color/%04i.ppm ~/Documents/Datasets/TUM-RGBD/$seq/depth/%04i.pgm

# TUM RGBD with list
../../build/Apps/InfiniTAM_cli/InfiniTAM_cli ../calibration/TUM_fr2.txt ~/Documents/Datasets/TUM-RGBD/rgbd_dataset_freiburg2_$seq/$traj.txt
