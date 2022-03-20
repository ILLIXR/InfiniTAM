seq=$1

# TUM RGBD with mask
# ../../build/Apps/InfiniTAM/InfiniTAM ../calibration/TUM_fr2.txt ~/Documents/Datasets/TUM-RGBD/$seq/color/%04i.ppm ~/Documents/Datasets/TUM-RGBD/$seq/depth/%04i.pgm

# TUM RGBD with list
../../build/Apps/InfiniTAM/InfiniTAM ../calibration/TUM_fr2.txt ~/Documents/Datasets/TUM-RGBD/rgbd_dataset_freiburg2_$seq/data.txt
