seq=$1

# TUM RGBD with mask
# cd ../build/Apps/InfiniTAM_cli/ && ./InfiniTAM_cli ~/Desktop/project-asp/NoICP_InfiniTAM/InfiniTAM/scripts/calibration/TUM_fr1.txt ~/Documents/Datasets/TUM-RGBD/$seq/color/%04i.ppm ~/Documents/Datasets/TUM-RGBD/$seq/depth/%04i.pgm

# TUM RGBD with list
cd ../build/Apps/InfiniTAM_cli/ && ./InfiniTAM_cli ~/Desktop/project-asp/NoICP_InfiniTAM/InfiniTAM/scripts/calibration/TUM_fr1.txt ~/Documents/Datasets/TUM-RGBD/$seq/matched_groundtruth.txt ~/Documents/Datasets/TUM-RGBD/$seq/color_list.txt ~/Documents/Datasets/TUM-RGBD/$seq/depth_list.txt
