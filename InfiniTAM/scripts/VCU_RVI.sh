seq=$1

# cd .. && ./InfiniTAM calibration/VCU_RVI.txt /home/bytian/Documents/Datasets/VCU_RVI/lab-easy-01/ppm/color/%04i.ppm /home/bytian/Documents/Datasets/VCU_RVI/lab-easy-01/ppm/depth/%04i.pgm
cd ../build/Apps/InfiniTAM_cli/ && ./InfiniTAM_cli ~/Desktop/project-asp/NoICP_InfiniTAM/InfiniTAM/scripts/calibration/VCU_RVI.txt /home/bytian/Documents/Datasets/VCU_RVI/Converted/$seq/matched_color_list.txt /home/bytian/Documents/Datasets/VCU_RVI/Converted/$seq/matched_depth_list.txt

