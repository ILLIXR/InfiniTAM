#!/usr/bin/gnuplot
filename=ARG1

# Graph settings
set datafile separator comma            # csv file
set style data lines                    # Line graph
set grid y lt 1 lw .75 lc "gray" dt '.' # Dotted y-axis lines
set tics nomirror                       # No ticks on top and right border
set key Left box reverse                # Left-justified legend with box around it, and lines first

# x-axis
set xlabel "Frame" font ":,12"

# y-axis
set ylabel "Average Voxels/Brick" font ":,12"

# Plot
set title "Average Voxels/Brick Per Frame" font ":Bold,15"
plot filename using 1:2 lw 2 title 'Launched', \
     filename using 1:3 lw 2 title 'Passed Camera', \
     filename using 1:4 lw 2 title 'Passed Projection', \
     filename using 1:5 lw 2 title 'Passed Depth', \
     filename using 1:6 lw 2 title 'Updated Voxels'

# Keep window open
pause mouse close
