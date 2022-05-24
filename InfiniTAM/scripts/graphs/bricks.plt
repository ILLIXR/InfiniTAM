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

# Left y-axis
set ytics nomirror                      # Don't mirror tickmarks on the right side
set ylabel "Bricks" font ":,12"

# Right y-axis
set y2tics                              # Make second y-axis visible
set y2label "Average Confidence/Voxel" font ":,12"

# Plot
set title "Bricks and Confidence/Voxel Per Frame" font ":Bold,15"
plot filename using 1:2 lw 2 lc 'purple' title 'New Bricks' axes x1y1, \
     filename using 1:3 lw 2 lc 'magenta' title 'Visible Bricks' axes x1y1, \
     filename using 1:4 lw 2 lc 'sea-green' title 'Confidence/Voxel' axes x1y2

# Keep window open
pause mouse close
