#!/usr/bin/gnuplot
filename=ARG1

# Graph settings
set datafile separator comma            # csv file
set grid y lt 1 lw .75 lc "gray" dt '.' # Dotted y-axis lines
set tics nomirror                       # No ticks on top and right border
set key Left box reverse                # Left-justified legend with box around it, and lines first

# x-axis
set xlabel "Frame" font ":,12"

# Left y-axis
set ytics nomirror                      # Don't mirror tickmarks on the right side
set ylabel "New Bricks" font ":,12"

# Right y-axis
set y2tics                              # Make second y-axis visible
set y2label "Frequency" font ":,12"

# Plot
set title "Camera Frequency Controller" font ":Bold,15"
plot filename using 1:2 with lines lw 2 title 'New Bricks' axes x1y1, \
     filename using 1:3 with steps lw 2 title 'Frequency' axes x1y2

# Keep window open
pause mouse close
