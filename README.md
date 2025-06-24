# Kalman-filter
implementation of kalman filter
Make sure to have sudo apt-get install gnuplot

To see the plot : 
gnuplot -e "splot 'estimated_positions.txt' with linespoints title 'Estimated Position'; pause -1"
QSocketNotifier: Can only be used with threads started with QThread
For 3D
gnuplot -e "plot 'estimated_positions.txt' using 1:2 with linespoints title 'Estimated XY'; pause -1"  
For 2D

Plot X position and X variance (dual y-axes):
gnuplot -persist <<EOF
set title "X Position and X Variance"
set xlabel "Time step"
set ylabel "X Position"
set y2label "X Variance"
set y2tics
plot 'estimated_positions.txt' using 0:1 with lines title 'X Position' axis x1y1 lc rgb 'blue', \
     'variance.txt' using 0:1 with lines title 'X Variance' axis x1y2 lc rgb 'red'
pause -1
EOF

Plot Y position and Y variance:
gnuplot -persist <<EOF
set title "Y Position and Y Variance"
set xlabel "Time step"
set ylabel "Y Position"
set y2label "Y Variance"
set y2tics
plot 'estimated_positions.txt' using 0:2 with lines title 'Y Position' axis x1y1 lc rgb 'green', \
     'variance.txt' using 0:2 with lines title 'Y Variance' axis x1y2 lc rgb 'magenta'
pause -1
EOF

Plot Z position and Z variance:
gnuplot -persist <<EOF
set title "Z Position and Z Variance"
set xlabel "Time step"
set ylabel "Z Position"
set y2label "Z Variance"
set y2tics
plot 'estimated_positions.txt' using 0:3 with lines title 'Z Position' axis x1y1 lc rgb 'blue', \
     'variance.txt' using 0:3 with lines title 'Z Variance' axis x1y2 lc rgb 'red'
pause -1
EOF

### Bibliographie:

https://www.ferdinandpiette.com/blog/2011/04/exemple-dutilisation-du-filtre-de-kalman/


https://pagespro.isae-supaero.fr/IMG/pdf/introKalman_vf_2008.pdf

https://apps.dtic.mil/sti/tr/pdf/ADA024377.pdf