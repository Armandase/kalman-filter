gnuplot -persist <<EOF

# X Position and X Variance in 3D
set title "X Position and X Variance"
set xlabel "Time step"
set ylabel "Value"
set zlabel "Dimension"
set grid
splot \
  'estimated_positions.txt' using 0:1:(0) with linespoints title 'X Position' lc rgb 'blue', \
  'variance.txt' using 0:1:(1) with linespoints title 'X Variance' lc rgb 'red'

pause -1

# Y Position and Y Variance in 3D
set title "Y Position and Y Variance"
set xlabel "Time step"
set ylabel "Value"
set zlabel "Dimension"
splot \
  'estimated_positions.txt' using 0:2:(0) with linespoints title 'Y Position' lc rgb 'green', \
  'variance.txt' using 0:2:(1) with linespoints title 'Y Variance' lc rgb 'magenta'

pause -1

# Z Position and Z Variance in 3D
set title "Z Position and Z Variance"
set xlabel "Time step"
set ylabel "Value"
set zlabel "Dimension"
splot \
  'estimated_positions.txt' using 0:3:(0) with linespoints title 'Z Position' lc rgb 'blue', \
  'variance.txt' using 0:3:(1) with linespoints title 'Z Variance' lc rgb 'red'

pause -1

EOF