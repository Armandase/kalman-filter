gnuplot -persist <<-EOF
    set title "Exemple 3D x y z"
    set xlabel "X"
    set ylabel "Y"
    set zlabel "Z"
    splot 'estimated_positions.txt' with linespoints title 'Points 3D'; pause -1
EOF