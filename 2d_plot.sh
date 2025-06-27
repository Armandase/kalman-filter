gnuplot -persist <<-EOF
    set title "Exemple 2D x y"
    set xlabel "X"
    set ylabel "Y"
    plot 'estimated_positions.txt' with linespoints title 'Courbe x^2'
EOF