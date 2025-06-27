#!/bin/bash

# Fichier de données
DATAFILE="variance.txt"

# Fichier script gnuplot temporaire
GNUPLOT_SCRIPT="plot_variance.gnu"

# Création du script gnuplot
cat > $GNUPLOT_SCRIPT <<EOF
set terminal qt size 800,600
set title "Modélisation de la variance en 3D"
set xlabel "X"
set ylabel "Y"
set zlabel "Z"
set grid

# Palette pour la variance (vx par exemple)
set palette defined (0 "blue", 0.01 "red")

# Affichage 3D en points colorés selon vx (colonne 4)
splot "$DATAFILE" using 1:2:3:4 with points pt 7 ps 1 palette notitle
pause -1
EOF

# Exécution de gnuplot
gnuplot $GNUPLOT_SCRIPT
