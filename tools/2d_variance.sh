#!/bin/bash

# Usage (décommenter pour usage) :
# ./plot_variance_position.sh x
# ./plot_variance_position.sh y
# ./plot_variance_position.sh z

if [ $# -ne 1 ]; then
  echo "Usage: $0 {x|y|z}"
  exit 1
fi

AXIS=$(echo "$1" | tr '[:upper:]' '[:lower:]')

if [[ "$AXIS" != "x" && "$AXIS" != "y" && "$AXIS" != "z" ]]; then
  echo "Argument invalide: $1"
  echo "Usage: $0 {x|y|z}"
  exit 1
fi

if [ ! -f estimated_positions.txt ] || [ ! -f variance.txt ]; then
  echo "Fichiers 'estimated_positions.txt' ou 'variance.txt' introuvables."
  exit 1
fi

# Index de colonne selon x=1, y=2, z=3
case "$AXIS" in
  x) COL=1; COLOR_POS="blue"; COLOR_VAR="red";;
  y) COL=2; COLOR_POS="green"; COLOR_VAR="magenta";;
  z) COL=3; COLOR_POS="blue"; COLOR_VAR="red";;
esac

gnuplot -persist <<EOF
set title "${AXIS^^} Position and ${AXIS^^} Variance"
set xlabel "Time step"
set ylabel "${AXIS^^} Position"
set y2label "${AXIS^^} Variance"
set y2tics
set ytics nomirror
plot \
  'estimated_positions.txt' using 0:${COL} with lines title '${AXIS^^} Position' axis x1y1 lc rgb '${COLOR_POS}', \
  'variance.txt' using 0:${COL} with lines title '${AXIS^^} Variance' axis x1y2 lc rgb '${COLOR_VAR}'
EOF
