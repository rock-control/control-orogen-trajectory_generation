#!/bin/bash

for i in 0 1 2 3 4 5
do
  echo "Plotting trajectory for joint $i..."
  export JNUM=$i
  gnuplot plot.gnuplot
  echo "... done"
done
