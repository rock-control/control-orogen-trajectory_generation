jnum = `echo $JNUM`

set terminal pdf solid font 'Arial,8' size 20cm,25cm
set output 'out'.jnum.'.pdf'

set multiplot layout 3,1

set ylabel "Position (rad)"
plot 'cmd.csv' u ($1-1379493948.2561631):1+jnum*4+1 with lines title "CMD",   'state.csv' u ($1-1379493948.2561631):1+jnum*4+1 with lines title "STATE"

set ylabel "Speed (rad/s)"
plot 'cmd.csv' u ($1-1379493948.2561631):1+jnum*4+2 with lines title "CMD",   'state.csv' u ($1-1379493948.2561631):1+jnum*4+2 with lines title "STATE"

set xlabel "Time (s)"
set ylabel "Effort (rad/s^2)"
plot 'cmd.csv' u ($1-1379493948.2561631):1+jnum*4+3 with lines title "CMD",   'state.csv' u ($1-1379493948.2561631):1+jnum*4+3 with lines title "STATE"

unset multiplot

