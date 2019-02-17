set terminal jpeg size 250,250 font 'Arial,10'
set output 'phasedata.jpg'
set autoscale
unset border
set polar
set angles degrees
set xtics axis
set ytics axis
set xtics scale 0
set ytics scale 0
set label "E" at 18.500000,0.000000 center font "Arial,18" textcolor "blue"
set label "60" at 16.021470,9.250000 center font "Arial,11" textcolor "red"
set label "30" at 9.250000,16.021470 center font "Arial,11" textcolor "red"
set label "N" at 0.000000,18.500000 center font "Arial,18" textcolor "blue"
set label "330" at -9.250000,16.021470 center font "Arial,11" textcolor "red"
set label "300" at -16.021470,9.250000 center font "Arial,11" textcolor "red"
set label "W" at -18.500000,0.000000 center font "Arial,18" textcolor "blue"
set label "240" at -16.021470,-9.250000 center font "Arial,11" textcolor "red"
set label "210" at -9.250000,-16.021470 center font "Arial,11" textcolor "red"
set label "S" at -0.000000,-18.500000 center font "Arial,18" textcolor "blue"
set label "150" at 9.250000,-16.021470 center font "Arial,11" textcolor "red"
set label "120" at 16.021470,-9.250000 center font "Arial,11" textcolor "red"
set grid polar lw 1 lt -1
plot "phasedata.dat" lw 9 lt rgb "red" notitle, "phasedata2.dat" lw 9 lt rgb "black" notitle
exit 0