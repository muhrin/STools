set xtics nomirror
set ytics nomirror
set style line 1 ps 1.5 pt 7 lc rgb '#000000'
set style line 2 ps 1.5 pt 7 lc rgb '#dd181f'
set style line 3 ps 0.6 pt 7 lc rgb '#0060ad'
set ylabel "Formation enthalpy"
set label "A" at -0.05, 0 centre
set label "B" at 1.05, 0 centre
set arrow 1 from 0, 0 to 1, 0 nohead linestyle 1
set arrow 2 from 1, 0 to 0, 0 nohead linestyle 1
set arrow 3 from 1, 0 to 0.8, -5.5211 nohead linestyle 1
set arrow 4 from 0.8, -5.5211 to 1, 0 nohead linestyle 1
set arrow 5 from 0, 0 to 0.333333, -8.62217 nohead linestyle 1
set arrow 6 from 0.333333, -8.62217 to 0, 0 nohead linestyle 1
set arrow 7 from 0.8, -5.5211 to 0.666667, -8.83807 nohead linestyle 1
set arrow 8 from 0.666667, -8.83807 to 0.8, -5.5211 nohead linestyle 1
set arrow 9 from 0.333333, -8.62217 to 0.5, -10.4224 nohead linestyle 1
set arrow 10 from 0.5, -10.4224 to 0.333333, -8.62217 nohead linestyle 1
set arrow 11 from 0.666667, -8.83807 to 0.5, -10.4224 nohead linestyle 1
set arrow 12 from 0.5, -10.4224 to 0.666667, -8.83807 nohead linestyle 1
set label "AB4" at 0.8, -5.5711 centre
set label "A2B" at 0.333333, -8.67217 centre
set label "A2B4" at 0.666667, -8.88807 centre
set label "AB" at 0.5, -10.4724 centre
plot "A_B.hull" i 0 ls 2 title "Hull point", "A_B.hull" i 1 ls 3 title "Off hull point"
