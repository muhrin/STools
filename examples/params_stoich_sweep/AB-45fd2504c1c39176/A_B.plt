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
set arrow 3 from 1, 0 to 0.666667, -12.4332 nohead linestyle 1
set arrow 4 from 0.666667, -12.4332 to 1, 0 nohead linestyle 1
set arrow 5 from 0, 0 to 0.333333, -12.0926 nohead linestyle 1
set arrow 6 from 0.333333, -12.0926 to 0, 0 nohead linestyle 1
set arrow 7 from 0.666667, -12.4332 to 0.5, -14.0493 nohead linestyle 1
set arrow 8 from 0.5, -14.0493 to 0.666667, -12.4332 nohead linestyle 1
set arrow 9 from 0.333333, -12.0926 to 0.5, -14.0493 nohead linestyle 1
set arrow 10 from 0.5, -14.0493 to 0.333333, -12.0926 nohead linestyle 1
set label "A2B4" at 0.666667, -12.4832 centre
set label "A2B" at 0.333333, -12.1426 centre
set label "A2B2" at 0.5, -14.0993 centre
plot "A_B.hull" i 0 ls 2 title "Hull point", "A_B.hull" i 1 ls 3 title "Off hull point"
