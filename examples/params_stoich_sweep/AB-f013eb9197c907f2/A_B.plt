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
set arrow 3 from 1, 0 to 0.8, -2.29387 nohead linestyle 1
set arrow 4 from 0.8, -2.29387 to 1, 0 nohead linestyle 1
set arrow 5 from 0, 0 to 0.2, -2.28293 nohead linestyle 1
set arrow 6 from 0.2, -2.28293 to 0, 0 nohead linestyle 1
set arrow 7 from 0.8, -2.29387 to 0.75, -2.72889 nohead linestyle 1
set arrow 8 from 0.75, -2.72889 to 0.8, -2.29387 nohead linestyle 1
set arrow 9 from 0.2, -2.28293 to 0.25, -2.72756 nohead linestyle 1
set arrow 10 from 0.25, -2.72756 to 0.2, -2.28293 nohead linestyle 1
set arrow 11 from 0.75, -2.72889 to 0.666667, -3.21998 nohead linestyle 1
set arrow 12 from 0.666667, -3.21998 to 0.75, -2.72889 nohead linestyle 1
set arrow 13 from 0.25, -2.72756 to 0.333333, -3.21417 nohead linestyle 1
set arrow 14 from 0.333333, -3.21417 to 0.25, -2.72756 nohead linestyle 1
set arrow 15 from 0.666667, -3.21998 to 0.6, -3.44478 nohead linestyle 1
set arrow 16 from 0.6, -3.44478 to 0.666667, -3.21998 nohead linestyle 1
set arrow 17 from 0.333333, -3.21417 to 0.4, -3.43895 nohead linestyle 1
set arrow 18 from 0.4, -3.43895 to 0.333333, -3.21417 nohead linestyle 1
set arrow 19 from 0.6, -3.44478 to 0.5, -3.67965 nohead linestyle 1
set arrow 20 from 0.5, -3.67965 to 0.6, -3.44478 nohead linestyle 1
set arrow 21 from 0.4, -3.43895 to 0.5, -3.67965 nohead linestyle 1
set arrow 22 from 0.5, -3.67965 to 0.4, -3.43895 nohead linestyle 1
set label "AB4" at 0.8, -2.34387 centre
set label "A4B" at 0.2, -2.33293 centre
set label "AB3" at 0.75, -2.77889 centre
set label "A3B" at 0.25, -2.77756 centre
set label "A2B4" at 0.666667, -3.26998 centre
set label "A4B2" at 0.333333, -3.26417 centre
set label "A2B3" at 0.6, -3.49478 centre
set label "A3B2" at 0.4, -3.48895 centre
set label "A4B4" at 0.5, -3.72965 centre
plot "A_B.hull" i 0 ls 2 title "Hull point", "A_B.hull" i 1 ls 3 title "Off hull point"
