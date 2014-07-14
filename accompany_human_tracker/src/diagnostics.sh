
#
# First collect data
#
# roslaunch accompany_uva trackRobotHouse.launch > ./tracker.log
#

grep state-match-score ./tracker.log | awk '{print $3}' > state.log
grep appearance-match-score ./tracker.log | awk '{print $3}' > appearance.log


gnuplot -e "set term wxt 0;
            plot \"state.log\" using 1 title 'state';
            set term wxt 1;
            plot \"appearance.log\" using 1 title 'appearance';
            pause -1;"

