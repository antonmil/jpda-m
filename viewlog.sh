#!/bin/sh
#66252[1].moby.cs.adela  amilan      batch    0822-1-1           8486     1      1    8gb  08:00:00 R  00:20:48   acvt-node32

if [[ $# -eq 2 ]]; then
	jobline=`qstat -u amilan -t -n -1 | grep '\['$1'\]' | grep $2`
elif [[ $# -eq 1 ]]; then
	jobline=`qstat -u amilan -t -n -1 | grep '\['$1'\]'`
else
        echo "Illegal number of parameters"
fi
jobid=${jobline:0:5}
node=${jobline:113:11}
echo $node
vim scp://$node//var/spool/torque/spool/$jobid-$1.moby.cs.adelaide.edu.au.OU 
