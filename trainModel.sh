#!/bin/sh
source ~/.bashrc
cd $PBS_O_WORKDIR

# e.g. 4
echo ${PBS_ARRAYID}

# eg 58973[4].moby.cs.adelaide.edu.au
echo ${PBS_JOBID}

# eg KITTI-4
echo $PBS_JOBNAME

# last parameter = max number of experiments for each learning iteration
matlab -nodesktop -nosplash -r "trainModel('$PBS_JOBNAME','$PBS_ARRAYID',99)"


