#!/bin/sh
#echo $1

# precomp helper structures 
#matlab -nodesktop -nosplash -r "precomputeForCluster('$PBS_JOBNAME'); quit;";

#qsub -t 1-20 -j oe -l walltime=08:00:00 -l vmem=16GB -W depend=afterokarray:69420 -o logs -N $1 trainModel.sh
qsub -t 1-20 -j oe -l walltime=2:00:00 -l vmem=12GB -o logs -N $1 trainModel.sh
