#!/bin/bash
#SBATCH --partition=compute
#SBATCH --account=uic193
#SBATCH --time=5:00
#SBATCH --nodes=2
#SBATCH --ntasks-per-node=1
#SBATCH --job-name=bw
#SBATCH --output=slurm_output.%x-o%j
#SBATCH --error=slurm_error.%x-o%j

PATH_TO_BUILD_DIR=${1:-./init/build/}
PATH_TO_EXE=${PATH_TO_BUILD_DIR}/benchmarks

echo "srun bw MPI"
srun --mpi=pmi2 ${PATH_TO_EXE}/mpi_pt2pt --send-window 64 --nthreads 1 --thread-pin 1 --max-msg-size 1048576 --nsteps 1000

echo "srun bw LCI 2m"
srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 2m --send-window 64 --nthreads 1 --thread-pin 1 --nsteps 1000
echo "srun bw LCI 2m dyn"
srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 2m --send-window 64 --nthreads 1 --thread-pin 1 --send-dyn 1 --recv-dyn 1 --nsteps 1000
echo "srun bw LCI 2l"
srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 2l --send-window 64 --nthreads 1 --thread-pin 1 --max-msg-size 1048576 --nsteps 1000
echo "srun bw LCI 2l reg"
srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 2l --send-window 64 --nthreads 1 --thread-pin 1 --send-reg 1 --recv--reg 1 --max-msg-size 1048576 --nsteps 1000

echo "srun bw LCI 1m"
srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 1m --send-window 64 --nthreads 1 --thread-pin 1 --nsteps 1000
echo "srun bw LCI 1m dyn"
srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 1m --send-window 64 --nthreads 1 --thread-pin 1 --send-dyn 1 --nsteps 1000
echo "srun bw LCI 1l"
srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 1l --send-window 64 --nthreads 1 --thread-pin 1 --max-msg-size 1048576 --nsteps 1000
echo "srun bw LCI 1l reg"
srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 1l --send-window 64 --nthreads 1 --thread-pin 1 --send-reg 1 --max-msg-size 1048576 --nsteps 1000

#echo "srun bw LCI --op 2m multithread 64"
#srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 2m --send-window 64 --nthreads 64 --thread-pin 1
#echo "srun bw LCI --op 2m multithread 64 dyn"
#srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 2m --send-window 64 --nthreads 64 --thread-pin 1 --send-dyn 1 --recv-dyn 1
#echo "srun bw LCI --op 2l multithread 64"
#srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 2l --send-window 64 --nthreads 64 --thread-pin 1 --max-msg-size 1048576
#echo "srun bw LCI --op 2l --nthreads 64 --send-reg 1 --recv--reg 1"
#srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 2l --send-window 64 --nthreads 64 --thread-pin 1 --send-reg 1 --recv--reg 1 --max-msg-size 1048576
#
#echo "srun bw LCI --op 1m multithread 64"
#srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 1m --send-window 64 --nthreads 64 --thread-pin 1
#echo "srun bw LCI --op 1m multithread 64 dyn"
#srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 1m --send-window 64 --nthreads 64 --thread-pin 1 --send-dyn 1
#echo "srun bw LCI --op 1l multithread 64"
#srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 1l --send-window 64 --nthreads 64 --thread-pin 1 --max-msg-size 1048576
#echo "srun bw LCI --op 1l multithread 64 pre-reg"
#srun --mpi=pmi2 ${PATH_TO_EXE}/lcitb_pt2pt --op 1l --send-window 64 --nthreads 64 --thread-pin 1 --send-reg 1 --max-msg-size 1048576
