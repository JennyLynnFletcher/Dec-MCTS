#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters"
    exit -1
fi


. ./devel/setup.bash
<<<<<<< HEAD
source venv/bin/activate
=======

>>>>>>> setup to run multiple versions concurrently that works on my PC
end=$(expr $1 + 100)
for i in $(seq $1 $end)
do
    for j in $(seq 6)
    do
	seed=$(expr $(expr $i \* 100) + $j)
	echo "iteration $i $j seed $seed"
	rosrun MCTS main.py withcomms_notimeout "$seed" 5 11 True 0 &>"logs/log1_$seed" &
	rosrun MCTS main.py withoutcomms_notimeout "$seed" 5 11 False 0 &>"logs/log2_$seed" &
    done	     
    for job in `jobs -p`
    do
	echo $job
	wait $job
    done
done

