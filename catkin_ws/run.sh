#!/bin/bash
. ./devel/setup.bash
source venv/bin/activate
end=$(expr $1 + 100)
for i in $(seq $1 $end)
do
    echo "iteration $i"
    rosrun MCTS main.py withcomms_notimeout $i 5 11 True 0 &>"logs/log1_${i}"
    rosrun MCTS main.py withoutcomms_notimeout $i 5 11 False 0 &>"logs/log2_${i}"
    rosrun MCTS main.py withcomms_timeout $i 5 11 True 3 &>"logs/log3_${i}"
    rosrun MCTS main.py withoutcomms_timeout $i 5 11 False 3 &>"logs/log4_${i}"

    for job in `jobs -p`
    do
	echo $job
	wait $job
    done
done

