#!/bin/bash
CSV_FILE="benchmark_results.csv"

echo "World_ID,Heuristic,Success,Collided,Timeout,Time_Taken(s),Nav_Metric" > $CSV_FILE

for mode in "new"; do
    echo "========================================="
    echo " RUNNING BENCHMARK: HEURISTIC = $mode"
    echo "========================================="

    for i in {0...99}; do
        n=$((i * 3))

        for j in {1..5}; do
            echo "Testing World $n | Run $j/5 | Mode: $mode"
            ./launch_barn.sh $n $mode $j
            sleep 1  # small gap between runs to let cleanup finish
        done
    done
done

echo "Benchmark complete. Results in $CSV_FILE"