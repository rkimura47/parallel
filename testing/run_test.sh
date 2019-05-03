#!/bin/bash

while read -r rawLine; do
  line=( $rawLine )
  obj=${line[0]}
  nb=${line[1]}
  i=${line[2]}
  j=${line[3]}
  k=${line[4]}

  resultsDir=${obj^^}$nb
  outputDir=${i}x${j}_${k}
  instance=P${i}x${j}_${k}

  mkdir -p $resultsDir/$outputDir
  cp ./instances/${instance}.data ./$resultsDir/$outputDir
  ../rob_parallel --dataDir=instances --outputDir=$resultsDir/$outputDir --$obj $instance $nb > $resultsDir/$outputDir/output
done < "params_list"
# Yes, Bash is weird and the filename goes at the END of the loop.
