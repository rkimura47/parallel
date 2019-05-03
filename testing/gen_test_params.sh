#!/bin/bash

for nb in 1 2 3; do
  for obj in cmax swct; do
    for i in 3 5 7; do
      for j in 10 15 20; do
        for k in `seq 0 9`; do
          echo $obj $nb $i $j $k
        done
      done
    done
  done
done
