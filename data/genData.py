#!/usr/bin/python3
import argparse
from collections import namedtuple
from random import randrange, gauss

def parse_arguments(pargs=None):
  parser = argparse.ArgumentParser(description='Generate parallel machine scheduling problem data.')
  parser.add_argument('fileName', help='Name of output file', type=str)
  parser.add_argument('--numJobs', help='Number of Jobs', type=int, default=10)
  parser.add_argument('--numMachines', help='Number of Machines', type=int, default=3)
  # Range of numJobs = 21, 27, 33
  # Range of numMachines = 3, 4, 5
  return parser.parse_args(pargs)

Job = namedtuple('Job', ['id', 'release', 'deadline'])

def randIntGauss(lb, ub):
  assert lb <= ub
  mu = (lb + ub)/2
  sigma = (ub-lb)/6
  r = gauss(mu, sigma)
  if r <= lb:
    return lb
  elif r >= ub:
    return ub
  else:
    return round(r)


def getData(fileName, numJobs, numMachines):
  # Other parameters
  maxReleaseTime = 50
  minProcTime = 11
  maxProcTime = 99
  maxDeadline = 1000

  # Randomly generate parameters
  jobs = []
  for j in range(numJobs):
    rel = randrange(maxReleaseTime)
    ddl = maxDeadline
    jobs.append(Job(j, rel, ddl))
  procTimes = []
  for m in range(numMachines):
    pTime = []
    for j in range(numJobs):
      pr = randIntGauss(minProcTime, maxProcTime)
      pTime.append(pr)
    procTimes.append(pTime)

  # Save data to file
  with open(fileName, 'w') as f:
    print('%d %d' % (numJobs, numMachines), file=f)
    print(' '.join(str(j.release) for j in jobs), file=f)
    for m in range(numMachines):
      print(' '.join(str(p) for p in procTimes[m]), file=f)

if __name__ == '__main__':
  args = parse_arguments()
  getData(args.fileName, args.numJobs, args.numMachines)
