#include <iostream>
#include <sstream>
#include <ilcp/cp.h>
#include "args.hxx" // Taywee/args

typedef IloArray<IloBoolArray> IloBoolArray2;
typedef IloArray<IloConstraintArray> IloConstraintArray2;

enum class ObjFuncType { cmax, swct };

/**
 * CPLEX error for failing to open file.
 */
class FileError: public IloException
{
public:
  FileError() : IloException("Cannot open data file") {}
};

/**
 * CPLEX error for attempting to invoke non-existent functionality.
 */
class NotImplementedError: public IloException
{
public:
  NotImplementedError() : IloException("Not implemented") {}
};

/**
 * Parse machine number from interval name.
 */
IloInt mchOf(std::string& jobName)
{
  std::string::size_type n = jobName.find("_", 2);
  if(n == std::string::npos)
  {
    return -1;
  }
  else
  {
    return std::stol(jobName.substr(1, n));
  }
}

/**
 * Parse job number from interval name.
 */
IloInt jobOf(std::string& jobName)
{
  std::string::size_type n = jobName.find("_", 2);
  if(n == std::string::npos)
  {
    return -1;
  }
  else
  {
    return std::stol(jobName.substr(n+2, jobName.size()-n-2));
  }
}



/**
 * Creates base model for use in logic-based Benders.
 *
 * Creates model using parameter data. Explicitly models delays as intervals.
 * Use to create scenario generation model.
 *
 * @param env Environment variable
 * @param nbJobs Number of jobs
 * @param nbMachines Number of machines
 * @param releaseTimes Release times of job j
 * @param processingTimes Processing time on machine i of job j
 * @param delays Interval variable container for delay of machine i job j
 * @param superJobs Interval variable container for machine i job j
 * @param sequences Sequence variable container for machine i's jobs
 * @param objExpr Expression for objective
 * @return Base scheduling model
 */
IloModel makeBaseModel(
  IloEnv                        env,
  IloInt                        nbJobs,
  IloInt                        nbMachines,
  IloIntArray                   releaseTimes,
  IloIntArray2                  processingTimes,
  ObjFuncType                   objType,
  IloIntervalVarArray2&         delays,
  IloIntervalVarArray2&         superJobs,
  IloIntervalSequenceVarArray&  sequences,
  IloIntExpr&                   objExpr)
{
  // Worst scenario subject to b_i breakdowns on machine i
  // Total no more than X Breaks overall
  IloModel model(env);

  // VARIABLES
  // For each job in solution, create fixed job (on machine), delay job, and super job
  delays = IloIntervalVarArray2(env, nbMachines);
  IloIntervalVarArray2 origJobs(env, nbMachines);
  superJobs = IloIntervalVarArray2(env, nbMachines);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    delays[i] = IloIntervalVarArray(env, nbJobs);
    origJobs[i] = IloIntervalVarArray(env, nbJobs);
    superJobs[i] = IloIntervalVarArray(env, nbJobs);
    for (IloInt j = 0; j < nbJobs; j++)
    {
      // Create delay job
      std::ostringstream name;
      name << "m" << i << "_j" << j;
      IloIntervalVar jobDelay(env, processingTimes[i][j]-1, name.str().c_str());
      jobDelay.setOptional();
      delays[i][j] = jobDelay;
      // Create orig job
      IloIntervalVar origJob(env, processingTimes[i][j]);
      origJob.setOptional();
      origJobs[i][j] = origJob;
      // Create super job
      name.str("");
      name << "M" << i << "_J" << j;
      IloIntervalVar sj(env, name.str().c_str());
      sj.setOptional();
      superJobs[i][j] = sj;
    }
  }
  // For each machine, a sequence variable to enforce sequence consistency
  sequences = IloIntervalSequenceVarArray(env, nbMachines);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    sequences[i] = IloIntervalSequenceVar(env, superJobs[i]);
  }

  // JOB CONSTRAINTS
  for (IloInt i = 0; i < nbMachines; i++)
  {
    for (IloInt j = 0; j < nbJobs; j++)
    {
      // Presence relations
      model.add(IloPresenceOf(env, origJobs[i][j]) == IloPresenceOf(env, superJobs[i][j]));
      model.add(IloPresenceOf(env, delays[i][j]) <= IloPresenceOf(env, superJobs[i][j]));
      // Length of super job is length of fixed + length of delay
      IloIntervalVarArray combo(env, 2);
      combo[0] = delays[i][j];
      combo[1] = origJobs[i][j];
      model.add(IloSpan(env, superJobs[i][j], combo));
      // Fixed job is immediately preceded by its delay.
      model.add(IloEndAtStart(env, delays[i][j], origJobs[i][j]));
      // Release times
      delays[i][j].setStartMin(releaseTimes[j]);
    }
  }
  // Each job can only go on one machine.
  for (IloInt j = 0; j < nbJobs; j++)
  {
    IloIntExpr jobPresences(env);
    for (IloInt i = 0; i < nbMachines; i++)
    {
      jobPresences += IloPresenceOf(env, superJobs[i][j]);
    }
    model.add(jobPresences == 1);
  }
  // SEQUENCE CONSTRAINTS
  for (IloInt i = 0; i < nbMachines; i++)
  {
    // No overlap constraints
    model.add(IloNoOverlap(env, sequences[i]));
  }
  // SCENARIO CONSTRAINTS
  // Added elsewhere

  // OBJECTIVE
  IloIntExprArray ends(env);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    for (IloInt j = 0; j < superJobs[i].getSize(); j++)
    {
      ends.add(IloEndOf(superJobs[i][j]));
    }
  }
  if (objType == ObjFuncType::cmax)
    objExpr = IloMax(ends);
  else if (objType == ObjFuncType::swct)
    objExpr = IloSum(ends);
  else
    throw NotImplementedError();

  return model;
}



/**
 * Creates simple scheduling model for use in logic-based Benders.
 *
 * Creates scheduling model from parameter data with one interval variable
 * per machine-job pair.  Use to generate the restricted master problem.
 *
 * @param env Environment variable
 * @param nbJobs Number of jobs
 * @param nbMachines Number of machines
 * @param releaseTimes Release times of job j
 * @param processingTimes Processing time on machine i of job j
 * @param delayedJobs Jobs that are delayed (and have double the processing time)
 * @param jobs Interval variable container for machine i job j
 * @param sequences Sequence variable container for machine i's jobs
 * @param objExpr Expression for objective
 * @return Simple scheduling model
 */
IloModel makeSimpleModel(
  IloEnv                        env,
  IloInt                        nbJobs,
  IloInt                        nbMachines,
  IloIntArray                   releaseTimes,
  IloIntArray2                  processingTimes,
  ObjFuncType                   objType,
  IloIntervalVarArray           delayedJobs,
  IloIntervalVarArray2&         jobs,
  IloIntervalSequenceVarArray&  sequences,
  IloIntExpr&                   objExpr)
{
  IloModel scenModel(env);
  // VARIABLES
  // Variables instantiating each job on each machine
  jobs = IloIntervalVarArray2(env, nbMachines);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    jobs[i] = IloIntervalVarArray(env, nbJobs);
    for (IloInt j = 0; j < nbJobs; j++)
    {
      std::ostringstream name;
      name << "M" << i << "_J" << j;
      IloIntervalVar job(env, processingTimes[i][j], name.str().c_str());
      job.setOptional();
      // Release times
      job.setStartMin(releaseTimes[j]);
      jobs[i][j] = job;
    }
  }
  // Adjust processing times for delayed jobs
  for (IloInt k = 0; k < delayedJobs.getSize(); k++)
  {
    IloIntervalVar itv = delayedJobs[k];
    std::string itvname = itv.getName();
    IloInt mchNum = mchOf(itvname);
    IloInt jobNum = jobOf(itvname);
    IloInt delProcTime = 2*processingTimes[mchNum][jobNum]-1;
    jobs[mchNum][jobNum].setSizeMin(delProcTime);
    jobs[mchNum][jobNum].setSizeMax(delProcTime);
  }
  // Variables for sequencing jobs on each machine
  sequences = IloIntervalSequenceVarArray(env, nbMachines);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    sequences[i] = IloIntervalSequenceVar(env, jobs[i]);
  }

  // CONSTRAINTS
  for (IloInt i = 0; i < nbMachines; i++)
  {
    scenModel.add(IloNoOverlap(env, sequences[i]));
  }
  for (IloInt j = 0; j < nbJobs; j++)
  {
    IloIntExpr jobPresences(env);
    for (IloInt i = 0; i < nbMachines; i++)
    {
      jobPresences += IloPresenceOf(env, jobs[i][j]);
    }
    scenModel.add(jobPresences == 1);
  }

  // Objective
  IloIntExprArray ends(env);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    for (IloInt j = 0; j < nbJobs; j++)
    {
      ends.add(IloEndOf(jobs[i][j]));
    }
  }
  if (objType == ObjFuncType::cmax)
    objExpr = IloMax(ends);
  else if (objType == ObjFuncType::swct)
    objExpr = IloSum(ends);
  else
    throw NotImplementedError();

  return scenModel;
}

/**
 * Creates deterministic version of model.
 *
 * Creates deterministic counterpart of robust scheduling model.
 *
 * @param env Environment variable
 * @param nbJobs Number of jobs
 * @param nbMachines Number of machines
 * @param releaseTimes Release times of job j
 * @param processingTimes Processing time on machine i of job j
 * @param jobs Interval variable container for machine i job j
 * @param sequences Sequence variable container for machine i's jobs
 * @return Deterministic model
 */
IloModel makeDetrModel(
  IloEnv                        env,
  IloInt                        nbJobs,
  IloInt                        nbMachines,
  IloIntArray                   releaseTimes,
  IloIntArray2                  processingTimes,
  ObjFuncType                   objType,
  IloIntervalVarArray2&         jobs,
  IloIntervalSequenceVarArray&  sequences)
{
  // Use makeSimpleModel as base
  IloModel model(env);
  IloIntervalVarArray delayedJobs(env);
  IloIntExpr objExpr;
  IloModel submodel = makeSimpleModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, objType, delayedJobs, jobs, sequences, objExpr);
  model.add(submodel);

  // OBJECTIVE
  IloObjective objective = IloMinimize(env, objExpr);
  model.add(objective);

  return model;
}

static void reportMaster(IloCP& masterCP, IloIntervalSequenceVarArray& masterSequences, std::ostream& out=std::cout)
{
  // Display master solution
  std::ostream& origOut = masterCP.out();
  masterCP.setOut(out);
  masterCP.out() << "MasterObjective \t: " << masterCP.getObjValue() << std::endl;
  for (IloInt i = 0; i < masterSequences.getSize(); i++)
  {
    IloIntervalSequenceVar seq = masterSequences[i];
    for (IloIntervalVar itv = masterCP.getFirst(seq); itv.getImpl() != 0; itv = masterCP.getNext(seq, itv))
    {
      std::string itvname = itv.getName();
      IloInt mchNum = mchOf(itvname);
      IloInt jobNum = jobOf(itvname);
      masterCP.out() << itvname << " [" << masterCP.getStart(itv) << "-" << masterCP.getEnd(itv) << "]" << std::endl;
    }
  }
  masterCP.setOut(origOut);
}

/**
 * Creates base scenario generation model for use in logic-based Benders.
 *
 * Creates base scenario generation model using parameter data. This model
 * will be re-used over the course of the algorithm. makeWorstDelayModel(...)
 * creates a complete model from the parameters.
 *
 * @param env Environment variable
 * @param nbJobs Number of jobs
 * @param nbMachines Number of machines
 * @param releaseTimes Release times of job j
 * @param processingTimes Processing time on machine i of job j
 * @param maxMchDelays Maximum number of delays for jobs on machine i
 * @param maxDelays Maximum number of delays over all jobs collectively
 * @param superJobs Interval variable container for machine i job j
 * @param sequences Sequence variable container for machine i's jobs
 * @return Scenario generation model
 */
IloModel makeScenGenModel(
  IloEnv                        env,
  IloInt                        nbJobs,
  IloInt                        nbMachines,
  IloIntArray                   releaseTimes,
  IloIntArray2                  processingTimes,
  ObjFuncType                   objType,
  IloIntArray                   maxMchDelays,
  IloInt                        maxDelays,
  IloIntervalVarArray2&         delays,
  IloIntervalVarArray2&         superJobs,
  IloIntervalSequenceVarArray&  sequences)
{
  // Use makeBaseModel as base
  IloModel model(env);
  IloIntExpr objExpr;
  IloModel submodel = makeBaseModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, objType, delays, superJobs, sequences, objExpr);
  model.add(submodel);

  // CONSTRAINTS
  // Sequence contiguity
  for (IloInt i = 0; i < nbMachines; i++)
  {
    for (IloInt j = 0; j < nbJobs; j++)
    {
      model.add( IloStartOf(superJobs[i][j], releaseTimes[j]) == IloMax(IloEndOfPrevious(sequences[i], superJobs[i][j], releaseTimes[j]), releaseTimes[j]));
    }
  }
  // Machine delay constraints
  IloIntExprArray presences(env, nbMachines);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    presences[i] = IloIntExpr(env);
    for (IloInt j = 0; j < delays[i].getSize(); j++)
    {
      presences[i] += IloPresenceOf(env, delays[i][j]);
    }
    model.add(presences[i] <= maxMchDelays[i]);
  }
  // Total delay constraints
  IloIntExpr allPresences(env);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    allPresences += presences[i];
  }
  model.add(allPresences <= maxDelays);

  // OBJECTIVE
  IloObjective objective = IloMaximize(env, objExpr);
  model.add(objective);

  return model;
}

static void reportSubmodel(IloCP& scenCP, IloIntervalVarArray2& scenDelays)
{
  // Display subproblem solution
  scenCP.out() << "SubprobObjective \t:" << scenCP.getObjValue() << std::endl;
  for (IloInt i = 0; i < scenDelays.getSize(); i++)
  {
    for (IloInt j = 0; j < scenDelays[i].getSize(); j++)
    {
      if (scenCP.isPresent(scenDelays[i][j]))
      {
        IloIntervalVar itv = scenDelays[i][j];
        std::string itvname = itv.getName();
        scenCP.out() << itvname << std::endl;
      }
    }
  }
}

static void recordDelay(IloCP& scenCP, IloInt iterNum, IloIntervalVarArray2& scenDelays, std::ostream& out=std::cout)
{
  // Record worst case delay found
  std::ostream& origOut = scenCP.out();
  scenCP.setOut(out);
  scenCP.out() << "Iter " << iterNum << ":";
  for (IloInt i = 0; i < scenDelays.getSize(); i++)
  {
    for (IloInt j = 0; j < scenDelays[i].getSize(); j++)
    {
      if (scenCP.isPresent(scenDelays[i][j]))
      {
        IloIntervalVar itv = scenDelays[i][j];
        std::string itvname = itv.getName();
        scenCP.out() << " " << itvname;
      }
    }
  }
  scenCP.out() << std::endl;
  scenCP.setOut(origOut);
}

/**
 * Creates scenario generation model for use in logic-based Benders.
 *
 * Creates scenario generation model using parameter data. This is a complete
 * model for finding the worst case scenario. makeScenGenModel(...) creates
 * a model that is intended to be re-used.
 *
 * @param env Environment variable
 * @param nbJobs Number of jobs
 * @param nbMachines Number of machines
 * @param releaseTimes Release times of job j
 * @param processingTimes Processing time on machine i of job j
 * @param maxMchDelays Maximum number of delays for jobs on machine i
 * @param maxDelays Maximum number of delays over all jobs collectively
 * @param soln Solution containing machine-job pairs to fix
 * @param delays Interval variable container for delay of machine i job j
 * @return Scenario generation model
 */
IloModel makeWorstDelayModel(
  IloEnv                  env,
  IloInt                  nbJobs,
  IloInt                  nbMachines,
  IloIntArray             releaseTimes,
  IloIntArray2            processingTimes,
  ObjFuncType             objType,
  IloIntArray             maxMchDelays,
  IloInt                  maxDelays,
  IloSolution&            soln,
  IloIntervalVarArray2&   delays)
{
  IloModel model(env);
  IloIntervalVarArray2 superJobs;
  IloIntervalSequenceVarArray sequences;
  IloModel submodel = makeScenGenModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, objType, maxMchDelays, maxDelays, delays, superJobs, sequences);
  model.add(submodel);

  // SOLUTION BINDING CONSTRAINTS
  // Job presence
  for (IloSolutionIterator<IloIntervalVar> it(soln); it.ok(); ++it)
  {
    IloIntervalVar itv = *it;
    std::string itvname = itv.getName();
    IloInt mchNum = mchOf(itvname);
    IloInt jobNum = jobOf(itvname);
    model.add(IloPresenceOf(env, superJobs[mchNum][jobNum]) == soln.isPresent(itv));
  }
  // Sequence binding
  for (IloSolutionIterator<IloIntervalSequenceVar> it(soln); it.ok(); ++it)
  {
    IloIntervalSequenceVar seq = *it;
    IloInt precJobNum = -1;
    for (IloIntervalVar itv = soln.getFirst(seq); itv.getImpl() != 0; itv = soln.getNext(seq, itv))
    {
      std::string itvname = itv.getName();
      IloInt mchNum = mchOf(itvname);
      IloInt jobNum = jobOf(itvname);
      if (precJobNum != -1)
      {
        model.add(IloPrevious(env, sequences[mchNum], superJobs[mchNum][precJobNum], superJobs[mchNum][jobNum]));
      }
      precJobNum = jobNum;
    }
  }

  return model;
}


/**
 * Generate list of job tuples.
 *
 * Generate list of job tuples. Only works for tuple lengths <= 3.

 * @param nbJobs Number of jobs
 * @param nbMachines Number of machines
 * @param tupLen Length of tuple
 * @return Array of job tuples
 */
IloArray<IloArray<IloIntArray> > generateJobTuples(IloEnv env, IloInt nbJobs, IloInt nbMachines, IloInt tupLen)
{
  IloArray<IloArray<IloIntArray> > allJobTuples(env);
  IloInt totElems = nbMachines*nbJobs;
  if (tupLen == 1)
  {
    for (IloInt i = 0; i < nbMachines; i++)
    {
      for (IloInt j = 0; j < nbJobs; j++)
      {
        IloArray<IloIntArray> monoTuple(env);
        monoTuple.add(IloIntArray(env, 2, i, j));
        allJobTuples.add(monoTuple);
      }
    }
    assert(allJobTuples.getSize() == totElems);
  }
  else if (tupLen == 2)
  {
    for (IloInt i = 0; i < nbMachines; i++)
    {
      for (IloInt j = 0; j < nbJobs; j++)
      {
        for (IloInt ii = 0; ii < nbMachines; ii++)
        {
          for (IloInt jj = 0; jj < nbJobs; jj++)
          {
            if (i > ii || (i == ii && j >= jj))
            {
              continue;
            }
            IloArray<IloIntArray> doubleTuple(env);
            doubleTuple.add(IloIntArray(env, 2, i, j));
            doubleTuple.add(IloIntArray(env, 2, ii, jj));
            allJobTuples.add(doubleTuple);
          }
        }
      }
    }
    assert(allJobTuples.getSize() == totElems*(totElems-1)/2);
  }
  else if (tupLen == 3)
  {
    for (IloInt i = 0; i < nbMachines; i++)
    {
      for (IloInt j = 0; j < nbJobs; j++)
      {
        for (IloInt ii = 0; ii < nbMachines; ii++)
        {
          for (IloInt jj = 0; jj < nbJobs; jj++)
          {
            if (i > ii || (i == ii && j >= jj))
            {
              continue;
            }
            for (IloInt iii = 0; iii < nbMachines; iii++)
            {
              for (IloInt jjj = 0; jjj < nbJobs; jjj++)
              {
                if (ii > iii || (ii == iii && jj >= jjj))
                {
                  continue;
                }
                IloArray<IloIntArray> tripleTuple(env);
                tripleTuple.add(IloIntArray(env, 2, i, j));
                tripleTuple.add(IloIntArray(env, 2, ii, jj));
                tripleTuple.add(IloIntArray(env, 2, iii, jjj));
                allJobTuples.add(tripleTuple);
              }
            }
          }
        }
      }
    }
    assert(allJobTuples.getSize() == totElems*(totElems-1)*(totElems-2)/6);
  }
  else
  {
    env.out() << "Cannot create tuples with length > 3" << std::endl;
    throw NotImplementedError();
  }
  return allJobTuples;
}

/**
 * Solve robust parallel scheduling problem with scenario enumeration.
 *
 * Solve robust parallel scheduling problem with scenario enumeration.
 * This method adds all of the scenarios at once and solves one model,
 * as opposed to scenario generation which solves the model many times
 * but only adds one scenario per iteration.
 *
 * @param env Environment variable
 * @param nbJobs Number of jobs
 * @param nbMachines Number of machines
 * @param releaseTimes Release times of job j
 * @param processingTimes Processing time on machine i of job j
 * @param maxMchDelays Maximum number of delays for jobs on machine i
 * @param maxDelays Maximum number of delays over all jobs collectively
 * @param outputDir Directory to output results
 * @param filename Name of input data file
 * @param numWorkers Maximum number of cores used by solver
 * @param timeLimit Global time limit of algorithm
 * @param failLimit Maximum number of failues allowed per search
 */
static void solveWithScenarioEnumeration(
  IloEnv                        env,
  IloInt                        nbJobs,
  IloInt                        nbMachines,
  IloIntArray                   releaseTimes,
  IloIntArray2                  processingTimes,
  ObjFuncType                   objType,
  IloIntArray                   maxMchDelays,
  IloInt                        maxDelays,
  std::string                   outputDir,
  std::string                   filename,
  IloInt                        numWorkers,
  IloNum                        timeLimit,
  IloInt                        failLimit)
{
  // Global timer
  IloTimer globalTimer(env);
  globalTimer.restart();
  // SCENARIO ENUMERATION
  IloIntervalVarArray delayedJobs(env);
  IloIntervalVarArray2 jobs;
  IloIntervalSequenceVarArray sequences;
  IloIntExpr objExpr;
  IloModel bigModel = makeSimpleModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, objType, delayedJobs, jobs, sequences, objExpr);
  IloArray<IloArray<IloIntArray> > delayScenarios = generateJobTuples(env, nbJobs, nbMachines, maxDelays);
  IloIntExprArray objExprArray(env);
  objExprArray.add(objExpr);
  for (IloInt k = 0; k < delayScenarios.getSize(); k++)
  {
    // Populate delayedJobs
    IloIntervalVarArray delayedJobs(env);
    for (IloInt t = 0; t < maxDelays; t++)
    {
      std::ostringstream name;
      name << "m" << delayScenarios[k][t][0] << "_j" << delayScenarios[k][t][1];
      delayedJobs.add(IloIntervalVar(env, 0, name.str().c_str()));
    }
    // Create and add submodel to master
    IloIntervalVarArray2 submodelJobs;
    IloIntervalSequenceVarArray submodelSequences;
    IloIntExpr submodelObjExpr;
    IloModel subModel = makeSimpleModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, objType, delayedJobs, submodelJobs, submodelSequences, submodelObjExpr);
    bigModel.add(subModel);
    // Add constraints linking submodel and master
    for (IloInt i = 0; i < nbMachines; i++)
    {
      bigModel.add(IloSameSequence(env, sequences[i], submodelSequences[i]));
      for (IloInt j = 0; j < nbJobs; j++)
      {
        bigModel.add(IloPresenceOf(env, jobs[i][j]) == IloPresenceOf(env, submodelJobs[i][j]));
      }
    }
    // Add submodel objective to objArray
    objExprArray.add(submodelObjExpr);
  }
  IloObjective objective = IloMinimize(env, IloMax(objExprArray));
  bigModel.add(objective);

  IloCP cp(bigModel);
  cp.setParameter(IloCP::TimeLimit, timeLimit);
  cp.setParameter(IloCP::FailLimit, failLimit);
  cp.setParameter(IloCP::Workers, numWorkers);
  // Algorithm paramters
  cp.setParameter(IloCP::NoOverlapInferenceLevel, IloCP::Extended);
  // Solve and report solution
  cp.solve();
  reportMaster(cp, sequences);
  std::ofstream statsFile(outputDir + "/statsEnum.csv");
  statsFile << "Phase,ObjVal,SolveTime,TotalTime" << std::endl;
  statsFile << "M0," << cp.getObjValue() << "," << cp.getInfo(IloCP::SolveTime) << "," << globalTimer.getTime() << std::endl;
  statsFile.flush();
}

/**
 * Solve robust parallel scheduling problem with scenario generation.
 *
 * Solve robust parallel scheduling problem with scenario generation.
 * This method adds one scenario at a time and re-solves the master
 * problem until it finds an optimal solution to the original problem.
 *
 * @param env Environment variable
 * @param nbJobs Number of jobs
 * @param nbMachines Number of machines
 * @param releaseTimes Release times of job j
 * @param processingTimes Processing time on machine i of job j
 * @param maxMchDelays Maximum number of delays for jobs on machine i
 * @param maxDelays Maximum number of delays over all jobs collectively
 * @param outputDir Directory to output results
 * @param filename Name of input data file
 * @param numWorkers Maximum number of cores used by solver
 * @param timeLimit Global time limit of algorithm
 * @param failLimit Maximum number of failues allowed per search
 */
static void solveWithScenarioGeneration(
  IloEnv                        env,
  IloInt                        nbJobs,
  IloInt                        nbMachines,
  IloIntArray                   releaseTimes,
  IloIntArray2                  processingTimes,
  ObjFuncType                   objType,
  IloIntArray                   maxMchDelays,
  IloInt                        maxDelays,
  std::string                   outputDir,
  std::string                   filename,
  IloInt                        numWorkers,
  IloNum                        timeLimit,
  IloInt                        failLimit)
{
  // Global timer
  IloTimer globalTimer(env);
  globalTimer.restart();
  IloNum localTimeLimit = 0;
  // SCENARIO GENERATION
  IloIntervalVarArray2 allDelayedJobs(env);
  allDelayedJobs.add(IloIntervalVarArray(env));

  IloIntervalVarArray2 masterJobs;
  IloIntervalSequenceVarArray masterSequences;
  IloIntExpr masterObjExpr;
  IloModel masterModel = makeSimpleModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, objType, allDelayedJobs[0], masterJobs, masterSequences, masterObjExpr);
  IloIntExprArray objExprArray(env);
  objExprArray.add(masterObjExpr);
  IloObjective masterObjective = IloMinimize(env, IloMax(objExprArray));
  masterModel.add(masterObjective);
  IloCP masterCP(masterModel);
  masterCP.setParameter(IloCP::FailLimit, failLimit);
  masterCP.setParameter(IloCP::Workers, numWorkers);
  // Algorithm paramters
  masterCP.setParameter(IloCP::NoOverlapInferenceLevel, IloCP::Extended);

  IloConstraintArray2 solnJobPres(env, nbMachines);
  IloConstraintArray2 solnOrder(env, nbMachines);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    solnJobPres[i] = IloConstraintArray(env);
    solnOrder[i] = IloConstraintArray(env);
  }
  IloIntervalVarArray2 scenDelays;
  IloIntervalVarArray2 scenJobs;
  IloIntervalSequenceVarArray scenSequences;
  IloModel sgModel = makeScenGenModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, objType, maxMchDelays, maxDelays, scenDelays, scenJobs, scenSequences);
  IloCP scenCP(sgModel);
  scenCP.setParameter(IloCP::FailLimit, failLimit);
  scenCP.setParameter(IloCP::Workers, numWorkers);
  // Algorithm paramters
  scenCP.setParameter(IloCP::NoOverlapInferenceLevel, IloCP::Extended);

  std::ofstream statsFile(outputDir + "/stats.csv");
  statsFile << "Phase,ObjVal,SolveTime,TotalTime" << std::endl;
  for(IloInt currIter = 0 ; ; currIter++)
  {
    masterCP.out() << "ITERATION " << currIter << std::endl;
    // Optimize over current master problem
    localTimeLimit = IloMax(0, timeLimit - globalTimer.getTime());
    masterCP.setParameter(IloCP::TimeLimit, localTimeLimit);
    masterCP.solve();
    if (globalTimer.getTime() >= timeLimit)
    {
      env.out() << "Exceeded global time limit." << std::endl;
      break;
    }
    if (masterCP.getInfo(IloCP::FailStatus) == IloCP::SearchStoppedByLimit)
    // Starting with CPLEX 12.9.0, use the following instead:
    //if (masterCP.getInfo(IloCP::SearchStatus) == IloCP::SearchStopped && masterCP.getInfo(IloCP::SearchStopCause) == IloCP::SearchStoppedByLimit)
    {
      env.out() << "Search stopped by limit." << std::endl;
      break;
    }
    IloNum masterObjVal = masterCP.getObjValue();
    reportMaster(masterCP, masterSequences);
    std::ofstream masterSolnFile(outputDir + "/soln" + std::to_string(currIter) + ".out");
    reportMaster(masterCP, masterSequences, masterSolnFile);
    masterSolnFile.close();
    statsFile << "M" << currIter << "," << masterCP.getObjValue() << "," << masterCP.getInfo(IloCP::SolveTime) << "," << globalTimer.getTime() << std::endl;
    statsFile.flush();

    // Update scenario generation submodel based on master solution
    for (IloInt i = 0; i < nbMachines; i++)
    {
      for (IloInt k = 0; k < solnJobPres[i].getSize(); k++)
      {
        sgModel.remove(solnJobPres[i][k]);
      }
      solnJobPres[i].clear();
      assert( solnJobPres[i].getSize() == 0 );
      for (IloInt j = 0; j < nbJobs; j++)
      {
        IloConstraint cc = (IloPresenceOf(env, scenJobs[i][j]) == masterCP.isPresent(masterJobs[i][j]));
        solnJobPres[i].add(cc);
        sgModel.add(cc);
      }
    }
    for (IloInt i = 0; i < nbMachines; i++)
    {
      for (IloInt k = 0; k < solnOrder[i].getSize(); k++)
      {
        sgModel.remove(solnOrder[i][k]);
      }
      solnOrder[i].clear();
      assert( solnOrder[i].getSize() == 0 );
      IloIntervalSequenceVar seq = masterSequences[i];
      IloInt prec = -1;
      for (IloIntervalVar itv = masterCP.getFirst(seq); itv.getImpl() != 0; itv = masterCP.getNext(seq, itv))
      {
        std::string itvname = itv.getName();
        IloInt curr = jobOf(itvname);
        if(prec != -1)
        {
          IloConstraint cc = IloPrevious(env, scenSequences[i], scenJobs[i][prec], scenJobs[i][curr]);
          solnOrder[i].add(cc);
          sgModel.add(cc);
        }
        prec = curr;
      }
    }
    
    // Solve scenario generation submodel
    localTimeLimit = IloMax(0, timeLimit - globalTimer.getTime());
    scenCP.setParameter(IloCP::TimeLimit, localTimeLimit);
    scenCP.solve();
    if (globalTimer.getTime() >= timeLimit)
    {
      env.out() << "Exceeded global time limit." << std::endl;
      break;
    }
    if (masterCP.getInfo(IloCP::FailStatus) == IloCP::SearchStoppedByLimit)
    // Starting with CPLEX 12.9.0, use the following instead:
    //if (masterCP.getInfo(IloCP::SearchStatus) == IloCP::SearchStopped && masterCP.getInfo(IloCP::SearchStopCause) == IloCP::SearchStoppedByLimit)
    {
      env.out() << "Search stopped by limit." << std::endl;
      break;
    }
    reportSubmodel(scenCP, scenDelays);
    std::ofstream submodelSolnFile(outputDir + "/delays.out", std::ios_base::app);
    recordDelay(scenCP, currIter, scenDelays, submodelSolnFile);
    submodelSolnFile.close();
    statsFile << "O" << currIter << "," << scenCP.getObjValue() << "," << scenCP.getInfo(IloCP::SolveTime) << "," << globalTimer.getTime() << std::endl;
    statsFile.flush();

    // If optimal value of submodel is no worse than master, we are done.
    IloNum sgObjVal = scenCP.getObjValue();
    if (sgObjVal <= masterObjVal)
    {
      break;
    }
    // Else...,
    // Extract scenario
    IloIntervalVarArray delayedJobs(env);
    for (IloInt i = 0; i < nbMachines; i++)
    {
      for (IloInt j = 0; j < nbJobs; j++)
      {
        if (scenCP.isPresent(scenDelays[i][j]))
        {
          delayedJobs.add(scenDelays[i][j]);
        }
      }
    }
    allDelayedJobs.add(delayedJobs);
    // Add scenario subproblem to master
    IloIntervalVarArray2 submodelJobs;
    IloIntervalSequenceVarArray submodelSequences;
    IloIntExpr submodelObjExpr;
    IloModel subModel = makeSimpleModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, objType, delayedJobs, submodelJobs, submodelSequences, submodelObjExpr);
    masterModel.add(subModel);
    // Constraints linking subproblem and master
    for (IloInt i = 0; i < nbMachines; i++)
    {
      masterModel.add(IloSameSequence(env, masterSequences[i], submodelSequences[i]));
      for (IloInt j = 0; j < nbJobs; j++)
      {
        masterModel.add(IloPresenceOf(env, masterJobs[i][j]) == IloPresenceOf(env, submodelJobs[i][j]));
      }
    }
    masterModel.remove(masterObjective);
    objExprArray.add(submodelObjExpr);
    masterObjective = IloMinimize(env, IloMax(objExprArray));
    masterModel.add(masterObjective);
  }
  statsFile.close();

  // Display statistics
  env.out() << "Final Objective: " << masterCP.getObjValue() << std::endl;
  env.out() << "Num Generated Scenarios: " << allDelayedJobs.getSize() << std::endl;
}

int main(int argc, const char* argv[])
{
  // Parse command line
  const IloInt numWorkersDefaultValue = 1;
  const IloNum timeLimitDefaultValue = 3600;
  const IloInt failLimitDefaultValue = IloIntMax;
  const std::string dataDirDefaultValue = "data";
  const std::string outputDirDefaultValue = ".";
  args::ArgumentParser parser("Robust parallel machine scheduling.");
  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::ValueFlag<IloInt> numWorkersP(parser, "numWorkers", "Number of cores to use", {'W', "numWorkers"}, numWorkersDefaultValue);
  args::ValueFlag<IloNum> timeLimitP(parser, "timeLimit", "Maximum number of seconds spent during search", {'T', "timeLimit"}, timeLimitDefaultValue);
  args::ValueFlag<IloInt> failLimitP(parser, "failLimit", "Maximum number of failures during search", {'F', "failLimit"}, failLimitDefaultValue);
  args::ValueFlag<std::string> dataDirP(parser, "dataDir", "Directory containing data files", {"dataDir"}, dataDirDefaultValue);
  args::ValueFlag<std::string> outputDirP(parser, "outputDir", "Directory to store output", {"outputDir"}, outputDirDefaultValue);
  args::Group objGroup(parser, "Objective (specify EXACTLY one)", args::Group::Validators::Xor);
  args::Flag cmaxF(objGroup, "cmax", "CMAX objective", {"cmax"});
  args::Flag swctF(objGroup, "swct", "SWCT objective", {"swct"});
  args::Flag solveWithEnumF(parser, "solveWithEnum", "Solve using scenario enumeration instead of generation", {"solveWithEnum"});
  args::Positional<std::string> filenameP(parser, "file", "Input filename", "default");
  args::Positional<int> maxNbDelaysP(parser, "maxNbDelays", "Maximum number of delays", 1);
  try
  {
    parser.ParseCLI(argc, argv);
  }
  catch (args::Help)
  {
    std::cout << parser;
    return 0;
  }
  catch (args::ParseError e)
  {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }
  catch (args::ValidationError e)
  {
    if(cmaxF == swctF)
    {
      std::cerr << "Must specify EXACTLY ONE of --cmax or --swct." << std::endl;
    }
    else
    {
      std::cerr << e.what() << std::endl;
    }
    std::cerr << parser;
    return 1;
  }

  IloEnv env;
  try
  {
    // Get command line parameters
    IloInt numWorkers = args::get(numWorkersP);
    IloNum timeLimit = args::get(timeLimitP);
    IloInt failLimit = args::get(failLimitP);
    std::string dataDir = args::get(dataDirP);
    std::string outputDir = args::get(outputDirP);
    std::string filename = args::get(filenameP);
    IloInt maxNbDelays = args::get(maxNbDelaysP);
    ObjFuncType objType;
    if (cmaxF)
      objType = ObjFuncType::cmax;
    else if (swctF)
      objType = ObjFuncType::swct;
    else
      throw NotImplementedError();
    IloBool solveWithEnum = solveWithEnumF;

    // Open input data file
    std::ifstream file(dataDir + "/" + filename + ".data");
    if (!file)
    {
      throw FileError();
    }

    // Read in input data from file
    IloInt nbJobs, nbMachines;
    file >> nbJobs;
    file >> nbMachines;
    IloIntArray releaseTimes(env, nbJobs);
    for (IloInt j = 0; j < nbJobs; j++)
    {
      file >> releaseTimes[j];
    }
    IloIntArray2 processingTimes(env, nbMachines);
    for (IloInt i = 0; i < nbMachines; i++)
    {
      processingTimes[i] = IloIntArray(env, nbJobs);
      for (IloInt j = 0; j < nbJobs; j++)
      {
        file >> processingTimes[i][j];
      }
    }
    file.close();

    // Set other parameterized input data
    IloIntArray maxMchDelays(env, nbMachines);
    for(IloInt i = 0; i < nbMachines; i++)
    {
      maxMchDelays[i] = maxNbDelays;
    }
    IloInt maxDelays = maxNbDelays;

    // Output basic info
    env.out() << "Instance \t: " << filename << std::endl;
    env.out() << "NumJobs  \t: " << nbJobs << std::endl;
    env.out() << "NumMachines\t: " << nbMachines << std::endl;
    env.out() << "NumDelays\t: " << maxNbDelays << std::endl;
    // Write command to file
    std::ofstream cmdLineCopy(outputDir + "/" + filename + ".command");
    cmdLineCopy << argv[0];
    if (objType == ObjFuncType::cmax)
      cmdLineCopy << " --cmax";
    else if (objType == ObjFuncType::swct)
      cmdLineCopy << " --swct";
    else
      throw NotImplementedError();
    if (solveWithEnum)
      cmdLineCopy << " --solveWithEnum";
    if (numWorkers != numWorkersDefaultValue)
      cmdLineCopy << " --numWorkers " << numWorkers;
    if (timeLimit != timeLimitDefaultValue)
      cmdLineCopy << " --timeLimit " << timeLimit;
    if (failLimit != failLimitDefaultValue)
      cmdLineCopy << " --failLimit " << failLimit;
    if (outputDir != outputDirDefaultValue)
      cmdLineCopy << " --outputDir " << outputDir;
    cmdLineCopy << " " << filename << " " << maxNbDelays << std::endl;
    cmdLineCopy.close();

    if (solveWithEnum)
        solveWithScenarioEnumeration(env, nbJobs, nbMachines, releaseTimes, processingTimes, objType, maxMchDelays, maxDelays, outputDir, filename, numWorkers, timeLimit, failLimit);
    else
        solveWithScenarioGeneration(env, nbJobs, nbMachines, releaseTimes, processingTimes, objType, maxMchDelays, maxDelays, outputDir, filename, numWorkers, timeLimit, failLimit);

  }
  catch(const IloException& e)
  {
    std::cerr << "CPO Error: " << e << std::endl;
  }
  catch(...)
  {
    std::cerr << "Unknown Error" << std::endl;
  }
  env.end();
  return 0;
}
