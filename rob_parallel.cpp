#include <iostream>
#include <sstream>
#include <ilcp/cp.h>
#include "args.hxx"

typedef IloArray<IloBoolArray> IloBoolArray2;

/**
 * CPLEX error for failing to open file.
 */
class FileError: public IloException
{
public:
  FileError() : IloException("Cannot open data file") {}
};

/**
 * Parse
 */
IloBool isBaseJob(std::string& jobName)
{
  return jobName.find("_", 2) != std::string::npos;
}

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



IloModel makeBaseModel(
  IloEnv                        env,
  IloInt                        nbJobs,
  IloInt                        nbMachines,
  IloIntArray                   releaseTimes,
  IloIntArray2                  processingTimes,
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
    }
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
  objExpr = IloMax(ends);

  return model;
}



/**
 * Creates simple scenario model for use in logic-based Benders.
 *
 * Creates scenario model using parameter data. Primarily for use with logic-based Benders and scenario generation strategy.
 *
 * @param env Environment variable
 * @param nbJobs Number of jobs
 * @param nbMachines Number of machines
 * @param releaseTimes Release times of job j
 * @param processingTimes Processing time on machine i of job j
 * @param sequences Sequence variable container for machine i's jobs
 * @param objExpr Expression for objective
 * @return Scenario model
 */
IloModel makeSimpleModel(
  IloEnv                        env,
  IloInt                        nbJobs,
  IloInt                        nbMachines,
  IloIntArray                   releaseTimes,
  IloIntArray2                  processingTimes,
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
      jobs[i][j] = job;
    }
  }
  // Variables for sequencing jobs on each machine
  sequences = IloIntervalSequenceVarArray(env, nbMachines);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    sequences[i] = IloIntervalSequenceVar(env, jobs[i]);
  }
  // Variables recording where jobs actually end up
  IloIntervalVarArray actualJobs(env, nbJobs);
  for (IloInt j = 0; j < nbJobs; j++)
  {
    actualJobs[j] = IloIntervalVar(env);
  }

  // CONSTRAINTS
  for (IloInt i = 0; i < nbMachines; i++)
  {
    scenModel.add(IloNoOverlap(env, sequences[i]));
  }
  for (IloInt j = 0; j < nbJobs; j++)
  {
    IloIntervalVarArray jobChoices(env, nbMachines);
    for (IloInt i = 0; i < nbMachines; i++)
    {
      jobChoices[i] = jobs[i][j];
    }
    scenModel.add(IloAlternative(env, actualJobs[j], jobChoices));
  }

  // Objective
  IloIntExprArray ends(env);
  for (IloInt j = 0; j < nbJobs; j++)
  {
    ends.add(IloEndOf(actualJobs[j]));
  }
  objExpr = IloMax(ends);

  return scenModel;
}

/**
 * Creates deterministic version of model.
 *
 * Creates deterministic version of scheduling model.
 *
 * @param env Environment variable
 * @param nbJobs Number of jobs
 * @param nbMachines Number of machines
 * @param releaseTimes Release times of job j
 * @param processingTimes Processing time on machine i of job j
 * @return Scenario model
 */
IloModel makeDetrModel(
  IloEnv                        env,
  IloInt                        nbJobs,
  IloInt                        nbMachines,
  IloIntArray                   releaseTimes,
  IloIntArray2                  processingTimes,
  IloIntervalVarArray2&         jobs,
  IloIntervalSequenceVarArray&  sequences)
{
  // Use makeSimpleModel as base
  IloModel model(env);
  IloIntExpr objExpr;
  IloModel submodel = makeSimpleModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, jobs, sequences, objExpr);
  model.add(submodel);

  // OBJECTIVE
  IloObjective objective = IloMinimize(env, objExpr);
  model.add(objective);

  return model;
}

IloModel makeInitialMasterModel(
  IloEnv                        env,
  IloInt                        nbJobs,
  IloInt                        nbMachines,
  IloIntArray                   releaseTimes,
  IloIntArray2                  processingTimes,
  IloIntervalVarArray2&         delays,
  IloIntervalVarArray2&         superJobs,
  IloIntervalSequenceVarArray&  sequences)
{
  // Use makeBaseModel as base
  IloModel model(env);
  IloIntExpr objExpr;
  IloModel submodel = makeBaseModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, delays, superJobs, sequences, objExpr);
  model.add(submodel);

  // OBJECTIVE
  IloObjective objective = IloMinimize(env, objExpr);
  model.add(objective);

  return model;
}

IloModel makeScenGenModel(
  IloEnv                        env,
  IloInt                        nbJobs,
  IloInt                        nbMachines,
  IloIntArray                   releaseTimes,
  IloIntArray2                  processingTimes,
  IloIntArray                   maxMchDelays,
  IloInt                        maxDelays,
  IloIntervalVarArray2&         delays,
  IloIntervalVarArray2&         superJobs,
  IloIntervalSequenceVarArray&  sequences)
{
  // Use makeBaseModel as base
  IloModel model(env);
  IloIntExpr objExpr;
  IloModel submodel = makeBaseModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, delays, superJobs, sequences, objExpr);
  model.add(submodel);

  // CONSTRAINTS
  // Sequence contiguity
  for (IloInt i = 0; i < nbMachines; i++)
  {
    for (IloInt j = 0; j < nbJobs; j++)
    {
      model.add( IloStartOf(superJobs[i][j]) == IloEndOfPrevious(sequences[i], superJobs[i][j], 0) );
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

IloModel makeWorstDelayModel(
  IloEnv                  env,
  IloInt                  nbJobs,
  IloInt                  nbMachines,
  IloIntArray             releaseTimes,
  IloIntArray2            processingTimes,
  IloIntArray             maxMchDelays,
  IloInt                  maxDelays,
  IloSolution&            soln,
  IloIntervalVarArray2&   delays)
{
  IloModel model(env);
  IloIntervalVarArray2 superJobs;
  IloIntervalSequenceVarArray sequences;
  IloModel submodel = makeScenGenModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, maxMchDelays, maxDelays, delays, superJobs, sequences);
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

int main(int argc, const char* argv[])
{
  // Parse command line
  args::ArgumentParser parser("Robust parallel machine scheduling.");
  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::ValueFlag<int> numWorkersP(parser, "numWorkers", "Number of cores to use", {'W', "numWorkers"}, 1);
  args::Positional<std::string> filenameP(parser, "file", "Input filename", "data/default.data");
  args::Positional<int> failLimitP(parser, "failLimit", "Maximum number of failues", 10000);
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

  IloEnv env;
  try
  {
    // Get command line parameters
    IloInt numWorkers = args::get(numWorkersP);
    const char* filename = args::get(filenameP).c_str();
    IloInt failLimit = args::get(failLimitP);
    std::ifstream file(filename);
    if (!file)
    {
      env.out() << "usage: " << argv[0] << " <file> <failLimit>" << std::endl;
      throw FileError();
    }

    // Read in input data
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
    // Other input data
    IloIntArray maxMchDelays(env, nbMachines);
    for(IloInt i = 0; i < nbMachines; i++)
    {
      maxMchDelays[i] = 2;
    }
    IloInt maxDelays = 3;

/*
    IloIntervalVarArray2 jobs;
    IloIntervalSequenceVarArray sequences;
    IloModel masterModel = makeDetrModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, jobs, sequences);
    IloCP cp(masterModel);
    cp.setParameter(IloCP::FailLimit, failLimit);
    cp.setParameter(IloCP::Workers, numWorkers);
    cp.out() << "Instance \t: " << filename << std::endl;

    IloBoolArray2 solnJobPres(env, nbMachines);
    IloConstraintArray2 solnOrder(env, nbMachines);
    for (IloInt i = 0; i < nbMachines; i++)
    {
      solnJobPres[i] = IloBoolArray(env, nbJobs);
      solnOrder[i] = IloConstraintArray(env);
    }
    IloIntervalVarArray2 delays;
    IloIntervalVarArray2 superJobs;
    IloIntervalSequenceVarArray scenSequences;
    IloModel sgModel = makeScenGenModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, maxMchDelays, maxDelays, solnJobPres, delays, superJobs, scenSequences);
    IloCP sgCP(sgModel);
    //sgCP.setParameter(IloCP::FailLimit, failLimit);
    sgCP.setParameter(IloCP::Workers, numWorkers);

    while(true)
    {
      // Optimize over current master problem
      cp.solve();
      IloNum masterObjVal = cp.getObjValue();

      // Update scenario generation submodel based on master solution
      for (IloInt i = 0; i < nbMachines; i++)
      {
        for (IloInt j = 0; j < nbJobs; j++)
        {
          solnJobPres[i][j] = cp.isPresent(jobs[i][j]);
        }
      }
      for (IloInt i = 0; i < nbMachines; i++)
      {
        solnOrder[i].endElements();
        //solnOrder[i].clear();
        assert( solnOrder[i].getSize() == 0 );
        IloIntervalSequenceVar seq = sequences[i];
        IloInt prec = -1;
        for (IloIntervalVar itv = cp.getFirst(seq); itv.getImpl() != 0; itv = cp.getNext(seq, itv))
        {
          if(itv != cp.getFirst(seq))
          {
            std::string itvname = itv.getName();
            IloInt curr = jobOf(itvname);
            IloConstraint cc = IloPrevious(env, scenSequences[i], superJobs[i][prec], superJobs[i][curr]);
          }
          prec = curr;
        }
      }
      
      // Solve scenario generation submodel
      sgCP.solve();
      IloNum sgObjVal = sgCP.getObjValue();
      // If optimal value is no worse we are done.
      if (sgObjVal <= masterObjVal)
      {
        break;
      }
      // Else add worst case scenario to master problem
*/

      /*
      if (sgCP.solve())
      {
        sgCP.out() << "WorstObj \t: " << sgCP.getObjValue() << std::endl;
        for (IloInt i = 0; i < nbMachines; i++)
        {
          for (IloInt j = 0; j < delays[i].getSize(); j++)
          {
            if (sgCP.isPresent(delays[i][j]))
            {
              IloIntervalVar itv = delays[i][j];
              std::string itvname = itv.getName();
              sgCP.out() << itvname << std::endl;
            }
          }
        }
      }
    }
*/
/*
    // Master model
    IloModel model(env);
    IloArray <IloIntervalVarArray> allJobs(env);
    IloIntExprArray allObjExpr(env);

    // Create and add scenario model
    IloIntervalVarArray scenarioJobs;
    IloIntExpr scenarioObjExpr;
    IloModel scenario = makeSimpleModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, scenarioJobs, scenarioObjExpr);
    model.add(scenario);
    allJobs.add(scenarioJobs);
    allObjExpr.add(scenarioObjExpr);

    // Master objective
    IloObjective objective = IloMinimize(env,IloMax(allObjExpr));
    model.add(objective);
*/
    IloIntervalVarArray2 jobs;
    IloIntervalSequenceVarArray sequences;
    IloModel model = makeDetrModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, jobs, sequences);

    // SOLVE MODEL
    IloCP cp(model);
    //cp.setParameter(IloCP::FailLimit, failLimit);
    cp.setParameter(IloCP::Workers, numWorkers);
    cp.out() << "Instance \t: " << filename << std::endl;
    if (cp.solve())
    {
      cp.out() << "Makespan \t: " << cp.getObjValue() << std::endl;
      // Get solution
      IloSolution currSoln(env);
      for (IloInt i = 0; i < nbMachines; i++)
      {
        currSoln.add(jobs[i]);
      }
      currSoln.add(sequences);
      /*
      for (IloInt i = 0; i < sequences.getSize(); i++)
      {
        IloIntervalSequenceVar seq = sequences[i];
        for (IloIntervalVar itv = cp.getFirst(seq); itv.getImpl() != 0; itv = cp.getNext(seq, itv))
        {
          currSoln.add(itv);
        }
      }
      */
      cp.store(currSoln);

      // Show solution
      for (IloSolutionIterator<IloIntervalSequenceVar> it(currSoln); it.ok(); ++it)
      {
        IloIntervalSequenceVar seq = *it;
        for (IloIntervalVar itv = cp.getFirst(seq); itv.getImpl() != 0; itv = cp.getNext(seq, itv))
        {
          std::string itvname = itv.getName();
          IloInt mchNum = mchOf(itvname);
          IloInt jobNum = jobOf(itvname);
          //cp.out() << itvname << ": " << cp.getStart(itv) << "-" << cp.getEnd(itv) << std::endl;
          cp.out() << itvname << ": " << currSoln.getStart(itv) << "-" << currSoln.getEnd(itv) << std::endl;
        }
      }
      cp.out() << std::endl;

      // Find worst case
      IloIntArray maxMchDelays(env, nbMachines);
      for(IloInt i = 0; i < nbMachines; i++)
      {
        maxMchDelays[i] = 2;
      }
      IloInt maxDelays = 3;
      IloIntervalVarArray2 delays;
      IloModel sgModel = makeWorstDelayModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, maxMchDelays, maxDelays, currSoln, delays);
      IloCP sgCP(sgModel);
      //sgCP.setParameter(IloCP::FailLimit, failLimit);
      sgCP.setParameter(IloCP::Workers, numWorkers);
      if (sgCP.solve())
      {
        sgCP.out() << "WorstObj \t: " << sgCP.getObjValue() << std::endl;
        for (IloInt i = 0; i < nbMachines; i++)
        {
          for (IloInt j = 0; j < delays[i].getSize(); j++)
          {
            if (sgCP.isPresent(delays[i][j]))
            {
              IloIntervalVar itv = delays[i][j];
              std::string itvname = itv.getName();
              sgCP.out() << itvname << std::endl;
            }
          }
        }
      }
      else
      {
        sgCP.out() << "WARNING: ScenGenModel is infeasible!!!"  << std::endl;
      }
    }
    else
    {
      cp.out() << "No solution found."  << std::endl;
    }
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
