#include <iostream>
#include <sstream>
#include <ilcp/cp.h>
#include "args.hxx"

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

/**
 * Creates scenario model for use in logic-based Benders.
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
IloModel makeScenModel(
  IloEnv                        env,
  IloInt                        nbJobs,
  IloInt                        nbMachines,
  IloIntArray                   releaseTimes,
  IloIntArray2                  processingTimes,
  IloIntervalSequenceVarArray&  sequences,
  IloIntExpr&                   objExpr)
{
  IloModel scenModel(env);
  // VARIABLES
  // Variables instantiating each job on each machine
  IloIntervalVarArray2 jobs(env, nbMachines);
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
    std::ostringstream name;
    name << "Mch" << i;
    sequences[i] = IloIntervalSequenceVar(env, jobs[i], name.str().c_str());
  }
  // Variables recording where jobs actually end up
  IloIntervalVarArray actualJobs(env, nbJobs);
  for (IloInt j = 0; j < nbJobs; j++)
  {
    std::ostringstream name;
    name << "Job" << j;
    actualJobs[j] = IloIntervalVar(env, name.str().c_str());
  }

  // CONSTRAINTS
  for (IloInt i = 0; i < nbMachines; i++)
  {
    std::ostringstream name;
    name << "NoOverlap" << i;
    scenModel.add(IloNoOverlap(env, sequences[i], 0, name.str().c_str()));
  }
  for (IloInt j = 0; j < nbJobs; j++)
  {
    IloIntervalVarArray jobChoices(env, nbMachines);
    for (IloInt i = 0; i < nbMachines; i++)
    {
      jobChoices[i] = jobs[i][j];
    }
    std::ostringstream name;
    name << "Altern" << j;
    scenModel.add(IloAlternative(env, actualJobs[j], jobChoices, name.str().c_str()));
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
  IloIntervalSequenceVarArray&  sequences)
{
  IloModel model(env);
  // Leverage makeScenModel to do most of the work for us.
  IloIntExpr objExpr;
  IloModel submodel = makeScenModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, sequences, objExpr);
  model.add(submodel);

  // Objective
  IloObjective objective = IloMinimize(env, objExpr);
  model.add(objective);

  return model;
}

IloModel makeScenGenModel(
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
  // Worst scenario subject to b_i breakdowns on machine i
  // Total no more than X Breaks overall
  IloModel model(env);

  // VARIABLES
  // For each job in solution, create fixed job (on machine), delay job, and super job
  IloIntervalVarArray2 machJobs(env, nbMachines);
  delays = IloIntervalVarArray2(env, nbMachines);
  IloIntervalVarArray2 superJobs(env, nbMachines);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    machJobs[i] = IloIntervalVarArray(env);
    delays[i] = IloIntervalVarArray(env);
    superJobs[i] = IloIntervalVarArray(env);
  }
  for (IloSolutionIterator<IloIntervalSequenceVar> it(soln); it.ok(); ++it)
  {
    IloIntervalSequenceVar seq = *it;
    for (IloIntervalVar itv = soln.getFirst(seq); itv.getImpl() != 0; itv = soln.getNext(seq, itv))
    {
      std::string itvname = itv.getName();
      IloInt mchNum = mchOf(itvname);
      IloInt jobNum = jobOf(itvname);
      // Create fixed job
      IloIntervalVar fixedJob(env, soln.getSize(itv));
      machJobs[mchNum].add(fixedJob);
      // Create delay job
      std::ostringstream name;
      name << "m" << mchNum << "_j" << jobNum;
      IloIntervalVar jobDelay(env, soln.getSize(itv)-1, name.str().c_str());
      jobDelay.setOptional();
      delays[mchNum].add(jobDelay);
      // Create super job
      IloIntervalVar sj(env);
      superJobs[mchNum].add(sj);
      // CONSTRAINT: Length of super job is length of fixed + length of delay
      IloIntervalVarArray combo(env, 2);
      combo[0] = jobDelay;
      combo[1] = fixedJob;
      model.add(IloSpan(env, sj, combo));
      // CONSTRAINT: Fixed job is immediately preceded by its delay.
      model.add(IloEndAtStart(env, jobDelay, fixedJob));
    }
  }
  // For each machine, a sequence variable to enforce sequence consistency
  IloIntervalSequenceVarArray sequences(env, nbMachines);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    sequences[i] = IloIntervalSequenceVar(env, superJobs[i]);
  }

  // CONSTRAINTS
  // No overlap constraints
  for (IloInt i = 0; i < nbMachines; i++)
  {
    model.add(IloNoOverlap(env, sequences[i]));
    for (IloInt j = 0; j < superJobs[i].getSize(); j++)
    {
      // Sequence binding constraints
      if (j == 0)
      {
        model.add(IloStartOf(superJobs[i][j]) == 0);
      }
      else
      {
        model.add(IloBefore(env, sequences[i], superJobs[i][j-1], superJobs[i][j]));
        model.add( IloStartOf(superJobs[i][j]) == IloEndOf(superJobs[i][j-1]) );
      }
    }
  }
  // Delay constraints
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
  IloIntExpr allPresences(env);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    allPresences += presences[i];
  }
  model.add(allPresences <= maxDelays);


  // OBJECTIVE
  IloIntExprArray ends(env);
  for (IloInt i = 0; i < nbMachines; i++)
  {
    for (IloInt j = 0; j < superJobs[i].getSize(); j++)
    {
      ends.add(IloEndOf(superJobs[i][j]));
    }
  }
  IloObjective objective = IloMaximize(env, IloMax(ends));
  model.add(objective);

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

/*
    // Master model
    IloModel model(env);
    IloArray <IloIntervalVarArray> allJobs(env);
    IloIntExprArray allObjExpr(env);

    // Create and add scenario model
    IloIntervalVarArray scenarioJobs;
    IloIntExpr scenarioObjExpr;
    IloModel scenario = makeScenModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, scenarioJobs, scenarioObjExpr);
    model.add(scenario);
    allJobs.add(scenarioJobs);
    allObjExpr.add(scenarioObjExpr);

    // Master objective
    IloObjective objective = IloMinimize(env,IloMax(allObjExpr));
    model.add(objective);
*/
    IloIntervalSequenceVarArray sequences;
    IloModel model = makeDetrModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, sequences);

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
      currSoln.add(sequences);
      for (IloInt i = 0; i < sequences.getSize(); i++)
      {
        IloIntervalSequenceVar seq = sequences[i];
        for (IloIntervalVar itv = cp.getFirst(seq); itv.getImpl() != 0; itv = cp.getNext(seq, itv))
        {
          currSoln.add(itv);
        }
      }
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
      IloModel sgModel = makeScenGenModel(env, nbJobs, nbMachines, releaseTimes, processingTimes, maxMchDelays, maxDelays, currSoln, delays);
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
