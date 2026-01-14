import sys
import os
import importlib
import argparse

#import param
import optuna
import optunahub
from processing import logResults

# Argument parser
parser = argparse.ArgumentParser(description='Optimize an objective function.')
parser.add_argument('objective',
                    metavar='module.py',
                    type=str,
                    nargs=1,
                    help='a python script that should contain an objective ' \
                         'function with the following signature: '
                         'def objective(trial). See objective.py in this directory for example')

parser.add_argument('-n',
                    '--nTrials',
                    type=int, nargs='?',
                    help='number of trials (default: 20)',
                    default=20)

args = parser.parse_args()

# Import the objective module
print("Importing objective module from: ", args.objective[0])
module = args.objective[0]
sys.path.append(os.path.dirname(os.path.abspath(module)))
sys.path.append(os.path.dirname(os.path.abspath(sys.argv[0])))
sys.path.append(os.path.dirname(os.path.abspath(sys.argv[0]))+"/../..")
o = importlib.import_module(os.path.splitext(os.path.basename(module))[0])

# Create a study for optimization
module = optunahub.load_module(package="samplers/auto_sampler")
study = optuna.create_study(storage = optuna.storages.InMemoryStorage(),
                            sampler = module.AutoSampler(), #Sampler is determined based on the study parameters
                            pruner=optuna.pruners.ThresholdPruner(upper=5), #Prune the study if the score exceeds the threashold
                         )

# Optimize the objective function
print(o.objective.__doc__)
study.optimize(func=o.objective,
               gc_after_trial=True, #Auto garbage clean to not saturate the memory
               # n_trials=args.nTrials,
               show_progress_bar=True,
               callbacks=[optuna.study.MaxTrialsCallback(n_trials=args.nTrials,  # Keeps running until 10 trials are completed
                                                         states=[optuna.trial.TrialState.COMPLETE])]
               )

# Log optimization results
logResults(study)

# Visualize the optimization history
fig = optuna.visualization.plot_optimization_history(study)
fig.show()
