import logging
from json import loads, dump
import os
from optuna.trial import TrialState
import param

#Data Handling
def importSimulationData(filepath:str):
    with open(filepath,'r',encoding="utf-8") as f:
        data = loads(f.read())
    return data

def exportSimulationData(data:dict,filepath:str):
    with open(filepath,'w',encoding="utf-8") as f:
        dump(data,f)

def createFolder(study):
    os.chdir(os.path.dirname(os.path.realpath(__file__)))
    os.makedirs('results/'+study.study_name)

def logResults(study):
    os.chdir(os.path.dirname(os.path.realpath(__file__)))
    Trials = study.get_trials()
    nTrials = len(Trials)
    # Completed trials
    cTrials = study.get_trials(states=[TrialState.COMPLETE])
    ncTrials = len(cTrials)

    # Pruned trials
    pTrials = study.get_trials(states=[TrialState.PRUNED])
    p1Trials = [trial for trial in pTrials if trial.user_attrs['prune']=='Unfeasible shape']
    p2Trials = [trial for trial in pTrials if trial.user_attrs['prune']=='Unattained target']
    npTrials = len(pTrials)
    np1Trials = len(p1Trials)
    np2Trials = len(p2Trials)

    # Pruned trials
    nfTrials = len(study.get_trials(states=[TrialState.FAIL]))

    # Results Logging
    logging.basicConfig(level=logging.INFO,
                        format='%(message)s',
                        filename='results/'+study.study_name+'/Summary.txt',
                        filemode='w')
    logging.getLogger().addHandler(logging.StreamHandler())

    logging.info('Study                  : %s', study.study_name)
    logging.info('- Start                : %s', study.trials[0].datetime_start)
    logging.info('- End                  : %s', study.trials[-1].datetime_complete)
    logging.info('- Duration             : %s', study.trials[-1].datetime_complete-study.trials[0].datetime_start)
    logging.info('- Sampler              : %s', study.trials[-1].system_attrs["auto:sampler"])
    logging.info('- Pruner               : %s', study.pruner.__class__.__name__)
    #logging.info('- Storage              : %s', study._storage.__class__.__name__)
    logging.info('- Trials               : %d', nTrials)
    logging.info('-- Completed trials    : %d, %.2f %%',ncTrials, ncTrials/nTrials*100)
    logging.info('-- Pruned trials       : %d, %.2f %%',npTrials, npTrials/nTrials*100)
    logging.info('--- Unfeasible shapes  : %d, %.2f %%',np1Trials, np1Trials/npTrials*100)
    logging.info('--- Unattained targets : %d, %.2f %%',np2Trials, np2Trials/npTrials*100)
    logging.info('-- Failed trials       : %d, %.2f %%',nfTrials, nfTrials/nTrials*100)
    logging.info('-- Parameters          :')
    for parameter, value in study.trials[0].distributions.items():
        logging.info('                         %s\t: [%d,%d]', parameter, value.low, value.high)
    logging.info('-- Results             : ')

    if ncTrials == 0:
        logging.info('                         There are no complete trials !')
    else:
        cTrials.sort(key=lambda x:x.value)
        for trial in cTrials:
            logging.info('                         Trial number : %d, Value : %f, Duration : %s', trial.number, trial.value, str(trial.duration))
            data={}
            param.sCL.setFromOptuna(trial.params)
            param.lCL.setFromOptuna(trial.params)
            data[param.sLName] = param.sCL.getControlPoints()
            data[param.lLName] = param.lCL.getControlPoints()
            
            os.chdir(os.path.dirname(os.path.realpath(__file__)))
            os.makedirs(f'results/{study.study_name}/{trial.number}')
            savePath = os.path.dirname(os.path.realpath(__file__))+f'/results/{study.study_name}/{trial.number}/'

            exportSimulationData(data,savePath + 'params.json')
            
            param.sCL.exportSTL(path = savePath)
            param.sCL.exportPlot(path = savePath,
                                 title=f'Leg : {param.sCL.name}, study : {study.study_name}, trial : {trial.number}')
            
            param.lCL.exportSTL(path = savePath)
            param.lCL.exportPlot(path = savePath,
                                 title=f'Leg : {param.lCL.name}, study : {study.study_name}, trial : {trial.number}')
            
    study.trials_dataframe().to_csv(savePath + 'results.csv')

def appendDataset(study,name):
    os.chdir(os.path.dirname(os.path.realpath(__file__)))
    df=study.trials_dataframe()
    df=df[df.state=="COMPLETE"]
    df.to_csv(f'results/{name}', header=False, mode='a')