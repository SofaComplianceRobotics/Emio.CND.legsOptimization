import logging
import os
from optuna.trial import TrialState, FrozenTrial
from optuna.study import Study
from optimization.parameters import low_leg, high_leg, center_part
from optimization.processing import exportSimulationData,exportTrialParams

#Data Handling


class LogCompletedTrial:
    def __init__(self) -> None:
        self._trial_count = 0

    def __call__(self,study:Study,trial:FrozenTrial)->None:
        logging.basicConfig(level=logging.INFO,
                            force=True,
                            format='%(asctime)s - %(message)s',
                            datefmt='%Y-%m-%d %H:%M:%S')
        logging.getLogger().addHandler(logging.StreamHandler())
        if trial.state == TrialState.COMPLETE and not TrialState.PRUNED:
            self._trial_count += 1
            logging.info("Completed Trial %d, id = %d, value = %f and parameters : %s \n",self._trial_count,trial.number,trial.value,trial.params)

class SaveCompletedTrial:
    def __init__(self,study_name:str,savePlot:bool=True,saveSTL:bool=True,saveParams:bool=True) -> None:
        self.savePlot = savePlot
        self.saveSTL = saveSTL
        self.saveParams = saveParams
        os.chdir(os.path.dirname(os.path.realpath(__file__))+'/../')
        os.makedirs('results/'+study_name)

    def __call__(self,study:Study,trial:FrozenTrial)->None:
        if trial.state == TrialState.COMPLETE:
            os.chdir(os.path.dirname(os.path.realpath(__file__))+'/../')
            os.makedirs(f'results/{study.study_name}/{trial.number}')
            savePath = os.path.dirname(os.path.realpath(__file__))+f'/../results/{study.study_name}/{trial.number}/'
            low_leg.update(params=trial.params)
            high_leg.update(params=trial.params)

            if self.savePlot:
                low_leg.centerLine.exportPlot(path = savePath,
                                           title=f'Leg : {low_leg.centerLine.name}, study : {study.study_name}, trial : {trial.number}')
                high_leg.centerLine.exportPlot(path = savePath,
                                           title=f'Leg : {high_leg.centerLine.name}, study : {study.study_name}, trial : {trial.number}')
            
            if self.saveSTL:
                low_leg.exportSTLCadQuery(path = savePath)
                high_leg.exportSTLCadQuery(path = savePath)

            if self.saveParams:
                data = {}
                data[low_leg.centerLine.name] = low_leg.centerLine.getControlPoints()
                data[high_leg.centerLine.name] = high_leg.centerLine.getControlPoints()
                #exportSimulationData(data,savePath + 'params.json')
                exportTrialParams(low_leg,high_leg,center_part,'params.json')

class RepeatTrial:
    def __init__(self,maxIteration:int):
        self.maxIteration = maxIteration
        self.loops = 0
    def __call__(self,study:Study,trial:FrozenTrial)->None:
        if trial.state == TrialState.PRUNED: 
            if self.loops < self.maxIteration:
                study.enqueue_trial(trial.params)
                self.loops +=1
            else:
                self.loops = 0
