# pyright: reportMissingImports=false, reportAttributeAccessIssue=false
# pylint: disable=import-error
import numpy as np
import optuna

import Sofa
import SofaRuntime

# import param
from optimization.parameters import low_leg, high_leg, center_part, movements
from scene import createScene

def objective(trial)->float:
    
    # Generate study samples
    generateSamples(trial)

    # Create the Sofa simulation scene
    SofaRuntime.importPlugin('Sofa.Component')
    SofaRuntime.importPlugin('Sofa.GUI.Component')
    SofaRuntime.importPlugin('Sofa.GL.Component')

    root = Sofa.Core.Node("root")
    root = createScene(root)

    if root is None:
        trial.report(140,0)
        trial.set_user_attr('prune','Unfeasible shape')
        raise optuna.TrialPruned()

    Sofa.Simulation.init(root)

    Sofa.Simulation.animateNSteps(root,n_steps=movements.steps[0])

    score_per_movement=[0]*(len(movements.steps)-1)

    #print(root.Modelling.Target.getMechanicalState().position.value[0][1])
    print(movements.steps)
    for idx in range(len(movements.steps)-1):

        remaining_steps = range(movements.steps[idx]+1,movements.steps[idx+1]+1)
        print(remaining_steps)
        for step in remaining_steps:
            Sofa.Simulation.animate(root, 0)
            if step == (remaining_steps[-1]+remaining_steps[0]+1)/2:
                score_per_movement[idx] = getScore(root)

            #print(f'({step}, {root.Modelling.Target.getMechanicalState().position.value[0][1]})')
            trial.report(getScore(root), step)

            # if trial.should_prune():
            #     Sofa.Simulation.unload(root)
            #     trial.set_user_attr('prune','Unattained target')
            #     raise optuna.TrialPruned()
    
    Sofa.Simulation.unload(root)
    
    return getCumulatedScore(score_per_movement)

def getScore(node)->float:

    desired_position = node.Modelling.Target.getMechanicalState().position.value[0][0:3]
    maximum_position = node.Simulation.Sam.CenterPart.getMechanicalState().position.value[0][0:3]
    return np.linalg.norm(np.subtract(desired_position,maximum_position))-center_part.legHeightDifference/2

def getCumulatedScore(scores)->float:
    
    return np.linalg.norm(scores)

def generateSamples(trial)->None:
    # param.sLPulley = trial.suggest_float(name = sCL.name+'Pulley',
    #                                      low = 5,
    #                                      high = 10,
    #                                      step = 5)
    # param.lLPulley = trial.suggest_float(name = lCL.name+'Pulley',
    #                                      low = 5,
    #                                      high = 10,
    #                                      step = 5)
    # param.sLMotor = trial.suggest_float(name = sCL.name+'Motor',
    #                                      low = 70,
    #                                      high = 120,
    #                                      step = 5)
    # param.lLMotor = trial.suggest_float(name = lCL.name+'Motor',
    #                                      low = 70,
    #                                      high = 120,
    #                                      step = 5)

    for idx in range(1,low_leg.centerLine.nPts-1):
    #for idx in range(low_leg.centerLine.nPts):
        #small Leg : Right Handle Length
        sRL = trial.suggest_float(name = low_leg.name+'Pt.rL'+str(idx),
                                  low = 10.0,
                                  high = 100.0)
        #small Leg : Left Handle Length
        # sLL = trial.suggest_float(name = sCL.name+'Pt.lL'+str(idx),
        #                           low = 10.0,
        #                           high = 100.0)
        
        #leg update
        # sCL.points[idx].update(rightHandleLength = sRL,
        #                        leftHandleLength = sLL)
        
        if idx == 1:
            low_leg.centerLine.points[idx].update(rightHandleLength = sRL,
                                                  sameHandleLength=False)
        if idx == low_leg.centerLine.nPts-2:
            low_leg.centerLine.points[idx].update(rightHandleLength = 1,
                                               leftHandleLength = sRL)
        
        if idx > 1 and idx < (low_leg.centerLine.nPts-2):
        #if idx > 0 and idx < (low_leg.centerLine.nPts-1):
            low_leg.centerLine.points[idx].update(rightHandleLength = sRL,
                                                sameHandleLength = True)
            #small Leg : x coordinate
            sX  = trial.suggest_float(name = low_leg.name+'Pt.x'+str(idx),
                                      low = low_leg.centerLine.startCoordinates[1],
                                      high = low_leg.centerLine.endCoordinates[1])
            #small Leg : y coordinate
            sY  = trial.suggest_float(name = low_leg.name+'Pt.y'+str(idx),
                                      low = low_leg.centerLine.startCoordinates[0],
                                      high = low_leg.centerLine.endCoordinates[0])
            #small Leg : Right Handle Angle
            sRA = trial.suggest_float(name = low_leg.name+'Pt.rA'+str(idx),
                                      low = -80,
                                      high = -20)
            #small Leg : Left Handle Angle
            # sLA = trial.suggest_float(name = sLegCL.name+'Pt.lA'+str(idx),
            #                            low = 90,
            #                            high = 180)
            
            # param.sLegCL.point[idx].update(coordinates=[sX,sY],
            #                                rightHandleAngle=sRA,
            #                                leftHandleAngle=sLA)
            low_leg.centerLine.points[idx].update(coordinates = [sX,sY],
                                               rightHandleAngle = sRA,
                                               angleUnit = 'deg')
        

    for idx in range(1,high_leg.centerLine.nPts-1):
    #for idx in range(high_leg.centerLine.nPts):
        #small Leg : Right Handle Length
        lRL = trial.suggest_float(name = high_leg.name+'Pt.rL'+str(idx),
                                  low = 10.0,
                                  high = 100.0)
        #small Leg : Left Handle Length
        # lLL = trial.suggest_float(name = lCL.name+'Pt.lL'+str(idx),
        #                           low = 10.0,
        #                           high = 100.0)
        # lCL.points[idx].update(rightHandleLength=lRL,
        #                        leftHandleLength=lLL)
        if idx == 1:
            high_leg.centerLine.points[idx].update(rightHandleLength = lRL,
                                               sameHandleLength=False)
        if idx == high_leg.centerLine.nPts-2:
            high_leg.centerLine.points[idx].update(rightHandleLength = 1,
                                               leftHandleLength = lRL)

        if idx > 1 and idx < (high_leg.centerLine.nPts-2):
        #if idx > 0 and idx < (high_leg.centerLine.nPts-1):
            high_leg.centerLine.points[idx].update(rightHandleLength = lRL,
                                                sameHandleLength = True)
            #small Leg : x coordinate
            lX  = trial.suggest_float(name = high_leg.name+'Pt.x'+str(idx),
                                      low = high_leg.centerLine.startCoordinates[0],
                                      high = high_leg.centerLine.endCoordinates[0])
            #small Leg : y coordinate
            lY  = trial.suggest_float(name = high_leg.name+'Pt.y'+str(idx),
                                      low = high_leg.centerLine.startCoordinates[1],
                                      high = high_leg.centerLine.endCoordinates[1])
            #small Leg : Right Handle Angle
            lRA = trial.suggest_float(name = high_leg.name+'Pt.rA'+str(idx),
                                      low = -80,
                                      high = -20)
            #small Leg : Left Handle Angle
            # lLA = trial.suggest_float(name = lLegCL.name+'Pt.lA'+str(idx),
            #                           low = 90,
            #                           high = 180)
            # param.lLegCL.point[idx].update(coordinates=[lX,lY],
            #                                rightHandleAngle=lRA,
            #                                leftHandleAngle=lLA)
            high_leg.centerLine.points[idx].update(coordinates = [lX,lY],
                                               rightHandleAngle = lRA,
                                               angleUnit = 'deg')
    high_leg.update()
    low_leg.update()
    high_leg.centerLine.exportPlot(name=f'{high_leg.name}-{trial.number}')
    low_leg.centerLine.exportPlot(name=f'{low_leg.name}-{trial.number}')

    