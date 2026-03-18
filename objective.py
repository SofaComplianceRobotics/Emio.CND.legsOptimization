# pyright: reportMissingImports=false, reportAttributeAccessIssue=false
# pylint: disable=import-error
from math import dist
import optuna

import Sofa
import SofaRuntime

# import param
from param import sCL, lCL, duration, durationPerOperation
import param
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
        trial.report(6,0)
        trial.set_user_attr('prune','Unfeasible shape')
        if trial.should_prune():
            raise optuna.TrialPruned()

    Sofa.Simulation.init(root)
        
    SimSteps = int(duration/root.getDt())
    pauseSteps = int(durationPerOperation[0]/root.getDt())
    remainingSteps = range(pauseSteps+1,SimSteps+1)

    Sofa.Simulation.animateNSteps(root,n_steps=pauseSteps)

    for step in remainingSteps:
        Sofa.Simulation.animate(root, 0)
        score = getScore(root)
        trial.report(score, step)
        if trial.should_prune():
            Sofa.Simulation.unload(root)
            trial.set_user_attr('prune','Unattained target')
            raise optuna.TrialPruned()
    
    sCL.exportPlot(name=f'{sCL.name}-{trial.number}')
    lCL.exportPlot(name=f'{lCL.name}-{trial.number}')
    
    Sofa.Simulation.unload(root)
    
    return score

def getScore(node)->float:

    desiredPosition = node.Modelling.TopTarget.getMechanicalState().position.value[-1][0:3]
    maximumPosition = node.Simulation.Sam.CenterPart.getMechanicalState().position.value[-1][0:3]
    score = abs(dist(desiredPosition,maximumPosition)-30)
    return score

def generateSamples(trial):
    param.sLPulley = trial.suggest_float(name = sCL.name+'Pulley',
                                         low = 5,
                                         high = 10,
                                         step = 5)
    param.lLPulley = trial.suggest_float(name = lCL.name+'Pulley',
                                         low = 5,
                                         high = 10,
                                         step = 5)
    param.sLMotor = trial.suggest_float(name = sCL.name+'Motor',
                                         low = 70,
                                         high = 120,
                                         step = 5)
    param.lLMotor = trial.suggest_float(name = lCL.name+'Motor',
                                         low = 70,
                                         high = 120,
                                         step = 5)
    for idx in range(sCL.nPts):
        #small Leg : Right Handle Length
        sRL = trial.suggest_float(name = sCL.name+'Pt.rL'+str(idx),
                                  low = 10.0,
                                  high = 100.0)
        #small Leg : Left Handle Length
        sLL = trial.suggest_float(name = sCL.name+'Pt.lL'+str(idx),
                                  low = 10.0,
                                  high = 100.0)
        
        #leg update
        sCL.points[idx].update(rightHandleLength = sRL,
                               leftHandleLength = sLL)
        #parameters.sLegCL.point[idx].update(rightHandleLength=sRl,
        #                                   sameHandleLength=True)
        if idx > 0 and idx < (sCL.nPts-1):
            #small Leg : x coordinate
            sX  = trial.suggest_float(name = sCL.name+'Pt.x'+str(idx),
                                      low = 30,
                                      high = 50.0)
            #small Leg : y coordinate
            sY  = trial.suggest_float(name = sCL.name+'Pt.y'+str(idx),
                                      low = 10,
                                      high = 30.0)
            #small Leg : Right Handle Angle
            sRA = trial.suggest_float(name = sCL.name+'Pt.rA'+str(idx),
                                      low = -80,
                                      high = -20)
            #small Leg : Left Handle Angle
            # sLA = trial.suggest_float(name = sLegCL.name+'Pt.lA'+str(idx),
            #                            low = 90,
            #                            high = 180)
            
            # param.sLegCL.point[idx].update(coordinates=[sX,sY],
            #                                rightHandleAngle=sRA,
            #                                leftHandleAngle=sLA)
            sCL.points[idx].update(coordinates=[sX,sY],
                                   rightHandleAngle=sRA,
                                   angleUnit='deg')
        

    for idx in range(lCL.nPts):
        #small Leg : Right Handle Length
        lRL = trial.suggest_float(name = lCL.name+'Pt.rL'+str(idx),
                                  low = 10.0,
                                  high = 100.0)
        #small Leg : Left Handle Length
        lLL = trial.suggest_float(name = lCL.name+'Pt.lL'+str(idx),
                                  low = 10.0,
                                  high = 100.0)
        lCL.points[idx].update(rightHandleLength=lRL,
                               leftHandleLength=lLL)
        #parameters.lLegCL.point[idx].update(rightHandleLength=lRl,
        #                                    sameHandleLength=True)

        if idx > 0 and idx < (lCL.nPts-1):
            #small Leg : x coordinate
            lX  = trial.suggest_float(name = lCL.name+'Pt.x'+str(idx),
                                      low = 30,
                                      high = 50)
            #small Leg : y coordinate
            lY  = trial.suggest_float(name = lCL.name+'Pt.y'+str(idx),
                                      low = 10,
                                      high = 30)
            #small Leg : Right Handle Angle
            lRA = trial.suggest_float(name = lCL.name+'Pt.rA'+str(idx),
                                      low = -80,
                                      high = -20)
            #small Leg : Left Handle Angle
            # lLA = trial.suggest_float(name = lLegCL.name+'Pt.lA'+str(idx),
            #                           low = 90,
            #                           high = 180)
            # param.lLegCL.point[idx].update(coordinates=[lX,lY],
            #                                rightHandleAngle=lRA,
            #                                leftHandleAngle=lLA)
            lCL.points[idx].update(coordinates=[lX,lY],
                                   rightHandleAngle=lRA,
                                   angleUnit='deg')
    lCL.update(endcoordinates=[param.lLMotor+param.lLPulley+1,75])
    sCL.update(endcoordinates=[param.sLMotor+param.sLPulley+1,75])