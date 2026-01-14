# pyright: reportMissingImports=false, reportAttributeAccessIssue=false
# pylint: disable=import-error
from math import dist
import optuna
from scene import createScene
import param
import SofaRuntime
import Sofa

def getScore(node)->float:

    desiredPosition = node.Modelling.TopTarget.getMechanicalState().position.value[-1][0:3]
    maximumPosition = node.Simulation.Emio.CenterPart.getMechanicalState().position.value[-1][0:3]
    score = abs(dist(desiredPosition,maximumPosition)-30)
    return score

def objective(trial)->float:
    # param.sLPulley = trial.suggest_float(name = param.sLName+'Pulley',
    #                                      low = 22.5,
    #                                      high = 40)
    # param.lLPulley = trial.suggest_float(name = param.sLName+'Pulley',
    #                                      low = 22.5,
    #                                      high = 40)
    for idx in range(param.sLegNPts):
        #small Leg : Right Handle Length
        sRL = trial.suggest_float(name = param.sLName+'Pt.rL'+str(idx),
                                  low = 10.0,
                                  high = 100.0)
        #small Leg : Left Handle Length
        sLL = trial.suggest_float(name = param.sLName+'Pt.lL'+str(idx),
                                  low = 10.0,
                                  high = 100.0)
        
        #leg update
        param.sLegCL.point[idx].update(rightHandleLength = sRL,
                                       leftHandleLength = sLL)
        #parameters.sLegCL.point[idx].update(rightHandleLength=sRl,
        #                                   sameHandleLength=True)
        if idx > 0 and idx < (param.sLegNPts-1):
            #small Leg : x coordinate
            sX  = trial.suggest_float(name = param.sLName+'Pt.x'+str(idx),
                                      low = 15,
                                      high = 65.0)
            #small Leg : y coordinate
            sY  = trial.suggest_float(name = param.sLName+'Pt.y'+str(idx),
                                      low = 15,
                                      high = 65.0)
            #small Leg : Right Handle Angle
            sRA = trial.suggest_float(name = param.sLName+'Pt.rA'+str(idx),
                                      low = -60,
                                      high = 0)
            #small Leg : Left Handle Angle
            # sLA = trial.suggest_float(name = param.sLName+'Pt.lA'+str(idx),
            #                            low = 90,
            #                            high = 180)
            
            # param.sLegCL.point[idx].update(coordinates=[sX,sY],
            #                                rightHandleAngle=sRA,
            #                                leftHandleAngle=sLA)
            param.sLegCL.point[idx].update(coordinates=[sX,sY],
                                                rightHandleAngle=sRA,
                                                sameHandleAngle=True)

    for idx in range(param.lLegNPts):
        #small Leg : Right Handle Length
        lRL = trial.suggest_float(name = param.lLName+'Pt.rL'+str(idx),
                                  low = 10.0,
                                  high = 100.0)
        #small Leg : Left Handle Length
        lLL = trial.suggest_float(name = param.lLName+'Pt.lL'+str(idx),
                                  low = 10.0,
                                  high = 100.0)
        param.lLegCL.point[idx].update(rightHandleLength=lRL,
                                            leftHandleLength=lLL)
        #parameters.lLegCL.point[idx].update(rightHandleLength=lRl,
        #                                    sameHandleLength=True)

        if idx > 0 and idx < (param.lLegNPts-1):
            #small Leg : x coordinate
            lX  = trial.suggest_float(name = param.lLName+'Pt.x'+str(idx),
                                      low = 15,
                                      high = 65)
            #small Leg : y coordinate
            lY  = trial.suggest_float(name = param.lLName+'Pt.y'+str(idx),
                                      low = 15,
                                      high = 65)
            #small Leg : Right Handle Angle
            lRA = trial.suggest_float(name = param.lLName+'Pt.rA'+str(idx),
                                      low = -60,
                                      high = 0)
            #small Leg : Left Handle Angle
            # lLA = trial.suggest_float(name = param.lLName+'Pt.lA'+str(idx),
            #                           low = 90,
            #                           high = 180)
            # param.lLegCL.point[idx].update(coordinates=[lX,lY],
            #                                rightHandleAngle=lRA,
            #                                leftHandleAngle=lLA)
            param.lLegCL.point[idx].update(coordinates=[lX,lY],
                                               rightHandleAngle=lRA,
                                               sameHandleAngle=True)

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
    SimSteps = int(param.duration/root.getDt())
    pauseSteps = int(param.durationPerOperation[0]/root.getDt())
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
        
    Sofa.Simulation.unload(root)
    return score

def detectCollision(node):
    from itertools import combinations
    cP=node.Simulation.Emio.CenterPart.bbox.value
    l0=node.Simulation.Emio.Leg0.bbox.value
    l1=node.Simulation.Emio.Leg1.bbox.value
    l2=node.Simulation.Emio.Leg2.bbox.value
    l3=node.Simulation.Emio.Leg3.bbox.value
    l4=node.Simulation.Emio.Leg4.bbox.value
    l5=node.Simulation.Emio.Leg5.bbox.value
    
    objects = {'Center part':cP,
              f'Leg 0 ({param.sLName})':l0,
              f'Leg 1 ({param.lLName})':l1,
              f'Leg 2 ({param.sLName})':l2,
              f'Leg 3 ({param.lLName})':l3,
              f'Leg 4 ({param.sLName})':l4,
              f'Leg 5 ({param.lLName})':l5}
    intersection ={}
    for item in list(combinations(objects.items(),2)):
        item = dict(item)
        key = str(list(item.keys())[0])+' & '+str(list(item.keys())[1])
        IoU = getIntersection(*item.values())
        intersection[key] = IoU
        print(f'objects : {key}, IoU : {IoU}')

def getIntersection(a,b)->float:
    
    bbox = [[max(aa,bb) for (aa,bb) in zip(a[0],b[0])],
            [min(aa,bb) for (aa,bb) in zip(a[1],b[1])]]
    
    if bbox[0][0]>bbox[1][0] or bbox[0][1]>bbox[1][1] or bbox[0][2]>bbox[1][2]:
        return 0
    else:
        Vinter = (bbox[1][0]-bbox[0][0])*(bbox[1][1]-bbox[0][1])*(bbox[1][2]-bbox[0][2])
        Va = (a[1][0]-a[0][0])*(a[1][1]-a[0][1])*(a[1][2]-a[0][2])
        Vb = (b[1][0]-b[0][0])*(b[1][1]-b[0][1])*(b[1][2]-b[0][2])
        Vunion = Va+Vb-Vinter
        return Vinter/Vunion