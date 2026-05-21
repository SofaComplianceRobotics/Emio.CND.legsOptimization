# pyright: reportMissingImports=false, reportAttributeAccessIssue=false
# pylint: disable=import-error
import numpy as np
from scipy.spatial.transform import Rotation
import optuna

import Sofa
import SofaRuntime
import os
# import param
from optimization.parameters import low_leg, high_leg, center_part, movements
from scene import createScene

def objective(trial)->float:
    # initialize Opt classes with yaml
    # pydantic check IRT Jules Verne

    generate_samples(trial) 

    # Create the Sofa simulation scene
    SofaRuntime.importPlugin('Sofa.Component')
    SofaRuntime.importPlugin('Sofa.GUI.Component')
    SofaRuntime.importPlugin('Sofa.GL.Component')

    root = Sofa.Core.Node("root")
    root = createScene(root)

    if root is None:
        #trial.report(140,0)
        trial.set_user_attr('prune','Unfeasible shape')
        raise optuna.TrialPruned()

    Sofa.Simulation.init(root)

    score_per_movement = [0]

    for idx, _ in enumerate(movements.steps):
        if 'p' == list(movements.durationPerMovements.keys())[idx]:
            Sofa.Simulation.animateNSteps(root,n_steps = movements.steps[idx])

        else:
            remaining_steps = range(movements.steps[idx-1]+1,movements.steps[idx]+1)

            for step in remaining_steps:
                Sofa.Simulation.animate(root, 0)
                track_trial(root,step,trial.number)

                if step == (remaining_steps[-1]+remaining_steps[0]+1)/2:
                    score_per_movement.append(get_score(root))

                trial.report(get_score(root), step)
    
    Sofa.Simulation.unload(root)
    
    return get_total_score(score_per_movement)

def get_score(node)->float:

    top_target_position = node.Modelling.Target.getMechanicalState().position.value[0][0:3]
    effector_position = node.Simulation.Sam.CenterPart.getMechanicalState().position.value[0][0:3]
    
    distance_score = abs(np.linalg.norm(np.subtract(top_target_position,effector_position))-center_part.height/2)
    
    top_target_angle = node.Simulation.Sam.CenterPart.getMechanicalState().position.value[0][3:] 
    effector_angle =node.Modelling.Target.getMechanicalState().position.value[0][3:] 
    effector_angle = (Rotation.from_euler('y',180,degrees=True) * Rotation.from_quat(effector_angle)).as_quat()

    #angle_score = np.linalg.norm(np.subtract(top_target_angle,effector_angle))
    angle_score = 1-np.dot(top_target_angle,effector_angle)**2

    return np.linalg.norm(np.array([distance_score,angle_score]))

def get_total_score(scores)->float:
    
    return np.linalg.norm(scores)

def generate_samples(trial)->None:

    for idx in range(1,low_leg.centerLine.nPts-1):
        
        low_leg_right_handle_length = trial.suggest_float(name = f'{low_leg.name}Pt.rL{idx}',
                                                          low = 1.0,
                                                          high = 80.0)
        
        low_leg_left_handle_length = trial.suggest_float(name = f'{low_leg.name}Pt.lL{idx}',
                                                         low = 1.0,
                                                         high = 80.0)
        
        if idx == 1:
            low_leg.centerLine.points[idx].update(rightHandleLength = low_leg_right_handle_length,
                                                  sameHandleLength = False)
        if idx == low_leg.centerLine.nPts-2:
            low_leg.centerLine.points[idx].update(rightHandleLength = 1,
                                                  leftHandleLength = low_leg_left_handle_length)
        
        if idx > 1 and idx < (low_leg.centerLine.nPts-2):

            low_leg_x  = trial.suggest_float(name = f'{low_leg.name}Pt.x{idx}',
                                             low = low_leg.centerLine.startCoordinates[0],
                                             high = low_leg.centerLine.endCoordinates[0])

            low_leg_y  = trial.suggest_float(name = f'{low_leg.name}Pt.y{idx}',
                                             low = low_leg.centerLine.startCoordinates[1],
                                             high = low_leg.centerLine.endCoordinates[1])

            low_leg_right_handle_angle = trial.suggest_float(name = f'{low_leg.name}Pt.rA{idx}',
                                                             low = -85,
                                                             high = -5)
            
            # low_leg_left_handle_angle = trial.suggest_float(name = f'{low_leg.name}Pt.lA{idx}',
            #                                                 low = 80,
            #                                                 high = 170)
            
            low_leg.centerLine.points[idx].update(coordinates = [low_leg_x,low_leg_y],
                                                  rightHandleAngle = low_leg_right_handle_angle,
                                                  #leftHandleAngle = low_leg_left_handle_angle,
                                                  rightHandleLength = low_leg_right_handle_length,
                                                  leftHandleLength = low_leg_left_handle_length,
                                                  sameHandleLength = False,
                                                  sameHandleAngle = True,
                                                  angleUnit = 'deg')
        

    for idx in range(1,high_leg.centerLine.nPts-1):

        high_leg_right_handle_length = trial.suggest_float(name = f'{high_leg.name}Pt.rL{idx}',
                                                    low = 1.0,
                                                    high = 80.0)
        
        high_leg_left_handle_length = trial.suggest_float(name = f'{high_leg.name}Pt.lL{idx}',
                                                          low = 1.0,
                                                          high = 80.0)
        
        if idx == 1:
            high_leg.centerLine.points[idx].update(rightHandleLength = high_leg_right_handle_length,
                                                   sameHandleLength = False)
        if idx == high_leg.centerLine.nPts-2:
            high_leg.centerLine.points[idx].update(leftHandleLength = high_leg_left_handle_length,
                                                   sameHandleLength = False)

        if idx > 1 and idx < (high_leg.centerLine.nPts-2):

            high_leg_x  = trial.suggest_float(name = f'{high_leg.name}Pt.x{idx}',
                                              low = high_leg.centerLine.startCoordinates[0],
                                              high = high_leg.centerLine.endCoordinates[0])

            high_leg_y  = trial.suggest_float(name = f'{high_leg.name}Pt.y{idx}',
                                              low = high_leg.centerLine.startCoordinates[1],
                                              high = high_leg.centerLine.endCoordinates[1])

            high_leg_right_handle_angle = trial.suggest_float(name = f'{high_leg.name}Pt.rA{idx}',
                                                              low = -85,
                                                              high = -5)
            
            # high_leg_left_handle_angle = trial.suggest_float(name = f'{high_leg.name}Pt.lA{idx}',
            #                                                  low = 80,
            #                                                  high = 170)
            
            high_leg.centerLine.points[idx].update(coordinates = [high_leg_x,high_leg_y],
                                                   rightHandleAngle = high_leg_right_handle_angle,
                                                   #leftHandleAngle = high_leg_left_handle_angle,
                                                   rightHandleLength = high_leg_right_handle_length,
                                                   leftHandleLength = high_leg_left_handle_length,
                                                   sameHandleLength = False,
                                                   sameHandleAngle = True,
                                                   angleUnit = 'deg')
            
    high_leg.update()
    low_leg.update()
    
    high_leg.centerLine.exportPlot(name=f'{high_leg.name}-{trial.number}')
    low_leg.centerLine.exportPlot(name=f'{low_leg.name}-{trial.number}')

def track_trial(node,step,trial)->None:
    target_position = node.Modelling.Target.getMechanicalState().position.value[0][1]
    center_part_position = node.Simulation.Sam.CenterPart.getMechanicalState().position.value[0][1]
    path = os.path.dirname(os.path.realpath(__file__))+f"/data/scores/{trial}.txt"
    with open(path,'a',encoding="utf-8") as f:
        f.write(f'Step : {step}, Score = {get_score(node)}, Target : {target_position}, TCP : {center_part_position}\n')


