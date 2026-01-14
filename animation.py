from re import split
from scipy.spatial.transform import Rotation
import numpy as np

def operationsSequence(sequence:str,defaultOperationDuration:float=0)->dict:
    delimiters = r'(p|\+tx|\-tx|\+ty|\-ty|\+tz|\-tz|\+rx|\-rx|\+ry|\-ry|\+rz|\-rz)'
    sequenceList = split(delimiters,sequence)[1:]
    duration = {}
    ids=0
    for idx, operation in enumerate(sequenceList):
        if idx <len(sequenceList)-1 and idx%2==0:
            if sequenceList[idx+1] == '':
                duration[operation+str(ids)] = defaultOperationDuration
            else:
                duration[operation+str(ids)] = float(sequenceList[idx+1])
            ids+=1
    return duration

def performOperations(position, movementSequence:str, displacementLimits:dict, factor:float, defaultOperationDuration:float=0):
    sequence = operationsSequence(movementSequence.lower(),defaultOperationDuration)
    simulationDuration = sum(sequence.values())
    indices = {'tx':0,'ty':1,'tz':2,'rx':3,'ry':4,'rz':5}
    progressDuration = 0

    for movement, operationDuration in sequence.items():
        progressDuration += operationDuration
        movement = movement[0:3]
        if factor*simulationDuration <= progressDuration:
            if 'p' in movement:
                break
            else:
                direction = int(movement[0]+'1')
                operation = movement[1]
                axis = movement[2]
                operationIdx = indices[operation+axis]
                scaledFactor = (factor*simulationDuration-(progressDuration-operationDuration))/operationDuration
                if operation == 't':
                    position[operationIdx] += displacementLimits[movement]*direction*scaledFactor

                elif operation == 'r':
                    angle=displacementLimits[movement]*direction*scaledFactor
                    rotation = Rotation.from_euler(axis, angle).inv()
                    offset = [0.0, 105-30, 0.0]
                    position[:3] -= offset
                    position[:3] = rotation.apply(position[:3]) + offset
                break

    return position

def animation(topTarget, iniPosition, movementSequence, displacementLimits,defaultOperationDuration,factor):
    iniPosition=[0, 105, 0, 0, 0, 0, 1]
    p = performOperations(np.copy(iniPosition),
                          movementSequence,
                          displacementLimits,
                          factor,
                          defaultOperationDuration)
    topTarget.value = [p]

    # p[1]-=65
    # botTarget.value = [p]
