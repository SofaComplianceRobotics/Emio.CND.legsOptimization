from re import split
from scipy.spatial.transform import Rotation
import numpy as np
from math import dist

class Movements():
    """
    A sequence is defined as a string of movements\n
    Movements are defined as follow : Direction&Operation&Axis&OperationDuration
    Direction can be either + or -
    Operation can be either t for translation, r for rotation or p for pause
    Operation duration can be defined individualy, or for all the operations using the default variable
    """
    def __init__(self, 
                 sequence : str='p0.75+ty',
                 defaultMovementsDuration : float = 4, 
                 movementsLimits : dict = {'+ty':80},
                 dt = 0.01):

        delimiters = r'(p|\+tx|\-tx|\+ty|\-ty|\+tz|\-tz|\+rx|\-rx|\+ry|\-ry|\+rz|\-rz)'
        
        self.sequence = split(delimiters,sequence.lower())[1:]
        self.defaultMovementsDuration = defaultMovementsDuration
        self.MovementsLimits = movementsLimits
        self.dt=dt

        durationPerMovements = {}
        
        #ids=0
        for idx, operation in enumerate(self.sequence):
            if idx <len(self.sequence)-1 and idx%2==0:
                if self.sequence[idx+1] == '':
                    durationPerMovements[operation] = self.defaultMovementsDuration #operation+str(ids)
                else:
                    durationPerMovements[operation] = float(self.sequence[idx+1]) #operation+str(ids)
                #ids+=1

        self.durationPerMovements = durationPerMovements
        movements = list(self.durationPerMovements.values())
        
        self.duration = sum(movements)
        self.steps = []*len(movements)

        for idx in range(len(movements)):
            self.steps.append(int(sum(movements[:idx+1])/self.dt))

    def getTargetPosition(self, initialPosition:list,targetHeight , factor:float)->list:
        indices = {'tx':0, 'ty':1, 'tz':2, 'rx':3, 'ry':4, 'rz':5}
        progressDuration = 0

        for movement, duration in self.durationPerMovements.items():
            progressDuration += duration
            movement = movement[0:3]
            if factor*self.duration <= progressDuration:
                if 'p' in movement:
                    break
                else:
                    direction = int(movement[0]+'1')
                    operation = movement[1]
                    axis = movement[2]
                    operationIdx = indices[operation+axis]
                    scaledFactor = (factor*self.duration-(progressDuration-duration))/duration
                    
                    if scaledFactor > 0.5: scaledFactor = 1-scaledFactor
        
                    transformation = self.MovementsLimits[movement]*direction*scaledFactor*2

                    match operation:
                        case 't':
                            initialPosition[0][operationIdx] += transformation
                            initialPosition[1][operationIdx] += transformation
                        case 'r':
                            rotation = Rotation.from_euler(axis, transformation).inv()
            
                            high_tcp = np.array([0,targetHeight,0])
                            
                            initialPosition[0][:3] = initialPosition[0][:3] + rotation.apply(high_tcp) - high_tcp
                            initialPosition[0][3:] = rotation.as_quat()

                            initialPosition[1][:3] = initialPosition[1][:3] + rotation.apply(-1*high_tcp) + high_tcp
                            initialPosition[1][3:] = rotation.as_quat()
                    break

        return initialPosition


def animation(target:list, initialPosition:list, movements:Movements, height:float ,factor:float):
    
    initialPosition = np.copy(initialPosition)
    p = movements.getTargetPosition(initialPosition, height, factor)
    target.value = [*p]

def distance(desiredPosition, maximumPosition,factor):
    score = abs(dist(desiredPosition.getMechanicalState().position.value[-1][0:3],maximumPosition.getMechanicalState().position.value[-1][0:3]))
    print(score)

if __name__ == "__main__":
    movement = Movements()