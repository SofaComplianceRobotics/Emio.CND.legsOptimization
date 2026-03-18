"""_summary_
Contains the optimization parameters related to:
   - the geometry of the legs
   - the materials of the legs
   - the movement sequence and its duration in the simulation
   - the simulation targets (weight, maximum displacement per each degree of freedom) 
"""

from math import pi
from json import loads
from centerLine import CenterLine
from animation import operationsSequence

# Mode

# 3 possible modes are available :
#  1 - optimization : for running the optimization with Optuna.
#  2 - trial : for running a specific successful trial, requires setting the study name and trial number
#  3 - test : for using a customized shape which can be defined below at Leg Creation section

mode = 'trial' 
studyName = 'goodShapes'
trialNumber = 'g-1227'

# Short Leg parameters
sLegNPts = 3
sLName = 'sLeg'
sLCrossSection = [10,5]
sLNBeams = 20
sLYoungModulus = 3.5e4
sLPoissonRatio = 0.45
sLDensity = 1.220e-6
sLPulley = 10
sLMotor = 70

# Long leg parameters
lLegNPts = 3
lLName = 'lLeg'
lLCrossSection = [10,5]
lLNBeams = 20
lLYoungModulus = 3.5e4
lLPoissonRatio = 0.45
lLDensity = 1.220e-6
lLPulley = 10
lLMotor = 100

# Movement sequence

# A sequence is defined as a string of movements
# Movements are defined as follow : Direction&Operation&Axis&OperationDuration
# Direction can be either + or -
# Operation can be either t for translation, r for rotation or p for pause 
# Operation duration can be defined individualy, or for all the operations using the default variable

sequence = 'p1+ty'
defaultOperationDuration = 1

# Simulation targets

weight = 0.750
displacementLimits = {'-tx':50,'+tx':50,
                      '-ty':20,'+ty':80,
                      '-tz':50,'+tz':50,
                      '-rx':pi/4,'+rx':pi/4,
                      '-ry':pi/4,'+ry':pi/4,
                      '-rz':pi/4,'+rz':pi/4}

# Debugging
debug = {'plot':'','time':'','cad':False}


# Leg Creation

operations = operationsSequence(sequence,defaultOperationDuration)
durationPerOperation = list(operations.values())
duration = sum(durationPerOperation)

match mode:
   case 'test':
      sCP = [[[0,0],20,90],
            [[50,50],50,-20],
            [[75,75],20,90]]
      lCP = [[[0,0],20,90],
            [[50,50],50,-45],
            [[75,75],20,90]]
   
   case 'trial':
      with open('results/'+studyName+'/'+trialNumber+'/params.json','r',encoding="utf-8") as f:
         data = loads(f.read())
      sCP = data[sLName]
      lCP = data[lLName]

   case 'optimization':
      sCP = None
      lCP = None

sCL = CenterLine(name = sLName,
                 numberOfPoints = sLegNPts,
                 crossSection = sLCrossSection,
                 controlPoints = sCP,
                 origin = [-sLPulley,-30],
                 endCoordinates=[sLMotor+sLPulley+1,75])

lCL = CenterLine(name = lLName,
                 numberOfPoints = lLegNPts,
                 controlPoints = lCP,
                 crossSection = lLCrossSection,
                 origin = [-lLPulley,-30],
                 endCoordinates=[lLMotor+lLPulley+1,75])


