#from centerLine import legCenterLine
import os
from math import pi
from centerLine import legCenterLine
from animation import operationsSequence

#Trials
dataPath = os.path.dirname(os.path.abspath(__file__))+"/data/displacement/data.json"
choice = ''
studyName = 'no-name-b182cfa6-d9b0-42fd-a947-a925491e9211'
trialId = '2460'

#parameters

# short Leg
sLegNPts = 3
sLName = 'sLeg'
sLCrossSection = [10,5]
sLNBeams = 20
sLYoungModulus = 3.5e4
sLPoissonRatio = 0.45
sLDensity = 1.220e-6
sLPulley = 22.5

# long leg
lLegNPts = 3
lLName = 'lLeg'
lLCrossSection = [10,5]
lLNBeams = 20
lLYoungModulus = 3.5e4
lLPoissonRatio = 0.45
lLDensity = 1.220e-6
lLPulley = 22.5

debug = {'plot':'','time':'','cad':False}

#Simulation
sequence = 'p1+ty'
defaultOperationDuration = 5
durationPerOperation = list(operationsSequence(sequence,defaultOperationDuration).values())
duration = sum(durationPerOperation)

#Variables

# sLegCL = legCenterLine(legName=shortLegName,
#                        numberOfPoints=sLegNPts,
#                        legCrossSection=shortLegCrossSection,
#                        origin=[0,-pulleyShortLeg])

# lLegCL = legCenterLine(legName=longLegName,
#                        numberOfPoints=lLegNPts,
#                        legCrossSection=longLegCrossSection,
#                        origin=[0,-pulleyLongLeg])

sLegCL = legCenterLine(legInitials=sLName,
                       numberOfPoints=sLegNPts)

lLegCL = legCenterLine(legInitials=lLName,
                       numberOfPoints=lLegNPts)

sCL = [[[0,0],20,90],
       [[50,50],50,-20],
       [[75,75],20,90]]

lCL = [[[0,0],20,90],
       [[50,50],50,-45],
       [[75,75],20,90]]

#Targets

weight = 750

dX = [50,50]
dY = [20,80]
dZ = [50,50]

rX = [pi/4,pi/4]
rY = [pi/4,pi/4]
rZ = [pi/4,pi/4]

displacementLimits = {'+tx':dX[1],'-tx':dX[0],
                      '+ty':dY[1],'-ty':dY[0],
                      '+tz':dZ[1],'-tz':dZ[0],
                      '+rx':rX[1],'-rx':rX[0],
                      '+ry':rY[1],'-ry':rY[0],
                      '+rz':rZ[1],'-rz':rZ[0]}
