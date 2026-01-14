# pyright: reportMissingImports=false, reportAttributeAccessIssue=false
# pylint: disable=import-error
from math import sin, cos, pi
import os
from json import loads
import numpy as np

from parts.motor import Motor 
from parts.leg import Leg
from parts.centerpart import CenterPart
from parts.controllers.assemblycontroller import AssemblyController
from parts.emio import Emio

from utils import getColorFromFilename
from utils.topology import applyRotation, applyTranslation
from utils.header import addHeader, addSolvers

from splib3.numerics import to_radians
from splib3.animation import AnimationManager, animate

import param
from centerLine import generateLeg
from animation import animation
#from processing import importSimulationData

import Sofa

def createScene(rootnode):
    
    match param.choice:
        case 'trial':
            os.system("clear||cls")
            #data=importSimulationData('results/'+param.studyName+'/'+param.trialId+'.json')
            with open('results/'+param.studyName+'/'+param.trialId+'.json','r',encoding="utf-8") as f:
                data = loads(f.read())
            param.sLegCL.fromArray(data[param.sLName])
            param.lLegCL.fromArray(data[param.lLName])
        case 'test':
            os.system("clear||cls")
            param.sLegCL.fromArray(param.sCL)
            param.lLegCL.fromArray(param.lCL)

    shortLegCenterLine = param.sLegCL.asArray()
    longLegCenterLine = param.lLegCL.asArray()

    _ , modelling, simulation = addHeader(rootnode, inverse=True) # _ : settings
    addSolvers(simulation)
    rootnode.dt = 0.01
    rootnode.gravity = [0., -9810., 0.]
    rootnode.VisualStyle.displayFlags.value = ["hideVisuals","showBehaviorModels","showForceFields","showWireframe"]
    rootnode.addObject(AnimationManager(rootnode))

    #Leg generation
    shortLegBeams = generateLeg(legName=param.sLName,
                                legCrossSection=param.sLCrossSection,
                                legCenterLine=shortLegCenterLine,
                                numberOfBeams=param.sLNBeams,
                                origin=[0,-param.sLPulley],
                                debug=param.debug)

    longLegBeams = generateLeg(legName=param.lLName,
                               legCrossSection=param.lLCrossSection,
                               legCenterLine=longLegCenterLine,
                               numberOfBeams=param.lLNBeams,
                               origin=[0,-param.lLPulley],
                               debug=param.debug)

    if (longLegBeams is None or shortLegBeams is None):
        rootnode.removeObject(AnimationManager(rootnode))
        return None

    # Add Emio to the scene
    emio = ndtRobot(name='Emio',
                   legsName = [param.sLName,param.lLName]*3,
                   legsModel = ['beam'],
                   legsPositionOnMotor = ['clockwiseup'],
                   legsPositions = [shortLegBeams, longLegBeams]*3,
                   legsMassDensity = [param.sLDensity, param.lLDensity]*3,
                   legsPoissonRatio = [param.sLPoissonRatio, param.lLPoissonRatio]*3,
                   legsYoungModulus = [param.sLYoungModulus, param.lLYoungModulus]*3,
                   centerPartName = "Hexaconnector",
                   centerPartType = "rigid",
                   centerPartModel = "beam",
                   centerPartMassDensity = 1.220e-6,
                   centerPartPoissonRatio = 0.45,
                   centerPartYoungModulus = 3.5e4,
                   centerPartClass = CenterPart,
                   extended=False)

    if not emio.isValid():    
        rootnode.removeObject(AnimationManager(rootnode))
        return None
    
    simulation.addChild(emio)
    emio.attachCenterPartToLegs()
    emio.addObject(AssemblyController(emio))

    # Add effector
    emio.effector.addObject("MechanicalObject",
                            template="Rigid3",
                            position=[0, 30, 0, 0, 0, 0, 1])
    emio.effector.addObject("RigidMapping",
                            index=0)

    # Top Target
    effectorTopTarget = modelling.addChild('TopTarget')
    effectorTopTarget.addObject('EulerImplicitSolver',
                             firstOrder=True)
    effectorTopTarget.addObject('CGLinearSolver',
                             iterations=50,
                             tolerance=1e-10,
                             threshold=1e-10)
    effectorTopTarget.addObject('MechanicalObject',
                             template='Rigid3',
                             position=[0, 105, 0, 0, 0, 0, 1],
                             showObject=True,
                             showObjectScale=20)
    emio.addInverseComponentAndGUI(effectorTopTarget.getMechanicalState().position.linkpath,withGUI=False)

    # bot Target
    # effectorBotTarget = modelling.addChild('BottomTarget')
    # effectorBotTarget.addObject('EulerImplicitSolver',
    #                          firstOrder=True)
    # effectorBotTarget.addObject('CGLinearSolver',
    #                          iterations=50,
    #                          tolerance=1e-10,
    #                          threshold=1e-10)
    # effectorBotTarget.addObject('MechanicalObject',
    #                          template='Rigid3',
    #                          position=[0, 40, 0, 0, 0, 0, 1],
    #                          showObject=True,
    #                          showObjectScale=20)

    # emio.effector.addObject('PositionEffector',
    #                     template='Rigid3', indices=[1],
    #                     useDirections=[1, 1, 1, 0, 0, 0],
    #                     weight=1,
    #                     maxSpeed=1000,
    #                     limitShiftToTarget=True,
    #                     maxShiftToTarget=20,  # mm
    #                     effectorGoal=effectorBotTarget.getMechanicalState().position.linkpath, name="EffectorCoord")
    
    animate(onUpdate = animation,
            params = {'topTarget':effectorTopTarget.getMechanicalState().position,
                      'iniPosition':effectorTopTarget.getMechanicalState().position.value[0],
                      'movementSequence':param.sequence,
                      'displacementLimits':param.displacementLimits,
                      'defaultOperationDuration':param.defaultOperationDuration
                      },
            duration=param.duration)
    
    return rootnode

class ndtLeg(Leg):
    def __init__(self, *args, **kwargs):
        
        super().__init__(*args, **kwargs)
        
        if 'meshData' in kwargs:
            self.meshData = kwargs['meshData']


    def _checkData(self):
        """
        Check if the data needed to create the leg is present.
        The leg is created from a mesh, and the data should be in the data/meshes/legs directory.
        Expected files:
            - legName.stl: surface mesh for the visual model. On only used for `tetra` model, for `beam` and `cosserat` models, the mesh is created from the positions.
            - legName.vtk: volume mesh for the tetra model
            - legName.txt: file containing the positions of the leg (for beam and cosserat models)
        """

        if self.model.value in ["beam", "cosserat"]:
            if not self.positions.value.any():
                filename = self.legName.value + ".txt"
                if not self._checkFile(filename):
                    return False
                self.positions.value = np.loadtxt(self._getFilePath(filename))
                if not self.positions.value.any():
                    Sofa.msg_error(self.getName(), 'Empty positions. We cannot model the leg without a list of '
                                                'Rigid3 positions describing the curve when using the '
                                                'beam or cosserat model')
                    return False
        elif self.model.value == "tetra":
            if self.legName.value is None:
                Sofa.msg_error(self.getName(),
                               'Empty legName. We cannot model the leg with tetra without a volume mesh.')
                return False
            
            if not self._checkFile(self.legName.value + ".vtk"):
                return False
        else:
            Sofa.msg_error(self.getName(), 'Unknown model, value should be "beam", "cosserat", or "tetra".')
            return False

        
        filePath = self._getFilePath(self.legName.value + ".stl")
        if filePath is None:
            return False

        volume = self.addChild("Volume")
        #volume.addObject("MeshSTLLoader", filename=filePath)
        volume.addObject("VolumeFromTriangles",
                         position=volume.MeshSTLLoader.position.value,
                         triangles=volume.MeshSTLLoader.triangles.value)
        volume.init()
        self.totalMass = volume.VolumeFromTriangles.volume.value * self.massDensity.value

        return True

class ndtRobot(Emio) :
    def __init__(self, *args, **kwargs,):

        if 'centerPartClass' in kwargs:
            centerPartClass = kwargs['centerPartClass']
        else:
            centerPartClass = CenterPart

        if 'legsPositions' in kwargs:
            legsPositions = kwargs['legsPositions']
        else:
            legsPositions = []

        Sofa.Prefab.__init__(self, *args, **kwargs)
        self._centerPartClass = centerPartClass
        self.addData(name="nbLegs", type="int", value=6)
        centerPartPositions = []
        # Add motors and legs
        self.motors = []
        self.legs = []
        distances = [70 for i in range(self.nbLegs.value)]
        legsPositionOnMotor = self.legsPositionOnMotor.value
        legsModel = self.legsModel.value
        legsMassDensity = self.legsMassDensity.value
        legsYoungModulus = self.legsYoungModulus.value
        legsPoissonRatio = self.legsPoissonRatio.value

        assert len(legsPositionOnMotor) > 0, "At least one position is expected"
        assert len(legsModel) > 0, "At least one model is expected"
        assert len(legsMassDensity) > 0, "At least one mass density is expected"
        assert len(legsYoungModulus) > 0, "At least one Young's modulus is expected"
        assert len(legsPoissonRatio) > 0, "At least one Poisson's ratio is expected"

        for i in range(self.nbLegs.value):
            angle = 2. * pi / self.nbLegs.value * i

            radius = distances[i] if i < len(distances) else distances[0]
            translation = [radius * sin(angle), 0, radius * cos(angle)]
            if i ==0 or i == 2 :
                rotation = [0, [180, 240, 300, 0, 60, 120][i], 0]
            else:
                rotation = [180, [180, 240, 300, 0, 60, 120][i], 0]

            # Motor
            motor = Motor(name="Motor" + str(i), translation=translation, rotation=rotation,
                          tempvisurotation=[[-90, 90, -90, 90, 90, 90][i], 180, 0], color=[0., 0., 0., 0.])
            self.addChild(motor)
            self.motors.append(motor)

            rotation = [0, [180, 240, 300, 0, 60, 120][i], 0]
            radius = radius + 20
            translation = [radius * sin(angle), 30, radius * cos(angle)]

            if (i < len(self.legsName.value) and self.legsName.value[i] == "None"):
                self.legs.append(None)
            else:
                # Leg
                positionOnMotor=legsPositionOnMotor[i] if i < len(legsPositionOnMotor) else legsPositionOnMotor[0]
                model=legsModel[i] if i < len(legsModel) else legsModel[0]
                leg = Leg(name="Leg" + str(i),
                        legName=self.legsName.value[i] if i < len(self.legsName.value) else self.legsName.value[0],
                        positionOnMotor=positionOnMotor,
                        model=model,
                        translation=translation, rotation=rotation,
                        massDensity=legsMassDensity[i] if i < len(legsMassDensity) else legsMassDensity[0],
                        poissonRatio=legsPoissonRatio[i] if i < len(legsPoissonRatio) else legsPoissonRatio[0],
                        youngModulus=legsYoungModulus[i] if i < len(legsYoungModulus) else legsYoungModulus[0],
                        positions = legsPositions[i] if i < len(legsPositions) else legsPositions[0]
                        )
                self.legs.append(leg)
                if not leg.isValid():
                    self._validState = False
                    Sofa.msg_error(self.getName(), "At least one leg is not valid, cannot create Emio.")
                    break
                else:
                    self.addChild(leg)
                    # Attach leg to motor
                    leg.attachBase(motor.Parts, 1)

                    # Store leg extremity's position to define the center part of the robot
                    position = list(leg.extremity.getMechanicalState().position.value[0])
                    applyRotation([position], to_radians(rotation))
                    applyTranslation([position], translation)
                    centerPartPositions += [position]

        # Robot's center part
        if self._validState:
            color = getColorFromFilename(self.centerPartName.value) if "blue" not in self.centerPartName.value else RGBAColor.lightblue
            self.centerpart = centerPartClass(name="CenterPart",
                                              positions=centerPartPositions,
                                              partName=self.centerPartName.value,
                                              model=self.centerPartModel.value,
                                              massDensity=self.centerPartMassDensity.value,
                                              poissonRatio=self.centerPartPoissonRatio.value,
                                              youngModulus=self.centerPartYoungModulus.value,
                                              type=self.centerPartType.value,
                                              color=color,
                                              rotation=[0, 0, 0] if "down" in legsPositionOnMotor[0] else [180, 180, 0]
                                              )
            if self.centerPartType.value == "rigid":
                self.effector = self.centerpart.addChild("Effector")
            else:
                self.effector = self.centerpart.attach.addChild("Effector")
            self.addChild(self.centerpart)
            self._addBox()
            self._addCamera()