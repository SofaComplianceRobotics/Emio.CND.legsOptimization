# pyright: reportMissingImports=false, reportAttributeAccessIssue=false
# pylint: disable=import-error
from math import sin, cos, pi
import os
import numpy as np

# EmioLabs libraries
from parts.motor import Motor
from parts.leg import Leg
from parts.centerpart import CenterPart
from parts.controllers.assemblycontroller import AssemblyController
from parts.emio import Emio

from utils import getColorFromFilename
from utils.topology import applyRotation, applyTranslation,getIndicesInBox,getExtremityFromBase
from utils.header import addHeader, addSolvers

# Sofa Python Libraries
from splib3.numerics import to_radians
from splib3.animation import AnimationManager, animate

# Sofa Library
import Sofa

# Scene Libraries
import param
from animation import animation

def createScene(rootnode):

    _, modelling, simulation = addHeader(rootnode, inverse=True)
    addSolvers(simulation)

    rootnode.dt = 0.01
    rootnode.gravity = [0., -9810., 0.]
    rootnode.VisualStyle.displayFlags.value = ["hideVisuals",
                                               "showBehaviorModels",
                                               "showWireframe"]
    rootnode.addObject(AnimationManager(rootnode))

    # Leg generation
    if (not param.sCL.isValid() or not param.lCL.isValid()):
        rootnode.removeObject(AnimationManager(rootnode))
        return

    if (not param.sCL.exportSTL() or not param.lCL.exportSTL()):
        rootnode.removeObject(AnimationManager(rootnode))
        return

    # Add SAM to the scene
    sam = ndtRobot(name='Sam',
                   legsName = [param.sCL.name, 
                               param.lCL.name]*3,
                   legsPositions = [param.sCL.getBeams(param.sLNBeams),
                                    param.lCL.getBeams(param.lLNBeams)]*3,
                   legsMassDensity = [param.sLDensity,
                                      param.lLDensity]*3,
                   legsPoissonRatio = [param.sLPoissonRatio,
                                       param.lLPoissonRatio]*3,
                   legsYoungModulus = [param.sLYoungModulus,
                                       param.lLYoungModulus]*3,
                   legsMeshData = [param.sCL.getTopologyGmsh(),
                                   param.lCL.getTopologyGmsh()]*3,
                   motorsDistanceToCenter = [param.sLMotor,
                                             param.lLMotor]*3,
                   platformLevel = 2,
                   centerPartName = "Hexaconnector",
                   centerPartMassDensity = 1.220e-6,
                   centerPartPoissonRatio = 0.45,
                   centerPartYoungModulus = 3.5e4,
                   centerPartClass = CenterPart,
                    )
    
    if not sam.isValid():
        rootnode.removeObject(AnimationManager(rootnode))
        return
    
    simulation.addChild(sam)
    sam.attachCenterPartToLegs()
    sam.addObject(AssemblyController(sam))

    # Add effector
    sam.effector.addObject("MechanicalObject", 
                            template = "Rigid3", 
                            position = [0, 30, 0, 0, 0, 0, 1])
    sam.effector.addObject("RigidMapping",
                            index = 0)

    # Top Target
    effectorTopTarget = modelling.addChild('TopTarget')
    effectorTopTarget.addObject('EulerImplicitSolver',
                                firstOrder = True)
    effectorTopTarget.addObject('CGLinearSolver',
                                iterations = 50,
                                tolerance = 1e-10,
                                threshold = 1e-10)
    effectorTopTarget.addObject('MechanicalObject',
                                template = 'Rigid3',
                                position = [0, 105, 0, 0, 0, 0, 1],
                                showObject = True,
                                showObjectScale = 20)
    #Sensor
    # sensor = emio.CenterPart.addChild('Sensor')
    # sensor.addObject('MechanicalObject',
    #                  position = [[0.0, 40.0, 0.0]],
    #                  showObject=True,
    #                 #  showObjectScale=1,
    #                  drawMode=2)
    # sensor.addObject("UniformMass", totalMass=param.weight)
    #     
    sam.addInverseComponentAndGUI(effectorTopTarget.getMechanicalState().position.linkpath,withGUI=False)

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

    prefabNodes = {'name': 'nodes', 'type': 'vector<Vec3d>', 'help': '', 'default': None}
    prefabTriangles = {'name': 'triangles', 'type': 'vector<Triangle>', 'help': '', 'default': None}
    prefabData = Leg.prefabData + [prefabNodes,prefabTriangles]
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args,**kwargs)

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
        
        if filePath is None and (self.nodes.value is None or self.triangles.value is None):
            return False
        else:
            volume = self.addChild("Volume")
            if self.nodes.value is not None and self.triangles.value is not None:
                volume.addObject("MeshTopology",
                    position = self.nodes.value,
                    triangles = self.triangles.value)
                position = volume.MeshTopology.position.value
                triangles = volume.MeshTopology.triangles.value

            else:
                volume.addObject("MeshSTLLoader", filename=filePath)
                position = volume.MeshSTLLoader.position.value
                triangles = volume.MeshSTLLoader.triangles.value
       
            volume.addObject("VolumeFromTriangles",
                             position = position,
                             triangles = triangles)

        volume.init()
        self.totalMass = volume.VolumeFromTriangles.volume.value * self.massDensity.value

        return True

    def _addVisualModel(self):
        """
        Adds a visual model to the leg. In case of the tetra model, we need the corresponding surface mesh.
        Otherwise, the surface will be generated from the leg positions.
        """
        if self.nodes.value is not None and self.triangles.value is not None:
            visual = self.leg.addChild("Visual")
            visual.addObject("MeshTopology",
                             position = self.nodes.value,
                             triangles = self.triangles.value)
            visual.addObject("OglModel",
                             src = visual.MeshTopology.linkpath,
                             color = self.color,
                             rotation = self.rotation.value,
                             translation = self.translation.value)
            visual.addObject('SkinningMapping')
        else:
            visual = self.leg.addChild("Visual")
            visual.addObject("MeshSTLLoader",
                             filename=self._getFilePath(self.legName.value + ".stl"))
            visual.addObject("OglModel",
                             src=visual.MeshSTLLoader.linkpath,
                             color=self.color,
                             rotation=self.rotation.value,
                             translation=self.translation.value)
            visual.addObject('SkinningMapping')

    def _getIndicesDistribution(self, topology):
        """
        Get the indices of the rigidified and deformable parts of the leg.
        The rigidified parts are the base and the extremity, and the deformable part is the rest of the leg.
        The indices are used to create the mapping between the rigidified and deformable parts.
        The function also returns the index pairs for the mapping.
        """
        indicesRigidified1 = []
        indicesRigidified2 = []
        indicesDeformable = []
        positions = topology.position.value

        def getIndicesInOrientedBox(positions, obox):
            indicesIn = []
            indicesOut = []

            p0 = np.array([obox[0], obox[1], obox[2]])
            p1 = np.array([obox[3], obox[4], obox[5]])
            p2 = np.array([obox[6], obox[7], obox[8]])
            depth = obox[9]

            normal = np.cross(p1-p0,p2-p0)
            normal /= np.linalg.norm(normal)
            
            p3 = p0 + (p2-p1)
            p6 = p2 + normal * depth

            plane0 = np.cross(p1-p0,normal)
            plane0 /= np.linalg.norm(plane0)
            
            plane1 = np.cross(p2-p3,p6-p3)
            plane1 /= np.linalg.norm(plane1)

            plane2 = np.cross(p3-p0,normal)
            plane2 /= np.linalg.norm(plane2)

            plane3 = np.cross(p2-p1,p6-p2)
            plane3 /= np.linalg.norm(plane3)

            width = abs(np.dot((p2-p0),plane0))
            length = abs(np.dot((p2-p0),plane2))
            
            for index, pos in enumerate(positions):
                
                pv0 = np.subtract(pos,p0)
                pv1 = np.subtract(pos,p2)

                if abs(np.dot(pv0, plane0)) <= width and abs(np.dot(pv1, plane1)) <= width:
                    if abs(np.dot(pv0, plane2)) <= length and abs(np.dot(pv1, plane3)) <= length:
                        if not (abs(np.dot(pv0, normal)) <= abs(depth/2)) :
                            indicesOut.append(index)
                            continue
                    else:
                        indicesOut.append(index)
                        continue

                else:
                    indicesOut.append(index)
                    continue
                indicesIn.append(index)

            return indicesIn, indicesOut
    
        #First the base
        
        boxes = [[0, -5., -5., 10., 15., 5.]]
        
        tr = positions[0]
        indice = int(self.name.value[-1])
        te = indice*pi/6
        
        rot = [[cos(te),0,-sin(te)],[0,1,0],[sin(te),0,cos(te)]]
        
        obox = [[]*10]*len(boxes)
        for idx,box in enumerate(boxes):
            obox[idx] = [0,box[1],box[2],
                         0,box[1],box[5],
                         0,box[4],box[5],
                         box[3]-box[0]]
            
            obox[idx][0:3] = np.dot(rot,obox[idx][0:3]) + tr
            obox[idx][3:6] = np.dot(rot,obox[idx][3:6]) + tr
            obox[idx][6:9] = np.dot(rot,obox[idx][6:9]) + tr

            indR, indD = getIndicesInOrientedBox(positions=positions, obox=obox[idx])
            # print(f'Ins : {indR1}, Outs : {indD1}')

            indicesRigidified1 = list(np.unique(indicesRigidified1 + indR))
            indicesDeformable = indD if len(indicesDeformable) == 0 else np.intersect1d(indicesDeformable, indD)
        indexExtremity = getExtremityFromBase(topology, indicesRigidified1[0])
        positionExtremity = positions[indexExtremity]

        self.leg.addObject('BoxROI',
                           name = 'legEndMotor', 
                           template ='Vec3',
                           position = positions,
                           orientedBox = obox,
                           drawBoxes = '1',
                           drawPoints = '1')


        # Second the extremity
        boxes = [[0, -5., -5., 10., 5., 5.]]
        for idx,box in enumerate(boxes):
            obox[idx] = [0,box[1],box[2],
                         0,box[1],box[5],
                         0,box[4],box[5],
                         box[3]-box[0]]
            
            obox[idx][0:3] = np.dot(rot,obox[idx][0:3]) + positionExtremity
            obox[idx][3:6] = np.dot(rot,obox[idx][3:6]) + positionExtremity
            obox[idx][6:9] = np.dot(rot,obox[idx][6:9]) + positionExtremity

            indR, indD = getIndicesInOrientedBox(positions = positions,
                                                 obox = obox[idx])
            indicesRigidified2 = list(np.unique(indicesRigidified2 + indR))
            indicesDeformable = np.intersect1d(indicesDeformable, indD)

        self.leg.addObject('BoxROI',
                           name = 'legEndTCP',
                           template ='Vec3',
                           position = positions,
                           orientedBox = obox,
                           drawBoxes = '1',
                           drawPoints = '1')

        indexPairs = []
        incr = [0, 0, 0]
        for index in range(len(positions)):
            if index in indicesRigidified1:
                indexPairs.append([0, incr[0]])
                incr[0] += 1
            elif index in indicesRigidified2:
                indexPairs.append([1, incr[1]])
                incr[1] += 1
            else:
                indexPairs.append([2, incr[2]])
                incr[2] += 1

        assert len(indicesRigidified1) != 0, "The position of the leg seems to be incorrect."
        assert len(indicesRigidified2) != 0, "The position of the leg seems to be incorrect."
        assert len(indicesDeformable) != 0, "The position of the leg seems to be incorrect."
        return indicesRigidified1, indicesRigidified2, indicesDeformable, indexPairs

class ndtRobot(Emio) :
    
    prefabData = Emio.prefabData
    prefabData[-1] = {'name': 'motorsDistanceToCenter', 'type': 'vector<double>', 'help': '', 'default': [70, 70, 70, 70, 70, 70]}
    
    def __init__(self, *args, **kwargs,):
        Sofa.Prefab.__init__(self, *args, **kwargs)
        centerPartClass = kwargs.get('centerPartClass',CenterPart)
        legsPositions = kwargs.get('legsPositions',[None])
        legsMeshData = kwargs.get('legsMeshData',[None])

        self._centerPartClass = centerPartClass
        self.addData(name="nbLegs", type="int", value=6) #value=4
        centerPartPositions = []

        # Add motors and legs
        self.motors = []
        self.legs = []

        distances = self.motorsDistanceToCenter.value
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

                leg = ndtLeg(name="Leg" + str(i),
                        legName=self.legsName.value[i] if i < len(self.legsName.value) else self.legsName.value[0],
                        positionOnMotor = positionOnMotor,
                        model = model,
                        translation = translation, rotation=rotation,
                        massDensity = legsMassDensity[i] if i < len(legsMassDensity) else legsMassDensity[0],
                        poissonRatio = legsPoissonRatio[i] if i < len(legsPoissonRatio) else legsPoissonRatio[0],
                        youngModulus = legsYoungModulus[i] if i < len(legsYoungModulus) else legsYoungModulus[0],
                        positions = legsPositions[i] if i < len(legsPositions) else legsPositions[0],
                        nodes = legsMeshData[i][0] if i < len(legsMeshData) else legsMeshData[0][0],
                        triangles = legsMeshData[i][1] if i < len(legsMeshData) else legsMeshData[0][1]
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
            self.centerpart = centerPartClass(name = "CenterPart",
                                              positions = centerPartPositions,
                                              partName = self.centerPartName.value,
                                              model = self.centerPartModel.value,
                                              massDensity = self.centerPartMassDensity.value,
                                              poissonRatio = self.centerPartPoissonRatio.value,
                                              youngModulus = self.centerPartYoungModulus.value,
                                              type = self.centerPartType.value,
                                              color = color,
                                              rotation = [0, 0, 0] if "down" in legsPositionOnMotor[0] else [180, 180, 0]
                                              )
            if self.centerPartType.value == "rigid":
                self.effector = self.centerpart.addChild("Effector")
            else:
                self.effector = self.centerpart.attach.addChild("Effector")
            self.addChild(self.centerpart)
            self._addBox()
            self._addCamera()
    
    def _addBox(self):
        pass
    
    def _addCamera(self):
        pass
    
    def _addPlatform(self):
        pass