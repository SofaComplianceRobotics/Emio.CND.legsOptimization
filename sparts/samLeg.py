import os.path
import sys
from math import sin, cos, pi
import numpy as np

# EmioLabs Libraries
from splib3.loaders import getLoadingLocation
from splib3.numerics import Quat, to_radians, to_degrees
from utils.topology import getExtremityFromBase
# from utils.topology import applyRotation
from scipy.spatial.transform import Rotation

# Sofa Library
import Sofa


class SamLeg(Sofa.Prefab):
    """
    The `Leg` class represents a leg component of the Sam robot. It includes rigid base, rigid extremity, 
    and deformable part, and supports multiple modeling options such as beam, cosserat, and tetra models.
    By default, the legs are added to the Sam class.

    Class Overview:
        - `base`: Represents the rigid base of the leg, typically attached to the motor.
        - `extremity`: Represents the rigid extremity part of the leg, typically attached to a connector.
        - `deformable`: Represents the deformable part of the leg.
        - `leg`: Represents the entire leg model.

    Key Features:
        - Supports multiple modeling techniques: "beam", "cosserat", and "tetra".
        - Automatically adjusts rotation and translation based on the motor position.
        - Includes visual and physical modeling components.

    Class Variables:
        - `legName` (`string`): Name of the leg, should have corresponding meshes in the "data/meshes/legs" directory.
        - `positionOnMotor` (`string`): Specifies the position on the motor. Options are: "clockwiseup", "counterclockwiseup", "clockwisedown", "counterclockwisedown"
        - `model` (`string`): Specifies the modeling technique. Options are: "beam", "cosserat", "tetra"
        - `massDensity` (`float`): Mass density of the leg material.
        - `poissonRatio` (`float`): Poisson's ratio of the leg material.
        - `youngModulus` (`float`): Young's modulus of the leg material.
        - `rotation` (`Vec3d`): Rotation of the leg.
        - `translation` (`Vec3d`): Translation of the leg.
        - `positions` (`Rigid3::VecCoord`, optional): Optional list of Rigid3 positions describing the leg's rod shape (for `beam` and `cosserat` models). If none is given, the "data/meshes/legs" directory should contain a file named "legName.txt" with the positions.
        - `crossSectionShape` (`string`): Shape of the cross-section. Options are: "circular", "rectangular"
        - `radius` (`float`): Radius of the leg, if the `crossSectionShape` is `circular` (for `beam` and `cosserat` models).
        - `thickness` (`float`): Thickness of the leg, if the `crossSection` is `rectangular` (for `beam` and `cosserat` models).
        - `width` (`float`): Width of the leg, if the `crossSection` is `rectangular` (for `beam` and `cosserat` models).

    Expected files in the "data/meshes/legs" directory:
        - legName.stl: surface mesh for the visual model. Only used for `tetra` model, for `beam` and `cosserat` models, the mesh is created from the positions.
        - legName.vtk: volume mesh for the tetra model
        - legName.txt: file containing the positions of the leg (for beam and cosserat models)

    Example Usage:
    ```python
    from utils.header import addHeader, addSolvers

    def createScene(root):
        # Header of the simulation
        settings, modelling, simulation = addHeader(root)
        addSolvers(simulation)
        # Create a leg instance
        leg = Leg(name="Leg",
                    legName="blueleg",
                    model="beam",
                    positionOnMotor="clockwiseup")
        if not leg.isValid():
            return
        simulation.addChild(leg)
    ```
    """
    prefabData = [
        {'name': 'legName', 'type': 'string', 'help': '', 'default': None},        
        {'name': 'positionOnMotor', 'type': 'string', 'help': 'clockwiseup, counterclockwiseup, clockwisedown, or counterclockwisedown', 'default': 'clockwiseup'},
        {'name': 'model', 'type': 'string', 'help': '["beam", "cosserat", "tetra"]', 'default': "beam"},
       
        {'name': 'massDensity', 'type': 'float', 'help': '', 'default': 1.22e-6},
        {'name': 'poissonRatio', 'type': 'float', 'help': '', 'default': 0.45},
        {'name': 'youngModulus', 'type': 'float', 'help': '', 'default': 3.5e4},
        {'name': 'rotation', 'type': 'Vec3d', 'help': '', 'default': [0.0, 0.0, 0.0]},
        {'name': 'translation', 'type': 'Vec3d', 'help': '', 'default': [0.0, 0.0, 0.0]},
       
        {'name': 'positions', 'type': 'Rigid3::VecCoord', 'help': '', 'default': None},
        {'name': 'crossSectionShape', 'type': 'string', 'help': '"circular" or "rectangular"', 'default': "rectangular"},
        {'name': 'radius', 'type': 'float', 'help': '', 'default': 5.},
        {'name': 'thickness', 'type': 'float', 'help': '', 'default': 5},
        {'name': 'width', 'type': 'float', 'help': '', 'default': 10},
        {'name': 'nodes', 'type': 'vector<Vec3d>', 'help': '', 'default': None},
        {'name': 'triangles', 'type': 'vector<Triangle>', 'help': '', 'default': None},
        {'name': 'collisionNodes', 'type': 'vector<Vec3d>', 'help': '', 'default': None},
        {'name': 'collisionTriangles', 'type': 'vector<Triangle>', 'help': '', 'default': None},
        {'name': 'mode', 'type': 'string', 'help': '', 'default': 'trial'}

    ]
    
    _validState = False
    
    def __init__(self, *args, **kwargs):

        Sofa.Prefab.__init__(self, *args, **kwargs)

        self._addRequiredPlugins()

        self.base = self.addChild(self.name.value + 'RigidBase')
        self.extremity = self.addChild(self.name.value + 'RigidExtremity')
        self.deformable = self.addChild(self.name.value + 'DeformablePart')

        self.leg = Sofa.Core.Node("Leg")

        self.color = [197/255, 179/255, 88/255, 1]
        self.attachSpring = None

        i = ["clockwiseup", "counterclockwiseup", "clockwisedown", "counterclockwisedown"].index(self.positionOnMotor.value)
        rotation = [[0., 0., 0.], [0., 180., 0.], [180., 180., 0.], [180., 0., 0.]][i]
        
        q = Quat.createFromEuler(to_radians(self.rotation.value))
        q.rotateFromEuler(to_radians(rotation))
        self.rotation.value = to_degrees(q.getEulerAngles())

        #self.rotation.value = (Rotation.from_euler('xyz',rotation,degrees=True) * Rotation.from_euler('xyz',self.rotation.value,degrees=True)).as_euler('xyz')

        if self._checkData():
            self._addBeamModel()
            if self.mode.value == 'trial':
                self._addVisualModel()
            self._validState = True

        if self.positionOnMotor.value in ["clockwiseup", "clockwisedown"]:
            with self.extremity.getMechanicalState().position.writeable() as position:
                q1 = Quat(position[0][3:7])
                q2 = Quat([1., 0., 0., 0.])
                q1.rotateFromQuat(q2)
                q1.normalize()
                position[0][3:7] = q1
                
                #position[0][3:7] = (Rotation.from_quat(position[0][3:7])*Rotation.from_quat([1,0,0,0])).as_quat()

        if self.collisionNodes.value.any():
            self._addCollision()
    
    def _getFilePath(self, filename) -> str:
        """
        Get the file path of the given filename in the data/meshes/legs directory.
        Returns the full path if the file exists, otherwise returns None.
        """
        
        # First check relative to the simulation file
        filePath = getLoadingLocation(os.path.dirname(os.path.abspath(sys.argv[0])) + '/data/meshes/legs/' + filename, __file__)
        if os.path.isfile(filePath):
            return filePath
            
        # Then check relative to the leg.py file
        filePath = getLoadingLocation(os.path.dirname(os.path.abspath(__file__)) + "/../data/meshes/legs/" + filename, __file__)
        if os.path.isfile(filePath):
            return filePath
        
        return None

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
                    Sofa.msg_error(self.getName(),
                                   'Empty positions. We cannot model the leg without a list of '
                                   'Rigid3 positions describing the curve when using the '
                                   'beam or cosserat model')
                    return False
        else:
            Sofa.msg_error(self.getName(),
                           'Unknown model, value should be "beam", "cosserat", or "tetra".')
            return False

        filePath = self._getFilePath(self.legName.value + ".stl")

        if filePath is None and (not self.nodes.value.any() or not self.triangles.value.any()):
            return False
        else:
            volume = self.addChild("Volume")
            if self.nodes.value.any() and self.triangles.value.any():
                volume.addObject("MeshTopology",
                                 position = self.nodes.value,
                                 triangles = self.triangles.value)
                position = volume.MeshTopology.position.value
                triangles = volume.MeshTopology.triangles.value
            else:
                volume.addObject("MeshSTLLoader", filename = filePath)
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
        
        if self.nodes.value.any() and self.triangles.value.any():
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

    def _addBeamModel(self):
        """
        FEM line model from the plugin BeamAdapter.
        base, extremity, deformable
        (all parents of leg)
        """
        positions = np.copy([np.copy(pos) for pos in self.positions.value])
        #applyRotation(positions, to_radians(self.rotation.value))
        #applyTranslation(positions, self.translation.value)
        
        for position in positions:
            er = Rotation.from_euler('xyz',self.rotation.value,degrees=True)
            qr = Rotation.from_quat(position[3:7])
            position[:3] = er.apply(position[:3])
            position[3:7] = (er * qr).as_quat()
            position[:3] += self.translation.value        
        
        nbPoints = len(positions)
        nbSections = nbPoints - 1

        # The entire model
        self.leg.addObject('MeshTopology', 
                           position=[pos[0:3] for pos in positions],
                           edges=[[i, i + 1] for i in range(nbSections)])
        self.leg.addObject('MechanicalObject', template='Rigid3', position=positions)
        self.leg.addObject('BeamInterpolation', straight=False, dofsAndBeamsAligned=False,
                           crossSectionShape=self.crossSectionShape.value,
                           lengthY=self.width.value,
                           lengthZ=self.thickness.value,
                           defaultYoungModulus=self.youngModulus.value,  
                           defaultPoissonRatio=self.poissonRatio.value,
                           radius=self.radius.value)
        self.leg.addObject('AdaptiveBeamForceFieldAndMass', computeMass=True,
                           massDensity=self.massDensity.value)

        # The extremity and the base of the leg are attached to something (either the motor or a support)
        # Thus, we need to rigidify these parts.
        indicesRigidified1, indicesRigidified2, indicesDeformable, indexPairs = self._getIndicesDistribution(self.leg.MeshTopology)

        # The rigid base part
        self.base.addObject('MechanicalObject', template='Rigid3', position=positions[0])
        baseGroup = self.base.addChild("RigidifiedPoints")
        baseGroup.addObject('MechanicalObject', template='Rigid3', position=positions[indicesRigidified1],
                            showObject=False, showObjectScale=10)
        baseGroup.addObject("RigidMapping", globalToLocalCoords=True)
        baseGroup.addChild(self.leg)

        # The rigid extremity part
        self.extremity.addObject('MechanicalObject', template="Rigid3",
                                 position=positions[-1],
                                 showObject=False, showObjectScale=10)
        extremityGroup = self.extremity.addChild("ExtremityGroup")
        extremityGroup.addObject('MechanicalObject', position=positions[indicesRigidified2],
                                 template="Rigid3",
                                 showObject=False, showObjectScale=10)
        extremityGroup.addObject("RigidMapping", globalToLocalCoords=True)
        extremityGroup.addChild(self.leg)

        # The deformable part
        self.deformable.addObject('MechanicalObject', template='Rigid3', position=positions[indicesDeformable],
                                  showObject=False, showObjectScale=10)
        self.deformable.addChild(self.leg)

        # Mapping separating rigid and deformable parts
        self.leg.addObject('SubsetMultiMapping', template="Rigid3,Rigid3",
                           input=[baseGroup.getMechanicalState().linkpath,
                                  extremityGroup.getMechanicalState().linkpath,
                                  self.deformable.getMechanicalState().linkpath],
                           output=self.leg.getMechanicalState().linkpath,
                           indexPairs=indexPairs)

    def _getIndicesInOrientedBox(self, positions, obox):
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

            indR, indD = self._getIndicesInOrientedBox(positions=positions, obox=obox[idx])
            # print(f'Ins : {indR1}, Outs : {indD1}')

            indicesRigidified1 = list(np.unique(indicesRigidified1 + indR))
            indicesDeformable = indD if len(indicesDeformable) == 0 else np.intersect1d(indicesDeformable, indD)
        indexExtremity = getExtremityFromBase(topology, indicesRigidified1[0])
        positionExtremity = positions[indexExtremity]

        self.leg.addObject('BoxROI',
                           name = 'legEndMotor', 
                           template ='Vec3',
                           position = positions,
                           orientedBox = obox)


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

            indR, indD = self._getIndicesInOrientedBox(positions = positions, obox = obox[idx])
            indicesRigidified2 = list(np.unique(indicesRigidified2 + indR))
            indicesDeformable = np.intersect1d(indicesDeformable, indD)

        self.leg.addObject('BoxROI',
                           name = 'legEndTCP',
                           template ='Vec3',
                           position = positions,
                           orientedBox = obox)

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
    
    def _addCollision(self):
        collision = self.leg.addChild("Collision")
        collision.addObject("MeshTopology",
                            position = self.collisionNodes.value,
                            triangles = self.collisionTriangles.value)
        
        collision.addObject("MechanicalObject",
                            rotation=self.rotation.value,
                            translation=self.translation.value)
        
        collision.addObject("TriangleCollisionModel", group=1)
        collision.addObject("LineCollisionModel", group=1)
        collision.addObject("PointCollisionModel", group=1)
        collision.addObject("SkinningMapping")
    
    def _addRequiredPlugins(self):
        """
        Add the RequiredPlugins of the Leg class.
        """
        plugins = self.addChild("RequiredPlugins")
        plugins.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')
        # Needed to use components [MeshOBJLoader]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic')
        # Needed to use components [EdgeSetTopologyContainer]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')
        # Needed to use components [MeshTopology]
        plugins.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')
        # Needed to use components [OglModel]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Grid')
        # Needed to use components [RegularGridTopology]
        plugins.addObject('RequiredPlugin', name='SoftRobots')
        plugins.addObject('RequiredPlugin', name='BeamAdapter')
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')
        # Needed to use components [SubsetMultiMapping]
        plugins.addObject('RequiredPlugin', name='Cosserat')
        plugins.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')
        # Needed to use components [TetrahedronFEMForceField]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')
        # Needed to use components [RigidMapping]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective')
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Engine.Generate')
        # Needed to use components [VolumeFromTriangles]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select')

    def attachBase(self, attach, index) -> None:
        """
        Attach the base of the leg to the motor.

        Function Parameters:
            - `attach` (`Sofa.Node`): The node to which the leg's base will be attached.
            - `index` (`int`): The index of the degrees of freedom (DOFs) in the `attach` node to connect to.
        """
        if not self._validState:
            return

        base = self.leg if self.model.value == "cosserat" else self.base
        baseIndex = len(self.leg.getMechanicalState().position) - 1 if self.model.value == "cosserat" else 0

        legAttach = attach.addChild("LegAttach")
        legAttach.addObject("MechanicalObject", template="Rigid3", showObject=False, showObjectScale=10,
                            position=base.getMechanicalState().position.value[baseIndex])
        legAttach.addObject("RigidMapping", globalToLocalCoords=True, index=index)

        self._attach(part=base, attach=legAttach, indexPart=baseIndex, indexAttach=0)

    def attachExtremity(self, attach, index) -> None:
        """
        Attach the extremity of the leg to the motor.

        Function Parameters:
            - `attach` (`Sofa.Node`): The node to which the leg's extremity will be attached.
            - `index` (`int`): The index of the degrees of freedom (DOFs) in the `attach` node to connect to.
        """
        if not self._validState:
            return
        self._attach(part=self.extremity, attach=attach, indexPart=0, indexAttach=index)

    def _attach(self, part, attach, indexPart, indexAttach):
        difference = part.addChild("Difference"+str(indexAttach))
        attach.addChild(difference)
        difference.addObject("MechanicalObject", template="Rigid3", position=[[0, 0, 0, 0, 0, 0, 1]])
        self.attachSpring = difference.addObject('RestShapeSpringsForceField', points=[0],
                                                 stiffness=1e7, angularStiffness=1e14)
        difference.addObject('RigidDistanceMapping',
                             input1=part.getMechanicalState().linkpath,
                             input2=attach.getMechanicalState().linkpath,
                             output=difference.getMechanicalState().linkpath,
                             first_point=[indexPart], second_point=[indexAttach])

    def isValid(self) -> bool:
        """
        Check if the leg is in a valid state. Returns True if the leg is in a valid state, False otherwise.
        """
        return self._validState

    def _checkFile(self, filename) -> bool:
        """
        Check if the file exists in the data/meshes/legs directory.
        Returns True if it exists, False otherwise and logs an error.
        """
        if self._getFilePath(filename) is not None:
            return True
            
        Sofa.msg_error(self.getName(), f'Missing file: {filename} in data/meshes/legs directory.')
        return False
    