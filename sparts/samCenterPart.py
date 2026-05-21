import Sofa
import os, sys
import numpy as np
import json
from splib3.loaders import getLoadingLocation
from utils.topology import getIndicesInBox

class SamCenterPart(Sofa.Prefab):
    """
    Represents the center part of the Sam robot that connected the legs together (also called connector).
    
    Class Variables:
        - `partName` (`string`): Name of the center part (e.g., "whitepart", "yellowpart", "bluepart"), should have corresponding meshes in the "data/meshes/centerparts" directory.
        - `type` (`string`): Type of the center part, either "deformable" or "rigid".
        - `model` (`string`): Model type between "tetra" and "beam", if deformable.
        - `massDensity` (`float`): Mass density of the center part.
        - `poissonRatio` (`float`): Poisson's ratio of the material, if deformable.
        - `youngModulus` (`float`): Young's modulus of the material, if deformable.
        - `color` (`Vec4d`): Color of the center part, for rendering.
        - `rotation` (`Vec3d`): Rotation of the center part in degrees.

    Class Members:
        - `attach`: Node containing the positions to attach the legs.
        - `deformable`: Node containing the deformable part, if deformable.

    Expected files in the "data/meshes/centerparts" directory:
        - partName.stl: surface mesh for the visual model. 
        - partName.json: file containing the initial position of the center part and the position of the legs'attach in local coordinates.

    Example Usage:
    ```python
    from centerpart import CenterPart
    from utils import addHeader, addSolvers
    def createScene(root):
        settings, modelling, simulation = addHeader(root)
        addSolvers(simulation)

        centerpart = root.addChild(CenterPart(name="CenterPart",
                                             partName="whitepart",
                                             color=[1, 1, 1, 1]))
    ```
    """

    prefabData = [
        {'name': 'positions', 'type': 'Rigid3::VecCoord', 'help': '', 'default': None},
        {'name': 'partName', 'type': 'string', 'help': '', 'default': None},
        {'name': 'model', 'type': 'string', 'help': 'model between tetra and beam', 'default': 'beam'},
        {'name': 'massDensity', 'type': 'float', 'help': '', 'default': 1.220e-6},
        {'name': 'poissonRatio', 'type': 'float', 'help': '', 'default': 0.45},
        {'name': 'youngModulus', 'type': 'float', 'help': '', 'default': 3.5e4}, 
        {'name': 'color', 'type': 'Vec4d', 'help': '', 'default': [1, 1, 1, 1]},
        {'name': 'rotation', 'type': 'Vec3d', 'help': '', 'default': [0, 0, 0]},
        {'name': 'collisionNodes', 'type': 'vector<Vec3d>', 'help': '', 'default': None},
        {'name': 'collisionTriangles', 'type': 'vector<Triangle>', 'help': '', 'default': None},
        {'name': 'mode', 'type': 'string', 'help': '', 'default': 'trial'}
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

        self._addRequiredPlugins()

        filename = self.partName.value +'.json'
        self._checkFile(filename)
        self._params = json.load(open(self._getFilePath(filename)))

        self.flip = False
        if self.rotation[0] == 180:  # the legs are pointing upward, we flip the center part
            self.flip = True

        self._checkFile(self.partName.value + '.stl')
        self._addRigidCenterPart()

        if self.mode.value!='optimization': #TO do replace with Enum class
            self._addVisualModel()
        self._addCollision()
    
    def _getFilePath(self, filename) -> str:
        """
        Get the file path of the given filename in the data/meshes/centerparts directory.
        Returns the full path if the file exists, otherwise returns None.
        """
 
        # First check relative to the simulation file
        filePath = getLoadingLocation(os.path.dirname(os.path.abspath(sys.argv[0])) + '/data/meshes/centerparts/' + filename, __file__)
        if os.path.isfile(filePath):
            return filePath
            
        # Then check relative to the centerpart.py file
        filePath = getLoadingLocation(os.path.dirname(os.path.abspath(__file__)) + "/../data/meshes/centerparts/" + filename, __file__)

        if os.path.isfile(filePath):
            return filePath
            
        return None
    
    def _checkFile(self, filename) -> bool:
        """
        Check if the file exists in the data/meshes/centerparts directory.
        Returns True if it exists, False otherwise and logs an error.
        """
        if self._getFilePath(filename) is not None:
            return True
            
        Sofa.msg_error(self.getName(), f'Missing file: {filename} in data/meshes/centerparts directory.')
        return False
    
    def _addRigidCenterPart(self):
        """
        Add a rigid center part to the simulation.
        """

        # Load the mesh and create the topology, mechanical object, and mass
        self.addObject('MechanicalObject', 
                       template='Rigid3', 
                       position=self._params["initialPosition"],
                       showObject=False, 
                       showObjectScale=20,
                       rotation=self.rotation)
        mass = self.addChild("ComputeMass")
        mass.addObject("MeshSTLLoader", 
                       filename=self._getFilePath(self.partName.value + ".stl"))
        mass.addObject('GenerateRigidMass', 
                       src=mass.MeshSTLLoader.linkpath, 
                       density=self.massDensity.value)
        self.addObject('UniformMass', 
                       vertexMass=mass.GenerateRigidMass.rigidMass.linkpath)

        # This node contains the positions to attach the legs
        self.attach = self.addChild("LegsAttach")
        attachposition = self._params["attachPositionInLocalCoord"]
        if self.flip:  # the legs are pointing upward, we flip the center part
            temp = attachposition[1]
            attachposition[1] = attachposition[3]
            attachposition[3] = temp
            
        self.attach.addObject("MechanicalObject", 
                              position=attachposition, 
                              template="Rigid3",
                              showObject=False,
                              showObjectScale=10,
                              showIndices=False,
                              showIndicesScale=0.02)
        self.attach.addObject("RigidMapping", globalToLocalCoords=False)

    def _getIndicesDistribution(self, topology):
        """
        Get the indices of the rigidified and deformable parts based on the attachment positions.
        The rigidified parts are the ones that are attached to the legs, while the deformable parts are the rest.
        Also create the index pairs for the mapping between the rigidified and deformable parts.
        """
        indicesRigidified = []
        indicesDeformable = []
        positions = topology.position.value

        attachposition = self._params["attachPositionInLocalCoord"]
        if self.flip:  # the legs are pointing upward, we flip the center part
            for pos in attachposition:
                pos[1] = -pos[1]

        for k in range(4):
            box = [-5., -5., -5., 5., 5., 5.]
            for i in range(3):
                box[i] += attachposition[k][i]
                box[i + 3] += attachposition[k][i]

            indR, indD = getIndicesInBox(positions=positions, box=box)
            indicesRigidified += [indR]
            indicesDeformable = indD if len(indicesDeformable) == 0 else np.intersect1d(indicesDeformable, indD)

        indexPairs = []
        incr = [0, 0]
        for index in range(len(positions)):
            for k in range(4):
                if index in indicesRigidified[k]:
                    indexPairs.append([0, incr[0]])
                    incr[0] += 1
            if index in indicesDeformable:
                indexPairs.append([1, incr[1]])
                incr[1] += 1

        return indicesRigidified, indicesDeformable, indexPairs

    def _addVisualModel(self):
        """
        Add the visual model of the center part to the simulation.
        The visual model is created using a mesh loaded from an STL file. The color of the model is set based on the parameters provided.
        The visual model is added to the appropriate node based on the type of center part (deformable or rigid).
        """

        visual = self.addChild("Visual")
        visual.addObject("MeshSTLLoader", filename=self._getFilePath(self.partName.value + ".stl"))
        visual.addObject("OglModel", src=visual.MeshSTLLoader.linkpath, color=self.color.value)
        visual.addObject('RigidMapping')

    def _addRequiredPlugins(self):
        plugins = self.addChild("RequiredPlugins")
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')
        # Needed to use components [MeshTopology]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Grid')
        # Needed to use components [RegularGridTopology]
        plugins.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')
        # Needed to use components [OglModel]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Engine.Generate')
        # Needed to use components [GenerateRigidMass]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select')
        
    def _addCollision(self):
        collision = self.addChild('Collision')
        collision.addObject("MeshTopology",
                            position = self.collisionNodes.value ,
                            triangles = self.collisionTriangles.value)
        collision.addObject("MechanicalObject")
        collision.addObject("TriangleCollisionModel")
        collision.addObject("LineCollisionModel")
        collision.addObject("PointCollisionModel")
        collision.addObject("RigidMapping")