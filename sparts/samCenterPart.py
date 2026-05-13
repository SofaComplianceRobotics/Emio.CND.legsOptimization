import Sofa
import os, sys
import numpy as np
import json
from splib3.loaders import getLoadingLocation
from utils.topology import getIndicesInBox
from parts.centerpart import CenterPart

class SamCenterPart(CenterPart):
    prefabData = CenterPart.prefabData
    prefabData.append({'name': 'collisionNodes', 'type': 'vector<Vec3d>', 'help': '', 'default': None})
    prefabData.append({'name': 'collisionTriangles', 'type': 'vector<Triangle>', 'help': '', 'default': None})
    prefabData.append({'name': 'mode', 'type': 'string', 'help': '', 'default': 'trial'})

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

        self._addRequiredPlugins()

        filename = self.partName.value + '.json'
        self._checkFile(filename)
        self._params = json.load(open(self._getFilePath(filename)))

        self.flip = False
        if self.rotation[0] == 180:  # the legs are pointing upward, we flip the center part
            self.flip = True

        self._checkFile(self.partName.value + '.stl')
        match self.type.value:
            case "rigid":
                self._addRigidCenterPart()
            case _:
                Sofa.msg_error("centerpart.py", 'Unknown model, value should be "deformable", or "rigid".')
                return
        if self.mode.value!='optimization':
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
    
    def _addRigidCenterPart(self):
        """
        Add a rigid center part to the simulation.
        """

        # Load the mesh and create the topology, mechanical object, and mass
        self.addObject('MechanicalObject', template='Rigid3', position=self._params["initialPosition"],
                       showObject=False, showObjectScale=20,
                       rotation=self.rotation)
        mass = self.addChild("ComputeMass")
        mass.addObject("MeshSTLLoader", filename=self._getFilePath(self.partName.value + ".stl"))
        mass.addObject('GenerateRigidMass', src=mass.MeshSTLLoader.linkpath, density=self.massDensity.value)
        self.addObject('UniformMass', vertexMass=mass.GenerateRigidMass.rigidMass.linkpath)

        # This node contains the positions to attach the legs
        self.attach = self.addChild("LegsAttach")
        attachposition = self._params["attachPositionInLocalCoord"]
        if self.flip:  # the legs are pointing upward, we flip the center part
            temp = attachposition[1]
            attachposition[1] = attachposition[3]
            attachposition[3] = temp
        self.attach.addObject("MechanicalObject", position=attachposition, template="Rigid3",
                              showObject=False, showObjectScale=10, showIndices=False, showIndicesScale=0.02)
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
        match self.type.value:
            case "rigid":
                visual = self.addChild("Visual")
                visual.addObject("MeshSTLLoader", filename=self._getFilePath(self.partName.value + ".stl"))
                visual.addObject("OglModel", src=visual.MeshSTLLoader.linkpath, color=self.color.value)
                visual.addObject('RigidMapping')
            case _:
                visual = self.part.addChild("Visual")
                visual.addObject("MeshSTLLoader", filename=self._getFilePath(self.partName.value + ".stl"), rotation=self.rotation)
                visual.addObject("OglModel", src=visual.MeshSTLLoader.linkpath, color=self.color.value)
                visual.addObject("BarycentricMapping")

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