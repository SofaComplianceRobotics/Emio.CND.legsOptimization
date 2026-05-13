import os.path
import sys
from math import sin, cos, pi
import numpy as np

# EmioLabs Libraries
from parts.leg import Leg
from splib3.loaders import getLoadingLocation
from utils.topology import getExtremityFromBase

# Sofa Library
import Sofa

class SamLeg(Leg):
    
    prefabData = Leg.prefabData
    prefabData.append({'name': 'nodes', 'type': 'vector<Vec3d>', 'help': '', 'default': None})
    prefabData.append({'name': 'triangles', 'type': 'vector<Triangle>', 'help': '', 'default': None})
    prefabData.append({'name': 'collisionNodes', 'type': 'vector<Vec3d>', 'help': '', 'default': None})
    prefabData.append({'name': 'collisionTriangles', 'type': 'vector<Triangle>', 'help': '', 'default': None})
    prefabData.append({'name': 'mode', 'type': 'string', 'help': '', 'default': 'trial'})
    
    def __init__(self, *args, **kwargs):

        super().__init__(self,*args,**kwargs)
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
                
        elif self.model.value == "tetra":
            if self.legName.value is None:
                Sofa.msg_error(self.getName(),
                               'Empty legName. We cannot model the leg with tetra without a volume mesh.')
                return False
            
            if not self._checkFile(self.legName.value + ".vtk"):
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
        if self.mode.value == 'trial':
            if self.nodes.value.any() and self.triangles.value.any():
                visual = self.leg.addChild("Visual")
                visual.addObject("MeshTopology",
                                position = self.nodes.value,
                                triangles = self.triangles.value)
                visual.addObject("OglModel",
                                src = visual.MeshTopology.linkpath,
                                color = [197/255, 179/255, 88/255, 1],
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

            indR, indD = getIndicesInOrientedBox(positions = positions,
                                                 obox = obox[idx])
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