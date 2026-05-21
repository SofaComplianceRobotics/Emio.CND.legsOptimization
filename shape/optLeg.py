from __future__ import annotations
import os
from math import pi
# from ..tools import checkModule

# checkModule('gmsh')
# checkModule('pyclipper')
# checkModule('cadquery')

import gmsh
import cadquery as cq
from json import loads
from beziers.path import BezierPath
from beziers.line import Line
from beziers.point import Point
from beziers.segment import Segment

if __name__ =='__main__' or __name__=='optLeg':
    from centerLine import CenterLine
else:
    from shape.centerLine import CenterLine

CAD_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/../data/meshes/legs/'
FIG_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/../data/'#+'/data/figs/'
STL_DIRECTORY = os.path.dirname(os.path.realpath(__file__))#+'/data/cad/

class OptLeg:
    def __init__(self, 
             name:str = 'leg',
             ltype:str = 'low_leg',
             crossSection:list = [10,5],
             legHeight:float = 30, 
             youngModulus:float = 3.5e4, 
             poissonRatio:float = 0.45, 
             density:float = 1.22e-6, 
             pulleyRadius:float = 30, 
             motorDistanceToCenter:float = 50,
             nBeams:int = 10,
             centerLine:CenterLine = CenterLine(name =' leg',
                                                numberOfPoints = 3)):
        
        self.name = name
        self.crossSection = crossSection
        self.legHeight = legHeight
        self.youngModulus = youngModulus
        self.poissonRatio = poissonRatio
        self.density = density
        self.pulleyRadius = pulleyRadius
        self.motorDistanceToCenter = motorDistanceToCenter
        centerLine.name = name
        self.centerLine = centerLine
        self.nBeams = nBeams
        self.beams = centerLine.getBeams(nBeams)
        self.type = ltype
    
    def fromTrial(self,filepath):
        with open(filepath,'r',encoding="utf-8") as f:
            data = loads(f.read())
        data = data[self.type]
        self.name = data['name']
        self.crossSection = data['crossSection']
        self.legHeight = data['legHeight']
        self.youngModulus = data['youngModulus']
        self.poissonRatio = data['poissonRatio']
        self.density = data['density']
        self.pulleyRadius = data['pulleyRadius']
        self.motorDistanceToCenter = data['motorDistanceToCenter']
        self.nBeams = data['nBeams']
        self.centerLine.setCurvePoints(data['controlPoint'])
        self.centerLine.computePath()
        #self.update(controlPoint = data['controlPoint'])
        self.beams = self.centerLine.getBeams(self.nBeams)
    
    def _initializeGmsh(self, debug:bool=False):
        """
        _summary_
            Sets GMSH options for meshing and debugging.
        Args:
            debug (bool) : Shows all warning and error messages if it is set to true. Default to false.
        """ 
        if not gmsh.isInitialized():
            gmsh.initialize()
            
            #debugging
            if debug:
                gmsh.option.setNumber("General.Terminal", 1) #to capture every terminal event
                gmsh.option.setNumber("General.Verbosity",99) # to debug or 99
            else:
                gmsh.option.setNumber("General.Terminal", 0)
            
            #Meshing
            gmsh.option.setNumber('General.AbortOnError',1)
            
            gmsh.option.setNumber('Mesh.Binary',1)
            gmsh.option.setNumber('Mesh.MaxRetries',0)
            #gmsh.option.setNumber('Mesh.MaxNumThreads2D',22)
            gmsh.option.setNumber('Mesh.AlgorithmSwitchOnFailure',0)

            gmsh.option.setNumber('Mesh.Algorithm', 6) #(1: MeshAdapt, 2: Automatic, 3: Initial mesh only, 5: Delaunay, 6: Frontal-Delaunay, 7: BAMG, 8: Frontal-Delaunay for Quads, 9: Packing of Parallelograms, 11: Quasi-structured Quad)
            #gmsh.option.setNumber('Mesh.MeshSizeFromCurvatureIsotropic',1)
            gmsh.option.setNumber('Mesh.MeshSizeFromCurvature',24)
            #gmsh.option.setNumber('Mesh.MeshSise',self.crossSection[1])
            #gmsh.option.setNumber('Mesh.MeshSizeFactor',0.5)

            # gmsh.option.setNumber('Mesh.Algorithm', 11)
            # gmsh.option.setNumber('Mesh.MeshSizeFromCurvature',24)

    def _renderMeshGmsh(self, inputExtension=None, debug:bool=False)->bool:
        """
        _summary_
            Generates a 2D mesh of a 3D model using GMSH, and returns True if successful.
        Args:
            inputExtension (str) : . Default to None.
            debug (bool) : Shows a preview of the mesh using GMSH gui if it is set to True. Default to False.
        Returns:
            success (bool) : True if geometry is meshed.
        """ 
        
        self._initializeGmsh(debug)
        
        if inputExtension is not None:
            gmsh.open(self.name+'.'+inputExtension)
        gmsh.model.mesh.generate(2)
        
        if debug:
            gmsh.fltk.run() # to view the mesh
        
        if gmsh.logger.getLastError():
            gmsh.finalize()
            return False
        else:
            gmsh.write(self.name+'.stl')
            gmsh.finalize()
            if not debug and inputExtension is not None:
                os.remove(self.name+'.'+inputExtension)
            return True
        
    def _getShapeGmsh(self,debug=False):
        curves=self.centerLine.path.asSegments()
        self._initializeGmsh(debug)
        occGeo = gmsh.model.occ() #OpenCascade kernel
        corner = [-self.crossSection[0]/2, 
                  -self.crossSection[1]/2+curves[0].start.y,
                  curves[0].start.x]
        crossSection= [(2,occGeo.addRectangle(*corner,*self.crossSection))]
        occGeo.rotate(crossSection,
                      0,
                      curves[0].start.y,
                      curves[0].start.x,
                      1,0,0,angle=pi/2)
        bezier = [(1,occGeo.addBezier([occGeo.addPoint(0, 
                                                       curve.start.y,
                                                       curve.start.x),
                                       occGeo.addPoint(0,
                                                       curve.points[1].y,
                                                       curve.points[1].x),
                                       occGeo.addPoint(0,
                                                       curve.points[2].y,
                                                       curve.points[2].x),
                                       occGeo.addPoint(0,
                                                       curve.end.y,
                                                       curve.end.x)])) for curve in curves]
        fusedBezier,_ = occGeo.fuse(bezier,bezier)
        _,fusedBezierTag = zip(*fusedBezier)
        
        if len(fusedBezierTag)>1:
            return False
        
        wireTag = occGeo.addWire(list(fusedBezierTag))
        occGeo.addPipe(crossSection,wireTag)
        occGeo.remove(fusedBezier)
        isolatedPoints = occGeo.getEntities(dim=0)
        occGeo.remove(isolatedPoints)
        occGeo.synchronize()
        return True

    def exportSTLGmsh(self,path=CAD_DIRECTORY,debug=False)->bool:
        os.chdir(path)
        shapeState = self._getShapeGmsh(debug=debug)
        meshState = self._renderMeshGmsh(debug=debug)
        return shapeState and meshState
    
    def getTopologyGmsh(self,debug:bool=False)->list:
        if not self._getShapeGmsh(debug):
            gmsh.finalize()
            return []
        gmsh.model.mesh.generate(2)

        if gmsh.logger.getLastError():
            gmsh.finalize()
            return []

        _, coord,_ = gmsh.model.mesh.getNodes()

        if not coord.any():
            gmsh.finalize()
            return []

        elementType = gmsh.model.mesh.getElementTypes(dim=2)[0]
        nodesPerElement = gmsh.model.mesh.getElementProperties(elementType)[3]
        nodes, _,_ = gmsh.model.mesh.getNodesByElementType(elementType)
        nodes = [x-1 for x in nodes]
        elements = [list(map(int,nodes[nodesPerElement*idx:nodesPerElement*(idx+1)])) for idx in range(int(len(nodes)/nodesPerElement))]
        positions = [coord[3*idx:3*(idx+1)] for idx in range(int(len(coord)/nodesPerElement))]
        gmsh.finalize()
    
        return [positions, elements]
        
    def _getShapeCADQuery(self,debug:bool=False)->bool:
        curves = self.centerLine.path.asSegments()
        start = curves[0].start
        crossSection = cq.Workplane("XZ",origin=(0,start.y,start.x)).rect(*self.crossSection)
        wire = cq.Workplane("YZ")
        for curve in curves:
            wire = wire.bezier([(curve.start.y,curve.start.x),
                                (curve.points[1].y,curve.points[1].x),
                                (curve.points[2].y,curve.points[2].x),
                                (curve.end.y,curve.end.x)])
        volume = crossSection.sweep(wire,multisection=True,transition='round').val()
        
        if volume.isValid() and not volume.isNull():
            volume.exportBrep(self.name+'.brep')
            return True
        else:
            return False
    
    def exportSTLCadQuery(self,path=CAD_DIRECTORY,debug=False)->bool:
        os.chdir(path)
        curves = self.centerLine.path.asSegments()
        start = curves[0].start
        crossSection = cq.Workplane("XZ",origin=(0,start.y,start.x)).rect(*self.crossSection)
        wire = cq.Workplane("YZ")
        for curve in curves:
            wire = wire.bezier([(curve.start.y,curve.start.x),
                                (curve.points[1].y,curve.points[1].x),
                                (curve.points[2].y,curve.points[2].x),
                                (curve.end.y,curve.end.x)])
        volume = crossSection.sweep(wire,multisection=True,transition='round').val()
        
        if volume.isValid() and not volume.isNull():
            volume.exportBrep(self.name+'.brep')
            if not self._renderMeshGmsh('brep',debug):
                return volume.exportStl(self.name+'.stl')
            else:
                return True
        else:
            if debug:
                volume.exportBrep(self.name+'.brep')
            return False

    def exportCadQuery(self, path=CAD_DIRECTORY, name=None, extension='.step'):
        os.chdir(path)
        if name is None:
            name=self.name
        curves = self.centerLine.path.asSegments()
        start = curves[0].start
        crossSection = cq.Workplane("XZ",origin=(0,start.y,start.x)).rect(*self.crossSection)
        wire = cq.Workplane("YZ")
        for curve in curves:
            wire = wire.bezier([(curve.start.y,curve.start.x),
                                (curve.points[1].y,curve.points[1].x),
                                (curve.points[2].y,curve.points[2].x),
                                (curve.end.y,curve.end.x)])
        volume = crossSection.sweep(wire,multisection=True,transition='round')
        evalVol = volume.val()
        if evalVol.isValid() and not evalVol.isNull():
            return volume.export(name+extension)
        else:
            return False

    def getTopologyCADQuery(self,debug=False)->list:
        self._getShapeCADQuery(debug)
        
        self._initializeGmsh(debug)
        gmsh.open(self.name+'.brep')
        gmsh.model.mesh.generate(2)
        
        if debug:
            gmsh.fltk.run() # to view the mesh

        _, coord,_ = gmsh.model.mesh.getNodes()
        elementType = gmsh.model.mesh.getElementTypes(dim=2)[0]
        nodesPerElement = gmsh.model.mesh.getElementProperties(elementType)[3]
        nodes, _,_ = gmsh.model.mesh.getNodesByElementType(elementType)
        nodes = [x-1 for x in nodes]
        elements = [list(map(int,nodes[nodesPerElement*idx:nodesPerElement*(idx+1)])) for idx in range(int(len(nodes)/nodesPerElement))]
        positions = [coord[3*idx:3*(idx+1)] for idx in range(int(len(coord)/nodesPerElement))]

        gmsh.finalize()
    
        return [positions, elements]

    def _getCollisionShapeGmsh(self,center_part=None,debug=False):
        
        curves = self.centerLine.path.asSegments()
        length = self.centerLine.path.length
        tolerance = 0.01

        if center_part:
            ending_distance = length-center_part.hThickness-center_part.hDepth
            starting_distance = length-center_part.height
            ending_time = ending_distance/length
            starting_time = starting_distance/length

            while abs(ending_distance-self.centerLine.path.lengthAtTime(ending_time))/ending_distance >= tolerance:
                if ending_distance <= self.centerLine.path.lengthAtTime(ending_time):
                    ending_time -= 0.001
                else:
                    ending_time += 0.001 

            while abs(starting_distance-self.centerLine.path.lengthAtTime(starting_time))/starting_distance >= tolerance:
                if starting_distance <= self.centerLine.path.lengthAtTime(starting_time):
                    starting_time -= 0.001
                else:
                    starting_time += 0.001 

        else:
            ending_time = 2.5/3
            starting_time = 2/3

        start = self.centerLine.path.pointAtTime(t=starting_time)
        end = self.centerLine.path.pointAtTime(t=ending_time)

        self._initializeGmsh(debug)
        occGeo = gmsh.model.occ() #OpenCascade kernel
        
        start_p1 = occGeo.addPoint(x = -self.crossSection[0]/2, 
                                   y = start.y,
                                   z = +self.crossSection[1]/2+start.x)
        
        start_p2 = occGeo.addPoint(x = self.crossSection[0]/2,
                                   y = start.y,
                                   z = +self.crossSection[1]/2+start.x)
        
        start_line = occGeo.addLine(start_p1,start_p2)
        
        end_p1 = occGeo.addPoint(x = -self.crossSection[0]/2,
                                 y = end.y,
                                 z = +self.crossSection[1]/2+end.x)
        
        end_p2 = occGeo.addPoint(x = self.crossSection[0]/2, 
                                 y = end.y,
                                 z = +self.crossSection[1]/2+end.x)
        
        end_line = occGeo.addLine(end_p1,end_p2)


        bezier = [(1,occGeo.addBezier([occGeo.addPoint(-self.crossSection[0]/2, 
                                                       curve.start.y,
                                                       self.crossSection[1]/2+curve.start.x),
                                       occGeo.addPoint(-self.crossSection[0]/2,
                                                       curve.points[1].y,
                                                       self.crossSection[1]/2+curve.points[1].x),
                                       occGeo.addPoint(-self.crossSection[0]/2,
                                                       curve.points[2].y,
                                                       self.crossSection[1]/2+curve.points[2].x),
                                       occGeo.addPoint(-self.crossSection[0]/2,
                                                       curve.end.y,
                                                       self.crossSection[1]/2+curve.end.x)])) for curve in curves]
        
        fusedBezier,_ = occGeo.fuse(bezier,bezier)
        mini_curve = occGeo.cut(fusedBezier,[(1,start_line),(1,end_line)],removeTool=True)
        occGeo.remove([mini_curve[0][0],mini_curve[0][-1]])
        occGeo.extrude([mini_curve[0][1]],self.crossSection[0],0,0)
        isolatedPoints = occGeo.getEntities(dim=0)
        occGeo.remove(isolatedPoints)
        occGeo.synchronize()

    def getCollisionTopologyGmsh(self, debug:bool=False):
        
        self._getCollisionShapeGmsh(debug)
        gmsh.model.mesh.generate(2)
        if gmsh.logger.getLastError():
            gmsh.finalize()
            return []
        
        _, coord,_ = gmsh.model.mesh.getNodes()

        if not coord.any():
            gmsh.finalize()
            return []
        
        elementType = gmsh.model.mesh.getElementTypes(dim=2)[0]
        nodesPerElement = gmsh.model.mesh.getElementProperties(elementType)[3]
        nodes, _,_ = gmsh.model.mesh.getNodesByElementType(elementType)
        nodes = [x-1 for x in nodes]
        elements = [list(map(int,nodes[nodesPerElement*idx:nodesPerElement*(idx+1)])) for idx in range(int(len(nodes)/nodesPerElement))]
        positions = [coord[3*idx:3*(idx+1)] for idx in range(int(len(coord)/nodesPerElement))]

        gmsh.finalize()
    
        return [positions, elements]

    def exportSTL(self, path=CAD_DIRECTORY, debug:bool=False)->bool:
        if self.centerLine.isSmooth:
            return self.exportSTLGmsh(path,debug)
        else:
            return self.exportSTLCadQuery(path,debug)

    def isValid(self)->bool:
        rightOffsetPath = self.centerLine.path.offset(self.centerLine.path.asSegments()[0].normalAtTime(0)*self.crossSection[1]/2)
        rightOffsetPath.closed = False
        rightOffsetIntersection = rightOffsetPath.getSelfIntersections() #pre sweep verification
        
        leftOffsetPath = self.centerLine.path.offset(self.centerLine.path.asSegments()[0].normalAtTime(0)*self.crossSection[1]/2*-1)
        leftOffsetPath.closed = False
        leftOffsetIntersection = leftOffsetPath.getSelfIntersections() #pre sweep verification
        
        legEndPointX = self.centerLine.path.asSegments()[-1].end.x+self.crossSection[1]/2
        bBoxRightX = self.centerLine.path.bounds().right

        collisionStatus = bool(legEndPointX >= bBoxRightX)

        centerLineStatus = (not bool(leftOffsetIntersection)) and (not bool(rightOffsetIntersection)) and self.centerLine.isValid()
        #shapeStatus = bool(self.getTopologyGmsh())

        return centerLineStatus and collisionStatus #and shapeStatus #and goodMesh

    def update(self,**kwargs):
        if 'controlPoint' in kwargs:
            self.centerLine.update(controlPoint = kwargs['controlPoint'])
        if 'nbeams' in kwargs:
            self.beams = self.centerLine.getBeams(kwargs['nbeams'])
        self.centerLine.update(kwargs=kwargs)

if __name__ == "__main__":
    leg = OptLeg(name='test')
    #leg.getTopologyGmsh()
    #leg.getCollisionTopologyGmsh()
    #leg.centerLine.plot()
    print(leg.isValid())
