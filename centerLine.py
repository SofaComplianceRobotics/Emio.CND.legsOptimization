"""_summary_

Returns:
    _type_: beam vector
"""
from __future__ import annotations
import os
from math import pi,sin,cos, floor
from tools import checkModule

checkModule('matplotlib')
import matplotlib.pyplot as plt

checkModule('scipy')
from scipy.spatial.transform import Rotation

checkModule('gmsh')
import gmsh

checkModule('pyclipper')
checkModule('cadquery')
import cadquery as cq

checkModule('beziers',gitLink='https://github.com/simoncozens/beziers.py.git')
from beziers.cubicbezier import CubicBezier
from beziers.point import Point
from beziers.path import BezierPath

CAD_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/../EmioLabs/assets/data/meshes/legs/'
FIG_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/data/'#+'/data/figs/'
STL_DIRECTORY = os.path.dirname(os.path.realpath(__file__))#+'/data/cad/'

def main():
    
    os.system("clear||cls")
    sCP = [[[0,0],20,90],
          [[50,50],50,-20],
          [[75,75],20,90]]
          
    #debug = {'plot':'','time':'','cad':True}

    l = CenterLine(name='test',
                      crossSection=[10,5],
                      numberOfPoints=4,
                      controlPoints=sCP,
                      origin=[0,-22.5]
                      )
    l.exportSTLGmsh(path="C:/Users/Ahmed Amine Chafik/Softs",debug=True)
    
    #l.plot()
    
class _CurvePoint:
    """
    _summary_
        Python class that represent a Bézier point.
    Args:
        idx (int): An integer index or identifier.
        distanceUnit (str) : the distance unit (mm/cm), default to mm.
        angleUnit (str) : the angle unit (rad/deg), default to deg.
        coordinates (list[float,float]) : point coordinates [x, y].
        x (float) : x coordinate of the point.
        y (float) : y coordinate of the point.
        rightHandleLength (float) : the length of the right handle (tangent), default to 0.
        leftHandleLength (float) : the length of the left handle (tangent), default to None.
        rightHandleAngle (float) : the angle of the right handle (tangent), default to 0.
        leftHandleAngle (float) : the angle of the left handle (tangent), default to None.        
    """    
    
    def __init__(self, idx:int, coordinates:list[float], rightHandleLength:float=0, rightHandleAngle:float=0, leftHandleLength:float | None = None, leftHandleAngle:float | None=None, distanceUnit:str='mm', angleUnit:str='deg') -> None:
        self.idx = idx
        self.distanceUnit = distanceUnit
        self.angleUnit = angleUnit
        self.coordinates = coordinates

        self.rightHandleAngle = rightHandleAngle
        self.rightHandleLength = rightHandleLength
   
        self.leftHandleAngle = leftHandleAngle        
        self.leftHandleLength = leftHandleLength
        
        self.sameHandleAngle : bool
        self.sameHandleLength : bool

        self._angCoeff : float
        self._distCoeff : float

        self.leftPole : list
        self.rightPole : list

        self._initCoeffs()
        self._initCoor()
        self._initFlagLength()
        self._initLength()
        self._initFlagAngle()
        self._initAngle()  
        self._initPoles()

    def _initCoeffs(self,af='',at='rad',df='',dt='mm'):
        """
        _summary_
            Updates conversion coefficent per unit
        """
        unitDict = {'mm' : 1, 'cm' : 10, 'rad' : 1, 'deg' : pi/180}
        if af=='':
            af = self.angleUnit
        if df=='':
            df = self.distanceUnit
        self._angCoeff = unitDict[af]/unitDict[at]
        self._distCoeff = unitDict[df]/unitDict[dt]

    def _initCoor(self):
        """
        _summary_
            Updates point coordinates.
        """
        self.coordinates = [x*self._distCoeff for x in self.coordinates]
        [self.x, self.y] = self.coordinates
    
    def _initFlagLength(self):
        """
        _summary_
            Determins if the handles have the samelength.
        """
        if self.leftHandleLength is None:
            self.sameHandleLength = True        
        elif abs(self.rightHandleLength-self.leftHandleLength) == 0:
            self.sameHandleLength = True
        else:
            self.sameHandleLength = False
    
    def _initFlagAngle(self):
        """
        _summary_
            Determins if the handles have the samelength.
        """
        if self.leftHandleAngle is None:
            self.sameHandleAngle = True
        elif abs(self.rightHandleAngle-self.leftHandleAngle) == pi:
            self.sameHandleAngle = True
        else:
            self.sameHandleAngle = False

    def _initLength(self):
        """
        _summary_
            Updates/initializes both the left and right handles lengths.
        """
        self.rightHandleLength *= self._distCoeff
        if self.sameHandleLength:
            self.leftHandleLength = self.rightHandleLength
        else:
            self.leftHandleLength *= self._distCoeff

    def _initAngle(self):
        """
        _summary_
            Updates/initializes both the left and right handles angles.
        """        
        self.rightHandleAngle *= self._angCoeff
        if self.sameHandleAngle:
            self.leftHandleAngle = self.rightHandleAngle+pi
        else:
            self.leftHandleAngle *= self._angCoeff

    def _initPoles(self):
        """
        _summary_
            Updates/initializes the leftPole and the rightPole coordinates of the Bézier point
        """        
        self.leftPole = [self.x+self.leftHandleLength*cos(self.leftHandleAngle), 
                         self.y+self.leftHandleLength*sin(self.leftHandleAngle)]
        self.rightPole = [self.x+self.rightHandleLength*cos(self.rightHandleAngle), 
                          self.y+self.rightHandleLength*sin(self.rightHandleAngle)]
    
    def update(self,**kwargs) -> None:
        """
        _summary_
            Updates the attributes of a bezier point, using keyword arguments.
        Args:
            idx (int): An integer index or identifier.
            sameHandleAngle (bool): set to True if left and right handles are colinear.
            sameHandleLength (bool): set to True if left and right handles length is equal.
            distanceUnit (str) : the distance unit (mm/cm).
            angleUnit (str) : the angle unit (rad/deg).
            coordinates (list[float,float]) : point coordinates [x, y].
            x (float) : x coordinate of the point.
            y (float) : y coordinate of the point.
            rightHandleLength (float) : the length of the right handle (tangent).
            leftHandleLength (float) : the length of the left handle (tangent).
            rightHandleAngle (float) : the angle of the right handle (tangent)
            leftHandleAngle (float) : the angle of the left handle (tangent)
        Return:
            None
        """
        self._initCoeffs(af='rad',df='mm')
        
        if 'idx' in kwargs:
            self.idx = kwargs.get('idx')
        
        if 'sameHandleAngle' in kwargs:
            self.sameHandleAngle = kwargs['sameHandleAngle']

        if 'sameHandleLength' in kwargs:
            self.sameHandleLength = kwargs['sameHandleLength']
        
        if 'distanceUnit' in kwargs:
            self._initCoeffs(df=kwargs['distanceUnit'])
            self.distanceUnit = kwargs['distanceUnit']
        
        if 'angleUnit' in kwargs:
            self._initCoeffs(af=kwargs['angleUnit'])
            self.angleUnit = kwargs['angleUnit']
 
        if 'coordinates' in kwargs:
            self.coordinates = kwargs['coordinates']

        if 'x' in kwargs:
            self.coordinates[0] = kwargs['x']

        if 'y' in kwargs:
            self.coordinates[1] = kwargs['y']

        if 'rightHandleLength' in kwargs:
            self.rightHandleLength = kwargs['rightHandleLength']
            if 'sameHandleLength' not in kwargs:
                self.sameHandleLength = True

        if 'leftHandleLength' in kwargs:
            self.leftHandleLength = kwargs['leftHandleLength']
            self._initFlagLength()

        if 'rightHandleAngle' in kwargs:
            self.rightHandleAngle = kwargs['rightHandleAngle']
            if 'sameHandleAngle' not in kwargs:
                self.sameHandleAngle = True 

        if 'leftHandleAngle' in kwargs:
            self.leftHandleAngle = kwargs['leftHandleAngle']
            self._initFlagAngle()

        self._initCoor()
        self._initLength()
        self._initAngle()
        self._initPoles()

class CenterLine:
    """
    _summary_
        Python class that represent a center line as a Bézier curve.
        A Bézier curve is defined by curvePoints.
    Args:
        name (str) : the name of the center line.  
        numberOfPoints (int) : the number Bézier point of the centerline, default to -1.
        controlPoints (list[list[list[int|float]|int|float]]) : default to None. 
        startCoordinates (list[float,float]) : the coordinates of the start point, default to [0, 0].
        endCoordinates (list[float,float]) : the coordinates of the end point, default to [75,75].
        crossSection: (list[float,float]) : the dimensions of the cross section, default to [10,5].
        origin (list[float, float] : the origin of the centerline, useful for a translation, default to None        
    """ 
    def __init__(self, name:str, numberOfPoints:int=-1, controlPoints=None, startCoordinates:list =[0, 0],endCoordinates:list=[75,75],crossSection:list=[10,5], origin=None) -> None:
        self.name = name
        self.nPts = numberOfPoints
        self.controlPoints = controlPoints
        self.crossSection = crossSection 
        self.origin = origin
        self.points : list[_CurvePoint] = [] 
        self.isSmooth : bool = True
        self.startCoordinates = startCoordinates
        self.endCoordinates = endCoordinates 
        
        if controlPoints is None:
            self._initControlPoints()
        else:
            self.setCurvePoints(controlPoints)
        
        if  origin is not None:
            self._initOrigin()
        
        self.computePath()

    def computePath(self):
        self.path = self._getBezierpath()

    def _initControlPoints(self):
        """
        _summary_
            Updates/initializes the control points (_curvePoints) of the centerline .
        """ 
        width = (self.endCoordinates[0]-self.startCoordinates[0])
        height = (self.endCoordinates[1]-self.startCoordinates[1])        
        self.points.append(_CurvePoint(0,self.startCoordinates, rightHandleLength=height/2, rightHandleAngle=90))
        for idx in range(1,self.nPts-1):
            coordinates = [(self.endCoordinates[0]-self.startCoordinates[0])*idx/(self.nPts-1),
                           (self.endCoordinates[1]-self.startCoordinates[1])*(self.nPts-1-idx)/(self.nPts-1)]
            self.points.append(_CurvePoint(idx,coordinates,rightHandleLength=width/(self.nPts-1),rightHandleAngle=-45))
        self.points.append(_CurvePoint(self.nPts-1,self.endCoordinates,rightHandleLength=height/2,rightHandleAngle=90))

    def _initOrigin(self):
        """
        _summary_
            Updates/initializes the origin coordinates of the center line.
        """ 
        dx = self.origin[0]-self.points[0].x
        dy = self.origin[1]-self.points[0].y
        for point in self.points:
            point.update(coordinates = [point.x+dx,point.y+dy] )
    
    def setCurvePoints(self,arrayCP:list[list[list[int|float]|int|float]]) -> None:
        """
        _summary_
            Updates/initializes the curve points coordinates, left/right angle and handle length.
        Args:
            arrayCP (list[list[list[int|float]|int|float]]) : array of curve point following this structure [[[x,y],right handle length, right handle angle, left handle length, left handle angle ],..]
        """        
        self.nPts = len(arrayCP)
        for idx, point in enumerate(arrayCP):
            if len(point)>4:
                if abs(point[2]-point[4]) == pi:
                    self.isSmooth = True
                else:
                    self.isSmooth = False
            point.insert(0,idx)
            self.points.append(_CurvePoint(*point,angleUnit='deg'))
        
    def _getBezierpath(self)->BezierPath:
        """
        _summary_
            Generates Bézier path instance.
        Returns:
            path (BezierPath) : instance of class BezierPath.
        """ 
        curves: list[CubicBezier]
        curves=[]
        for idx in range(len(self.points)-1):
            curves.append(CubicBezier(Point(*self.points[idx].coordinates),
                                      Point(*self.points[idx].rightPole),
                                      Point(*self.points[idx+1].leftPole),
                                      Point(*self.points[idx+1].coordinates)))
    
        path = BezierPath.fromSegments(curves)
        path.closed=False
        path = path.removeIrrelevantSegments()

        return path

    def getBeams(self, numberOfBeams:int)->list[list[int|float]]:
        """
        _summary_
            Generates an array of beams coordinates, which the size is an input variable.
        Args:
            numberOfBeams (int) : number of beams.
        Returns:
            beams (list[list[int|float]]) : vector of beams coordinates.
        """ 
        beams = []
        path = self._getBezierpath()

        beamStep = path.length/(numberOfBeams-1)    
        curves = path.asSegments()
        remainingDistance = beamStep #Forcing starting distance to be 0

        for curve in curves:
            numberOfBeamsPerCurve = floor(curve.lengthAtTime(1)/beamStep)+1

            startingDistance = beamStep-remainingDistance
            remainingDistance = (curve.lengthAtTime(1)-startingDistance) % beamStep

            startParameter = startingDistance/curve.lengthAtTime(1)
            stepParameter = beamStep/curve.lengthAtTime(1)
            
            curveParameters=[startParameter+stepParameter*i for i in range(numberOfBeamsPerCurve)] 

            for parameter in curveParameters:
                
                point = curve.pointAtTime(parameter)
                theta = curve.tangentAtTime(parameter).angle-pi/2

                quaternion = Rotation.from_euler('z', -pi/2).inv() * Rotation.from_euler('y', theta) * Rotation.from_euler('x', pi)
                quaternion = quaternion.as_quat()

                beams.append([0,point.y,point.x,*quaternion.astype(list)])

        return beams

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
                gmsh.option.setNumber("General.Verbosity",2) # to debug or 99
            else:
                gmsh.option.setNumber("General.Terminal", 0)
            #gmsh.clear()
            
            #Meshing
            gmsh.option.setNumber("General.AbortOnError",1)
            gmsh.option.setNumber('Mesh.Binary',1)
            gmsh.option.setNumber('Mesh.MaxRetries',0)
            gmsh.option.setNumber('Mesh.AlgorithmSwitchOnFailure',0)
            gmsh.option.setNumber('Mesh.Algorithm', 9) #(1: MeshAdapt, 2: Automatic, 8: Frontal-Delaunay for Quads, 9: Packing of Parallelograms)
            #gmsh.option.setNumber('Mesh.MeshSizeFromCurvatureIsotropic',1)
            gmsh.option.setNumber('Mesh.MeshSizeFromCurvature',30)
            #gmsh.option.setNumber('Mesh.MeshSizeFactor',0.25)

    def _renderMeshGmsh(self, inputExtension:str='', debug=False)->bool:
        """
        _summary_
            Generates a 2D mesh of a 3D model using GMSH, and returns a True flag if successful.
        Args:
            inputExtension (str) : . Default to False.
            debug (bool) : Shows a preview of the mesh using GMSH gui if it is set to True. Default to False.
        Returns:
            success (bool) : True if geometry is meshed.
        """ 
        self._initializeGmsh(debug)
        
        if inputExtension:
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
            if not debug and inputExtension:
                os.remove(self.name+'.'+inputExtension)
            return True
        
    def _getShapeGmsh(self,debug=False):
        curves=self.path.asSegments()
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
        occGeo.synchronize()

    def exportSTLGmsh(self,path=CAD_DIRECTORY,debug=False)->bool:
        os.chdir(path)
        self._getShapeGmsh(debug=debug)
        exportState = self._renderMeshGmsh(debug=debug)
        return exportState
    
    def getTopologyGmsh(self,debug=False):
        self._getShapeGmsh(debug)
        gmsh.model.mesh.generate(2)
        
        _, coord,_ = gmsh.model.mesh.getNodes()
        elementType = gmsh.model.mesh.getElementTypes(dim=2)[0]
        nodesPerElement = gmsh.model.mesh.getElementProperties(elementType)[3]
        nodes, _,_ = gmsh.model.mesh.getNodesByElementType(elementType)

        nodes = [x-1 for x in nodes]
        elements = [nodes[3*idx:3*idx+3] for idx in range(int(len(nodes)/nodesPerElement))]
        positions = [coord[3*idx:3*idx+3] for idx in range(int(len(coord)/nodesPerElement))]

        gmsh.finalize()
    
        return [positions, elements]
        
    def _getShapeCADQuery(self,debug=False):
        curves = self.path.asSegments()
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
            if not self._renderMeshGmsh(self.name,'brep',debug):
                return volume.exportStl(self.name+'.stl')
            else:
                return True
        else:
            if debug:
                volume.exportBrep(self.name+'.brep')
            return False
    
    def exportSTLCadQuery(self,path=CAD_DIRECTORY,debug=False)->bool:
        os.chdir(path)
        curves = self.path.asSegments()
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
        
    def exportSTL(self, path=CAD_DIRECTORY, debug:bool=False)->bool:
        if self.isSmooth:
            return self.exportSTLGmsh(path,debug)
        else:
            return self.exportSTLCadQuery(path,debug)

    def isValid(self)->bool:
        intersection = self.path.getSelfIntersections() # self-intersection Verification

        rightOffsetPath = self.path.offset(self.path.asSegments()[0].normalAtTime(0)*self.crossSection[1]/2)
        rightOffsetPath.closed = False
        rightOffsetIntersection = rightOffsetPath.getSelfIntersections() #pre sweep verification

        leftOffsetPath = self.path.offset(self.path.asSegments()[0].normalAtTime(0)*-self.crossSection[1]/2)
        leftOffsetPath.closed = False
        leftOffsetIntersection = leftOffsetPath.getSelfIntersections() #pre sweep verification

        return (not bool(intersection)) and (not bool(leftOffsetIntersection)) and (not bool(rightOffsetIntersection))

    def plot(self,name=None,title=None,show=True):
        if name is None:
            name=self.name
        if title is None:
            title = name+' centerline'
        _, ax = plt.subplots()
        ax.set_title(label=title)
        ax.set_aspect(aspect="auto")
        ax.grid(True)
        self.path.plot(ax)
        for line in ax.lines: line.set_color("red")
        # plt.rcParams["figure.figsize"] = (15,15)
        if show:
            plt.show()
    
    def exportPlot(self,name=None,title=None,path=FIG_DIRECTORY):
        os.chdir(path)
        if name is None:
            name=self.name
        self.plot(name = name,
                  title = title,
                  show = False)
        plt.savefig(fname=name+'.png',
                    bbox_inches = 'tight',
                    )
        plt.close()

    def getControlPoints(self) -> list[list[list[int|float]|int|float]]:
        leg=[]
        for point in self.points:
            if point.sameHandleLength and point.sameHandleAngle:
                leg.append([point.coordinates,point.rightHandleLength,point.rightHandleAngle*180/pi])

            else:
                leg.append([point.coordinates,point.rightHandleLength,point.rightHandleAngle*180/pi,point.leftHandleLength,point.leftHandleAngle*180/pi])           
        
        return  leg
    
    def setFromOptuna(self,trialParams:dict) -> None:

        l = len(self.name)
        
        lL = [ int(key[l+5:]) for key in trialParams if key[l+3:l+5]=='lL' and key[:l]==self.name]
        lA = [ int(key[l+5:]) for key in trialParams if key[l+3:l+5]=='lA' and key[:l]==self.name]
        
        for key in trialParams.keys():
            if key[:l] == self.name:
                sameAngle=True
                sameLength=True

                if key[l+3:l+4] == 'x':
                    self.points[int(key[l+4:])].update(x = float(trialParams[key]))
                
                if key[l+3:l+4] == 'y':
                    self.points[int(key[l+4:])].update(y = float(trialParams[key]))
                
                if key[l+3:l+5] == 'rL':
                    if int(key[l+5:]) in lL:
                        sameLength = False
                    self.points[int(key[l+5:])].update(rightHandleLength = float(trialParams[key]),
                                                       sameHandleLength = sameLength)
                    
                if key[l+3:l+5] == 'rA':
                    if int(key[l+5:]) in lA:
                        sameAngle = False
                    self.points[int(key[l+5:])].update(rightHandleAngle = float(trialParams[key]),
                                                       angleUnit = 'deg',
                                                       sameHandleAngle = sameAngle)

                if key[l+3:l+5] == 'lL':
                    self.points[int(key[l+5:])].update(leftHandleLength = float(trialParams[key]))

                if key[l+3:l+5] == 'lA':
                    self.points[int(key[l+5:])].update(leftHandleAngle = float(trialParams[key]),
                                                       angleUnit = 'deg') 
    
    def update(self,**kwargs)->None:
        """
            _summary_
                Updates the attributes of a center line class , using keyword arguments.
            Args:
                origin (list[float,float]): the coordinates of the origin of the centerline.
                crossSection (list[float,float]): the dimensions of the cross section to be sweepted along the centerline .
                endCoordinates (list[float,float]): the coordinates of the end point of the center line.
        """
        if 'origin' in kwargs:
            self.origin = kwargs['origin']
            self._initOrigin()
        
        if 'crossSection' in kwargs:
            self.crossSection = kwargs['crossSection']
        if 'endCoordinates' in kwargs:
            self.endCoordinates = kwargs['endCoordinates']
            self._initControlPoints()

        self.computePath()

if __name__=="__main__":
    main()
    