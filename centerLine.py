"""_summary_

Returns:
    _type_: beam vector
"""
from __future__ import annotations
from importlib import import_module

import os
import sys
from subprocess import run,DEVNULL,PIPE

def checkModule(module:str,libraryName:str='',gitLink:str=''):
    try:
        import_module(module)
    except ImportError:
        import pip
        if not libraryName and not gitLink:
            pip.main(['install',module])
        elif not libraryName:
            pip.main(['install','git+'+gitLink])
        elif not gitLink:
            pip.main(['install',libraryName])
    
checkModule('beziers',gitLink='git+https://github.com/simoncozens/beziers.py/tree/main/src/beziers')
checkModule('gmsh')
checkModule('solid2',libraryName='solidpython2')
checkModule('matplotlib')
checkModule('cadquery')
#checkModule('numpy')
checkModule('scipy')
checkModule('pyclipper')

from math import pi,sin,cos, floor
import gmsh

from scipy.spatial.transform import Rotation

from beziers.point import Point
from beziers.cubicbezier import CubicBezier
from beziers.path import BezierPath

from solid2.extensions.bosl2 import rect, bezpath_curve, path_sweep
from solid2.core import scad_render_to_file

import cadquery as cq

import random
import string


match os.name:
    case 'nt':
        OPENSCAD_DIRECTORY = 'C:/Program Files/OpenSCAD/' 
    case 'posix':
        OPENSCAD_DIRECTORY = '/usr/bin/openscad'

CAD_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/../EmioLabs/assets/data/meshes/legs/'
FIG_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/data/figs/'
STL_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/data/cad/'

def main():
    
    os.system("clear||cls")

    legName = ''.join(random.choices(string.ascii_letters + string.digits, k=8))
    legCrossSection = [10,5]
    pulley =[22.5,0]
    numberOfBeams = 10
    
    # leg=[[[x,y],rightHandleLength,rightHandleAngle,leftHandleLength,leftHandleAngle],.....]
    # legCenterLine=[[[0,0],200,90],
    #                  [[100,100],100,-45],
    #                  [[250,200],200,90]]
    # legCenterLine = [[[0,0],20,90],
    #                  [[40,40],50,-45,20,40],
    #                  [[75,75],20,90]]
    # legCenterLine = [[[0,0],20,90],
    #                  [[50,50],50,-45],
    #                  [[75,75],20,90]]
    data = {'sPt.rL0': 64.92305858998935, 'sPt.rL1': 68.11944020329142, 'sPt.x1': 39.14679130942243, 'sPt.y1': 32.58772944462524, 'sPt.rA1': 9.501390743726446, 'sPt.rL2': 69.51926420437991, 'lPt.rL0': 29.29504519912233, 'lPt.rL1': 67.4816563447648, 'lPt.x1': 46.475222143426365, 'lPt.y1': 42.21131726554306, 'lPt.rA1': -8.195176261888683, 'lPt.rL2': 65.28619633249619}
    
    l = legCenterLine('s',3)
    l.updateFromOptuna(data)
    debug = {'plot':'pre','time':'all','cad':True}
    beams=generateLeg(legName,
                      legCrossSection,
                      l.asArray(),
                      numberOfBeams,
                      pulley,
                      debug)

def generateLeg(legName:str, legCrossSection:list[int|float], legCenterLine:list[list[list[int|float]|int|float]], numberOfBeams:int, origin:list[float|int]=[0,0], debug:dict = {'plot':'','cad':False,'time':''})->list[list[int|float]]:
    """Generates beams coordinates of a composite cubic bézier curve, given a leg vector and number of beams

    Args:
        leg (list[list[list[int  |  float]]]): List describing the differents properties of the interpolation points.
        numberOfBeams (int): The number of beams.

    Returns:
        beams (list[list[float]|float]): Coordinates composite cubic bézier curve expressed in [[x y z qx qy qz qw],...].
    """  

    import time

    start=time.perf_counter()
    if origin and origin != [0,0]:
        legCenterLine = changeLegOrigin(legCenterLine,origin)
    points, smooth = generateControlPoints(legCenterLine)
    path = generateBezierPath(points)
    
    if debug['plot']=='pre':
        plotCurvesMatplotlib(legName, path)
    
    if isPathNotValid(path):
         return
    
    beams = generateBeams(path, numberOfBeams)
    beamTime = time.perf_counter()-start

    if not generateSTL(legName, path, legCrossSection, not smooth, debug['cad']):
        return
    
    stlTime = time.perf_counter() - beamTime
    if debug['plot']=='post': 
        plotCurvesMatplotlib(legName, path)
    if debug['time'] =='beam' or debug['time'] =='all':
        print("Beams generation : {:0.6f} s, {:0.2f} %".format(beamTime, beamTime/(stlTime+beamTime)*100))
    if debug['time'] =='cad' or debug['time'] =='all':
        print("STL generation   : {:0.6f} s, {:0.2f} %".format(stlTime, stlTime/(stlTime+beamTime)*100))
    return beams

def changeLegOrigin(legCenterLine:list[list[list[int|float]|int|float]], origin:list[int|float])->list[list[list[int|float]|int|float]]:
    dx = origin[0]-legCenterLine[0][0][0]
    dy = origin[1]-legCenterLine[0][0][1]
    
    for point in legCenterLine:
        point[0]=[point[0][0]+dx, point[0][1]+dy]
    return legCenterLine

def generateControlPoints(legCenterLine:list[list[list[int|float]|int|float]])->tuple[list[curvePoint]|bool]:
    """Generates list of interpolation points from leg vector

    Args:
        leg (list[list[list[int  |  float]]]): List describing the differents properties of the interpolation points.

    Returns:
        points (list[curvePoint]): List of interpolated points
    """    
    points:list[curvePoint]
    points=[]
    smooth = True
    for idx, point in enumerate(legCenterLine):
        
        # if len(point)==3:
        #     point.extend([point[1],point[2]+180])
        # else:
        #     smooth =False

        point.insert(0,idx)
        points.append(curvePoint(*point,angleUnit='deg'))

    return points, smooth

def generateBezierPath(curvePoints:list[curvePoint])->BezierPath:
    """Generates a composite cubic Bézier curve given a set of curve points

    Args:
        curvePoints (list[curvePoint]): List of interpolation points

    Returns:
        curves (list[Part.BezierCurve]): Composite cubic Bézier curve, List of independent cubic Bézier curves
    """    
    curves: list[CubicBezier]
    curves = []
        
    for idx in range(len(curvePoints)-1):
        curves.append(CubicBezier(Point(*curvePoints[idx].coordinates),
                                  Point(*curvePoints[idx].rightPole),
                                  Point(*curvePoints[idx+1].leftPole),
                                  Point(*curvePoints[idx+1].coordinates)))
    
    path = BezierPath.fromSegments(curves)
    path.closed=False
    path = path.removeIrrelevantSegments()

    return path

def generateBeams(path:BezierPath, numberOfBeams:int)->list[list[int|float]]:
    """Generates beams coordinates of a composite cubic bézier curve, given a set of cubic Bézier curves and number of beams

    Args:
        curves (list[CubicBezier]): List of cubic bezier curves created using Beziers.py Library.
        numberOfBeams (int):The number of beams.

    Returns:
        beams (list[list[int|float]]): Coordinates composite cubic bézier curve expressed in [[x y z qx qy qz qw],...]
    """    
    beams = []
    
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
    
def generateSTL(legName:str, sweepPath:BezierPath, legCrossSection:list[int|float], smooth:bool, debug:bool=False)->bool:
    os.chdir(CAD_DIRECTORY)
    if smooth:
        return generateSTLGmsh(legName, sweepPath, legCrossSection, debug)
    else:
        return generateSTLCadQuery(legName, sweepPath, legCrossSection, debug)
         
        #generateShapeOpenScad(legName, sweepPath, legCrossSection, debug)

def generateShapeOpenScad(legName:str, sweepPath:BezierPath, legCrossSection:list[int|float], debug:bool=False):              
    crossSection = rect(legCrossSection)
    sweepPath = [[0, point.y,point.x] for point in sweepPath.asNodelist()]
    volume = path_sweep(crossSection,bezpath_curve(sweepPath))   
    scad_render_to_file(volume,legName+'.scad')
    command = OPENSCAD_DIRECTORY + 'openscad --export-format=binstl -o ' + legName + '.stl ' + legName + '.scad'
    test = run(command,stdout=PIPE,stderr=DEVNULL)
    print(test)
    if not debug:
        os.remove(legName+'.scad')

def generateSTLGmsh(legName:str, sweepPath:BezierPath, legCrossSection:list[int|float], debug:bool=False)->bool:

    curves=sweepPath.asSegments()
    initilizeGmsh(debug)
    occGeo = gmsh.model.occ() #OpenCascade kernel
    corner = [-legCrossSection[0]/2, -legCrossSection[1]/2+curves[0].start.y, curves[0].start.x]
    crossSection= [(2,occGeo.addRectangle(*corner,*legCrossSection))]
    occGeo.rotate(crossSection,0,curves[0].start.y,curves[0].start.x,1,0,0,angle=pi/2)
    bezier = [(1,occGeo.addBezier([occGeo.addPoint(0, curve.start.y, curve.start.x),
                                   occGeo.addPoint(0, curve.points[1].y, curve.points[1].x),
                                   occGeo.addPoint(0, curve.points[2].y, curve.points[2].x),
                                   occGeo.addPoint(0, curve.end.y, curve.end.x)])
                                   ) for curve in curves]
    fusedBezier,_ = occGeo.fuse(bezier,bezier)
    _,fusedBezierTag = zip(*fusedBezier)
    
    if len(fusedBezierTag)>1:
        return False
    
    wireTag = occGeo.addWire(list(fusedBezierTag))
    occGeo.addPipe(crossSection,wireTag)
    occGeo.synchronize()
    return renderMeshGmsh(legName,debug=debug)

def generateSTLCadQuery(legName:str, sweepPath:BezierPath, legCrossSection:list[int|float], debug=False)->bool:
    curves = sweepPath.asSegments()
    start = curves[0].start
    crossSection = cq.Workplane("XZ",origin=(0,start.y,start.x)).rect(*legCrossSection)
    wire = cq.Workplane("YZ")
    for curve in curves:
        wire = wire.bezier([(curve.start.y,curve.start.x),
                            (curve.points[1].y,curve.points[1].x),
                            (curve.points[2].y,curve.points[2].x),
                            (curve.end.y,curve.end.x)])
    volume = crossSection.sweep(wire,multisection=True,transition='round').val()
    
    if volume.isValid() and not volume.isNull():
        volume.exportBrep(legName+'.brep')
        if not renderMeshGmsh(legName,'brep',debug):
            return volume.exportStl(legName+'.stl')
        else:
            return True
    else:
        if debug:
            volume.exportBrep(legName+'.brep')
        return False

def initilizeGmsh(debug:bool=False):
    if not gmsh.isInitialized():
        gmsh.initialize()

        if debug:
            gmsh.option.setNumber("General.Terminal", 1) #to capture every terminal event
            gmsh.option.setNumber("General.Verbosity",2) # to debug or 99
        else:
            gmsh.option.setNumber("General.Terminal", 0)
        
        #gmsh.clear()
        gmsh.option.setNumber("General.AbortOnError",1)
        gmsh.option.setNumber('Mesh.Binary',1)
        gmsh.option.setNumber('Mesh.MaxRetries',0)
        gmsh.option.setNumber('Mesh.AlgorithmSwitchOnFailure',0)
        gmsh.option.setNumber('Mesh.Algorithm', 8) #(1: MeshAdapt, 2: Automatic, 8: Frontal-Delaunay for Quads, 9: Packing of Parallelograms)

        #gmsh.option.setNumber('Mesh.MeshSizeFromCurvatureIsotropic',1)
        #gmsh.option.setNumber('Mesh.MeshSizeFromCurvature',30)
        gmsh.option.setNumber('Mesh.MeshSizeFactor',0.25)

def renderMeshGmsh(legName:str, inputExtension:str='', debug=False)->bool:

    initilizeGmsh(debug)
    
    if inputExtension:
        gmsh.open(legName+'.'+inputExtension)
    gmsh.model.mesh.generate(2)
    if debug:
        gmsh.fltk.run() # to view the mesh
    
    if gmsh.logger.getLastError():
        gmsh.finalize()
        return False
    else:
        gmsh.write(legName+'.stl')
        gmsh.finalize()
        if not debug and inputExtension:
            os.remove(legName+'.'+inputExtension)
        return True

def isPathNotValid(path:BezierPath)->bool:
    
    intersection = path.getSelfIntersections() # self-intersection Verification
    
    rightOffsetPath = path.offset(path.asSegments()[0].normalAtTime(0)*2.5)
    rightOffsetPath.closed = False
    rightOffsetIntersection = rightOffsetPath.getSelfIntersections() #pre sweep verification

    leftOffsetPath = path.offset(path.asSegments()[0].normalAtTime(0)*-2.5)
    leftOffsetPath.closed = False
    leftOffsetIntersection = leftOffsetPath.getSelfIntersections() #pre sweep verification
    
    return bool(intersection) or bool(leftOffsetIntersection) or bool(rightOffsetIntersection)

def plotCurvesMatplotlib(legName:str, path:BezierPath):
    import matplotlib.pyplot as plt
    _, ax = plt.subplots()
    plt.title(legName)
    ax.set_aspect(aspect="auto")
    ax.grid(True)
    path.plot(ax)
    [line.set_color("red") for line in ax.lines]
    plt.savefig(FIG_DIRECTORY+legName+'.png')
    plt.show()


class curvePoint:
    global units
    units = {'mm' : 0.1, 'cm' : 1, 'rad' : 1, 'deg' : pi/180}
    def __init__(self, idx:int, coordinates:list[float], rightHandleLength:float=0, rightHandleAngle:float=0, leftHandleLength:float=0, leftHandleAngle:float=370, distanceUnit:str='cm', angleUnit:str='deg') -> None:
        
        self.idx = idx
        self.distanceUnit = distanceUnit
        self.angleUnit = angleUnit
        self.coordinates = [x*units[distanceUnit] for x in coordinates]
        [self.x, self.y] = self.coordinates
        
        self._setHandlesLengths(rightHandleLength,leftHandleLength)
        self._setHandlesAngles(rightHandleAngle,leftHandleAngle)
        self._setPoles()

    def _setHandlesLengths(self,rightHandleLength,leftHandleLength,distanceUnit='cm'):
        self.rightHandleLength = rightHandleLength*units[distanceUnit]
        self.leftHandleLength = leftHandleLength*units[distanceUnit]
        
        if leftHandleLength == 0:
            self.leftHandleLength = rightHandleLength*units[distanceUnit] 
        
        if (abs(self.leftHandleLength-self.rightHandleLength) == 0):
            self.sameHandleLength = True
        else:
            self.sameHandleLength = False   

    def _setHandlesAngles(self,rightHandleAngle,leftHandleAngle,angleUnit='deg'):
        self.rightHandleAngle = rightHandleAngle*units[angleUnit]
        self.leftHandleAngle = leftHandleAngle*units[angleUnit]

        if leftHandleAngle == 370:
            self.leftHandleAngle = rightHandleAngle*units[angleUnit]+pi
        
        if (abs(self.leftHandleAngle-self.rightHandleAngle) == pi):
            self.sameHandleAngle = True
        else:
            self.sameHandleAngle = False   

    def _setPoles(self):
        self.leftPole = [self.x+self.leftHandleLength*cos(self.leftHandleAngle), self.y+self.leftHandleLength*sin(self.leftHandleAngle)]
        self.rightPole = [self.x+self.rightHandleLength*cos(self.rightHandleAngle), self.y+self.rightHandleLength*sin(self.rightHandleAngle)]

    def update(self,**kwargs) -> None:
        """_summary_
        Args:
            idx (int): An integer index or identifier.
            distanceUnit (str): The unit of distance (e.g., 'meters', 'kilometers').
            angleUnit (str): The unit of angle (e.g., 'degrees', 'radians').
            coordinates (list[float]): A tuple or list representing coordinates (x, y).
            translation (list[float]): A tuple or list for translation values.
            rotation (float): A float value for rotation.
            handleLength (float): The length of the handle.
        Return:
            None
        """
        if 'idx' in kwargs:
            self.idx = kwargs['idx']
        
        if 'distanceUnit' in kwargs:
            self.distanceUnit = kwargs['distanceUnit']
        
        if 'angleUnit' in kwargs:
            self.angleUnit = kwargs['angleUnit']
        
        if 'coordinates' in kwargs:
            self.coordinates = [x*units[self.distanceUnit] for x in kwargs['coordinates']]
            [self.x, self.y] = self.coordinates
            self._setPoles()
        
        if 'x' in kwargs:
            self.x = kwargs['x']*units[self.distanceUnit]
            self.coordinates[0] = self.x
            self._setPoles()

        if 'y' in kwargs:
            self.y = kwargs['y']*units[self.distanceUnit]
            self.coordinates[1] = self.y
            self._setPoles()

        if 'rightHandleLength' in kwargs:
            self.rightHandleLength = kwargs['rightHandleLength']*units[self.distanceUnit]
            if 'sameHandleLength' in kwargs:
                self.sameHandleLength = kwargs['sameHandleLength']
                if self.sameHandleLength:
                    self.leftHandleLength = kwargs['rightHandleLength']*units[self.distanceUnit]
            if abs(self.rightHandleLength-self.leftHandleLength) == 0:
                self.sameHandleLength=True
            else:
                self.sameHandleLength=False  


        if 'rightHandleAngle' in kwargs:
            self.rightHandleAngle = kwargs['rightHandleAngle']*units[self.angleUnit]
            if 'sameHandleAngle' in kwargs:
                self.sameHandleAngle = kwargs['sameHandleAngle']
                if self.sameHandleAngle:
                    self.leftHandleAngle = kwargs['rightHandleAngle']*units[self.angleUnit]+pi
            if abs(self.rightHandleAngle-self.leftHandleAngle) == pi:
                self.sameHandleAngle=True
            else:
                self.sameHandleAngle=False  

        if 'leftHandleLength' in kwargs:
            self.leftHandleLength = kwargs['leftHandleLength']*units[self.distanceUnit]
            if abs(self.rightHandleLength-self.leftHandleLength) == 0:
                self.sameHandleLength=True
            else:
                self.sameHandleLength=False  

        if 'leftHandleAngle' in kwargs:
            self.leftHandleAngle = kwargs['leftHandleAngle']*units[self.angleUnit]
            if abs(self.rightHandleAngle-self.leftHandleAngle) == pi:
                self.sameHandleAngle=True
            else:
                self.sameHandleAngle=False  
                
class legCenterLine:

    def __init__(self, legInitials:str,numberOfPoints:int=3, startCoordinates:list[float]=[0,0], endCoordinates:list[float]=[75,75]) -> None:
        self.isSmooth = True
        self.numberOfPoints = numberOfPoints
        self.legInitials = legInitials
        self.point : list[curvePoint]
        self.point=[]    

        self.point.append(curvePoint(0,startCoordinates,rightHandleAngle=90))
        for idx in range(1,numberOfPoints-1):
            coordinates = [(endCoordinates[0]-startCoordinates[0])/(idx+1),
                           (endCoordinates[1]-startCoordinates[1])/(idx+1)]
            self.point.append(curvePoint(idx,coordinates))
        self.point.append(curvePoint(numberOfPoints-1,endCoordinates,rightHandleAngle=90))

    def asArray(self) -> list[list[list[int|float]|int|float]]:
        leg=[]
        for point in self.point:
            if point.sameHandleLength and point.sameHandleAngle:
                leg.append([point.coordinates,point.rightHandleLength,point.rightHandleAngle*180/pi])

            else:
                leg.append([point.coordinates,point.rightHandleLength,point.rightHandleAngle*180/pi,point.leftHandleLength,point.leftHandleAngle*180/pi])           
        
        return  leg
    
    def fromArray(self,array:list[list[list[int|float]|int|float]]) -> None:
        self.numberOfPoints = len(array)
        self.point.clear()

        for idx, point in enumerate(array):
            point.insert(0,idx)
            self.point.append(curvePoint(*point,angleUnit='deg'))

    def updateFromOptuna(self,results:dict) -> None:

        l = len(self.legInitials)
        
        lL = [ int(key[l+5:]) for key in results if key[l+3:l+5]=='lL' and key[:l]==self.legInitials]
        lA = [ int(key[l+5:]) for key in results if key[l+3:l+5]=='lA' and key[:l]==self.legInitials]
        
        for key in results.keys():
            if key[:l] == self.legInitials:
                sameAngle=True
                sameLength=True

                if key[l+3:l+4]=='x':
                    self.point[int(key[l+4:])].update(x=float(results[key]))
                
                if key[l+3:l+4]=='y':
                    self.point[int(key[l+4:])].update(y=float(results[key]))
                
                if key[l+3:l+5]=='rL':
                    if int(key[l+5:]) in lL:
                        sameLength=False
                    self.point[int(key[l+5:])].update(rightHandleLength=float(results[key]),sameHandleLength=sameLength)
                    
                if key[l+3:l+5]=='rA':
                    if int(key[l+5:]) in lA:
                        sameAngle=False
                    self.point[int(key[l+5:])].update(rightHandleAngle=float(results[key]),angleUnit='deg',sameHandleAngle=sameAngle)

                if key[l+3:l+5]=='lL':
                    self.point[int(key[l+5:])].update(leftHandleLength=float(results[key]))

                if key[l+3:l+5]=='lA':
                    self.point[int(key[l+5:])].update(leftHandleAngle=float(results[key]),angleUnit='deg') 

if __name__=="__main__":
    main()
    