from __future__ import annotations
import os
from math import pi, floor
# from ..tools import checkModule

# checkModule('matplotlib')
# checkModule('scipy')
# checkModule('gmsh')
# checkModule('pyclipper')
# checkModule('cadquery')
# checkModule('beziers',gitLink='https://github.com/simoncozens/beziers.py.git')

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from beziers.cubicbezier import CubicBezier
from beziers.point import Point
from beziers.path import BezierPath
import cadquery as cq

CAD_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/../data/meshes/legs/'
FIG_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/../data/'#+'/data/figs/'
STL_DIRECTORY = os.path.dirname(os.path.realpath(__file__))#+'/data/cad/

if __name__ =='__main__' or __name__ == 'centerLine':
    from curvePoint import CurvePoint
else:
    from shape.curvePoint import CurvePoint

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
    def __init__(self, 
                 name:str, 
                 numberOfPoints:int=-1, 
                 controlPoints=None, 
                 startCoordinates:list =[0, 0],
                 endCoordinates:list=[75,75],
                 origin=None) -> None:
        
        self.name = name
        self.nPts = numberOfPoints
        self.controlPoints = controlPoints
        self.origin = origin
        self.points : list[CurvePoint] = [] 
        self.isSmooth : bool = True
        self.startCoordinates = startCoordinates
        self.endCoordinates = endCoordinates 
        
        if controlPoints is None:
            self._initControlPoints()
        else:
            self.setCurvePoints(controlPoints)
        
        if  origin is not None:
            self._initOrigin()
        
        self.startCoordinates = self.points[0].coordinates
        self.endCoordinates = self.points[-1].coordinates
        self.computePath()

    def computePath(self):
        self.path = self._getBezierpath()

    def _initControlPoints(self):
        """
        _summary_
            Updates/initializes the control points (CurvePoints) of the centerline .
        """ 
        width = (self.endCoordinates[0]-self.startCoordinates[0])
        height = (self.endCoordinates[1]-self.startCoordinates[1])        
        self.points.append(CurvePoint(0,self.startCoordinates, rightHandleLength=height/2, rightHandleAngle=90))
        for idx in range(1,self.nPts-1):
            coordinates = [(self.endCoordinates[0]-self.startCoordinates[0])*idx/(self.nPts-1),
                           (self.endCoordinates[1]-self.startCoordinates[1])*(self.nPts-1-idx)/(self.nPts-1)]
            self.points.append(CurvePoint(idx,coordinates,rightHandleLength=width/(self.nPts-1),rightHandleAngle=-45))
        self.points.append(CurvePoint(self.nPts-1,self.endCoordinates,rightHandleLength=height/2,rightHandleAngle=90))

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
        self.points = []
        for idx, point in enumerate(arrayCP):
            if len(point)>4:
                if abs(point[2]-point[4]) == pi:
                    self.isSmooth = True
                else:
                    self.isSmooth = False
            point.insert(0,idx)
            self.points.append(CurvePoint(*point,angleUnit='deg'))
        
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

    def getBeams(self, numberOfBeams:int, regular=True)->list[list[int|float]]:
        """
        _summary_
            Generates an array of beams coordinates, which the size is an input variable.
        Args:
            numberOfBeams (int) : number of beams.
            regular (bool) : equi-distant distribution of beams
        Returns:
            beams (list[list[int|float]]) : vector of beams coordinates.
        """ 
        beams = []
        path = self._getBezierpath()
        curves = path.asSegments()
        
        if regular:
            parameters = path.regularSampleTValue(numberOfBeams)
        else:
            parameters = [i/(numberOfBeams-1) for i in range(numberOfBeams)]

        for parameter in parameters:
            if parameter == 1:
                point = curves[-1].pointAtTime(1)
                theta = curves[-1].tangentAtTime(1).angle-pi/2
            else:
                parameter *= len(curves)
                curve = curves[int(floor(parameter))]
                point = curve.pointAtTime(parameter - floor(parameter))
                theta = curve.tangentAtTime(parameter - floor(parameter)).angle-pi/2

            quaternion = Rotation.from_euler('z', -pi/2).inv() * Rotation.from_euler('y', theta) * Rotation.from_euler('x', pi)
            quaternion = quaternion.as_quat()

            beams.append([0,point.y,point.x,*quaternion.astype(list)])

        return beams

    def getBeamsDep(self, numberOfBeams:int)->list[list[int|float]]:
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

    def isValid(self)->bool:
        intersection = self.path.getSelfIntersections() # self-intersection Verification
        return not bool(intersection)

    def plot(self,name=None,title=None,show=True):
        if name is None:
            name=self.name
        if title is None:
            title = name+' centerline'
        _, ax = plt.subplots()
        ax.set_title(label=title)
        ax.set_aspect(aspect="auto")
        ax.grid(True, alpha=0.3)
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

    def _nodesSvg(self, close=False):
        nodes = self.path.asNodelist()
        if not nodes:
            return ""
        parts = [f"M{nodes[0].x},{nodes[0].y}"]
        parts += [f"L{node.x},{node.y}" for node in nodes[1:]]
        if close:
            parts.append("Z")
        return " ".join(parts)
    
    def exportSvg(self,name=None,withNodes=True,path=CAD_DIRECTORY):
        os.chdir(path)
        if name is None:
            name=self.name

        path_data = self.path.asSVGPath()
        if withNodes:
            bezier_data = self._nodesSvg()
        else:
            bezier_data = ''
        
        width = self.path.bounds().width
        height = self.path.bounds().height

        svg_content = f'''<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">
        <path d="{path_data}" stroke="black" fill="none" stroke-width="2" transform="rotate(-180 100 100) translate(-300 150) scale(3.8) "/>
        <path d="{bezier_data}" stroke="black" fill="none" stroke-width="2" transform="rotate(-180 100 100) translate(-300 150) scale(3.8) "/>
        </svg>'''
        with open(name+'.svg', 'w',encoding='utf-8') as f:
            f.write(svg_content)

    def getControlPoints(self) -> list[list[list[int|float]|int|float]]:
        leg=[]
        for point in self.points:
            if point.sameHandleLength and point.sameHandleAngle:
                leg.append([point.coordinates,point.rightHandleLength,point.rightHandleAngle*180/pi])

            else:
                leg.append([point.coordinates,point.rightHandleLength,point.rightHandleAngle*180/pi,point.leftHandleLength,point.leftHandleAngle*180/pi])           
        
        return  leg
    
    def fromTrialParams(self,trialParams:dict) -> None:

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
    
    def plotBeams(self,nBeams:int,show=True,regular=True)->None:
        beams = self.getBeams(nBeams,regular)
        x = [beams[i][2] for i in range(len(beams))]
        y = [beams[i][1] for i in range(len(beams))]

        # Create the plot
        plt.plot(x, y, marker='*')
        plt.title(f'{self.name} - {nBeams} beams')
        plt.grid(True, alpha=0.3)
        if show:
            plt.show()

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
        if 'endCoordinates' in kwargs:
            self.endCoordinates = kwargs['endCoordinates']
            self._initControlPoints()
        if 'params' in kwargs:
            self.fromTrialParams(trialParams=kwargs['params'])
        if 'controlPoint' in kwargs:
            self.setCurvePoints(arrayCP=kwargs['controlPoint'])

        self.endCoordinates = self.points[-1].coordinates
        self.startCoordinates = self.points[0].coordinates
        self.computePath()
    
    def getTFromLength(self,
                        target_length, 
                        derivative_at_t=None, 
                        initial_guess=None,
                        max_iterations=20, 
                        tolerance=1e-6):
        """Newton-Raphson refinement for t from target arc length."""
        
        # Auto-compute initial guess if needed
        if initial_guess is None:
            total_length = self.path.length
            if total_length == 0:
                return 0.0
            initial_guess = min(target_length / total_length, 1)
        
        t = initial_guess
        
        for iteration in range(max_iterations):
            current_length = self.path.lengthAtTime(t)
            error = current_length - target_length
            
            # Converged?
            if abs(error) < tolerance:
                return t
            
            # Get speed
            if derivative_at_t is not None:
                speed = derivative_at_t(t)
            else:
                dt = 1e-8
                speed = (self.path.lengthAtTime(t + dt) - self.path.lengthAtTime(t - dt)) / (2 * dt)
            
            # Avoid division by zero
            if abs(speed) < 1e-12:
                return t
            
            # Update
            t_new = t - error / speed
            t_new = max(0, min(1, t))  # Clamp to [0, 1]
            
            # Check if we're making progress
            if abs(t_new - t) < 1e-10:
                return t  # Stuck, return current
            
            t = t_new
        
        return t

    def exportDxf(self,path=CAD_DIRECTORY, name=None):
        os.chdir(path)
        if name is None:
            name = self.name
        curves = self.path.asSegments()
        wire = cq.Workplane("YZ")
        for curve in curves:
            wire = wire.bezier([(curve.start.y,curve.start.x),
                                (curve.points[1].y,curve.points[1].x),
                                (curve.points[2].y,curve.points[2].x),
                                (curve.end.y,curve.end.x)])
        wire.export(name+'.dxf')
        
if __name__=="__main__":    
    os.system("clear||cls")
    # sCP = [[[0,0],20,90],
    #       [[75/2,75/2],40,-45],
    #       [[75,75],20,90]]
    sCP = [[[20,-30],0.1,90],
        [[20,-15],20,90,0.1,270],
        [[20,20],35,-20],
        [[50,60-15],0.1,90,20,270],
        [[50,60],0.1,90]]
    
    #debug = {'plot':'','time':'','cad':True}

    l = CenterLine(name='test leg',
                   numberOfPoints=5,
                   #controlPoints=sCP
                   )
    l.exportDxf()