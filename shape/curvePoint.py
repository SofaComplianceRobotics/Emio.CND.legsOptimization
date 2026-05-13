from __future__ import annotations
from math import pi,sin,cos

class CurvePoint:
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
    
    def __init__(self, 
                 idx:int,
                 coordinates:list[float],
                 rightHandleLength:float = 0,
                 rightHandleAngle:float = 0,
                 leftHandleLength:float | None = None,
                 leftHandleAngle:float | None = None, 
                 distanceUnit:str='mm', 
                 angleUnit:str='deg') -> None:
        
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