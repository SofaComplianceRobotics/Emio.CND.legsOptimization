from math import sin, cos, pi

# EmioLabs libraries
from parts.emio import Emio
from parts.gripper import Gripper
from utils import getColorFromFilename
from utils.topology import applyRotation, applyTranslation,getIndicesInBox
from splib3.numerics import to_radians


# Sofa Library
import Sofa

# Sam Libraries
from sparts.samDynamixelMotors import MotorController
from sparts.samGUI import SamGUI
from sparts.samLeg import SamLeg
from sparts.samCenterPart import SamCenterPart
from sparts.samMotor import SamMotor

class Sam(Emio) :
    
    prefabData = Emio.prefabData
    prefabData.append({'name': 'mode', 'type': 'string', 'help': '', 'default': 'trial'})
    
    def __init__(self, *args, **kwargs,):
        Sofa.Prefab.__init__(self, *args, **kwargs)
        self._centerPartClass = kwargs.get('centerPartClass',SamCenterPart)
        optCenterPart = kwargs['centerpart']
        optLeg = kwargs['legs']

        self.addData(name="nbLegs", type="int", value=6) #value=4
        centerPartPositions = []
        # Add motors and legs
        self.motors = []
        self.legs = []

        if 'legs' in kwargs:
            n = len(kwargs['legs'])
            distances = [optLeg[i].motorDistanceToCenter for i in range(n)]
            legsPositionOnMotor = ['clockwiseup']*n 
            legsMassDensity = [optLeg[i].density for i in range(n)]
            legsYoungModulus = [optLeg[i].youngModulus for i in range(n)]
            legsPoissonRatio = [optLeg[i].poissonRatio for i in range(n)]
            legsPositions = [optLeg[i].beams for i in range(n)]
            legsMeshData = [optLeg[i].getTopologyGmsh() for i in range(n)]
            legsCollisionMeshData =  [optLeg[i].getCollisionTopologyGmsh(optCenterPart) if i%2!=0 else [None,None] for i in range(n)]
            legsModel = ['beam']*n
            legsName = [optLeg[i].name for i in range(n)]
        else:
            distances = self.motorsDistanceToCenter.value
            legsPositionOnMotor = self.legsPositionOnMotor.value
            legsModel = self.legsModel.value
            legsMassDensity = self.legsMassDensity.value
            legsYoungModulus = self.legsYoungModulus.value
            legsPoissonRatio = self.legsPoissonRatio.value
            legsName = self.legsName.value
            legsPositions = kwargs.get('legsPositions',[None])
            legsMeshData = kwargs.get('legsMeshData',[[None,None]])
        
        if 'centerpart' in kwargs:
            centerPartName = optCenterPart.name
            centerPartDensity = optCenterPart.density
            centerPartpoissonRatio = optCenterPart.poissonRatio
            centerPartYoungModulus = optCenterPart.youngModulus
            centerPartCollisionMesh = optCenterPart.getCollisionTopology()
            optCenterPart.exportStl()
            optCenterPart.exportJson()

        else:
            centerPartName = self.centerPartName.value
            centerPartDensity = self.centerPartMassDensity.value
            centerPartpoissonRatio = self.centerPartPoissonRatio.value
            centerPartYoungModulus = self.centerPartYoungModulus.value
            centerPartCollisionMesh = [None,None]
        
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

            motor = SamMotor(name="Motor" + str(i),
                             leg = optLeg[i], 
                             translation=translation, 
                             rotation=rotation,
                             tempvisurotation=[[-90, 90, -90, 90, 90, 90][i], 180, 0], 
                             color=[0., 0., 0., 0.],
                             mode = self.mode.value)

            self.addChild(motor)
            self.motors.append(motor)

            rotation = [0, [180, 240, 300, 0, 60, 120][i], 0]
            radius = radius + 20
            translation = [radius * sin(angle), 30, radius * cos(angle)]

            if (i < len(legsName) and legsName[i] == "None"):
                self.legs.append(None)
            else:
                # Leg

                leg = SamLeg(name = "Leg" + str(i),
                             legName = legsName[i] if i < len(legsName) else legsName[0],
                             positionOnMotor = legsPositionOnMotor[i] if i < len(legsPositionOnMotor) else legsPositionOnMotor[0],
                             model = legsModel[i] if i < len(legsModel) else legsModel[0],
                             translation = translation, 
                             rotation = rotation,
                             massDensity = legsMassDensity[i] if i < len(legsMassDensity) else legsMassDensity[0],
                             poissonRatio = legsPoissonRatio[i] if i < len(legsPoissonRatio) else legsPoissonRatio[0],
                             youngModulus = legsYoungModulus[i] if i < len(legsYoungModulus) else legsYoungModulus[0],
                             positions = legsPositions[i] if i < len(legsPositions) else legsPositions[0],
                             nodes = legsMeshData[i][0] if i < len(legsMeshData) else legsMeshData[0][0],
                             triangles = legsMeshData[i][1] if i < len(legsMeshData) else legsMeshData[0][1],
                             collisionNodes = legsCollisionMeshData[i][0] if i < len(legsCollisionMeshData) else legsCollisionMeshData[0][0],
                             collisionTriangles = legsCollisionMeshData[i][1] if i < len(legsCollisionMeshData) else legsCollisionMeshData[0][1],
                             mode = self.mode.value)

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
            self.centerpart = SamCenterPart(name = "CenterPart", #self._centerPartClass
                                            positions = centerPartPositions,
                                            partName = centerPartName,
                                            model = self.centerPartModel.value,
                                            massDensity = centerPartDensity,
                                            poissonRatio = centerPartpoissonRatio,
                                            youngModulus = centerPartYoungModulus,
                                            type = self.centerPartType.value,
                                            collisionNodes = centerPartCollisionMesh[0],
                                            collisionTriangles = centerPartCollisionMesh[1],
                                            color = color,
                                            rotation = [0, 0, 0] if "down" in legsPositionOnMotor[0] else [180, 180, 0],
                                            mode = self.mode.value
                                            )
            if self.centerPartType.value == "rigid":
                self.effector = self.centerpart.addChild("Effector")
            else:
                self.effector = self.centerpart.attach.addChild("Effector")
            self.addChild(self.centerpart)
            self._addBox()
            self._addCamera()

    def addConnectionComponents(self) -> None:
        """
        Adds the connection components to the Emio robot.
        The components are used to connect the simulation to the real robot.
        """

        actuators = []
        for motor in self.motors:
            if motor is not None and motor.getObject("JointActuator") is not None:
                if motor.JointActuator.findData("angle"):
                    actuators.append(motor.JointActuator.angle)
                elif motor.JointActuator.findData("displacement"):
                    actuators.append(motor.JointActuator.displacement)
            else:
                actuators.append(None)
        self.getRoot().addObject(MotorController(actuators, name="MotorController"))

    def _addBox(self):
        pass
    
    def _addCamera(self):
        pass
    
    def _addPlatform(self):
        pass

    def addInverseComponentAndGUI(self, 
                                  targetMechaLink,
                                  positionWeight=1.,
                                  orientationWeight=0.,
                                  withGUI=True,
                                  barycentric=False) -> None:
        if not self._validState:
            Sofa.msg_error(self.getName(), "Sam has not been correctly initialized, cannot add the "
                                           "inverse components.")
            return

        for i, motor in enumerate(self.motors):
            motor.addObject('JointActuator', maxAngleVariation=pi / 20, 
                            maxAngle=pi/3 if self.legs[i] is not None else 0, 
                            minAngle=-pi/3 if self.legs[i] is not None else 0
                            )

        if positionWeight > 0.:
            self.effector.addObject('PositionEffector' if not barycentric else "BarycentricCenterEffector",
                                    template='Rigid3', indices=[0,1],
                                    useDirections=[1, 1, 1, 0, 0, 0],
                                    weight=positionWeight,
                                    maxSpeed=1000,
                                    limitShiftToTarget=True,
                                    maxShiftToTarget=20,  # mm
                                    effectorGoal=targetMechaLink, name="EffectorCoord")
            
        if orientationWeight > 0.:
            self.effector.addObject('PositionEffector' if not barycentric else "BarycentricCenterEffector",
                                    template='Rigid3', indices=[0,1],
                                    useDirections=[0, 0, 0, 1, 1, 1],
                                    weight=orientationWeight,
                                    effectorGoal=targetMechaLink, name="EffectorOrientation")

        if self._centerPartClass == Gripper:
            self.centerpart.addGripperEffector()

        if withGUI:
            self.addObject(SamGUI(self))

        return

