from math import sin, cos, pi

# EmioLabs libraries
# from utils.topology import applyRotation
# from splib3.numerics import to_radians

from scipy.spatial.transform import Rotation

# Sofa Library
import Sofa

# Sam Libraries
from sparts.samDynamixelMotors import MotorController
from sparts.samGUI import SamGUI
from sparts.samLeg import SamLeg
from sparts.samCenterPart import SamCenterPart
from sparts.samMotor import SamMotor

class Sam(Sofa.Prefab) :

    """
    The `Sam` class represents the Sam robot in the simulation. It constructs the robot's structure, including its motors, legs, and center part, and provides methods for adding components to solve the inverse kinematics and integrate GUI elements.

    Class Variables:
        - `legsName` (`list[str]`): A list of names for the legs, corresponding to the mesh file names in the "data/meshes/legs" directory. The order follows the numbering of the motors. For a single type of leg, use a list like `["blueleg"]`. To skip a leg, use `"None"` as the name, e.g., `["blueleg", "blueleg", None, None]`.
        - `legsPositionOnMotor` (`list[str]`): A list of positions for each leg on the motor. Possible values are "clockwiseup", "counterclockwiseup", "clockwisedown", or "counterclockwisedown".
        - `legsModel` (`list[str]`): A list of models for each leg. Possible values are "beam", "cosserat", or "tetra".
        - `legsMassDensity` (`list[float]`): A list of mass densities for each leg. At least one value is expected, which will be applied to all legs in that case.
        - `legsPoissonRatio` (`list[float]`): A list of Poisson ratios for each leg. At least one value is expected, which will be applied to all legs in that case.
        - `legsYoungModulus` (`list[float]`): A list of Young's moduli for each leg. At least one value is expected, which will be applied to all legs in that case.
        - `centerPartName` (`str`): The name of the center part, which should correspond to the mesh file name in the "data/meshes/centerparts" directory.
        - `centerPartType` (`str`): The type of the center part. Possible values are "deformable", "rigid", or "gripper".
        - `centerPartModel` (`str`): The model of the center part. Possible values are "beam" or "tetra".
        - `centerPartMassDensity` (`float`): The mass density of the center part material.
        - `centerPartPoissonRatio` (`float`): The Poisson ratio of the center part material, if deformable.
        - `centerPartYoungModulus` (`float`): The Young's modulus of the center part material, if deformable.
        - `extended` (`bool`): A flag indicating whether the robot is in extended mode (True) or compact mode (False).
        - `platformLevel` (`int`): The level of the platform. Possible values are 0, 1, or 2.
        - `motorsDistanceToCenter` (`list[float]`): A list of distances from the motors to the center part. The default value is [70, 70, 70, 70, 70, 70] which correspond to the real device.
    
    Class Members:
        - `motors`: A list of motor objects.
        - `legs`: A list of leg objects.
        - `centerpart`: The center part object.
        - `effector`: The effector node, which is used for inverse kinematics.

    Example Usage:
    ```python
    from emio import Sam
    from utils import addHeader, addSolvers

    def createScene(root):
        settings, modelling, simulation = addHeader(root)
        addSolvers(simulation)

        emio = Sam(name="Sam", 
                    legsName=["blueleg"], 
                    centerPartName="yellowpart", 
                    centerPartType="rigid")
        if not emio.isValid():
            return
        simulation.addChild(emio)
        emio.attachCenterPartToLegs()
    ```
    """

    prefabData = [
        {'name': 'legsName', 'type': 'vector<string>', 
         'help': 'The name of the legs, which should correspond to the name of the mesh files, ex: ["blueleg"]. \
                  For a setup with different legs, give a list of names for motor n0 to n4, ex ["blueleg", "whiteleg", "blueleg", "blueleg"]. \
                  To skip a leg give "None" as a name, ["blueleg", "None", "blueleg", "None"]', 
         'default': ['blueleg']},
        {'name': 'legsPositionOnMotor', 'type': 'vector<string>', 'help': 'clockwiseup, counterclockwiseup, clockwisedown, or counterclockwisedown',
         'default': ["clockwiseup"]},
        {'name': 'legsModel', 'type': 'vector<string>', 'help': '["beam", "cosserat", "tetra"]', 'default': ["beam"]},
        {'name': 'legsMassDensity', 'type': 'vector<float>', 'help': 'List of mass density for each leg. At least one value is expected (applied to all legs in that case).', 'default': [1.22e-6]},
        {'name': 'legsPoissonRatio', 'type': 'vector<float>', 'help': 'List of Poisson ratio for each leg. At least one value is expected (applied to all legs in that case).', 'default': [0.45]},
        {'name': 'legsYoungModulus', 'type': 'vector<float>', 'help': 'List of Young modulus for each leg. At least one value is expected (applied to all legs in that case).', 'default': [3.5e4]},
        {'name': 'centerPartName', 'type': 'string', 'help': '', 'default': None},
        {'name': 'centerPartType', 'type': 'string', 'help': '["deformable", "rigid", "gripper"]', 'default': 'rigid'},
        {'name': 'centerPartModel', 'type': 'string', 'help': 'if deformable, model between tetra and beam', 'default': 'beam'},
        {'name': 'centerPartMassDensity', 'type': 'float', 'help': 'if deformable, mass density of the material', 'default': 1.220e-6},
        {'name': 'centerPartPoissonRatio', 'type': 'float', 'help': 'if deformable, Poisson ratio of the material', 'default': 0.45},
        {'name': 'centerPartYoungModulus', 'type': 'float', 'help': 'if deformable, Young modulus of the material', 'default': 3.5e4},
        {'name': 'extended', 'type': 'bool', 'help': '', 'default': False},
        {'name': 'motorsDistanceToCenter', 'type': 'vector<double>', 'help': '', 'default': [70, 70, 70, 70, 70, 70]},
        {'name': 'mode', 'type': 'string', 'help': 'to do', 'default': 'trial'}
    ]

    _validState = True

    def __init__(self, *args, **kwargs,):
        Sofa.Prefab.__init__(self, *args, **kwargs)
        #to do / to be declared in init arguments with their type
        optCenterPart = kwargs['centerpart']
        optLeg = kwargs['legs']

        self.addData(name="nbLegs", type="int", value=6) #value=4
        centerPartPositions = []

        # Add motors and legs
        self.motors = []
        self.legs = []

        # to do / create a parameter class to handle the declaration/initialization
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

        # to do /  create an intialization method for each SAM compoenent / code blocks
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
            translation = [radius * sin(angle), 0, radius * cos(angle)]

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
                    Sofa.msg_error(self.getName(), "At least one leg is not valid, cannot create Sam.")

                    break
                else:
                    self.addChild(leg)
                    # Attach leg to motor
                    leg.attachBase(motor.Parts, 1)

                    # Store leg extremity's position to define the center part of the robot
                    position = list(leg.extremity.getMechanicalState().position.value[0])

                    #applyRotation([position], to_radians(rotation))

                    er = Rotation.from_euler('xyz',rotation,degrees=True)
                    qr = Rotation.from_quat(position[3:7])
                    position[:3] = er.apply(position[:3])
                    position[3:7] = (er * qr).as_quat()
                    position[:3] += translation   
                    centerPartPositions += [position]

        # Robot's center part
        #TO DO import variables from yaml
        if self._validState:
            self.centerpart = SamCenterPart(name = "CenterPart",
                                            positions = centerPartPositions,
                                            partName = centerPartName,
                                            model = self.centerPartModel.value,
                                            massDensity = centerPartDensity,
                                            poissonRatio = centerPartpoissonRatio,
                                            youngModulus = centerPartYoungModulus,
                                            type = self.centerPartType.value,
                                            collisionNodes = centerPartCollisionMesh[0],
                                            collisionTriangles = centerPartCollisionMesh[1],
                                            color = [1,1,1,1],
                                            rotation = [0, 0, 0] if "down" in legsPositionOnMotor[0] else [180, 180, 0],
                                            mode = self.mode.value
                                            )
            
            self.effector = self.centerpart.addChild("Effector")

            self.addChild(self.centerpart)

    def addConnectionComponents(self) -> None:
        """
        Adds the connection components to the Sam robot.
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

    def addInverseComponentAndGUI(self, 
                                  targetMechaLink,
                                  positionWeight = 1.,
                                  orientationWeight = 0.,
                                  withGUI = True,
                                  barycentric = False) -> None:
        if not self._validState:
            Sofa.msg_error(self.getName(), "Sam has not been correctly initialized, cannot add the "
                                           "inverse components.")
            return

        for i, motor in enumerate(self.motors):
            motor.addObject('JointActuator', 
                            maxAngleVariation = pi/8, 
                            maxAngle = 3/8*pi if self.legs[i] is not None else 0, 
                            minAngle = -3/8*pi if self.legs[i] is not None else 0)

        if positionWeight > 0.:
            self.effector.addObject('PositionEffector' if not barycentric else "BarycentricCenterEffector",
                                    template = 'Rigid3', 
                                    indices = [0,1],
                                    useDirections = [1, 1, 1, 0, 0, 0],
                                    weight = positionWeight,
                                    maxSpeed = 1000,
                                    #limitShiftToTarget=False,
                                    #maxShiftToTarget=20,  # mm
                                    effectorGoal = targetMechaLink, 
                                    name = "EffectorCoord")
            
        if orientationWeight > 0.:
            self.effector.addObject('PositionEffector' if not barycentric else "BarycentricCenterEffector",
                                    template = 'Rigid3', 
                                    indices = [0,1],
                                    useDirections = [0, 0, 0, 1, 1, 1],
                                    weight = orientationWeight,
                                    effectorGoal = targetMechaLink, 
                                    name = "EffectorOrientation")

        if withGUI:
            gui = SamGUI(self)
            self.addObject(gui)

        return

    def isValid(self) -> bool:
        """
        Check if Sam is in a valid state. Returns True if Sam is in a valid state, False otherwise.
        """
        return self._validState
    
    def attachCenterPartToLegs(self) -> None:
        """
        Attaches the center part to the legs.
        The center part is attached to the legs at their extremities.
        The legs are attached to the motors at their base.
        """
        if not self._validState:
            Sofa.msg_error(self.getName(),
                           "Sam has not been correctly initialized, cannot attach the center part to the legs.")
            return

        for i, leg in enumerate(self.legs):
            if leg is not None:
                leg.attachExtremity(self.centerpart.attach, i)