from math import pi

# EmioLabs Libraries
# from parts.emio import EmioGUI

# Sofa Library
import Sofa
import Sofa.ImGui as MyGui

class SamGUI(Sofa.Core.Controller):
    """
    The `SamGUI` class setups the elements of the graphical user interface for SOFA robotics.
    It uses a user-friendly interface to control the Sam robot's parameters and visualize its state.
    The GUI includes windows for robot settings, movement controls, and plotting.

    Class Variables:
        - `sam`: The Sam robot instance.
        - `plotTorquesAngles`: A boolean flag to enable or disable torque and angle plotting.

    Requirements:
        - This class requires inverse components to be added to the Sam instance.
        - The instance of Sam should be added in the graph scene before creating the GUI: to allow the GUI to control the robot, SamGUI must access the inverse solver which is located at the root of the scene.
        - The instance of Sam should also have the effector and actuator components added: typically done by calling the `addInverseComponentAndGUI` method. 
        
    Elements added to the GUI:
        - My Robot: Allows users to adjust robot settings such as maximum speed and motor angle limits.
        - Move: Provides controls for moving the robot's TCP and adjusting motor angles.
        - Plotting: Visualizes torque and angle data for each motor.
    """
    def __init__(self, sam, plotTorquesAngles=True):
        Sofa.Core.Controller.__init__(self)
        self.name = "SamGUI"
        root = sam.getRoot()
        self.guiActuators = [0, 0, 0, 0, 0, 0]

        MyGui.setIPController(root.Modelling.Target, sam.effector, root.ConstraintSolver)

        # Simulation State Tab
        MyGui.SimulationState.addData("TCP", "Frame", sam.effector.getMechanicalState().position)
        for i in range(sam.nbLegs.value):
            MyGui.SimulationState.addData("Torques", "M" + str(i), sam.motors[i].JointActuator.effort)
        for i in range(sam.nbLegs.value):
            MyGui.SimulationState.addData("Angles", "M" + str(i), sam.motors[i].JointActuator.angle)

        # My Robot Tab
        MyGui.MyRobotWindow.addSetting("Max TCP speed (mm/s)", sam.effector.EffectorCoord.maxSpeed, 0, 2000)
        for i, leg in enumerate(sam.legs):
            if (leg is not None):
                group = "M" + str(i)
                MyGui.MyRobotWindow.addSettingInGroup("Min angle (rad)", sam.motors[i].JointActuator.minAngle, -pi/2, 0, group)
                MyGui.MyRobotWindow.addSettingInGroup("Max angle (rad)", sam.motors[i].JointActuator.maxAngle, 0, pi/2, group)

        # Move Tab
        MyGui.MoveWindow.setTCPLimits(-200, 200,
                                      -3/2*pi,-pi/2)
                                        #sam.motors[0].JointActuator.minAngle.value,
                                        #sam.motors[0].JointActuator.maxAngle.value)
        
        MyGui.MoveWindow.setActuatorsDescription("Motors Position (rad)")
        MyGui.MoveWindow.setActuators([sam.motors[i].JointActuator.angle for i in range(len(sam.motors))],
                                        [0, 1, 2, 3, 4, 5], "displacement")
        for i in range(len(sam.motors)):
            MyGui.MoveWindow.setActuatorLimits(i, 
                                               sam.motors[i].JointActuator.minAngle.value,
                                               sam.motors[i].JointActuator.maxAngle.value)

        # Plotting Tab
        if plotTorquesAngles:
            for i in range(len(sam.motors)):
                MyGui.PlottingWindow.addData(" torque M" + str(i) + " (Nm*"+str(1/root.dt.value*1e-6)+")", sam.motors[i].JointActuator.effort)
                #MyGui.PlottingWindow.addData(" angle M" + str(i) + " (rad) ", sam.motors[i].JointActuator.angle)