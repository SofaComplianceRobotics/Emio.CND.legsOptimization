from math import pi

# EmioLabs Libraries
from parts.emio import EmioGUI

# Sofa Library
import Sofa
import Sofa.ImGui as MyGui

class SamGUI(EmioGUI):
    def __init__(self, sam, plotTorquesAngles=False):
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
                MyGui.MyRobotWindow.addSettingInGroup("Min angle (rad)", sam.motors[i].JointActuator.minAngle, -pi/3, 0, group)
                MyGui.MyRobotWindow.addSettingInGroup("Max angle (rad)", sam.motors[i].JointActuator.maxAngle, 0, pi/3, group)

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
                MyGui.PlottingWindow.addData(" torque M" + str(i) + " 1e-3Nmm ", sam.motors[i].JointActuator.effort)
                MyGui.PlottingWindow.addData(" angle M" + str(i) + " (rad) ", sam.motors[i].JointActuator.angle)