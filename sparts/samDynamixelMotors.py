import Sofa
import Sofa.ImGui as MyGui
from dynamixelmotorsapi import DynamixelMotors
from dynamixelmotorsapi import listFTDIDevices
import threading
import time    

class samDynamixelMotors(DynamixelMotors):
    def __init__(self):
        super().__init__([{
            "id": [0, 1, 2, 3, 4, 5],
            "model": "XM430-W210",
            "pulley_radius": 30,
            "pulse_center": 2048,
            "max_vel": 1000,
            "baud_rate": 1000000
        }])

def connectRobot(samMotors: samDynamixelMotors):
    """
    This function runs in a separate thread to check the connection status of the robot.
    It will open the connection if the simulation/robot toggle button from the GUI is on and the robot is not connected,
    and close the connection if the toggle is off and the robot is connected.
    """
    samConnected = False
    
    while True:
        samConnected = samMotors.is_connected
        if MyGui.getRobotConnectionToggle() and not samConnected:
            try:
                index = samMotors.findAndOpen(device_name=MyGui.MyRobotWindow.getSelectedPort())
                if index == -1:
                    Sofa.msg_error("MotorController", "Could not find or connect Emio robots. Please check the connection.")
                    MyGui.MyRobotWindow.updateAvailablePorts()
                MyGui.setRobotConnectionToggle(index >= 0 ) # GUI toggle with the current connection status
            except Exception as e:
                Sofa.msg_error("MotorController", str(e))
                MyGui.setRobotConnectionToggle(False)
                MyGui.MyRobotWindow.updateAvailablePorts()
                samMotors.close()
        elif not MyGui.getRobotConnectionToggle() and samConnected:
            samMotors.close()

        time.sleep(2) # wait 2 seconds before checking again

class MotorController(Sofa.Core.Controller):
    
    def __init__(self, jointActuators, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "MotorController"
        self.jointActuators = jointActuators
        self.samMotors = samDynamixelMotors()
        MyGui.MyRobotWindow.listAvailablePortsCallback(self, "listAvailablePortsCallback")

        # Connection thread
        self.connectionThread = threading.Thread(target=connectRobot, args=[self.samMotors])
        self.connectionThread.daemon = True
        self.connectionThread.start()

    def onAnimateEndEvent(self, _):
        """
        This function is called at the end of each animation step.
        It checks the connection status of the robot and updates the joint angles if connected.
        """
        if MyGui.getRobotConnectionToggle() and self.samMotors.is_connected:
            angles = []
            for joint in self.jointActuators:
                if joint is not None:
                    angles.append(-joint.value)
                else:
                    angles.append(0)
            try:
                self.samMotors.angles = angles
            except Exception as e:
                Sofa.msg_error("MotorController - onAnimate", str(e))
                MyGui.setRobotConnectionToggle(False)
                self.samMotors.close()

    def listAvailablePortsCallback(self) -> list[str]:
        """
        Update the list of available Sam devices in the GUI.
        """
        ports = listFTDIDevices()
        if ports is not None and len(ports) != 0:
            return ports
        return ["No Sam robots found"]   

