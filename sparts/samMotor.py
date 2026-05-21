import Sofa
from splib3.loaders import getLoadingLocation
from shape.optDrum import OptDrum

class SamMotor(Sofa.Prefab):
    """
    Represents a motor in the SAM robot. 
    By default, motors are added to the SAM class with the rotation and translation matching the real device.

    Class Variables:
        - `rotation` (`Vec3d`): Orientation of the motor.
        - `translation` (`Vec3d`): Position of the motor. 
        - `scale3d` (`Vec3d`): Scale of the 3D model for rendering purposes.
        - `color` (`Vec4f`): Color of the motor for rendering purposes, in RGBA format.
    
    Example Usage:
    ```python
        def createScene(root):
            motor = root.addChild(Motor(name="Motor"))

            # Access the motor's angle
            angle = motor.getMechanicalState().position.value[0]
    ```
    """
    prefabData = [
        {'name': 'rotation', 'type': 'Vec3d', 'help': 'Rotation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'tempvisurotation', 'type': 'Vec3d', 'help': 'Rotation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Translation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'scale3d', 'type': 'Vec3d', 'help': 'Scale 3d', 'default': [1.0, 1.0, 1.0]},
        {'name': 'color', 'help': '', 'type': 'Vec4f', 'default': [0.2, 0.2, 0.2, 1.0]},
        {'name': 'mode', 'type': 'string', 'help': '', 'default': 'trial'}
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

        self._addRequiredPlugins()

        self.addObject('MechanicalObject',
                       template='Vec1',
                       position=[[0]])

        parts = self.addChild('Parts')
        parts.addObject('MechanicalObject',
                        template = 'Rigid3',
                        position = [[0., 0., 0., 0., 0., 0., 1.],
                                    [0., 0., 0., 0., 0., 0., 1.]],
                        showObjectScale = 3,
                        translation = self.translation.value,
                        rotation = self.rotation.value,
                        scale3d = self.scale3d.value)
        
        parts.addObject("UniformMass", 
                        totalMass = 0.01)
        
        parts.addObject('ArticulatedSystemMapping',
                        input1 = self.getMechanicalState().linkpath,
                        output = parts.getMechanicalState().linkpath)

        articulationCenter = self.addChild('ArticulationCenter')
        articulationCenter.addObject('ArticulationCenter',
                                     parentIndex = 0,
                                     childIndex = 1,
                                     posOnParent = [0., 0., 0.],
                                     posOnChild = [0., 0., 0.])
        
        articulation = articulationCenter.addChild('Articulations')

        articulation.addObject('Articulation',
                               translation = False,
                               rotation = True,
                               rotationAxis = [1, 0, 0],
                               articulationIndex = 0)
        
        self.addObject('ArticulatedHierarchyContainer')
        
        if self.mode.value == 'trial':
            leg = kwargs['leg']
            OptDrum(leg).exportStl()
            file = leg.name +"motorattachbase.stl"
            #file = 'legmotorattachbase.stl'
            translation = [leg.crossSection[0]+4.5,0,0]

            visual = parts.addChild('MotorVisual')
            visual.addObject('MeshSTLLoader',
                             name = 'loader',
                             filename = getLoadingLocation("../data/meshes/motor.stl", __file__),
                             rotation = self.tempvisurotation.value,
                             translation = translation)
            
            visual.addObject('MeshTopology',
                             src = '@loader')
            
            visual.addObject('OglModel',
                             color = [0, 0, 0, 1])
            
            visual.addObject('RigidMapping',
                             index = 0)            

            visual = parts.addChild('LegAttachBaseVisual')
            visual.addObject('MeshSTLLoader',
                             name = 'loader',
                             filename = getLoadingLocation("../data/meshes/"+file, __file__),
                             rotation = self.tempvisurotation.value)
            
            visual.addObject('MeshTopology',
                             src = '@loader')
            visual.addObject('OglModel',
                             color = [1, 1, 1, 1])
            visual.addObject('RigidMapping',
                             index = 1)

    def _addRequiredPlugins(self):
        """
        Private method to add required plugins. Create a dedicated node in the Motor node.
        Called at the beginning of __init__.
        """
        plugins = self.addChild("RequiredPlugins")
        plugins.addObject('RequiredPlugin', name="ArticulatedSystemPlugin")
        plugins.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')
        # Needed to use components [MeshOBJLoader,MeshSTLLoader]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')
        # Needed to use components [RigidMapping]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Mass')
        # Needed to use components [UniformMass]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring')
        # Needed to use components [RestShapeSpringsForceField]
        plugins.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')
        # Needed to use components [MeshTopology]
        plugins.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')
        # Needed to use components [OglModel]