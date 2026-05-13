import Sofa
from splib3.loaders import getLoadingLocation
from parts.motor import Motor
from shape.optDrum import OptDrum

class SamMotor(Motor):
    prefabData = Motor.prefabData
    prefabData.append({'name': 'mode', 'type': 'string', 'help': '', 'default': 'trial'})

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