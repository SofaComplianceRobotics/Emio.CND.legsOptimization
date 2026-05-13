# pyright: reportMissingImports=false, reportAttributeAccessIssue=false
# pylint: disable=import-error

# EmioLabs libraries
from parts.controllers.assemblycontroller import AssemblyController
from utils.header import addHeader, addSolvers

# Sofa Python Libraries
from splib3.animation import AnimationManager, animate

# Scene Libraries
from optimization.parameters import low_leg, high_leg, weight, mode, center_part, movements
from sparts.sam import Sam
from animation import animation
import numpy as np

def createScene(rootnode):
    rootnode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')
    rootnode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select')
    _, modelling, simulation = addHeader(rootnode, 
                                         inverse = True, 
                                         friction = 0, 
                                         withCollision = True
                                         )
    
    rootnode.LocalMinDistance.setDataValues(alarmDistance = 1 ,#center_part.hThickness/5, 
                                            contactDistance = 0.5)
    addSolvers(simulation)

    rootnode.dt = movements.dt
    rootnode.gravity = [0., -9810., 0.]
    rootnode.VisualStyle.displayFlags.value = ["showVisual",
                                               "showBehaviorModels",
                                               "showForceFields",
                                               "showWireframe",
                                               'showCollisionModels',
                                               'hideOptions']
    rootnode.addObject(AnimationManager(rootnode))

    #Leg generation
    if (not low_leg.isValid() or not high_leg.isValid()):
        rootnode.removeObject(AnimationManager(rootnode))
        return
    
    if (not low_leg.exportSTLCadQuery() or not high_leg.exportSTLCadQuery()):
        rootnode.removeObject(AnimationManager(rootnode))
        return

    # Add SAM to the scene
    sam = Sam(name = 'Sam',
              legs = [low_leg, high_leg]*3,
              centerpart = center_part,
              mode = mode,
              platformLevel = 2)
    
    if not sam.isValid():
        rootnode.removeObject(AnimationManager(rootnode))
        return
    
    simulation.addChild(sam)
    sam.attachCenterPartToLegs()
    assembly = AssemblyController(sam)
    assembly.duration = 0.01
    sam.addObject(assembly)

    # Add effector
    sam.effector.addObject("MechanicalObject", 
                            template = "Rigid3", 
                            position = [[0, center_part.height/2, 0, 0, 0, 0, 1],
                                        [0, -center_part.height/2, 0, 0, 0, 0, 1]],
                            showObject = False)
    sam.effector.addObject("RigidMapping",
                            index = 0)

    # Target
    effectorTarget = modelling.addChild('Target')

    effectorTarget.addObject('EulerImplicitSolver',
                             firstOrder = True)
    effectorTarget.addObject('CGLinearSolver',
                             iterations = 50,
                             tolerance = 1e-10,
                             threshold = 1e-10)
    
    effectorTarget.addObject('MechanicalObject',
                                template = 'Rigid3',
                                position = [[0, low_leg.legHeight+center_part.legHeightDifference+center_part.hThickness+40, 0, 0, 0, 0, 1],
                                            [0, low_leg.legHeight-center_part.lDepth+40, 0, 0, 0, 0, 1]],
                                showObject = True,
                                showObjectScale = 10)
    
    #Sensor
    sensor = sam.CenterPart.addChild('Sensor')
    sensor.addObject('MechanicalObject',
                     position = [[0.0, 40.0, 0.0]],
                     #showObject = False,
                     #drawMode = 2
                     )
    
    sensor.addObject("UniformMass",
                     totalMass = weight)
    
    sensor.addObject("RigidMapping",
                     index = 0)

    # sensor.addObject("ForcePointActuator", 
    #             name="ForcePointActuator",
    #             indices=[0],
    #             direction=[0, 1, 0],
    #             applyForce=False, # We only want to apply the force when the assembly is done
    #             showForce=True, 
    #             maxForceVariation = 1,
    #             visuScale=1.0,
    #             )
    
    if mode == 'optimization':   
        sam.addInverseComponentAndGUI(targetMechaLink = effectorTarget.getMechanicalState().position.linkpath,
                                      withGUI = False)
        animate(onUpdate = animation,
                params = {'target':effectorTarget.getMechanicalState().position,
                          'initialPosition':np.copy(effectorTarget.getMechanicalState().position.value),
                          'movements':movements,
                          'height':center_part.height/2
                        },
                duration = movements.duration)
    else:
        sam.addInverseComponentAndGUI(targetMechaLink = effectorTarget.getMechanicalState().position.linkpath,
                                      orientationWeight = 57)
        sam.addConnectionComponents()
        
        effectorTarget.MechanicalObject.position = [[0, low_leg.legHeight+center_part.legHeightDifference+center_part.hThickness+20, 0, 0, 0, 0, 1]]
        sam.effector.MechanicalObject.position = [[0, center_part.height/2, 0, 0, 0, 0, 1]]
        sam.effector.EffectorCoord.indices = [0]
        sam.effector.EffectorOrientation.indices = [0]

#         animate(onUpdate = distance,
#                 params = {'desiredPosition':rootnode.Modelling.Target
# ,
#                         'maximumPosition':rootnode.Simulation.Sam.CenterPart
#                         },
#                 duration = param.duration)  
    return rootnode
