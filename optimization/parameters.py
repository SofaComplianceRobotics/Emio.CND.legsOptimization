"""_summary_
Contains the optimization parameters related to:
   - the geometry of the legs
   - the materials of the legs
   - the movement sequence and its duration in the simulation
   - the simulation targets (weight, maximum displacement per each degree of freedom) 
   - Units are in mm, kg, s, rad, mN
   - Young modulus and density are in mN/mm^2 and kg/mm^3
"""
import os
from math import pi
from shape.centerLine import CenterLine
from shape.optLeg import OptLeg
from shape.optCenterPart import OptCenterPart
from animation import Movements

# Mode

# 3 possible modes are available :
#  1 - optimization : for running the optimization with Optuna.
#  2 - trial : for running a specific successful trial, requires setting the study name and trial number
#  3 - test : for using a customized shape which can be defined below at Leg Creation section

mode = 'trial' 
#config1
study_name = 'no-name-33d159f7-222c-479e-a350-10578a57c9ae'
trial_number = '32'

#config2
# study_name = 'no-name-24d6c80a-3090-4cc5-ae7e-8642c47ff59e'
# trial_number = '94'

trial_path = os.path.dirname(os.path.realpath(__file__))+'/../results/'+study_name+'/'+trial_number+'/params.json'

# Center part parameters todo center part parameters BaseParams / group by type (geometry, material, ...)
center_part_name = 'connector'
center_part_density = 1.24e-6
center_part_young_modulus = 2.5e6 #3.5e4
center_part_poisson_ratio = 0.35 #0.45
center_part_high_fixture_thickness = 2.5
center_part_high_diameter = 40
center_part_low_fixture_thickness = 2.5
center_part_low_diameter = 40
center_part_leg_height_difference = 60

# Short Leg parameters
low_leg_name = 'low'
low_leg_points = 5
low_leg_cross_section = [5,0.75]
low_leg_height = 80 #80
low_leg_beams = 30
low_leg_young_modulus = 210e6 #189e6
low_leg_poisson_ratio = 0.27
low_leg_density = 7.9e-6
low_leg_motor_distance_center = 100
low_leg_pulley_radius = 50
low_leg_pulley_fixture_depth = 30
low_leg_center_part_fixture_depth = 15

# Long leg parameters #TO Do create class parameters for motors, legs
high_leg_name = 'high'
high_leg_points = 5
high_leg_cross_section = [5,0.75]
high_leg_beams = 30
high_leg_young_modulus = 210e6 #189e3
high_leg_poisson_ratio = 0.27
high_leg_density = 7.9e-6
high_leg_motor_distance_center = 100
high_leg_pulley_radius = 50
high_leg_pulley_fixture_depth = 30
high_leg_center_part_fixture_depth = 15

# Movement sequence
sequence_of_movements = 'p0.5+ty-ty+tx-tx+tz-tz+rx-rx+rz-rz'
default_duration_per_movement = 3
movements_limits = {'-tx':50,'+tx':50,
                    '-ty':30,'+ty':120,
                    '-tz':50,'+tz':50,
                    '-rx':pi/3,'+rx':pi/3,
                    '-ry':pi/3,'+ry':pi/3,
                    '-rz':pi/3,'+rz':pi/3}
time_step = 0.01

# Simulation targets
weight = 0.750
#force = 1

low_leg_control_points = [[[-low_leg_pulley_radius, -low_leg_pulley_fixture_depth],0.1,90],
                          
                          [[-low_leg_pulley_radius, 0],20,90,0.1,270],
                          
                          
                          [[(low_leg_motor_distance_center-center_part_low_diameter/2-low_leg_pulley_radius)/2,
                            (low_leg_height-low_leg_pulley_fixture_depth-low_leg_center_part_fixture_depth)/2]
                            ,35,-45],
                          

                          [[low_leg_motor_distance_center-center_part_low_diameter/2,
                            low_leg_height-low_leg_pulley_fixture_depth-low_leg_center_part_fixture_depth]
                            ,0.1,90,20,270],
                          
                          [[low_leg_motor_distance_center-center_part_low_diameter/2,
                            low_leg_height-low_leg_pulley_fixture_depth]
                            ,0.1,90]]

high_leg_control_points = [[[-high_leg_pulley_radius, -high_leg_pulley_fixture_depth],0.1,90],
                           
                           [[-high_leg_pulley_radius, 0],20,90,0.1,270],
                           

                           [[(high_leg_motor_distance_center-center_part_high_diameter/2-high_leg_pulley_radius)/2,
                             (low_leg_height+center_part_leg_height_difference-high_leg_pulley_fixture_depth-high_leg_center_part_fixture_depth)/2]
                             ,35,-45],
                           

                           [[high_leg_motor_distance_center-center_part_high_diameter/2,
                             low_leg_height+center_part_leg_height_difference-high_leg_pulley_fixture_depth-high_leg_center_part_fixture_depth]
                             ,0.1,90,20,270],

                           [[high_leg_motor_distance_center-center_part_high_diameter/2,
                             low_leg_height+center_part_leg_height_difference-high_leg_pulley_fixture_depth]
                             ,0.1,90]]

center_part = OptCenterPart(name = center_part_name,
                            density = center_part_density,
                            youngModulus = center_part_young_modulus,
                            poissonRatio = center_part_poisson_ratio,
                            hCrossSection = high_leg_cross_section,
                            hDepth = high_leg_center_part_fixture_depth,
                            hThickness = center_part_high_fixture_thickness,
                            hDiameter = center_part_high_diameter,
                            lCrossSection = low_leg_cross_section,
                            lDepth = low_leg_center_part_fixture_depth,
                            lThickness = center_part_low_fixture_thickness,
                            lDiameter = center_part_low_diameter,
                            heightDifference = center_part_leg_height_difference)

low_leg = OptLeg(name = low_leg_name,
                 ltype = 'low_leg',
                 crossSection = low_leg_cross_section,
                 legHeight = low_leg_height,
                 youngModulus = low_leg_young_modulus,
                 poissonRatio = low_leg_poisson_ratio,
                 density = low_leg_density,
                 pulleyRadius = low_leg_pulley_radius,
                 motorDistanceToCenter = low_leg_motor_distance_center,
                 nBeams = low_leg_beams,
                 centerLine = CenterLine(name = low_leg_name,
                                         numberOfPoints = low_leg_points,
                                         controlPoints = low_leg_control_points))

high_leg = OptLeg(name = high_leg_name,
                  ltype = 'high_leg',
                  crossSection = high_leg_cross_section,
                  legHeight = low_leg_height+center_part_leg_height_difference,
                  youngModulus = high_leg_young_modulus,
                  poissonRatio = high_leg_poisson_ratio,
                  density = high_leg_density,
                  pulleyRadius = high_leg_pulley_radius,
                  motorDistanceToCenter = high_leg_motor_distance_center,
                  nBeams = high_leg_beams,
                  centerLine = CenterLine(name = high_leg_name,
                                          numberOfPoints = high_leg_points,
                                          controlPoints = high_leg_control_points))

movements = Movements(sequence = sequence_of_movements,
                      defaultMovementsDuration = default_duration_per_movement,
                      movementsLimits = movements_limits,
                      dt = time_step)

if mode == 'trial':
   high_leg.fromTrial(filepath = trial_path)
   low_leg.fromTrial(filepath = trial_path)
   center_part.fromTrial(filepath = trial_path)

  #  high_leg.crossSection=[5,0.75]
  #  low_leg.crossSection=[5,0.75]
  #  hname=study_name+'_'+trial_number+'_'+high_leg.name+'_'+str(high_leg.crossSection)
  #  lname=study_name+'_'+trial_number+'_'+low_leg.name+'_'+str(low_leg.crossSection)

  #  high_leg.exportCadQuery(name=hname,extension='.step')
  #  low_leg.exportCadQuery(name=lname,extension='.step')

  #  high_leg.centerLine.exportSvg(name=hname, withNodes=True)
  #  low_leg.centerLine.exportSvg(name=lname, withNodes=True)
  #  print('here')
  #  print(low_leg.centerLine.path.length)
  #  print(high_leg.centerLine.path.length)
  #  high_leg.centerLine.exportDxf(name=hname)
  #  low_leg.centerLine.exportDxf(name=lname)   
  #  low_leg.centerLine.plot()
  #  high_leg.centerLine.plot()

if __name__=="__main__":
   
   high_leg.fromTrial(filepath = trial_path)
   low_leg.fromTrial(filepath = trial_path)
   high_leg.exportCadQuery('.step')
   low_leg.exportCadQuery('.step')