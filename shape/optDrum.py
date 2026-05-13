from __future__ import annotations
import os
from math import pi, sin, cos
import cadquery as cq

if __name__ =='__main__':
    from optLeg import OptLeg
else:
    from shape.optLeg import OptLeg

CAD_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/../data/meshes/'
FIG_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/../data/'#+'/data/figs/'
STL_DIRECTORY = os.path.dirname(os.path.realpath(__file__))#+'/data/cad/

class OptDrum:
    def __init__(self, leg: OptLeg=OptLeg()):
        self.crossSection = leg.crossSection
        self.radius = leg.pulleyRadius
        self.name = leg.name

    def exportStl(self,path=CAD_DIRECTORY)-> bool:
        os.chdir(path)
        d = 19.1
        h = 5
        r = self.radius  
        cross_section = self.crossSection
        tolerance = 0.1
        offset = 2.5
        offset_leg = 2.5
        xoffset = -(cross_section[0]/2+offset-tolerance/2+h/2+offset_leg)

        cylinder = cq.Workplane("YZ",origin=(xoffset+h/2,0,0)).cylinder(radius=d/2, height=h)
        lever = cq.Workplane("YZ",origin=(xoffset+h/2,-r/2,-d/4)).box(length=r, width=d/2, height=h)

        pulley_cavity = cq.Workplane("YZ",origin=(xoffset+2.5/2,0,0)).cylinder(radius=8.4/2,height=2.5)
        axis = cq.Workplane("YZ",origin=(xoffset+5/2,0)).cylinder(radius=3/2,height=5)
        screw_cavity = cq.Workplane("YZ",origin=(xoffset+2/2+3,0,0)).cylinder(radius=5/2,height=2)
        axis_cavities = pulley_cavity.union(axis).union(screw_cavity)
        mini_screw_cavities = cq.Workplane("YZ")
        for i in range(8):
            angle_rad = (2 * pi / 8) * i
            x = 8 * cos(angle_rad)
            y = 8 * sin(angle_rad)
            mini_screw_thread_cavities = cq.Workplane("YZ",origin=(xoffset+5/2,y,x)).cylinder(radius=2.2/2,height=5)
            mini_screw_cap_cavities = cq.Workplane("YZ",origin=(xoffset+1.5+3.5/2,y,x)).cylinder(radius=4/2,height=3.5)
            mini_screw_cavities = mini_screw_cavities.union(mini_screw_cap_cavities).union(mini_screw_thread_cavities)

        pulley_cavities = mini_screw_cavities.union(axis_cavities)
        
        pulley = cylinder.union(lever).cut(pulley_cavities)

        leg_fixture_out = cq.Workplane("YZ",
                                    origin=(xoffset+(cross_section[0]+offset*2+tolerance+offset-tolerance/2+offset_leg)/2,
                                                    -r,
                                                    -d/2+d/4))
        leg_fixture_out = leg_fixture_out.box(length=cross_section[1]+offset*2+tolerance,
                                            width=d/2, 
                                            height=cross_section[0]+offset*2+tolerance+offset-tolerance/2+offset_leg)

        pulley = pulley.union(leg_fixture_out)
        pulley = pulley.edges("|Z").filter(lambda e: e.startPoint().y>-r and e.startPoint().x<xoffset+(cross_section[0]+offset*2+tolerance+offset-tolerance/2+offset_leg)/2 ).fillet(5)
        pulley = pulley.edges("|Z").fillet(2.5)

        leg_fixture_in = cq.Workplane("YZ",
                                    origin=(xoffset+(cross_section[0]/2+offset-tolerance/2)+h,
                                            -r,
                                            -d/2+d/4))
        leg_fixture_in = leg_fixture_in.box(length=cross_section[1]+tolerance, 
                                            width=d/2, 
                                            height=(cross_section[0]+tolerance-tolerance/2))    
        pulley = pulley.cut(leg_fixture_in)
        
        pulley = pulley.val()

        pulley.exportStl(fileName=self.name+'motorattachbase.stl',
                        tolerance=0.1,
                        angularTolerance=0.5)

if __name__ =='__main__':
    # drum = OptDrum()
    # drum.exportStl()
    os.chdir(CAD_DIRECTORY)
    d=19.1
    h=5
    r=30  
    cross_section=[10,0.5]
    tolerance=0.1
    offset = 2.5
    xoffset= -15
    offset_leg = 2.5
    
    cylinder = cq.Workplane("YZ",origin=(xoffset+h/2,0,0)).cylinder(radius=d/2, height=h)
    lever = cq.Workplane("YZ",origin=(xoffset+h/2,-r/2,-d/4)).box(length=r, width=d/2, height=h)

    pulley_cavity = cq.Workplane("YZ",origin=(xoffset+2.5/2,0,0)).cylinder(radius=8.4/2,height=2.5)
    axis = cq.Workplane("YZ",origin=(xoffset+5/2,0)).cylinder(radius=3/2,height=5)
    screw_cavity = cq.Workplane("YZ",origin=(xoffset+2/2+3,0,0)).cylinder(radius=5/2,height=2)
    axis_cavities = pulley_cavity.union(axis).union(screw_cavity)
    mini_screw_cavities = cq.Workplane("YZ")
    for i in range(8):
        angle_rad = (2 * pi / 8) * i
        x = 8 * cos(angle_rad)
        y = 8 * sin(angle_rad)
        mini_screw_thread_cavities = cq.Workplane("YZ",origin=(xoffset+5/2,y,x)).cylinder(radius=2.2/2,height=5)
        mini_screw_cap_cavities = cq.Workplane("YZ",origin=(xoffset+1.5+3.5/2,y,x)).cylinder(radius=4/2,height=3.5)
        mini_screw_cavities = mini_screw_cavities.union(mini_screw_cap_cavities).union(mini_screw_thread_cavities)

    pulley_cavities = mini_screw_cavities.union(axis_cavities)
    
    pulley = cylinder.union(lever).cut(pulley_cavities)

    leg_fixture_out = cq.Workplane("YZ",
                                   origin=(xoffset+(cross_section[0]+offset*2+tolerance+offset-tolerance/2)/2,
                                           -r,
                                           -d/2+d/4))
    leg_fixture_out = leg_fixture_out.box(length=cross_section[1]+offset*2+tolerance,
                                        width=d/2, 
                                        height=cross_section[0]+offset*2+tolerance+offset-tolerance/2)

    pulley = pulley.union(leg_fixture_out)
    pulley = pulley.edges("|Z").filter(lambda e: e.startPoint().y>-r and e.startPoint().x<xoffset+(cross_section[0]+offset*2+tolerance+offset-tolerance/2)/2 ).fillet(5)
    pulley = pulley.edges("|Z").fillet(2.5)

    leg_fixture_in = cq.Workplane("YZ",
                                origin=(xoffset+(cross_section[0]/2+h), #xoffset+(cross_section[0]/2+offset-tolerance/2
                                        -r,
                                        -d/2+d/4))
    leg_fixture_in = leg_fixture_in.box(length=cross_section[1]+tolerance, 
                                        width=d/2, 
                                        height=(cross_section[0]+tolerance-tolerance/2))    
    pulley = pulley.cut(leg_fixture_in)
    
    pulley = pulley.val()

    pulley.exportStl(fileName='testmotorattachbase.stl',
                    tolerance=0.1,
                    angularTolerance=0.5)
