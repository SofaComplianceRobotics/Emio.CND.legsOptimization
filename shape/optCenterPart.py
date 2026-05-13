from __future__ import annotations
import os
from math import pi, sin, cos, sqrt, acos
import gmsh
from json import dump
import cadquery as cq
from json import loads

CAD_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/../data/meshes/centerparts/'
FIG_DIRECTORY = os.path.dirname(os.path.realpath(__file__))+'/../data/'#+'/data/figs/'
STL_DIRECTORY = os.path.dirname(os.path.realpath(__file__))#+'/data/cad/

class OptCenterPart:
    def __init__(self,
                 name:str = 'connector',
                 density:float = 1.22e-6,
                 poissonRatio:float = 0.45,
                 youngModulus:float = 3.5e4,
                 lCrossSection:list[float]=[10,5],
                 lDepth:float=15,
                 lThickness:float=2.5,
                 lDiameter:float=40,
                 hCrossSection:list[float]=[10,5],
                 hDepth:float=15,
                 hThickness:float=2.5,
                 hDiameter:float=40,
                 heightDifference:float=40):
        
        self.name = name
        self.density = density
        self.poissonRatio = poissonRatio
        self.youngModulus = youngModulus
        self.lcrossSection = lCrossSection
        self.lDepth = lDepth
        self.lDiameter = lDiameter
        self.lThickness = lThickness
        self.hcrossSection = hCrossSection
        self.hDepth = hDepth
        self.legHeightDifference = heightDifference
        self.hDiameter = hDiameter
        self.hThickness = hThickness
        self.height = self.legHeightDifference+self.lDepth+self.hThickness

    def exportStl(self,path=CAD_DIRECTORY)-> bool:
        low_thickness = self.lThickness
        low_cross_section = self.lcrossSection
        low_diameter = self.lDiameter-low_cross_section[1]-2*self.lThickness
        low_fixture_width = 2*self.lThickness+low_cross_section[0]
        low_fixture_height = self.lDepth+self.lThickness
        low_fixture_thickness = 2*self.lThickness+low_cross_section[1]
        low_hexagone_height = self.lDepth+self.lThickness

        high_thickness = self.hThickness
        high_cross_section = self.hcrossSection
        high_diameter = self.hDiameter-high_cross_section[1]-2*self.hThickness
        high_fixture_width = 2*self.hThickness+high_cross_section[0]
        high_fixture_height = self.hDepth + self.hThickness
        high_fixture_thickness = 2*self.hThickness+high_cross_section[1]
        high_hexagon_height = self.hDepth + self.hThickness

        cone_height = self.height
        cone = cq.Workplane("ZX",origin=(0,0,0))
        cone = cone.cylinder(radius=high_diameter/2,
                             height=cone_height)
        
        low_hexagon = cq.Workplane("ZX",origin=(0,-cone_height/2,0))
        low_hexagon = low_hexagon.polygon(nSides=6,
                                          circumscribed=True,
                                          diameter=low_diameter)
        low_hexagon = low_hexagon.extrude(low_hexagone_height)
        low_hexagon = low_hexagon.edges('|Y').fillet(1/2*low_diameter-sqrt(3)/2*low_fixture_width)
        
        high_hexagon = cq.Workplane("ZX",origin=(0,cone_height/2-high_hexagon_height,0))
        high_hexagon = high_hexagon.polygon(nSides=6,
                                            circumscribed=True,
                                            diameter=high_diameter)
        high_hexagon = high_hexagon.extrude(high_hexagon_height)
        high_hexagon = high_hexagon.edges('|Y').fillet(1/2*high_diameter-sqrt(3)/2*high_fixture_width)

        high_fixtures = cq.Workplane("ZX")
        low_fixtures = cq.Workplane("ZX")

        for i in range(3):
            angle_rad = 2/3*i*pi+1/2*pi
            x = (high_diameter+high_fixture_thickness)/2 * cos(angle_rad)
            z = (high_diameter+high_fixture_thickness)/2 * sin(angle_rad)
            
            high_fixtures_out = cq.Workplane("ZX",
                                            origin = (x,
                                                    (cone_height-high_fixture_height)/2,
                                                    z))
            
            high_fixtures_out = high_fixtures_out.box(length = high_fixture_thickness,
                                                      width = high_fixture_width,
                                                      height = high_fixture_height)
            
            high_fixtures_out = high_fixtures_out.rotate(axisStartPoint = (x,0,z),
                                                        axisEndPoint = (x,1,z),
                                                        angleDegrees = 1/3*180*i)
            high_fixtures_out = high_fixtures_out.edges('|Y').filter(lambda e:e.startPoint().z**2+e.startPoint().x**2 > (x**2+z**2) )
            high_fixtures_out = high_fixtures_out.fillet(high_thickness)

            # x -= high_thickness/2 * cos(angle_rad)
            # z -= high_thickness/2 * sin(angle_rad)

            high_fixtures_in = cq.Workplane("ZX",
                                            origin=(x,
                                                    (cone_height-high_fixture_height-high_thickness)/2,
                                                    z))

            high_fixtures_in = high_fixtures_in.box(length=high_fixture_thickness-2*high_thickness,
                                                    width=high_fixture_width-2*high_thickness,
                                                    height=high_fixture_height-high_thickness)
            
            high_fixtures_in = high_fixtures_in.rotate(axisStartPoint = (x,0,z),
                                                       axisEndPoint = (x,1,z),
                                                       angleDegrees = 1/3*180*i)
            
            high_fixtures_out = high_fixtures_out.cut(high_fixtures_in)

            high_fixtures = high_fixtures.union(high_fixtures_out)


            # low fixture
            x = -(low_diameter+low_fixture_thickness)/2 * cos(angle_rad)
            z = -(low_diameter+low_fixture_thickness)/2 * sin(angle_rad)
            
            low_fixtures_out = cq.Workplane("ZX",
                                            origin = (x,
                                                    -(cone_height-low_fixture_height)/2,
                                                    z))
            
            low_fixtures_out = low_fixtures_out.box(length = low_fixture_thickness,
                                                    width = low_fixture_width,
                                                    height = low_fixture_height)
            
            low_fixtures_out = low_fixtures_out.rotate(axisStartPoint = (x,0,z),
                                                    axisEndPoint = (x,1,z),
                                                    angleDegrees = 1/3*180*i)
            
            low_fixtures_out = low_fixtures_out.edges('|Y').filter(lambda e:e.startPoint().z**2+e.startPoint().x**2 > (x**2+z**2) )
            low_fixtures_out = low_fixtures_out.fillet(low_thickness)

            # x += low_thickness/2 * cos(angle_rad)
            # z += low_thickness/2 * sin(angle_rad)

            low_fixtures_in = cq.Workplane("ZX",
                                        origin=(x,
                                                -(cone_height-low_fixture_height+low_thickness)/2,
                                                z))

            low_fixtures_in = low_fixtures_in.box(length=low_fixture_thickness-2*low_thickness,
                                                width=low_fixture_width-2*low_thickness,
                                                height=low_fixture_height-low_thickness)
            
            low_fixtures_in = low_fixtures_in.rotate(axisStartPoint = (x,0,z),
                                                    axisEndPoint = (x,1,z),
                                                    angleDegrees = 1/3*180*i)
            
            low_fixtures_out = low_fixtures_out.cut(low_fixtures_in)

            low_fixtures = low_fixtures.union(low_fixtures_out)
        
        connector = cone.union(low_hexagon).union(low_fixtures).union(high_hexagon).union(high_fixtures)
        connector = connector.val()
        return connector.exportStl(fileName=path+'connector.stl',
                                   tolerance=0.1,
                                   angularTolerance=0.5)

    def exportJson(self,path=CAD_DIRECTORY)->bool:
        data = {}
        cone_height = self.height
        
        # low_fixture_thickness = 2*self.lThickness+self.lcrossSection[1]
        # high_fixture_thickness = 2*self.hThickness+self.hcrossSection[1]

        # cone_high_diameter = self.hDiameter-high_fixture_thickness
        # cone_low_diameter = self.lDiameter-low_fixture_thickness

        high_z = cone_height/2-self.hThickness
        low_z = -cone_height/2+self.lDepth

        high_radius = self.hDiameter/2
        low_radius = self.lDiameter/2
 
        data['initialPosition'] = [[0, -180, 0, 1, 0, 0, 0]]
        data['attachPositionInLocalCoord'] = [[                    0,   low_z,  -low_radius,             0.000, 0.000,  -0.707,  -0.707],
                                              [                    0,   high_z,  high_radius,           -0.707, -0.707, 0.000, 0.000],
                                              [-low_radius*cos(pi/6),   low_z,   low_radius*sin(pi/6),  -0.610,-0.610, -0.350, -0.350],
                                              [-high_radius*cos(pi/6),  high_z, -high_radius*sin(pi/6), -0.350,-0.350, -0.610, -0.610],
                                              [ low_radius*cos(pi/6),   low_z,   low_radius*sin(pi/6),  -0.610,-0.610, 0.350, 0.350],
                                              [ high_radius*cos(pi/6),  high_z, -high_radius*sin(pi/6), -0.350,-0.350, 0.610, 0.610]]
        
        with open(path+self.name+'.json','w',encoding="utf-8") as f:
            dump(data,f)

    def getCollisionTopology(self,max_error=0.2)->list:

        # Initialize Gmsh
        gmsh.initialize()
        occGeo=gmsh.model.occ()
        
        # # Calculate n_points from maximum error and radius
        # max_radius = max(self.lDiameter,self.hDiameter)/2
        # n_points = max(6, int(pi / acos(1 - max_error / max_radius))) #6 sides are the minimum

        low_radius = (self.lDiameter-self.lcrossSection[1])/2-self.lThickness
        high_radius = (self.hDiameter-self.hcrossSection[1])/2-self.hThickness
        side_surface_arc_angle = 1.5*self.hcrossSection[0]/high_radius
        
        # Create profile line in r-z plane
        side_surface_y_min = -self.height/2
        side_surface_y_max = self.height/2-self.hThickness-self.hDepth

        p1 = occGeo.addPoint(low_radius, side_surface_y_min,0)
        p2 = occGeo.addPoint(high_radius, side_surface_y_max,0)
        
        line = occGeo.addLine(p1, p2)
        occGeo.rotate([(1, line)],0, 0, 0, 0, 1, 0,-pi/2-side_surface_arc_angle/2)

        # Revolve around z-axis to create lateral surface

        side_surface1 = occGeo.revolve([(1, line)], 0, 0, 0, 0, 1, 0,side_surface_arc_angle )
        side_surface2 = occGeo.copy(side_surface1)
        occGeo.rotate(side_surface2,0,0,0,0,1,0,2*pi/3)
        side_surface3 = occGeo.copy(side_surface1)
        occGeo.rotate(side_surface3,0,0,0,0,1,0,4*pi/3)
        isolatedPoints = occGeo.getEntities(dim=0)
        occGeo.remove(isolatedPoints)        
        # base = occGeo.addDisk(xc=0,yc=side_surface_y_min,zc=0,
        #                               rx=low_radius, ry=low_radius,
        #                               zAxis=[0,1,0])

        occGeo.synchronize()
        
        # Generate 2D mesh
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setNumber('Mesh.MaxNumThreads2D',22)
        #gmsh.option.setNumber("Mesh.CharacteristicLengthMax", mesh_size)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature",6)
        gmsh.model.mesh.generate(2)

        _, coord,_ = gmsh.model.mesh.getNodes()
        elementType = gmsh.model.mesh.getElementTypes(dim=2)[0]
        nodesPerElement = gmsh.model.mesh.getElementProperties(elementType)[3]
        nodes, _,_ = gmsh.model.mesh.getNodesByElementType(elementType)
        nodes = [x-1 for x in nodes]
        elements = [list(map(int,nodes[nodesPerElement*idx:nodesPerElement*(idx+1)])) for idx in range(int(len(nodes)/nodesPerElement))]
        positions = [coord[3*idx:3*(idx+1)] for idx in range(int(len(coord)/nodesPerElement))]

        gmsh.finalize()
    
        return [positions, elements]

    def fromTrial(self,filepath)->bool:
        
        with open(filepath,'r',encoding="utf-8") as f:
            data = loads(f.read())        
        data = data['center_part']
        self.name = data['name'] 
        self.density = data['density']
        self.youngModulus = data['youngModulus']
        self.poissonRatio = data['poissonRatio']
        self.hcrossSection = data['hCrossSection']
        self.hDepth = data['hDepth']
        self.hThickness = data['hThickness']
        self.hDiameter = data['hDiameter']
        self.lcrossSection = data['lCrossSection']
        self.lDepth = data['lDepth']
        self.lThickness = data['lThickness']
        self.lDiameter = data['lDiameter']
        self.legHeightDifference = data['height']

if __name__ =='__main__':
    center_part = OptCenterPart()
    center_part.getCollisionTopology()
    #center_part.exportStl()
   