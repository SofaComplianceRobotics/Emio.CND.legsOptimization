def getIndicesInOrientedBox(positions, obox):
    indicesIn = []
    indicesOut = []
    p0 = [obox[0], obox[1], obox[2]]
    p1 = [obox[3], obox[4], obox[5]]
    p2 = [obox[6], obox[7], obox[8]]
    depth = obox[9]

    p3 = p0 + (p2-p1)
    p6 = p2 + normal * depth
    

    normal = (p1-p0).cross(p2-p0)
    normal.normalize()



    plane0 = (p1-p0).cross(normal)
    plane0.normalize()

    plane1 = (p2-p3).cross(p6-p3)
    plane1.normalize()

    plane2 = (p3-p0).cross(normal)
    plane2.normalize()

    plane3 = (p2-p1).cross(p6-p2)
    plane3.normalize()

    width = abs(dot((p2-p0),plane0))
    length = abs(dot((p2-p0),plane2))


    for index, pos in enumerate(positions):
        pv0 = [pos[0]-p0[0], pos[1]-p0[1], pos[2]-p0[2]]
        pv1 = [pos[0]-p2[0], pos[1]-p2[1], pos[2]-p2[2]]

        if ( abs(dot(pv0, plane0)) <= width and abs(dot(pv1, plane1)) <= width ):
            if ( abs(dot(pv0, plane2)) <= length and abs(dot(pv1, plane3)) <= length ):
                if ( !(abs(dot(pv0, normal)) <= abs(depth/2)) ):
                    return False
            else:
                return False

        else
            return False


        if Vt == vObox:
            indicesIn.append(index)
        else:
            indicesOut.append(index)

    return indicesIn, indicesOut



