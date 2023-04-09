#include "processing.hpp"

bool calcTriangleAreas(std::vector<Point> &listOfPoints, // make const - need to alter classes of Point and face
                       std::vector<Face> &listOfFaces,
                       std::vector<double> &listOfAreas)

{
    // loop through the list of faces, for each face get the points, then use these in calc
    for (int i = 0; i < listOfFaces.size(); i++)
    {
        // listOfAreas.at(i) = (x1 * y2 - y1 * x2) + (x2 * y3 - y2 * x3) + (x3 * y1 - y3 * x1);
        listOfAreas.at(i) = (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x() * listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y() -
                             listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y() * listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x()) +
                            (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x() * listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y() -
                             listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y() * listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x()) +
                            (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x() * listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y() -
                             listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y() * listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x());
    }
    return true;
}

Eigen::VectorXi find_triangles(int indexWereAfter,
                               std::vector<Face> &listOfFaces)

{
    Eigen::VectorXi indexVector(2, 1); // assuming at least two triangles include a given point
    int count = 0;
    for (int i = 0; i < listOfFaces.size(); i++)
    {

        if (listOfFaces.at(i).get_aIndex() == indexWereAfter ||
            listOfFaces.at(i).get_bIndex() == indexWereAfter ||
            listOfFaces.at(i).get_cIndex() == indexWereAfter)
        { // here 0 is the index of the point (0,0,0)
          // std::cout << "Triangle: " << i << " contains index " << indexWereAfter << "\n";
            // patch_listOfFaces.at(patch_faceIndex) = Face(patch_faceIndex, listOfFaces.at(i).get_aIndex(), listOfFaces.at(i).get_bIndex(), listOfFaces.at(i).get_cIndex());
            // patch_faceIndex++;
            if (count >= indexVector.rows())
            {
                indexVector.conservativeResize(count + 1, 1); // conservative resize to not lose data during reallocation
            }
            indexVector(count) = i;
            count++;
        }
    }
    // indexVector.conservativeResize();
    return indexVector;
}

bool removeTriangles(std::vector<Point> &listOfPoints,
                     std::vector<Face> &listOfFaces,
                     std::vector<std::vector<int>> &faces,
                     Eigen::MatrixXi &faceMatrix)
{
    // remove triangles,
    // listofpoints, listoffaces, faces
    // return numelementsadded value

    double a, b, c;
    int listOfFacesSizeBefore = listOfFaces.size();
    int numElementsAdded = 0;
    int rowCounter = 0;
    double epsilon = 0.0001f;

    for (int i = 0; i < listOfFacesSizeBefore; i++)
    {
        a = listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z();
        b = listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_z();
        c = listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_z();

        // std::cout << "a: " << a << "\n";
        //  For radome - its y coordinate for bottom faces
        //  a = listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y();
        //  b = listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y();
        //  c = listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y();

        // copy elements that are NOT part of the base into a new list which will now use
        // if (!((a == b) && (b == c)))    //comparing doubles can be problematic, use function
        if (!(compare_double(a, b, epsilon) && compare_double(b, c, epsilon))) //&& a < -2.14 && b < -2.14 && c < -2.14))
        {
            // std::cout << "Triangle: " << i << " is not on x-y plane\n";
            //   listOfFaces.erase(listOfFaces.begin() + i);
            //  numElementsRemoved++;
            numElementsAdded++;

            // newListOfFaces.at(rowCounter) = Face(i, listOfFaces.at(i).get_aIndex(),listOfFaces.at(i).get_bIndex(), listOfFaces.at(i).get_cIndex());
            listOfFaces.at(rowCounter) = Face(i, listOfFaces.at(i).get_aIndex(), listOfFaces.at(i).get_bIndex(), listOfFaces.at(i).get_cIndex());

            faces[rowCounter][0] = listOfFaces.at(i).get_aIndex();
            faces[rowCounter][1] = listOfFaces.at(i).get_bIndex();
            faces[rowCounter][2] = listOfFaces.at(i).get_cIndex();

            rowCounter++;
        }
    }

    listOfFaces.resize(numElementsAdded);
    faces.resize(numElementsAdded, std::vector<int>(3)); // always 3  verticies per row
    faceMatrix.resize(listOfFaces.size(), 3);

    // Eigen::MatrixXi faceMatrix(listOfFaces.size(), 3);
    for (int i = 0; i < listOfFaces.size(); i++)
    {
        faceMatrix.row(i) = Eigen::VectorXi::Map(&faces[i][0], faces[i].size());
    }

    return true;
}

bool compare_double(double first, double second, double epsilon)

{
    if (fabs(first - second) < epsilon)
        return true; // they are same
    return false;    // they are not same
}

bool compareEdges(std::vector<Edge> &first, std::vector<Edge> &second)
{
    // return std::find_first_of(first.begin(), first.end(),
    //                           second.begin(), second.end()) != first.end();

    if (std::find(first.begin(), first.end(), second.at(0)) != first.end() ||
        std::find(first.begin(), first.end(), second.at(1)) != first.end() ||
        std::find(first.begin(), first.end(), second.at(2)) != first.end())
    {
        return true;
    }
    return false;
}

Edge whichEdgeShared(std::vector<Edge> &first, std::vector<Edge> &second, int &indexOfSharedEdge_second, int &indexOfSharedEdge_first)
{
    std::vector<Edge>::iterator it0;

    it0 = std::find(first.begin(), first.end(), second.at(0));

    if (it0 != first.end())
    {
        indexOfSharedEdge_second = 0;
        indexOfSharedEdge_first = it0 - first.begin();
        return second.at(0);
    }

    std::vector<Edge>::iterator it1; // reset/clear the iterator and reuse ? TODO
    it1 = std::find(first.begin(), first.end(), second.at(1));
    if (it1 != first.end())
    {
        indexOfSharedEdge_second = 1;
        indexOfSharedEdge_first = it1 - first.begin();
        return second.at(1);
    }

    std::vector<Edge>::iterator it2; // reset/clear the iterator and reuse ? TODO
    it2 = std::find(first.begin(), first.end(), second.at(2));
    if (it2 != first.end())
    {
        indexOfSharedEdge_second = 2;
        indexOfSharedEdge_first = it2 - first.begin();
        return second.at(2);
    }

    else
    {
        indexOfSharedEdge_second = -1;
        indexOfSharedEdge_first = -1;
        return Edge(-1, -1, -1);
    }
}

bool postProcess(std::vector<double> &outputAreas,
                 std::vector<double> &listOfAreas,
                 std::vector<Point> &outputPoints,
                 std::vector<Face> &listOfFaces,
                 Eigen::VectorXd &u_coords,
                 Eigen::VectorXd &v_coords,
                 Eigen::MatrixXi &faceMatrix,
                 std::vector<Point> &listOfPoints,
                 Eigen::VectorXi &pinnedVerticies,
                 int scaleFlag

)
{
    // scale UV mesh appropriately
    // consider distance between selected pinned points and between pinned UV

    // //d = sqrt ( ((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2 )

    if (scaleFlag == 1)
    {
        double distanceBetweenPinned = sqrt(((listOfPoints.at(pinnedVerticies(0)).get_x() - listOfPoints.at(pinnedVerticies(1)).get_x())) * ((listOfPoints.at(pinnedVerticies(0)).get_x() - listOfPoints.at(pinnedVerticies(1)).get_x())) +
                                            ((listOfPoints.at(pinnedVerticies(0)).get_y() - listOfPoints.at(pinnedVerticies(1)).get_y())) * ((listOfPoints.at(pinnedVerticies(0)).get_y() - listOfPoints.at(pinnedVerticies(1)).get_y())) +
                                            ((listOfPoints.at(pinnedVerticies(0)).get_z() - listOfPoints.at(pinnedVerticies(1)).get_z())) * ((listOfPoints.at(pinnedVerticies(0)).get_z() - listOfPoints.at(pinnedVerticies(1)).get_z())));

        std::cout << "Distance between pinned coords in xyz: " << distanceBetweenPinned << "   Distance in UV: 1\n";

        // scale up mesh by distance
        for (int i = 0; i < u_coords.rows(); i++)
        {
            u_coords(i) *= distanceBetweenPinned;
            v_coords(i) *= distanceBetweenPinned;
        }

        // outputUVfile(listOfFaces, faceMatrix, u_coords, v_coords, "Scaledoutput_UV.tri");
    }

    for (int i = 0; i < u_coords.rows(); i++)
    {
        std::cout << i << " " << u_coords(i) << " " << v_coords(i) << "\n";
        outputPoints.push_back(Point(i, u_coords(i), v_coords(i), 0));
    }
    calcTriangleAreas(outputPoints, listOfFaces, outputAreas);

    for (int i = 0; i < listOfFaces.size(); i++)
    {
        std::cout << "Face: " << i << "  Area before: " << abs(listOfAreas.at(i)) << "   Area after:  " << outputAreas.at(i) << "\t\t";
        std::cout << "Percentage change: " << ((abs(listOfAreas.at(i)) - outputAreas.at(i)) / listOfAreas.at(i)) * 100 << "\n";
    }

    return true;
}


bool rotateModel(std::vector<Point> &listOfPoints,
                 Eigen::MatrixXd &pointMatrix,
                 std::vector<std::vector<double>> &points,
                 char axis)
{
    for (int i = 0; i < listOfPoints.size(); i++)
    {
        pointMatrix.row(i) = Eigen::VectorXd::Map(&points[i][0], points[i].size());
    }
    // std::cout << pointMatrix << "\n\n";

    // Eigen::AngleAxisd rotate(3.14159, Eigen::Vector3d(0, 2.15, 0)); // M_PI
    // set all to zero, axis switch_case selects what to change
    Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitZ());

    switch (axis)
    {
    case 'x':
        // pitchAngle.angle() = 3.14159 / 2;
        pitchAngle.angle() = (3.14159 / 2) - 0.35; // want to rotate wing less so its 'flatter'
        break;

    case 'y':
        yawAngle.angle() = 3.14159 / 2;
        break;

    case 'z':
        rollAngle.angle() = 3.14159 / 2;
        break;
    }

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();

    pointMatrix = pointMatrix * rotationMatrix;

    // loop through list of points and change the values, leave the indexes
    // gonna require set functions for point
    for (int i = 0; i < listOfPoints.size(); i++) // update list of points after rotation
    {
        listOfPoints.at(i).set_x(pointMatrix(i, 0));
        listOfPoints.at(i).set_y(pointMatrix(i, 1));
        listOfPoints.at(i).set_z(pointMatrix(i, 2));
    }

    return true;
}