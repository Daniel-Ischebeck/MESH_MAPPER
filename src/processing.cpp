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
                 std::vector<double> &results,
                 std::vector<Eigen::MatrixXd> &listOfLocalBasis,
                 int scaleFlag

)
{
    // scale UV mesh appropriately
    // consider distance between selected pinned points and between pinned UV

    // //d = sqrt ( ((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2 )
    double totalAreaBefore = 0, totalAreaAfter = 0;
    double scaleFactor;
    if (scaleFlag == 1)
    {
        double distanceBetweenPinned = sqrt(((listOfPoints.at(pinnedVerticies(0)).get_x() - listOfPoints.at(pinnedVerticies(1)).get_x())) * ((listOfPoints.at(pinnedVerticies(0)).get_x() - listOfPoints.at(pinnedVerticies(1)).get_x())) +
                                            ((listOfPoints.at(pinnedVerticies(0)).get_y() - listOfPoints.at(pinnedVerticies(1)).get_y())) * ((listOfPoints.at(pinnedVerticies(0)).get_y() - listOfPoints.at(pinnedVerticies(1)).get_y())) +
                                            ((listOfPoints.at(pinnedVerticies(0)).get_z() - listOfPoints.at(pinnedVerticies(1)).get_z())) * ((listOfPoints.at(pinnedVerticies(0)).get_z() - listOfPoints.at(pinnedVerticies(1)).get_z())));

        // std::cout << "Distance between pinned coords in xyz: " << distanceBetweenPinned << "   Distance in UV: 1\n";
        for (int i = 0; i < listOfAreas.size(); i++)
        {
            totalAreaBefore = totalAreaBefore + abs(listOfAreas.at(i)); // problem with += maybe as std::vector
        }
        std::cout << "totalAreaBefore: " << totalAreaBefore << "\n";

        for (int i = 0; i < u_coords.rows(); i++)
        {
            // std::cout << i << " " << u_coords(i) << " " << v_coords(i) << "\n";
            outputPoints.push_back(Point(i, u_coords(i), v_coords(i), 0));
        }
        calcTriangleAreas(outputPoints, listOfFaces, outputAreas);

        for (int i = 0; i < outputAreas.size(); i++)
        {
            totalAreaAfter = totalAreaAfter + abs(outputAreas.at(i)); // problem with +=
        }
        std::cout << "totalAreaAfter: " << totalAreaAfter << "\n";

        scaleFactor = totalAreaBefore / totalAreaAfter;
        std::cout << "Scale factor: " << scaleFactor << "\n";

        // scale up mesh by area
        for (int i = 0; i < u_coords.rows(); i++)
        {
            u_coords(i) *= scaleFactor;
            v_coords(i) *= scaleFactor;
        }

        // // scale up mesh by distance
        // for (int i = 0; i < u_coords.rows(); i++)
        // {
        //     u_coords(i) *= distanceBetweenPinned;
        //     v_coords(i) *= distanceBetweenPinned;
        // }

        outputPoints.clear();
        for (int i = 0; i < u_coords.rows(); i++)
        {
            // std::cout << i << " " << u_coords(i) << " " << v_coords(i) << "\n";
            outputPoints.push_back(Point(i, u_coords(i), v_coords(i), 0));
        }
        calcTriangleAreas(outputPoints, listOfFaces, outputAreas);

        // outputUVfile(listOfFaces, faceMatrix, u_coords, v_coords, "Scaledoutput_UV.tri");
    }

    else
    {
        for (int i = 0; i < u_coords.rows(); i++)
        {
            // std::cout << i << " " << u_coords(i) << " " << v_coords(i) << "\n";
            outputPoints.push_back(Point(i, u_coords(i), v_coords(i), 0));
        }
        calcTriangleAreas(outputPoints, listOfFaces, outputAreas);
    }

    for (int i = 0; i < listOfFaces.size(); i++)
    {
        // std::cout << "Face: " << i << "  Area before: " << abs(listOfAreas.at(i)) << "   Area after:  " << outputAreas.at(i) << "\t\t";
        // results.at(i) = ((abs(listOfAreas.at(i)) - abs(outputAreas.at(i))) / abs(listOfAreas.at(i))) * 100;
        //#######################results.at(i) = (abs(listOfAreas.at(i)) / abs(outputAreas.at(i)));
        // std::cout << "Percentage change: " <<  results.at(i) << "\n";
    }

    // ################response direction test
    Eigen::MatrixXd TandB(3, 2);
    Eigen::MatrixXd pointMatrix(3, 2);
    Eigen::MatrixXd uvMatrix(2, 2);

    // ########################generating a local orthonomral basis for UV triangles
    std::vector<Eigen::MatrixXd> listOfUVBasis(listOfFaces.size());
    double averageEccentricity=0;

    for (int i = 0; i < listOfFaces.size(); i++)
    {

        double angle_between_AB_AC;

        Eigen::Vector3d vAB, vAC, vAB_norm, vAC_norm; // TODO: kind of already done above in temp, assignment there rather than redoing calc.

        vAB << (outputPoints.at(listOfFaces.at(i).get_bIndex()).get_x()) - (outputPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
            (outputPoints.at(listOfFaces.at(i).get_bIndex()).get_y()) - (outputPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
            (outputPoints.at(listOfFaces.at(i).get_bIndex()).get_z()) - (outputPoints.at(listOfFaces.at(i).get_aIndex()).get_z());

        vAC << (outputPoints.at(listOfFaces.at(i).get_cIndex()).get_x()) - (outputPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
            (outputPoints.at(listOfFaces.at(i).get_cIndex()).get_y()) - (outputPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
            (outputPoints.at(listOfFaces.at(i).get_cIndex()).get_z()) - (outputPoints.at(listOfFaces.at(i).get_aIndex()).get_z());

        // normalise
        double vAB_mag = sqrt(vAB.x() * vAB.x() + vAB.y() * vAB.y() + vAB.z() * vAB.z());
        vAB_norm << vAB.x() / vAB_mag, vAB.y() / vAB_mag, vAB.z() / vAB_mag;

        double vAC_mag = sqrt(vAC.x() * vAC.x() + vAC.y() * vAC.y() + vAC.z() * vAC.z());
        vAC_norm << vAC.x() / vAC_mag, vAC.y() / vAC_mag, vAC.z() / vAC_mag;

        double dotProduct = vAB_norm.x() * vAC_norm.x() + vAB_norm.y() * vAC_norm.y() + vAB_norm.z() * vAC_norm.z();

        angle_between_AB_AC = acos(dotProduct);

        double Xa, Xb, Xc;
        double Ya, Yb, Yc;

        Xa = 0; // A is at the origin
        Ya = 0;

        Xb = vAB.norm(); // vAB_norm.norm() / magX;
        Yb = 0;

        Xc = vAC.norm() * cos(angle_between_AB_AC); // vAC_mag   vAC_norm.norm()
        Yc = vAC.norm() * sin(angle_between_AB_AC); // vAC_norm.norm()

        Eigen::MatrixXd localBasis(3, 2);
        localBasis << Xa, Ya, Xb, Yb, Xc, Yc;
        listOfUVBasis.at(i) = localBasis;

        pointMatrix(0, 0) = listOfLocalBasis.at(i)(1, 0) - listOfLocalBasis.at(i)(0, 0);
        pointMatrix(0, 1) = listOfLocalBasis.at(i)(2, 0) - listOfLocalBasis.at(i)(0, 0);
        pointMatrix(1, 0) = listOfLocalBasis.at(i)(1, 1) - listOfLocalBasis.at(i)(0, 1);
        pointMatrix(1, 1) = listOfLocalBasis.at(i)(2, 1) - listOfLocalBasis.at(i)(0, 1);
        pointMatrix(2, 0) = 0;
        pointMatrix(2, 1) = 0;

        uvMatrix(0, 0) = localBasis(1, 0) - localBasis(0, 0);
        uvMatrix(0, 1) = localBasis(2, 0) - localBasis(0, 0);
        uvMatrix(1, 0) = localBasis(1, 1) - localBasis(0, 1);
        uvMatrix(1, 1) = localBasis(2, 1) - localBasis(0, 1);

        uvMatrix = uvMatrix.inverse();

        TandB = pointMatrix * uvMatrix;

        // std::cout << "\n\n################### reposne direction stuff\n\n";
        // std::cout << "pointMatrix: \n"
        //           << pointMatrix << "\n\n";
        // std::cout << "uvMatrix: \n"
        //           << uvMatrix << "\n\n";
        // std::cout << "T and B: \n"
        //           << TandB << "\n\n";

        Eigen::Vector3d T, B;
        T << TandB.col(0);
        B << TandB.col(1);

        std::cout<< "\n\ni:  " << i << "\n";
        std::cout << "length of T " << T.norm() << "\n";
        std::cout << "length of B " << B.norm() << "\n";
        std::cout << "T/B ratio: " << T.norm() / B.norm() << "\n";
        std::cout << "angle between T and B (degrees): " << std::atan2(T.cross(B).norm(), T.dot(B)) * (180 / 3.14159) << "\n";

        double aSquared, bSquared, eccentricity;
        aSquared = 0.5 * ((T.dot(T) + B.dot(B)) + sqrt((T.dot(T) - B.dot(B)) * (T.dot(T) - B.dot(B)) + 4 * ((T.dot(B)) * (T.dot(B)))));
        bSquared = 0.5 * ((T.dot(T) + B.dot(B)) - sqrt((T.dot(T) - B.dot(B)) * (T.dot(T) - B.dot(B)) + 4 * ((T.dot(B)) * (T.dot(B)))));

        eccentricity = sqrt(1 - (bSquared / aSquared));
        results.at(i) = eccentricity;
        averageEccentricity+=eccentricity;

        // std::cout << "dot: " << T.dot(T) << "   nomr*norm" << T.norm() * T.norm() << "\n";
        std::cout << "eccentricity: " << eccentricity << "\n";
    }
    std::cout << "Average eccentricity across whole mesh: " << averageEccentricity/listOfFaces.size() << "\n";
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