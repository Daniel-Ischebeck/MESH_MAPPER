#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <vector>

// include eigen dense
// later eigen sparse

// eigen is header only - link in cmake
// #include "Eigen"
// #include <Eigen/Core>
//

#include <Eigen/Dense>

/*Notes
Double vs float – template so user can choose speed vs accuracy?

//tri format, consider indexing, 0 or 1, general formatting
*/
#include "Point.hpp"
#include "Face.hpp"

int main()
{

    std::ifstream inputTRIFile("../files/indexed_straight_dome.tri"); // straight_dome.tri
    std::string line;                                                 // declare string to represent each line in the file

    std::vector<Point> listOfPoints;
    std::vector<Face> listOfFaces;

    std::vector<std::vector<double>> points;
    int numPointRows = 8; // wpuld actually be read from file, manual for now
    int numPointCols = 3;
    points.resize(numPointRows, std::vector<double>(numPointCols));

    std::vector<std::vector<int>> faces;
    int numFaceRows = 10; // wpuld actually be read from file, manual for now
    int numFaceCols = 3;
    faces.resize(numFaceRows, std::vector<int>(numFaceCols));

    if (!inputTRIFile.is_open())
    {
        std::cout << "Failed to open file";
        return (-1);
    }

    int currentLine = 1;
    int numPoints, numDimensions, numAttrPP;

    // int numFaces = 1, numVertPF = 2, numAttrPF = 3;

    while (getline(inputTRIFile, line))
    {
        std::istringstream iss(line);

        int _pointIndex;
        double _x, _y, _z;

        int _faceIndex;
        int _aIndex, _bIndex, _cIndex;

        if (!line.empty())
        {
            // std::cout << "Current Line: " << currentLine << "\n";
            if (currentLine == 1)
            {
                iss >> numPoints >> numDimensions >> numAttrPP;
                std::cout
                    << "Points: " << numPoints << "\n"
                    << "Dimensions: " << numDimensions << "\n"
                    << "Attributes: " << numAttrPP << "\n";

                listOfPoints.resize(numPoints);
            }
            else if ((iss >> _pointIndex >> _x >> _y >> _z) && currentLine < numPoints + 2)
            {
                // formatting for points
                std::cout << "**Line: " << currentLine << "\n";
                std::cout << _pointIndex << " " << _x << " " << _y << " " << _z << "\n";
                listOfPoints.at(_pointIndex) = Point(_pointIndex, _x, _y, _z);

                points[_pointIndex][0] = _x;
                points[_pointIndex][1] = _y;
                points[_pointIndex][2] = _z;
                /*
                point[i][j] where i is num row, j is num col
                point index gives the row number
                point[0][0] = x1
                point[0][1] = y1
                point[0][2] = z2

                point[1][0] = x2
                point[1][1] = y2
                point[1][2] = z2
                */
            }
            else if ((currentLine == numPoints + 2))
            { //(iss>>numFaces>>numVertPF>>numAttrPF) &&
                // std::cout << "Reading faces\n";
                std::istringstream iss(line); // for some reason repeat this? clear string stream before?
                int numFaces = 1, numVertPF = 2, numAttrPF = 3;
                std::cout << "?Line: " << currentLine << "\n";
                if (!(iss >> numFaces >> numVertPF >> numAttrPF))
                {
                    std::cout << "wrong formatting\n";
                    std::cout << numFaces << " " << numVertPF << " " << numAttrPF << "\n";
                }

                else
                {
                    std::cout
                        << "Faces: " << numFaces << "\n"
                        << "Dimensions: " << numVertPF << "\n"
                        << "Attributes: " << numAttrPF << "\n";

                    listOfFaces.resize(10);
                }
            }
            else if (currentLine > numPoints + 2 && currentLine < 21)
            {
                std::istringstream iss(line);
                iss >> _faceIndex >> _aIndex >> _bIndex >> _cIndex;
                std::cout << "!Line: " << currentLine << "\n";
                std::cout << _faceIndex << " " << _aIndex << " " << _bIndex << " " << _cIndex << "\n";
                listOfFaces.at(_faceIndex) = Face(_faceIndex, _aIndex, _bIndex, _cIndex);

                faces[_faceIndex][0] = _aIndex;
                faces[_faceIndex][1] = _bIndex;
                faces[_faceIndex][2] = _cIndex;
            }
            else
            {
                std::cout << "Other\n";
            }
            currentLine++;
        }

        // we now want to convert list to a matrix
    }

    /*
        Eigen::MatrixXd pointMatrix;
        pointMatrix.resize(numPoints, 3);

        std::cout << "\nPoints:\n";
        for (Point aPoint : listOfPoints)
        {
            std::cout << aPoint.get_x() << "\n";
        }

        for (int i = 0; i < numPoints; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                // pointMatrix(i,j) =
            }
        }
        */

    std::cout << "Eigen test\n";
    Eigen::MatrixXd pointMatrix(numPointRows, numPointCols);

    for (int i = 0; i < numPointRows; i++)
    {
        pointMatrix.row(i) = Eigen::VectorXd::Map(&points[i][0], points[i].size());
    }

    Eigen::MatrixXi faceMatrix(numFaceRows, numFaceCols);
    for (int i = 0; i < numFaceRows; i++)
    {
        faceMatrix.row(i) = Eigen::VectorXi::Map(&faces[i][0], faces[i].size());
    }

    std::cout << "Eigen point matrix:\n";
    std::cout << pointMatrix << "\n\n"
              << std::endl;

    std::cout << "Eigen face matrix:\n";
    std::cout << faceMatrix << "\n\n"
              << std::endl;

    // area of triangle
    // very inefficent quick calc
    std::vector<double> listOfAreas;
    listOfAreas.resize(numFaceRows);
    // loop through the list of faces, for each face get the points, then use these in calc
    double x1, y1, x2, y2, x3, y3;
    double dt;

    for (int i = 0; i < listOfFaces.size(); i++)
    {
        x1 = listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x();
        y1 = listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y();
        x2 = listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x();
        y2 = listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y();
        x3 = listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x();
        y3 = listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y();

        std::cout << "i: " << i << "\tx1: " << x1 << "\n";
        std::cout << "i: " << i << "\ty1: " << y1 << "\n";
        std::cout << "i: " << i << "\tx2: " << x2 << "\n";
        std::cout << "i: " << i << "\ty2: " << y2 << "\n";
        std::cout << "i: " << i << "\tx3: " << x3 << "\n";
        std::cout << "i: " << i << "\ty3: " << y3 << "\n";

        dt = (x1 * y2 - y1 * x2) + (x2 * y3 - y2 * x3) + (x3 * y1 - y3 * x1);
        listOfAreas.at(i) = dt;
        std::cout << "~~~~~~~";
        std::cout << "i: " << i << "\tArea: " << dt << "\n\n\n";
    }

    // assign to sparse matrix
    //  if vertex j belongs to triangle i, do some calc and place number, otherwise zero.
    // look at index numbers of triangle, and compare these to j, if theres a match, vertex j is part of triangle i

    std::cout << "\n\n\n\n";
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(10,8);
    //would be sparse in reality, dense for now.


    for (int i = 0; i < listOfFaces.size(); i++)
    { // loop through the triangles

        for (int j = 0; j < listOfPoints.size(); j++)
        { // loop through the points

            if (listOfFaces.at(i).get_aIndex() == j ||
                listOfFaces.at(i).get_bIndex() == j ||
                listOfFaces.at(i).get_cIndex() == j)
            { // if the first vertex of the triangle is equal to 0
                std::cout << "Triangle " << i  << " contains vertex " << j << "\n";
                M(i,j) = 1;
            }
        }
    }

    std::cout << "\n\nFilled matrix M:\n\n";
    std:: cout << M << "\n\n" << std::endl;

    return 0;
}