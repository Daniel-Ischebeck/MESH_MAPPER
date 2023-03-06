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

#include <igl/boundary_loop.h>

/*Notes
Double vs float – template so user can choose speed vs accuracy?

//tri format, consider indexing, 0 or 1, general formatting
*/
#include "Point.hpp"
#include "Face.hpp"

bool readFile(std::vector<Point> &listOfPoints,
              std::vector<Face> &listOfFaces,
              std::string filePath);

bool calcTriangleAreas(std::vector<Point> &listOfPoints,
                       std::vector<Face> &listOfFaces,
                       std::vector<double> &listOfAreas);

// briefly have a global varible for eigen matrix of faces
// igl boundary loop test


std::vector<std::vector<int>> faces;

int main()
{



    std::vector<Point> listOfPoints; // underscore just to check different
    std::vector<Face> listOfFaces;
    std::string filePath = "../files/indexed_straight_dome.tri";

    if (!readFile(listOfPoints, listOfFaces, filePath))
    {
        std::cout << "Exiting\n";
        return (-1);
    }

    std::vector<double> listOfAreas(listOfFaces.size());

    if (!calcTriangleAreas(listOfPoints, listOfFaces, listOfAreas))
    {
        std::cout << "Triangle calcualtion failed\n";
        return (-1);
    }


//
    Eigen::MatrixXi faceMatrix(listOfFaces.size(), 3);
    for (int i = 0; i < listOfFaces.size(); i++)
    {
        faceMatrix.row(i) = Eigen::VectorXi::Map(&faces[i][0], faces[i].size());
    }

    Eigen::VectorXi boundaryVerticies, pinnedVerticies(2, 1);
    igl::boundary_loop(faceMatrix, boundaryVerticies);

    pinnedVerticies(0) = boundaryVerticies(0);
    pinnedVerticies(1) = boundaryVerticies(boundaryVerticies.size() / 2);
    std::cout << "\n\nBoundary indexes\n"
              << boundaryVerticies << std::endl;
    std::cout << "\n\nPinned Verticies\n"
              << pinnedVerticies << std::endl;









    // assign to sparse matrix
    //  if vertex j belongs to triangle i, do some calc and place number, otherwise zero.
    // look at index numbers of triangle, and compare these to j, if theres a match, vertex j is part of triangle i

    std::cout << "\n\n\n\n";
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(listOfFaces.size(), listOfPoints.size());
    // would be sparse in reality, dense for now.

    Eigen::MatrixXd A_Mf1 = Eigen::MatrixXd::Zero(listOfFaces.size(), listOfPoints.size());
    Eigen::MatrixXd A_Mf2 = Eigen::MatrixXd::Zero(listOfFaces.size(), listOfPoints.size());

    for (int i = 0; i < listOfFaces.size(); i++)
    { // loop through the triangles

        for (int j = 0; j < listOfPoints.size(); j++)
        { // loop through the points

            if (listOfFaces.at(i).get_aIndex() == j) // Weight 1
            {
                // real part x3-x2
                M(i, j) = 1;
                A_Mf1(i, j) = (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x() -
                               listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x()) /
                              sqrt(abs(listOfAreas.at(i))); // W1/sqrt(dt)       //W1 = (x3-x2)+ i (y3-y2)
                                                            // because dividing by sqrt of negative, should this be in complex part?

                // complex part y3-y2
                A_Mf2(i, j) = (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y() -
                               listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y()) /
                              sqrt(abs(listOfAreas.at(i))); // W1/sqrt(dt)       //W1 = (x3-x2)+ i (y3-y2)
                                                            // because dividing by sqrt of negative, should this be in complex part?
            }

            if (listOfFaces.at(i).get_bIndex() == j) // Weight 2
            {
                // real part x1-x2
                M(i, j) = 2;
                A_Mf1(i, j) = (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x() -
                               listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x()) /
                              sqrt(abs(listOfAreas.at(i)));

                // complex part y1-y3
                A_Mf2(i, j) = (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y() -
                               listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y()) /
                              sqrt(abs(listOfAreas.at(i)));
            }

            if (listOfFaces.at(i).get_cIndex() == j) // Weight 3
            {
                // real part x2-x1
                M(i, j) = 3;
                A_Mf1(i, j) = (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x() -
                               listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()) /
                              sqrt(abs(listOfAreas.at(i)));

                // complex part y2-y1
                A_Mf2(i, j) = (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y() -
                               listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()) /
                              sqrt(abs(listOfAreas.at(i)));
            }
        }
    }

    std::cout << "\n\nFilled matrix M:\n\n";
    std::cout << M << "\n\n"
              << std::endl;

    // std::cout << "\n\nA_Mf1:\n\n";
    // std::cout << A_Mf1 << "\n\n"
    //           << std::endl;

    // std::cout << "\n\nA_Mf2:\n\n";
    // std::cout << A_Mf2 << "\n\n"
    //           << std::endl;

    // we want to copy pinned coordinate data to its own thing, remove these from the matrix
    // and resize the matrix

    // pinning verticies 4 and 6
    Eigen::MatrixXd b_Mp1 = Eigen::MatrixXd::Zero(listOfFaces.size(), 2);
    Eigen::MatrixXd b_Mp2 = Eigen::MatrixXd::Zero(listOfFaces.size(), 2);

    b_Mp1 << A_Mf1.block<10, 1>(0, pinnedVerticies(0)), A_Mf1.block<10, 1>(0, pinnedVerticies(1)); // concatting the two columns for pinned matrix
    // std::cout << "\n\nb_Mp1:\n\n";
    // std::cout << b_Mp1 << "\n\n"
    //           << std::endl;

    b_Mp2 << A_Mf2.block<10, 1>(0, pinnedVerticies(0)), A_Mf2.block<10, 1>(0, pinnedVerticies(1)); // concatting the two columns for pinned matrix
    // std::cout << "\n\nb_Mp2:\n\n";
    // std::cout << b_Mp2 << "\n\n"
    //           << std::endl;

    // now to remove the columns from matrix

    auto colToRemove = pinnedVerticies(0); // auto - was warning about possible loss of data (prev unsigned int)
    auto numRows = A_Mf1.rows();
    auto numCols = A_Mf1.cols() - 1;

    if (colToRemove < numCols)
    {
        A_Mf1.block(0, colToRemove, numRows, numCols - colToRemove) = A_Mf1.block(0, colToRemove + 1, numRows, numCols - colToRemove);
        A_Mf2.block(0, colToRemove, numRows, numCols - colToRemove) = A_Mf2.block(0, colToRemove + 1, numRows, numCols - colToRemove);
    }
    A_Mf1.conservativeResize(numRows, numCols);
    A_Mf2.conservativeResize(numRows, numCols);

    colToRemove = pinnedVerticies(1)-1; // as weve resized, to remove what was the 6, now 5
    numRows = A_Mf1.rows();
    numCols = A_Mf1.cols() - 1;
    if (colToRemove < numCols)
    {
        A_Mf1.block(0, colToRemove, numRows, numCols - colToRemove) = A_Mf1.block(0, colToRemove + 1, numRows, numCols - colToRemove);
        A_Mf2.block(0, colToRemove, numRows, numCols - colToRemove) = A_Mf2.block(0, colToRemove + 1, numRows, numCols - colToRemove);
    }
    A_Mf1.conservativeResize(numRows, numCols);
    A_Mf2.conservativeResize(numRows, numCols);

    // std::cout << "\n\nRemoved some stuff...A_Mf1:\n\n";
    // std::cout << A_Mf1 << "\n\n"
    //           << std::endl;

    // std::cout << "\n\nRemoved some stuff...A_Mf2:\n\n";
    // std::cout << A_Mf2 << "\n\n"
    //           << std::endl;

    // Form A, consists of 4 block matricies
    /*
    Mf1 -Mf2
    Mf2 Mf1
    */
    Eigen::MatrixXd A_top = Eigen::MatrixXd::Zero(listOfFaces.size(), listOfFaces.size() + 2);
    Eigen::MatrixXd A_bottom = Eigen::MatrixXd::Zero(listOfFaces.size(), listOfFaces.size() + 2);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * listOfFaces.size(), 2 * (listOfPoints.size() - 2));
    // A is 2(numFaces) x 2(numPoints-numPinnedpoints) matrix

    A_top << A_Mf1, -A_Mf2;
    A_bottom << A_Mf2, A_Mf1;

    A << A_top, A_bottom;

    std::cout << "#############  Final things   #########\n\n";

    std::cout << "A\n\n"
              << A << "\n\n"
              << std::endl;

    Eigen::MatrixXd Bmat_top = Eigen::MatrixXd::Zero(listOfFaces.size(), 4);
    Eigen::MatrixXd Bmat_bottom = Eigen::MatrixXd::Zero(listOfFaces.size(), 4);
    Eigen::MatrixXd Bmat = Eigen::MatrixXd::Zero(2 * listOfFaces.size(), 4);

    Bmat_top << b_Mp1, -b_Mp2;
    Bmat_bottom << b_Mp2, b_Mp1;

    Bmat << Bmat_top, Bmat_bottom;

    // b = - Bmat x PinnedVector          ??
    // do we assign values for Up1??

    std::cout << "Bmat:\n"
              << Bmat << "\n\n"
              << std::endl;

    Eigen::VectorXd pinnedUV(4, 1);
    pinnedUV << 0, 1, 0, 0; // choosing to pin in UV space, one coord at (1,1) other at (10,10)
    std::cout << "Pinned UV:\n\n"
              << pinnedUV << "\n\n"
              << std::endl;

    Eigen::MatrixXd RHS = -Bmat * pinnedUV; // should rhs be neg? Ax=b

    std::cout << "RHS\n"
              << RHS << "\n\n"
              << std::endl;

    // std::cout << "The least-squares solution is:\n"
    //     //<< A.template bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(Bmat) << std::endl;
    //      << A.colPivHouseholderQr().solve(RHS);

    Eigen::MatrixXd solution(listOfFaces.size() + 2, 1);
    solution = A.colPivHouseholderQr().solve(RHS);

    std::cout << "Solution:\n\n"
              << solution << std::endl;

    //  std::cout << "\n\n\n\n\nThe solution using normal equations is:\n"
    //  << (A.transpose() * A).ldlt().solve(A.transpose() * RHS) << std::endl;

    return 0;
}

bool readFile(std::vector<Point> &listOfPoints,
              std::vector<Face> &listOfFaces,
              std::string filePath)
{
    std::ifstream inputTRIFile(filePath); // straight_dome.tri
    std::string line;

    if (!inputTRIFile.is_open())
    {
        std::cout << "Failed to open file";
        return false;
    }

    int currentLine = 1;
    int numPoints, numDimensions, numAttrPP;
    int numFaces, numVertPF, numAttrPF;

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
                    << "Attributes: " << numAttrPP << "\n\n";

                listOfPoints.resize(numPoints);
            }
            else if ((iss >> _pointIndex >> _x >> _y >> _z) && currentLine < numPoints + 2)
            {
                // formatting for points
                // std::cout << "**Line: " << currentLine << "\n";
                // std::cout << _pointIndex << " " << _x << " " << _y << " " << _z << "\n";
                listOfPoints.at(_pointIndex) = Point(_pointIndex, _x, _y, _z);
            }
            else if ((currentLine == numPoints + 2))
            { //(iss>>numFaces>>numVertPF>>numAttrPF) &&
                // std::cout << "Reading faces\n";
                std::istringstream iss(line); // for some reason repeat this? clear string stream before?
                // int numFaces = 1, numVertPF = 2, numAttrPF = 3;
                //  std::cout << "?Line: " << currentLine << "\n";
                if (!(iss >> numFaces >> numVertPF >> numAttrPF))
                {
                    std::cout << "wrong formatting\n";
                }

                else
                {
                    std::cout
                        << "Faces: " << numFaces << "\n"
                        << "Dimensions: " << numVertPF << "\n"
                        << "Attributes: " << numAttrPF << "\n\n";

                    listOfFaces.resize(numFaces);
                    faces.resize(numFaces, std::vector<int>(numVertPF));    
                }
            }
            else if (currentLine > numPoints + 2 && currentLine < numPoints + numFaces + 3)
            // 21 = 1+numPoints(8)+1+numFaces(10)+1+1 - account for lines which give number of items, and line at end
            {
                std::istringstream iss(line);
                iss >> _faceIndex >> _aIndex >> _bIndex >> _cIndex;
                // std::cout << "!Line: " << currentLine << "\n";
                // std::cout << _faceIndex << " " << _aIndex << " " << _bIndex << " " << _cIndex << "\n";
                listOfFaces.at(_faceIndex) = Face(_faceIndex, _aIndex, _bIndex, _cIndex);

                // testing eigen matrix of faces for boundary loop
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
    }

    return true;
}

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