#include "FileIO.hpp"

bool outputUVfile(std::vector<Face> &listOfFaces,
                  Eigen::MatrixXi &faceMatrix,
                  Eigen::VectorXd &u_coords,
                  Eigen::VectorXd &v_coords,
                  std::string filePath) // testing face matrix
{

    std::ofstream outputUVfile;
    outputUVfile.open(filePath); // file name or file path

    outputUVfile << u_coords.rows() << " 2 0\n"; // uv output points will always have 2 dimensions and 0 attributes
    for (int i = 0; i < u_coords.rows(); i++)
    {
        outputUVfile << i << " " << u_coords(i) << " " << v_coords(i) << "\n";
    }

    outputUVfile << listOfFaces.size() << " 3 0\n"; // these attribute values may change later
    // outputUVfile << faceMatrix << "\n";
    for (int i = 0; i < listOfFaces.size(); i++)
    {
        outputUVfile << i << " " << listOfFaces.at(i).get_aIndex() << " " << listOfFaces.at(i).get_bIndex() << " " << listOfFaces.at(i).get_cIndex() << "\n";
    }

    outputUVfile << "Random\n";

    return true;
}

bool outputTRIfile(std::vector<Point> &listOfPoints,
                   std::vector<Face> &listOfFaces,
                   std::string filePath) // testing face matrix
{

    std::ofstream outputTRIfile;
    outputTRIfile.open(filePath); // file name or file path

    outputTRIfile << listOfPoints.size() << " 3 0\n"; // uv output points will always have 2 dimensions and 0 attributes
    for (int i = 0; i < listOfPoints.size(); i++)
    {
        outputTRIfile << listOfPoints.at(i).get_index() << " " << listOfPoints.at(i).get_x() << " "
                      << listOfPoints.at(i).get_y() << " "
                      << listOfPoints.at(i).get_z() << "\n";
        // outputTRIfile << i << " " << pointMatrix(i, 0) << " "
        //               << pointMatrix(i, 1) << " "
        //               << pointMatrix(i, 2) << "\n";
    }

    outputTRIfile << listOfFaces.size() << " 3 0\n"; // these attribute values may change later
    // outputUVfile << faceMatrix << "\n";
    for (int i = 0; i < listOfFaces.size(); i++)
    {
        outputTRIfile << i << " " << listOfFaces.at(i).get_aIndex() << " " << listOfFaces.at(i).get_bIndex() << " " << listOfFaces.at(i).get_cIndex() << "\n";
    }

    outputTRIfile << "Random\n";

    return true;
}

bool readFile(std::vector<Point> &listOfPoints,
              std::vector<Face> &listOfFaces,
              std::vector<std::vector<int>> &faces,
              std::vector<std::vector<double>> &points,
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

    int faceIndexCounter = 0;
    int pointIndexCounter = 0;

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
                points.resize(numPoints, std::vector<double>(numDimensions));
            }
            else if ((iss >> _pointIndex >> _x >> _y >> _z) && currentLine < numPoints + 2)
            {
                // formatting for points
                // std::cout << "**Line: " << currentLine << "\n";
                // std::cout << _pointIndex << " " << _x << " " << _y << " " << _z << "\n";
                listOfPoints.at(_pointIndex) = Point(_pointIndex, _x, _y, _z);

                points[pointIndexCounter][0] = _x;
                points[pointIndexCounter][1] = _y;
                points[pointIndexCounter][2] = _z;

                pointIndexCounter++;
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

                iss >> _faceIndex >> _aIndex >> _bIndex >> _cIndex; // there can be more attributes im not reading in here
                // std::cout << "!Line: " << currentLine << "\n";

                // faces arent indexed in phils example, use counter
                // std::cout << faceIndexCounter << " " << _aIndex << " " << _bIndex << " " << _cIndex << "\n";
                listOfFaces.at(faceIndexCounter) = Face(faceIndexCounter, _aIndex, _bIndex, _cIndex);

                // testing eigen matrix of faces for boundary loop
                faces[faceIndexCounter][0] = _aIndex;
                faces[faceIndexCounter][1] = _bIndex;
                faces[faceIndexCounter][2] = _cIndex;

                faceIndexCounter++;
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