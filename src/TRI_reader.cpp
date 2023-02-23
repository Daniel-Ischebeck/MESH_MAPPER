#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <vector>

/*Notes
Double vs float – template so user can choose speed vs accuracy?

//tri format, consider indexing, 0 or 1, general formatting
*/
#include "Point.hpp"
#include "Face.hpp"

int main()
{

    std::ifstream inputTRIFile("indexed_straight_dome.tri"); // straight_dome.tri
    std::string line;                                        // declare string to represent each line in the file

    std::vector<Point> listOfPoints;
    std::vector<Face> listOfFaces;

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
                // Point A = Point(1, 2.3, 4,2, 5.6);
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
            else if (currentLine > numPoints + 2)
            {
                std::istringstream iss(line);
                iss >> _faceIndex >> _aIndex >> _bIndex >> _cIndex;
                std::cout << "!Line: " << currentLine << "\n";
                std::cout << _faceIndex << " " << _aIndex << " " << _bIndex << " " << _cIndex << "\n";
                listOfFaces.at(_faceIndex) = Face(_faceIndex, _aIndex, _bIndex, _cIndex);
            }
            else
            {
                std::cout << "Other\n";
            }
            currentLine++;
        }
    }
    return 0;
}