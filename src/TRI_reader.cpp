#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <vector>

/*Notes
Double vs float – template so user can choose speed vs accuracy?

//tri format, consider indexing, 0 or 1, general formatting
*/



int main()
{

    std::ifstream inputTRIFile("indexed_straight_dome.tri");    //straight_dome.tri
    std::string line; // declare string to represent each line in the file


    int numCells, numVertPerCell, numAttrPC;

    std::vector<Point> listOfPoints;

    if (!inputTRIFile.is_open())
    {
        std::cout << "Failed to open file";
        return (-1);
    }

    int currentLine = 1;

    while (getline(inputTRIFile, line))
    {
        std::istringstream iss(line);

        int numPoints, numDimensions, numAttrPP;

        int _index;
        double _x, _y, _z;
        if (!line.empty())
        {
            if (currentLine == 1)
            {
                iss >> numPoints >> numDimensions >> numAttrPP;
                std::cout
                          << "Points: " << numPoints << "\n"
                          << "Dimensions: " << numDimensions << "\n"
                          << "Attributes: " << numAttrPP << "\n";

                listOfPoints.resize(numPoints);
                
            }
            if (iss>>_index>>_x>>_y>>_z){
                //formatting for points
                //listOfPoints.at(_index) = Point(_index,_x, _y, _z);
                Point A = Point(1, 2.3, 4,2, 5.6);
            }
            else{
                std::cout << "Line: " << currentLine << "\n";
            }
            currentLine++;
        }
        
    }
    return 0;
}