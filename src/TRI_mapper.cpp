#include "TRI_mapper.hpp"
#include "FileIO.hpp"

#include <algorithm> //for testing if indicies is in listofindicies for preventing duplication in patch creation



int main()
{

    std::vector<Point> OBJlistOfPoints;
    std::vector<Face> OBJlistOfFaces;
    std::string OBJfilePath = "lead_edge_v2.obj";    //wing_v3.obj
    readOBJ(OBJlistOfPoints, OBJlistOfFaces, OBJfilePath );
    outputTRIfile(OBJlistOfPoints, OBJlistOfFaces, "lead_edge_v2.tri");

    std::chrono::steady_clock::time_point overallBegin = std::chrono::steady_clock::now();

    std::vector<std::vector<int>> faces;
    std::vector<std::vector<double>> points;

    std::vector<Point> listOfPoints;
    std::vector<Face> listOfFaces;
    // std::string filePath = "../files/indexed_straight_dome.tri";
    // std::string filePath = "../files/part_sphere_low.tri"; // part_sphere_high "../files/double_dome.tri" aircraft_wing.tri
    std::string filePath = "wingv3.tri"; // hex_mesh //high_modifiedTRI  //actual_part_sphere  //wingv3.tri

    if (!readFile(listOfPoints, listOfFaces, faces, points, filePath))
    {
        std::cout << "Exiting\n";
        return (-1);
    }

    // we now need to preprocess before we can perform mapping
    // e.g. currently not topologically a disk

    // ########################
    // Rotate the radome
    // first need to have a point matrix, and then rotate this

    Eigen::MatrixXd pointMatrix(listOfPoints.size(), 3);

    Eigen::MatrixXi faceMatrix(listOfFaces.size(), 3);

    outputTRIfile(listOfPoints, listOfFaces, "pre.tri");
    // removeTriangles(listOfPoints, listOfFaces, faces, faceMatrix);
    // rotateModel(listOfPoints, pointMatrix, points, 'x'); // rotate the model so its orientated sensibly, y for double_dome, x for patch_antenna, x for wing
    // outputTRIfile(listOfPoints, listOfFaces, "modifiedTRI.tri");

    //  double dome
    //  we want to be able to select a seed point, and grow a patch from there
    //  obvious choice is start at 0,0,0
    //  currently my data strucutre means triangles dont know anything about their neighbours

    // start at 0,0,0
    // add all triangles that contain that point
    // starting at first triangle added, choose one of its other points, add all triangles that contain that point
    // beware this is all doubles so comparison function needeed
    // std::vector<Point> patch_listOfPoints; // underscore just to check different
    std::vector<Face> patch_listOfFaces;

    std::vector<int> listOfIndicies(100000);

    patch_listOfFaces.resize(100000); // guess, we can resize later
    // patch_listOfPoints.resize(listOfPoints.size() / 2);

    int patch_faceIndex = 0;

    // for (int i = 0; i < listOfFaces.size(); i++)
    // {
    //     // start point
    //     if (listOfFaces.at(i).get_aIndex() == 0 || listOfFaces.at(i).get_bIndex() == 0 || listOfFaces.at(i).get_cIndex() == 0) // 7,9,26,   index _ is good start point for patch_antenna, aircraft wing 0?
    //     {                                                                                                                               // here 0 is the index of the point (0,0,0) - for double dome
    //         // std::cout << "Triangle: " << i << " contains 0,0,0\n";
    //         patch_listOfFaces.at(patch_faceIndex) = Face(0, listOfFaces.at(i).get_aIndex(), listOfFaces.at(i).get_bIndex(), listOfFaces.at(i).get_cIndex());
    //         patch_faceIndex++;
    //     }
    // }

    // patch_antenna start point around (30.1,41.6, -36.5) //closer look (30.2,44, -36.5)

    //     // patch_listOfFaces.at(patch_faceIndex) = Face(patch_faceIndex, listOfFaces.at(195).get_aIndex(), listOfFaces.at(195).get_bIndex(), listOfFaces.at(195).get_cIndex());
    //     // listOfIndicies.push_back(patch_faceIndex);
    //     // patch_faceIndex++;
    //     // patch_listOfFaces.at()

    //     // weve got out first triangles aroung (0,0,0)
    //     // Triangle 0 contains points 0,1,7744

    //     // index = 0;
    //     // find_triangles(patch_listOfFaces.at(index).getbIndex()) - returns a vector of the indexes of the triangles in listOfFaces
    //     // i.e. find me the triangles that contain index 1
    //     // add these triangles to the patch_listOfFaces

    //     // for (int i = 0; i < patch_faceIndex; i++)
    //     // {
    //     //     std::cout << "i: " << patch_listOfFaces.at(i).get_faceIndex() << "\n";
    //     // }

    // Patch selection
    // #################################################
    //      // plant seed triangle - for dome
    //      // index 0 to allow comparison
    //      patch_listOfFaces.at(patch_faceIndex) = Face(0, listOfFaces.at(0).get_aIndex(), listOfFaces.at(0).get_bIndex(), listOfFaces.at(0).get_cIndex());
    //      patch_faceIndex++;

    //     int indexA, indexB, indexC;
    //     int j = 0;

    //     // std::vector<int> sharePoint;
    //     std::map<int, Eigen::VectorXi> sharePoint;

    // label:
    //     for (j; j < 2; j++)
    //     {
    //         // std::cout<<"j: " << j << "\n";

    //         indexB = patch_listOfFaces.at(j).get_bIndex();

    //         if (std::find(listOfIndicies.begin(), listOfIndicies.end(), indexB) != listOfIndicies.end())
    //         {
    //             // v contains x
    //             // j++;
    //             //  std::cout << "indexB: " <<  indexB << "\n";
    //             // goto label;
    //         }
    //         else
    //         {
    //             // std::cout << j << " B\n";
    //             listOfIndicies.push_back(indexB);
    //             Eigen::VectorXi resultVectorB = find_triangles(indexB, listOfFaces);
    //             std::cout << "j: " << j << "   result B: \n"
    //                     << resultVectorB << "\n";
    //             sharePoint[j] = resultVectorB;

    //             for (int i = 0; i < resultVectorB.rows(); i++)
    //             {
    //                 // only want to adda  new patch if it hasnt already been included before
    //                 Face theFace = Face(0, listOfFaces.at(resultVectorB(i)).get_aIndex(), listOfFaces.at(resultVectorB(i)).get_bIndex(), listOfFaces.at(resultVectorB(i)).get_cIndex());
    //                 if (std::find(patch_listOfFaces.begin(), patch_listOfFaces.end(), theFace) != patch_listOfFaces.end())
    //                 {
    //                     // patch_listOfFaces.at(patch_faceIndex) = Face(0, listOfFaces.at(resultVectorB(i)).get_aIndex(), listOfFaces.at(resultVectorB(i)).get_bIndex(), listOfFaces.at(resultVectorB(i)).get_cIndex());
    //                     // j++;
    //                     // goto label;
    //                     // break;
    //                 }
    //                 else
    //                 {
    //                     // if ((listOfPoints.at(theFace.get_aIndex()).get_z() > 1) || (listOfPoints.at(theFace.get_bIndex()).get_z() > 1) || (listOfPoints.at(theFace.get_bIndex()).get_z() > 1))
    //                     // {
    //                     //     goto done;
    //                     //     // break;
    //                     // }
    //                     if (1) // listOfZNormals.at(i) < 0)
    //                     {
    //                         patch_listOfFaces.at(patch_faceIndex) = theFace;
    //                         patch_faceIndex++;
    //                         std::cout << "B\tZ normal: " << listOfZNormals.at(i) << "\n";
    //                         // std::cout << "result vector B i: " << resultVectorB(i) << "\n";
    //                     }
    //                     // patch_listOfFaces.at(patch_faceIndex) = theFace;
    //                     // patch_faceIndex++;
    //                 }
    //                 // patch_listOfFaces.at(patch_faceIndex) = Face(0, listOfFaces.at(resultVectorB(i)).get_aIndex(), listOfFaces.at(resultVectorB(i)).get_bIndex(), listOfFaces.at(resultVectorB(i)).get_cIndex());
    //                 //  patch_faceIndex++;
    //             }
    //             j++;
    //             goto label;
    //         }

    //         indexC = patch_listOfFaces.at(j).get_cIndex();

    //         if (std::find(listOfIndicies.begin(), listOfIndicies.end(), indexC) != listOfIndicies.end())
    //         {
    //             // v contains x
    //             // j++;
    //             //  std::cout << "indexB: " <<  indexB << "\n";
    //             // goto label;
    //         }
    //         else
    //         {
    //             // std::cout << j << " B\n";
    //             listOfIndicies.push_back(indexC);
    //             Eigen::VectorXi resultVectorC = find_triangles(indexC, listOfFaces);
    //             std::cout << "j: " << j << "   result C: \n"
    //                     << resultVectorC << "\n";
    //             sharePoint[j] = resultVectorC;

    //             for (int i = 0; i < resultVectorC.rows(); i++)
    //             {
    //                 // only want to adda  new patch if it hasnt already been included before
    //                 Face theFace = Face(0, listOfFaces.at(resultVectorC(i)).get_aIndex(), listOfFaces.at(resultVectorC(i)).get_bIndex(), listOfFaces.at(resultVectorC(i)).get_cIndex());
    //                 if (std::find(patch_listOfFaces.begin(), patch_listOfFaces.end(), theFace) != patch_listOfFaces.end())
    //                 {
    //                     // patch_listOfFaces.at(patch_faceIndex) = Face(0, listOfFaces.at(resultVectorB(i)).get_aIndex(), listOfFaces.at(resultVectorB(i)).get_bIndex(), listOfFaces.at(resultVectorB(i)).get_cIndex());
    //                     // j++;
    //                     // goto label;
    //                     // break;
    //                 }
    //                 else
    //                 {

    //                     // if ((listOfPoints.at(theFace.get_aIndex()).get_z() > 1) || (listOfPoints.at(theFace.get_bIndex()).get_z() > 1) || (listOfPoints.at(theFace.get_bIndex()).get_z() > 1))
    //                     // {
    //                     //     goto done;
    //                     //     // break;
    //                     // }
    //                     if (1) // listOfZNormals.at(i) < 0)
    //                     {
    //                         patch_listOfFaces.at(patch_faceIndex) = theFace;
    //                         patch_faceIndex++;
    //                         std::cout << "C\tZ normal: " << listOfZNormals.at(i) << "\n";
    //                     }
    //                     // patch_listOfFaces.at(patch_faceIndex) = theFace;
    //                     // patch_faceIndex++;
    //                 }
    //                 // patch_listOfFaces.at(patch_faceIndex) = Face(0, listOfFaces.at(resultVectorB(i)).get_aIndex(), listOfFaces.at(resultVectorB(i)).get_bIndex(), listOfFaces.at(resultVectorB(i)).get_cIndex());
    //                 //  patch_faceIndex++;
    //             }
    //             j++;
    //             goto label;
    //         }

    //         indexA = patch_listOfFaces.at(j).get_aIndex();

    //         if (std::find(listOfIndicies.begin(), listOfIndicies.end(), indexA) != listOfIndicies.end())
    //         {
    //             // v contains x
    //             // j++;
    //             //  std::cout << "indexB: " <<  indexB << "\n";
    //             // goto label;
    //         }
    //         else
    //         {
    //             // std::cout << j << " B\n";
    //             listOfIndicies.push_back(indexA);
    //             Eigen::VectorXi resultVectorA = find_triangles(indexA, listOfFaces);
    //             std::cout << "j: " << j << "   result A: \n"
    //                     << resultVectorA << "\n";
    //             sharePoint[j] = resultVectorA;

    //             for (int i = 0; i < resultVectorA.rows(); i++)
    //             {
    //                 // only want to adda  new patch if it hasnt already been included before
    //                 Face theFace = Face(0, listOfFaces.at(resultVectorA(i)).get_aIndex(), listOfFaces.at(resultVectorA(i)).get_bIndex(), listOfFaces.at(resultVectorA(i)).get_cIndex());
    //                 if (std::find(patch_listOfFaces.begin(), patch_listOfFaces.end(), theFace) != patch_listOfFaces.end())
    //                 {
    //                     // patch_listOfFaces.at(patch_faceIndex) = Face(0, listOfFaces.at(resultVectorB(i)).get_aIndex(), listOfFaces.at(resultVectorB(i)).get_bIndex(), listOfFaces.at(resultVectorB(i)).get_cIndex());
    //                     // j++;
    //                     // goto label;
    //                     // break;
    //                 }
    //                 else
    //                 {
    //                     // if ((listOfPoints.at(theFace.get_aIndex()).get_z() > 1) || (listOfPoints.at(theFace.get_bIndex()).get_z() > 1) || (listOfPoints.at(theFace.get_bIndex()).get_z() > 1))
    //                     // {
    //                     //     goto done;
    //                     //     // break;
    //                     // }
    //                     if (1) // listOfZNormals.at(i) < 0)
    //                     {
    //                         patch_listOfFaces.at(patch_faceIndex) = theFace;
    //                         patch_faceIndex++;
    //                         std::cout << "A\tZ normal: " << listOfZNormals.at(i) << "\n";
    //                     }
    //                     // patch_listOfFaces.at(patch_faceIndex) = theFace;
    //                     // patch_faceIndex++;
    //                 }
    //                 // patch_listOfFaces.at(patch_faceIndex) = Face(0, listOfFaces.at(resultVectorB(i)).get_aIndex(), listOfFaces.at(resultVectorB(i)).get_bIndex(), listOfFaces.at(resultVectorB(i)).get_cIndex());
    //                 //  patch_faceIndex++;
    //             }
    //             j++;
    //             goto label;
    //         }

    //         // std::cout << "patch_faceIndex: " << patch_faceIndex << "\n";
    //     }

    // done:

    //     // testing shared point map
    //     for (std::map<int, Eigen::VectorXi>::iterator miter = sharePoint.begin(); miter != sharePoint.end(); ++miter)
    //     {
    //         std::cout << "\nKey = " << (*miter).first << "\nvalue =\n"
    //                 << (*miter).second;
    //     }
    //     std::cout << "\n\n";
    //     //     //     // currently we now want to work with patch_list_of_faces not the normal one
    //     //     //     // also as we havent removed triangles from this model, we havent generated the face matrix yet
    //     //     //     // ideally we would have options that we could select, i.e. preprocess -Y/N , remove tri - Y/N, select patch - Y/N etc/

    //     patch_listOfFaces.resize(patch_faceIndex);

    /// #################    end of patch selection

    // /###############################map faces needed if no remove triangles
    for (int i = 0; i < listOfFaces.size(); i++)
    {
        faces[i][0] = listOfFaces.at(i).get_aIndex();
        faces[i][1] = listOfFaces.at(i).get_bIndex();
        faces[i][2] = listOfFaces.at(i).get_cIndex();
    }
    faces.resize(listOfFaces.size(), std::vector<int>(3));
    faceMatrix.resize(listOfFaces.size(), 3);

    for (int i = 0; i < listOfFaces.size(); i++)
    {
        faceMatrix.row(i) = Eigen::VectorXi::Map(&faces[i][0], faces[i].size());
    }
    outputTRIfile(listOfPoints, listOfFaces, "modifiedTRI.tri");

    // ################################################

    //     // outputTRIfile(listOfPoints, patch_listOfFaces, "Selectedpatch.tri");

    //     // listOfFaces.swap(patch_listOfFaces);
    //     listOfFaces = patch_listOfFaces;
    //     // #################################
    //     // we want to remove unused points from both the list of points
    //     // go through list of faces and check indexes, create a list of indexes
    //     // problem is size of this when we have non consecutive ordering,

    //     std::vector<Point> temp_listOfPoints;
    //     int aIndex, bIndex, cIndex;
    //     int count = 0;
    //     // generaous guess of size for points from faces, can resize later
    //     // temp_listOfPoints.resize(listOfFaces.size()*3);

    //     for (int i = 0; i < listOfFaces.size(); i++)
    //     {
    //         aIndex = listOfFaces.at(i).get_aIndex();
    //         bIndex = listOfFaces.at(i).get_bIndex();
    //         cIndex = listOfFaces.at(i).get_cIndex();

    //         if ((std::find(temp_listOfPoints.begin(), temp_listOfPoints.end(), listOfPoints.at(aIndex)) != temp_listOfPoints.end()))
    //         {
    //             //     temp_listOfPoints.push_back(listOfPoints.at(listOfFaces.at(i).get_aIndex()));
    //             //     // std::cout << "i: " << i << "\taindex: " << listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_index() << "\n";
    //             //     count++;
    //         }
    //         else
    //         {
    //             temp_listOfPoints.push_back(listOfPoints.at(listOfFaces.at(i).get_aIndex()));
    //             // std::cout << "i: " << i << "\taindex: " << listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_index() << "\n";
    //             count++;
    //         }
    //         if ((std::find(temp_listOfPoints.begin(), temp_listOfPoints.end(), listOfPoints.at(bIndex)) != temp_listOfPoints.end()))
    //         {
    //             // temp_listOfPoints.push_back(listOfPoints.at(listOfFaces.at(i).get_bIndex()));
    //             // // std::cout << "i: " << i << "\tbindex: " << listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_index() << "\n";
    //             // count++;
    //         }
    //         else
    //         {
    //             temp_listOfPoints.push_back(listOfPoints.at(listOfFaces.at(i).get_bIndex()));
    //             // std::cout << "i: " << i << "\tbindex: " << listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_index() << "\n";
    //             count++;
    //         }
    //         if ((std::find(temp_listOfPoints.begin(), temp_listOfPoints.end(), listOfPoints.at(cIndex)) != temp_listOfPoints.end()))
    //         {
    //             // temp_listOfPoints.push_back(listOfPoints.at(listOfFaces.at(i).get_cIndex()));
    //             // // std::cout << "i: " << i << "\tcindex: " << listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_index() << "\n";
    //             // count++;
    //         }
    //         else
    //         {
    //             temp_listOfPoints.push_back(listOfPoints.at(listOfFaces.at(i).get_cIndex()));
    //             // std::cout << "i: " << i << "\tcindex: " << listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_index() << "\n";
    //             count++;
    //         }
    //     }
    //     // temp_listOfPoints.resize(count);
    //     std::cout << "temp size: " << temp_listOfPoints.size() << "\n";

    //     for (int i = 0; i < temp_listOfPoints.size(); i++)
    //     {
    //         std::cout << temp_listOfPoints.at(i).get_index() << " " << temp_listOfPoints.at(i).get_x() << " "
    //                 << temp_listOfPoints.at(i).get_y() << " "
    //                 << temp_listOfPoints.at(i).get_z() << "\n";
    //     }

    //     std::cout << "List of point size  (before): " << listOfPoints.size() << "\n";
    //     // listOfPoints.swap(temp_listOfPoints);
    //     // listOfPoints = temp_listOfPoints;

    //     std::cout << "List of point size  (after): " << listOfPoints.size() << "\n";

    //     // for(int i=0; i<listOfFaces.size(); i++)

    //     // for(int i=0; i<temp_listOfPoints.size();i++)
    //     // {
    //     //     std::cout << "i: " << temp_listOfPoints.
    //     // }

    //     // for(int i=0; i<10; i++)  //testing swap
    //     // {
    //     //     std::cout << "i: " << i << " lof: " << listOfFaces.at(i).get_bIndex() << "\n";
    //     // }

    //     // #################prep face matrix after patch selection
    //     for (int i = 0; i < listOfFaces.size(); i++)
    //     {
    //         faces[i][0] = listOfFaces.at(i).get_aIndex();
    //         faces[i][1] = listOfFaces.at(i).get_bIndex();
    //         faces[i][2] = listOfFaces.at(i).get_cIndex();
    //     }

    //     faces.resize(listOfFaces.size(), std::vector<int>(3));
    //     faceMatrix.resize(listOfFaces.size(), 3);

    //     for (int i = 0; i < listOfFaces.size(); i++)
    //     {
    //         faceMatrix.row(i) = Eigen::VectorXi::Map(&faces[i][0], faces[i].size());
    //     }

    //     // std::map<int, std::vector<int>> neighbours;
    //     std::vector<Eigen::VectorXi> theResults;

    //     std::vector<int> simulatedChange = {};

    //     for (std::map<int, Eigen::VectorXi>::iterator miter = sharePoint.begin(); miter != sharePoint.end(); ++miter)
    //     {
    //         std::cout << "\nKey = " << (*miter).first << "\nvalue =\n"
    //                 << (*miter).second;
    //         // std::cout<< "\nTest access first " << (*miter).second(1)  << "\n";
    //         Eigen::VectorXi theResult = (*miter).second;
    //         theResults.push_back(theResult);
    //         // for(auto sharedPoint:theResult)
    //         // {
    //         //     std::cout < "A shared point " << sharedPoint << "\n";
    //         // }
    //     }

    //     std::vector<Face> winding_listOfFaces;

    //     // make all triangles follow the winding direction of triangle 0
    //     // the actual order doesnt matter as ling as mesh is consistent

    //     // add a beginning triangle to list
    //     // find adjacent triangles to current triangle, check windinding direction. if it needs to be flipped, do so and add to list

    //     winding_listOfFaces.push_back(listOfFaces.at(0)); // seed
    //     listOfFaces.at(0).set_winding(1);

    //     std::cout << "\n\nPre any changes to ordering\n";
    //     for (int i = 0; i < listOfFaces.size(); i++)
    //     {
    //         std::cout << i << " " << listOfFaces.at(i).get_aIndex() << " " << listOfFaces.at(i).get_bIndex() << " " << listOfFaces.at(i).get_cIndex() << "\twinding " << listOfFaces.at(i).get_winding() << "\n";
    //     }
    //     int iterations = 0;
    // again:
    //     // #####################
    //     Edge anotherSharedEdge(-1, -1, -1);
    //     Edge anotherFailEdge(-1, -1, -1);
    //     std::map<int, std::vector<int>> neighbours;
    //     std::cout << "\nBrute\n";
    //     std::vector<int> shared; // = {8,7,6};
    //     // shared.resize(1);
    //     int indexShared_secondTri = -1;
    //     int indexShared_firstTri = -1;
    //     std::vector<Face> new_listOfFaces;
    //     // new_listOfFaces.push_back(listOfFaces.at(0)); // seed
    //     new_listOfFaces = listOfFaces;

    //     std::vector<int> visitedBefore;
    //     // visitedBefore.push_back(0);

    //     std::vector<std::pair<int, int>> thePairs;

    //     std::cout << "\n\n new list of faces\n";
    //     for (int i = 0; i < new_listOfFaces.size(); i++)
    //     {
    //         std::cout << i << " " << new_listOfFaces.at(i).get_aIndex() << " " << new_listOfFaces.at(i).get_bIndex() << " " << new_listOfFaces.at(i).get_cIndex() << "\twinding " << new_listOfFaces.at(i).get_winding() << "\n";
    //     }

    //     for (int i = 0; i < listOfFaces.size(); i++)
    //     {
    //         std::vector<int> shared;
    //         for (int j = 0; j < listOfFaces.size(); j++)
    //         {
    //             if ((compareEdges(listOfFaces.at(i).get_faceEdges(), listOfFaces.at(j).get_faceEdges())) && i != j)
    //             {

    //                 shared.push_back(j);
    //                 anotherSharedEdge = whichEdgeShared(listOfFaces.at(i).get_faceEdges(), listOfFaces.at(j).get_faceEdges(), indexShared_secondTri, indexShared_firstTri);

    //                 if (listOfFaces.at(i).get_faceEdges().at(indexShared_firstTri).get_index1() == listOfFaces.at(j).get_faceEdges().at(indexShared_secondTri).get_index1() &&
    //                     listOfFaces.at(i).get_faceEdges().at(indexShared_firstTri).get_index2() == listOfFaces.at(j).get_faceEdges().at(indexShared_secondTri).get_index2())
    //                 {

    //                     // if(std::find(visitedBefore.begin(), visitedBefore.end(), j ) != visitedBefore.end()) {
    //                     //     // change the winding of the latter
    //                     //     std::cout << "NEW!!\t";
    //                     //     new_listOfFaces.at(j) = Face(j, listOfFaces.at(j).get_cIndex(), listOfFaces.at(j).get_bIndex(), listOfFaces.at(j).get_aIndex());
    //                     // }
    //                     // visitedBefore.push_back(j);
    //                     std::pair<int, int> current = {i, j};

    //                     if (std::find(thePairs.begin(), thePairs.end(), current) != thePairs.end())
    //                     {
    //                     }
    //                     else
    //                     {
    //                         std::cout << "New combo";
    //                         if (iterations == 0)
    //                         {
    //                             new_listOfFaces.at(j) = Face(j, listOfFaces.at(j).get_cIndex(), listOfFaces.at(j).get_bIndex(), listOfFaces.at(j).get_aIndex());
    //                             new_listOfFaces.at(j).set_winding(1);
    //                         }
    //                         if (iterations == 1)
    //                         {
    //                             new_listOfFaces.at(j) = Face(j, listOfFaces.at(j).get_bIndex(), listOfFaces.at(j).get_aIndex(), listOfFaces.at(j).get_cIndex());
    //                             new_listOfFaces.at(j).set_winding(1);
    //                         }
    //                         thePairs.push_back(std::make_pair(i, j));
    //                         thePairs.push_back(std::make_pair(j, i)); // and the reverse ordering of the pair
    //                     }

    //                     std::cout << "\n################Triangle " << i << " and triangle " << j << " share an edge\t\t";
    //                     std::cout << "################Edge: " << anotherSharedEdge.get_index1() << ", " << anotherSharedEdge.get_index2() << "\tIndex first: " << indexShared_firstTri << " second: " << indexShared_secondTri << "\n\n";
    //                 }
    //                 else
    //                 {
    //                     std::cout << "Triangle " << i << " and triangle " << j << " share an edge\t\t";
    //                     std::cout << "Edge: " << anotherSharedEdge.get_index1() << ", " << anotherSharedEdge.get_index2() << "\tIndex first: " << indexShared_firstTri << " second: " << indexShared_secondTri << "\n";
    //                 }
    //                 // listOfFaces.at(i).get_faceEdges().at(indexShared).
    //                 // neighbours.insert({i, shared});
    //             }
    //         }
    //         neighbours.insert({i, shared});
    //         // shared.clear();
    //         //  shared.resize(1);
    //     }

    //     std::cout << "\n\n update new list of faces\n";
    //     for (int i = 0; i < new_listOfFaces.size(); i++)
    //     {
    //         std::cout << i << " " << new_listOfFaces.at(i).get_aIndex() << " " << new_listOfFaces.at(i).get_bIndex() << " " << new_listOfFaces.at(i).get_cIndex() << "\twinding " << new_listOfFaces.at(i).get_winding() << "\n";
    //     }

    //     outputTRIfile(listOfPoints, new_listOfFaces, "firstpostBrute.tri");

    //     listOfFaces = new_listOfFaces;
    //     if (iterations == 0)
    //     {
    //         std::cout << "first iteration complete\n";
    //         outputTRIfile(listOfPoints, new_listOfFaces, "secondpostBrute.tri");
    //         iterations++;
    //         goto again;
    //     }
    // ############################################

    // rather than having to search all triangles
    // we only need to search triangles we know that share a point with
    std::cout << "testing: " << listOfFaces.at(5).get_aIndex() << "\n";
    /// #############
    std::cout << "List: " << listOfFaces.size() << "\tMatrix: " << faceMatrix.rows() << "\tpoints: " << listOfPoints.size() << "\n\n";
    std::cout << "\nPre areas\n";
    std::vector<double> listOfAreas(listOfFaces.size());
    std::cout << "?\n";
    if (!calcTriangleAreas(listOfPoints, listOfFaces, listOfAreas))
    {
        std::cout << "Triangle calcualtion failed\n";
        return (-1);
    }

    // outputTRIfile(listOfPoints, patch_listOfFaces, "Selectedpatch.tri");
    std::cout << "1\n";
    Eigen::VectorXi boundaryVerticies, pinnedVerticies(2, 1);
    igl::boundary_loop(faceMatrix, boundaryVerticies);

    pinnedVerticies(0) = boundaryVerticies(0);
    pinnedVerticies(1) = boundaryVerticies(boundaryVerticies.size() / 2);
    std::cout << "\n\nBoundary indexes\n"
              << boundaryVerticies << std::endl;
    std::cout << "\n\nPinned Verticies\n"
              << pinnedVerticies << std::endl;

    std::cout << "\n\n\n\n";

    // // ####################### matrix declarations
    Eigen::SparseMatrix<double> A(2 * listOfFaces.size(), 2 * (listOfPoints.size() - 2));
    Eigen::MatrixXd RHS(2 * (listOfFaces.size()), 1);
    Eigen::VectorXd pinnedUV(4, 1);
    std::vector< Eigen::MatrixXd > listOfLocalBasis(listOfFaces.size());


    prepMatricies(listOfPoints, listOfFaces, listOfAreas, listOfLocalBasis, pinnedVerticies, pinnedUV, A, RHS);

    Eigen::VectorXd solution(2 * (listOfPoints.size() - 2), 1);

    std::cout << "Calculating...\n";

    // solution = A.colPivHouseholderQr().solve(RHS);

    // SOlver
    // Matrix needs to be compressed before it can be used with solver
    A.makeCompressed();
    

    double nonZeros = A.nonZeros();
    double numElements = A.rows()*A.cols(); 
    double nonZeroPercentage = (nonZeros/numElements) *100;  //cabove compoentns can be combined here, initialisation?
    std::cout << "Num non zeros: "  << A.nonZeros() << "\n";
    std::cout << "Non-zero precentage: " << nonZeroPercentage << "%\n";

    
    std::cout << "Dimensions (post compression) A: " << A.rows() << " x " << A.cols() << "\n";
    // std::ofstream A_sparseMatrixfile;
    // A_sparseMatrixfile.open("A_sparse.txt");
    // A_sparseMatrixfile << Eigen::MatrixXd(A);

    // Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<double>> lscg;

    std::chrono::steady_clock::time_point begin_compute = std::chrono::steady_clock::now();
    // solver.compute(A);
    lscg.compute(A);
    std::chrono::steady_clock::time_point end_compute = std::chrono::steady_clock::now();
    std::cout << "Time for compute (sec) = " << std::chrono::duration_cast<std::chrono::microseconds>(end_compute - begin_compute).count() / 1000000.0 << std::endl;

    // if (solver.info() != Eigen::Success)
    // {
    //     // decomposition failed
    //     return -1;
    // }

    std::chrono::steady_clock::time_point begin_solve = std::chrono::steady_clock::now();
    // solution = solver.solve(RHS);
    solution = lscg.solve(RHS);
    std::chrono::steady_clock::time_point end_solve = std::chrono::steady_clock::now();
    // if (solver.info() != Eigen::Success) //should this be solution###########################
    // {
    //     // solving failed
    //     return -1;
    // }

    // std::cout << "Least-Squares Solution (U coords, then V):\n\n"
    //           << solution << std::endl;

    std::cout << "Time for solver (sec) = " << std::chrono::duration_cast<std::chrono::microseconds>(end_solve - begin_solve).count() / 1000000.0 << std::endl;
    
    
    std::cout << "#iterations:     " << lscg.iterations() << std::endl;
    std::cout << "estimated error: " << lscg.error() << std::endl;
    
    //  std::cout << "\n\n\n\n\nThe solution using normal equations is:\n"
    //  << (A.transpose() * A).ldlt().solve(A.transpose() * RHS) << std::endl;

    Eigen::VectorXd u_coords(listOfPoints.size(), 1);
    Eigen::VectorXd v_coords(listOfPoints.size(), 1);

    prepSolutionOutput(u_coords, v_coords, pinnedVerticies, solution, pinnedUV, listOfPoints.size());

    // determing output triangle areas
    std::vector<double> outputAreas(listOfAreas.size());
    std::vector<Point> outputPoints;
    std::vector<double> results(listOfAreas.size());

    int attributeFlag = 0;
    outputUVfile(listOfFaces, faceMatrix, u_coords, v_coords, "output_UV.tri", attributeFlag, results);

    int scaleFlag = 0;
    postProcess(outputAreas, listOfAreas, outputPoints, listOfFaces, u_coords, v_coords, faceMatrix, listOfPoints, pinnedVerticies, results, listOfLocalBasis, scaleFlag);

    attributeFlag = 1;
    outputUVfile(listOfFaces, faceMatrix, u_coords, v_coords, "withAttributes_output_UV.tri", attributeFlag, results);

    std::chrono::steady_clock::time_point overallEnd = std::chrono::steady_clock::now();
    std::cout << "Time for overall execution (sec) = " << std::chrono::duration_cast<std::chrono::microseconds>(overallEnd - overallBegin).count() / 1000000.0 << std::endl;

    return 0;
}

bool prepMatricies(std::vector<Point> &listOfPoints,
                   std::vector<Face> &listOfFaces,
                   std::vector<double> &listOfAreas,
                   std::vector< Eigen::MatrixXd > &listOfLocalBasis,
                   Eigen::VectorXi &pinnedVerticies,
                   Eigen::VectorXd &pinnedUV,
                   Eigen::SparseMatrix<double> &A,
                   Eigen::MatrixXd &RHS)
{

    Eigen::SparseMatrix<double> M(listOfFaces.size(), listOfPoints.size());
    Eigen::SparseMatrix<double> A_Mf1(listOfFaces.size(), listOfPoints.size());
    Eigen::SparseMatrix<double> A_Mf2(listOfFaces.size(), listOfPoints.size());

    Eigen::MatrixXd b_Mp1 = Eigen::MatrixXd(listOfFaces.size(), 2);
    Eigen::MatrixXd b_Mp2 = Eigen::MatrixXd(listOfFaces.size(), 2);

    Eigen::SparseMatrix<double> A_Mf1_p1, A_Mf1_p2, A_Mf1_p3, A_Mf1_temp, A_Mf1_final;
    Eigen::SparseMatrix<double> A_Mf2_p1, A_Mf2_p2, A_Mf2_p3, A_Mf2_temp, A_Mf2_final;

    Eigen::SparseMatrix<double> A_top(listOfFaces.size(), 2 * (listOfPoints.size() - 2));
    Eigen::SparseMatrix<double> A_bottom(listOfFaces.size(), 2 * (listOfPoints.size() - 2));
    // Eigen::SparseMatrix<double> A(2 * listOfFaces.size(), 2 * (listOfPoints.size() - 2));

    Eigen::MatrixXd Bmat_top = Eigen::MatrixXd(listOfFaces.size(), 4); // 4 as two pinned verticeis
    Eigen::MatrixXd Bmat_bottom = Eigen::MatrixXd(listOfFaces.size(), 4);
    Eigen::MatrixXd Bmat = Eigen::MatrixXd(2 * listOfFaces.size(), 4);

    // Eigen::MatrixXd RHS(2 * (listOfFaces.size()), 1); // this is the only matrix we will use later

    // prepMatricies(Eigen::MatrixXd RHS)
    //

    // assign to sparse matrix
    // if vertex j belongs to triangle i, do some calc and place number, otherwise zero.
    // look at index numbers of triangle, and compare these to j, if theres a match, vertex j is part of triangle i

    for (int i = 0; i < listOfFaces.size(); i++)
    { // loop through the triangles

        for (int j = 0; j < listOfPoints.size(); j++)
        { // loop through the points

            if (listOfFaces.at(i).get_aIndex() == j) // Weight 1
            {
                // std::cout << "i: " << i << " j: " << j << "    Weight 1\n";
                //  real part x3-x2
                //  M(i, j) = 1;
                M.insert(i, j) = 1; // for sparse we need .insert()
                                    // A_Mf1.insert(i, j) = (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x() -
                                    //                       listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x()) /
                                    //                      sqrt(abs(listOfAreas.at(i))); // W1/sqrt(dt)       //W1 = (x3-x2)+ i (y3-y2)
                                    //                                                    // because dividing by sqrt of negative, should this be in complex part?

                // complex part y3-y2
                // A_Mf2.insert(i, j) = (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y() -
                //                       listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y()) /
                //                      sqrt(abs(listOfAreas.at(i))); // W1/sqrt(dt)       //W1 = (x3-x2)+ i (y3-y2)
                //                                                 // because dividing by sqrt of negative, should this be in complex part?

                Eigen::Vector3d X, n, Y;
                Eigen::Vector3d temp((listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
                                     (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
                                     (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_z()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z()));

                X = temp / temp.norm();

                // std::cout << "X\n"
                //           << X << "\n";

                temp << (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
                    (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
                    (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_z()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z());

                n = X.cross(temp) / ((X.cross(temp)).norm());

                Y = n.cross(X);
                // std::cout << "i: " << i << "\tn\n"
                //           << n << "\n";
                // std::cout << "i: " << i << " norm() " << n.norm() << "\n";

                double angle_between_AB_AC;

                Eigen::Vector3d vAB, vAC, vAB_norm, vAC_norm; // TODO: kind of already done above in temp, assignment there rather than redoing calc.

                vAB << (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
                    (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
                    (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_z()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z());

                vAC << (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
                    (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
                    (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_z()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z());

                // normalise
                double vAB_mag = sqrt(vAB.x() * vAB.x() + vAB.y() * vAB.y() + vAB.z() * vAB.z());
                vAB_norm << vAB.x() / vAB_mag, vAB.y() / vAB_mag, vAB.z() / vAB_mag;

                double vAC_mag = sqrt(vAC.x() * vAC.x() + vAC.y() * vAC.y() + vAC.z() * vAC.z());
                vAC_norm << vAC.x() / vAC_mag, vAC.y() / vAC_mag, vAC.z() / vAC_mag;

                double dotProduct = vAB_norm.x() * vAC_norm.x() + vAB_norm.y() * vAC_norm.y() + vAB_norm.z() * vAC_norm.z();

                angle_between_AB_AC = acos(dotProduct);

                // std::cout << "Angle (radians): " << angle_between_AB_AC << "\n"
                //           << "Angle (degrees): " << angle_between_AB_AC * (180 / 3.14159) << "\n";

                // Now to determine the values of the points in our new coordiante system
                // A is at the origin
                // B will be some scaling of our basis vector X
                // C will have components of both basis vectors X and Y.

                double Xa, Xb, Xc;
                double Ya, Yb, Yc;

                // Xa = 0; // A is at the origin
                // Ya = 0;

                // Xb = 1; // vAB_norm.norm()/magX; //1; // vAB_norm.norm()/magX;
                // Yb = 0;

                // Xc = vAC_norm.norm() * cos(angle_between_AB_AC); // vAC_mag
                // Yc = vAC_norm.norm() * sin(angle_between_AB_AC);
                Xa = 0; // A is at the origin
                Ya = 0;

                Xb = vAB.norm(); // vAB_norm.norm() / magX;
                Yb = 0;

                Xc = vAC.norm() * cos(angle_between_AB_AC); // vAC_mag   vAC_norm.norm()
                Yc = vAC.norm() * sin(angle_between_AB_AC); // vAC_norm.norm()
                // std::cout << "Results:\n"
                //           << "Xa: " << Xa << "\tYa: " << Ya << "\n"
                //           << "Xb: " << Xb << "\tYb: " << Yb << "\n"
                //           << "Xc: " << Xc << "\tYc: " << Yc << "\n";

                // real part Xc-Xb   - note these are from wight eauation not top graident in triangle equation
                A_Mf1.insert(i, j) = (Xc - Xb) / sqrt(abs(listOfAreas.at(i)));

                // complex part yc-yb
                A_Mf2.insert(i, j) = (Yc - Yb) / sqrt(abs(listOfAreas.at(i)));

                Eigen::MatrixXd localBasis(3,2);
                localBasis << Xa, Ya, Xb, Yb, Xc, Yc;
                // std::cout <<  "i: " << i << "   local basis 1 \n" << localBasis << "\n\n";
                listOfLocalBasis.at(i) = localBasis;
            }

            if (listOfFaces.at(i).get_bIndex() == j) // Weight 2
            {
                // std::cout << "i: " << i << " j: " << j << "    Weight 2\n";
                M.insert(i, j) = 2;

                Eigen::Vector3d X, n, Y;
                Eigen::Vector3d temp((listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
                                     (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
                                     (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_z()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z()));

                X = temp / temp.norm();

                // std::cout << "X\n"
                //           << X << "\n";

                temp << (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
                    (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
                    (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_z()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z());

                n = X.cross(temp) / ((X.cross(temp)).norm());

                Y = n.cross(X);
                // std::cout << "i: " << i << "\tn\n"
                //           << n << "\n";
                // std::cout << "i: " << i << " norm() " << n.norm() << "\n";
                double angle_between_AB_AC;

                Eigen::Vector3d vAB, vAC, vAB_norm, vAC_norm; // TODO: kind of already done above in temp, assignment there rather than redoing calc.

                vAB << (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
                    (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
                    (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_z()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z());

                vAC << (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
                    (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
                    (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_z()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z());

                // normalise
                double vAB_mag = sqrt(vAB.x() * vAB.x() + vAB.y() * vAB.y() + vAB.z() * vAB.z());
                vAB_norm << vAB.x() / vAB_mag, vAB.y() / vAB_mag, vAB.z() / vAB_mag;

                double vAC_mag = sqrt(vAC.x() * vAC.x() + vAC.y() * vAC.y() + vAC.z() * vAC.z());
                vAC_norm << vAC.x() / vAC_mag, vAC.y() / vAC_mag, vAC.z() / vAC_mag;

                double dotProduct = vAB_norm.x() * vAC_norm.x() + vAB_norm.y() * vAC_norm.y() + vAB_norm.z() * vAC_norm.z();

                angle_between_AB_AC = acos(dotProduct);

                // std::cout << "Angle (radians): " << angle_between_AB_AC << "\n"
                //           << "Angle (degrees): " << angle_between_AB_AC * (180 / 3.14159) << "\n";

                // Now to determine the values of the points in our new coordiante system
                // A is at the origin
                // B will be some scaling of our basis vector X
                // C will have components of both basis vectors X and Y.

                double Xa, Xb, Xc;
                double Ya, Yb, Yc;

                Xa = 0; // A is at the origin
                Ya = 0;

                Xb = vAB.norm(); // vAB_norm.norm() / magX;
                Yb = 0;

                Xc = vAC.norm() * cos(angle_between_AB_AC); // vAC_mag   vAC_norm.norm()
                Yc = vAC.norm() * sin(angle_between_AB_AC); // vAC_norm.norm()

                // real part Xc-Xb   - note these are from wight eauation not top graident in triangle equation
                A_Mf1.insert(i, j) = (Xa - Xc) / sqrt(abs(listOfAreas.at(i)));

                // complex part yc-yb
                A_Mf2.insert(i, j) = (Ya - Yc) / sqrt(abs(listOfAreas.at(i)));
                
                Eigen::MatrixXd localBasis(3,2);
                localBasis << Xa, Ya, 
                              Xb, Yb, 
                              Xc, Yc;
                // std::cout <<  "i: " << i <<  "   local basis 2 \n" << localBasis << "\n\n";
                listOfLocalBasis.at(i) = localBasis;
            }

            if (listOfFaces.at(i).get_cIndex() == j) // Weight 3
            {
                // std::cout << "i: " << i << " j: " << j << "     Weight 3\n";
                M.insert(i, j) = 3;

                Eigen::Vector3d X, n, Y;
                Eigen::Vector3d temp((listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
                                     (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
                                     (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_z()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z()));

                X = temp / temp.norm();

                // std::cout << "X\n"
                //           << X << "\n";

                temp << (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
                    (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
                    (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_z()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z());

                n = X.cross(temp) / ((X.cross(temp)).norm());

                Y = n.cross(X);
                // std::cout << "i: " << i << "\tn\n"
                //           << n << "\n";
                // std::cout << "i: " << i << " norm() " << n.norm() << "\n";
                double angle_between_AB_AC;

                Eigen::Vector3d vAB, vAC, vAB_norm, vAC_norm; // TODO: kind of already done above in temp, assignment there rather than redoing calc.

                vAB << (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
                    (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
                    (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_z()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z());

                vAC << (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()),
                    (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()),
                    (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_z()) - (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z());

                // normalise
                double vAB_mag = sqrt(vAB.x() * vAB.x() + vAB.y() * vAB.y() + vAB.z() * vAB.z());
                vAB_norm << vAB.x() / vAB_mag, vAB.y() / vAB_mag, vAB.z() / vAB_mag;

                double vAC_mag = sqrt(vAC.x() * vAC.x() + vAC.y() * vAC.y() + vAC.z() * vAC.z());
                vAC_norm << vAC.x() / vAC_mag, vAC.y() / vAC_mag, vAC.z() / vAC_mag;

                double dotProduct = vAB_norm.x() * vAC_norm.x() + vAB_norm.y() * vAC_norm.y() + vAB_norm.z() * vAC_norm.z();

                angle_between_AB_AC = acos(dotProduct);

                // std::cout << "Angle (radians): " << angle_between_AB_AC << "\n"
                //           << "Angle (degrees): " << angle_between_AB_AC * (180 / 3.14159) << "\n";

                // Now to determine the values of the points in our new coordiante system
                // A is at the origin
                // B will be some scaling of our basis vector X
                // C will have components of both basis vectors X and Y.
                double Xa, Xb, Xc;
                double Ya, Yb, Yc;

                Xa = 0; // A is at the origin
                Ya = 0;

                Xb = vAB.norm(); // vAB_norm.norm() / magX;
                Yb = 0;

                Xc = vAC.norm() * cos(angle_between_AB_AC); // vAC_mag   vAC_norm.norm()
                Yc = vAC.norm() * sin(angle_between_AB_AC); // vAC_norm.norm()

                // real part Xc-Xb   - note these are from wight eauation not top graident in triangle equation
                A_Mf1.insert(i, j) = (Xb - Xa) / sqrt(abs(listOfAreas.at(i)));

                // complex part yc-yb
                A_Mf2.insert(i, j) = (Yb - Ya) / sqrt(abs(listOfAreas.at(i)));

                Eigen::MatrixXd localBasis(3,2);
                localBasis << Xa, Ya, Xb, Yb, Xc, Yc;
                // std::cout << "i: " << i << "   local basis 3 \n" << localBasis << "\n\n";
                listOfLocalBasis.at(i) = localBasis;
            }
        }
    }

    std::cout << "Dimensions of M: " << M.rows() << " x " << M.cols() << "\n";

    // std::cout << "\n\nFilled matrix M:\n\n";
    // std::cout << Eigen::MatrixXd(M) << "\n\n"
    //           << std::endl; // for nice formatting of sparse convert to dense

    // std::cout << "\n\nA_Mf1:\n\n";
    // std::cout << Eigen::MatrixXd(A_Mf1) << "\n\n"
    //           << std::endl;

    // std::cout << "\n\nA_Mf2:\n\n";
    // std::cout << Eigen::MatrixXd(A_Mf2) << "\n\n"
    //           << std::endl;

    // we want to copy pinned coordinate data to its own thing, remove these from the matrix
    // and resize the matrix

    // pinning verticies
    // Eigen::MatrixXd b_Mp1 = Eigen::MatrixXd::Zero(listOfFaces.size(), 2); // dont think need these to be zero initalised, remove later, just working on something else rn
    // Eigen::MatrixXd b_Mp2 = Eigen::MatrixXd::Zero(listOfFaces.size(), 2);
    int testing = listOfFaces.size();
    // template paramters need to be known at compile time
    // b_Mp1 << A_Mf1.block<testing, 1>(0, pinnedVerticies(0)), A_Mf1.block<testing, 1>(0, pinnedVerticies(1)); // concatting the two columns for pinned matrix

    // You can't just concatenate sparse matricies
    // b_Mp1 << A_Mf1.col(pinnedVerticies(0)), A_Mf1.col(pinnedVerticies(1));

    // when extracting from sparse and copying to eigen need to first construct vector
    b_Mp1 << Eigen::VectorXd(A_Mf1.col(pinnedVerticies(0))), Eigen::VectorXd(A_Mf1.col(pinnedVerticies(1)));
    // std::cout << "\n\nb_Mp1:\n\n";
    // std::cout << b_Mp1 << "\n\n"
    //           << std::endl;

    // b_Mp2 << A_Mf2.block<10, 1>(0, pinnedVerticies(0)), A_Mf2.block<10, 1>(0, pinnedVerticies(1)); // concatting the two columns for pinned matrix
    b_Mp2 << Eigen::VectorXd(A_Mf2.col(pinnedVerticies(0))), Eigen::VectorXd(A_Mf2.col(pinnedVerticies(1)));
    // std::cout << "\n\nb_Mp2:\n\n";
    // std::cout << b_Mp2 << "\n\n"
    //           << std::endl;

    // // now to remove the columns from matrix

    auto colToRemove = pinnedVerticies(0); // auto - was warning about possible loss of data (prev unsigned int)
    auto numRows = A_Mf1.rows();
    auto numCols = A_Mf1.cols() - 1;

    // Eigen::SparseMatrix<double> anotherMatrix;

    // anotherMatrix = A_Mf1.block(10, 10, 10, 10); // start row, start column, blockRows, block collums
    // std::cout << "anothermatrix\n"
    //           << anotherMatrix << "\n\n"
    //           << std::endl;

    // Eigen::SparseMatrix<double> A_Mf1_p1, A_Mf1_p2, A_Mf1_p3, A_Mf1_temp, A_Mf1_final;

    A_Mf1_p1 = A_Mf1.block(0, 0, numRows, pinnedVerticies(0));
    A_Mf1_p2 = A_Mf1.block(0, pinnedVerticies(0) + 1, numRows, pinnedVerticies(1) - pinnedVerticies(0) - 1);
    A_Mf1_p3 = A_Mf1.block(0, pinnedVerticies(1) + 1, numRows, numCols - pinnedVerticies(1));

    // COmbine 1 with 2, into a temp, then combine temp with 3 to make final

    A_Mf1_temp.resize(A_Mf1_p1.rows(), A_Mf1_p1.cols() + A_Mf1_p2.cols());
    A_Mf1_temp.middleCols(0, A_Mf1_p1.cols()) = A_Mf1_p1;
    A_Mf1_temp.middleCols(A_Mf1_p1.cols(), A_Mf1_p2.cols()) = A_Mf1_p2;

    // std::cout << "Dimensions temp: " << A_Mf1_temp.rows() << " x " << A_Mf1_temp.cols() << "\n";

    A_Mf1_final.resize(A_Mf1_temp.rows(), A_Mf1_temp.cols() + A_Mf1_p3.cols());
    A_Mf1_final.middleCols(0, A_Mf1_temp.cols()) = A_Mf1_temp;
    A_Mf1_final.middleCols(A_Mf1_temp.cols(), A_Mf1_p3.cols()) = A_Mf1_p3;

    // std::cout << "Dimensions A_Mf1_final: " << A_Mf1_final.rows() << " x " << A_Mf1_final.cols() << "\n";

    // std::ofstream sparseMatrixfile;
    // sparseMatrixfile.open("sparse_A_Mf1.txt");
    // sparseMatrixfile << Eigen::MatrixXd(A_Mf1_final); // Need to convert to dense for representation

    // A_Mf2################################
    // Eigen::SparseMatrix<double> A_Mf2_p1, A_Mf2_p2, A_Mf2_p3, A_Mf2_temp, A_Mf2_final;

    A_Mf2_p1 = A_Mf2.block(0, 0, numRows, pinnedVerticies(0));
    A_Mf2_p2 = A_Mf2.block(0, pinnedVerticies(0) + 1, numRows, pinnedVerticies(1) - pinnedVerticies(0) - 1);
    A_Mf2_p3 = A_Mf2.block(0, pinnedVerticies(1) + 1, numRows, numCols - pinnedVerticies(1));

    A_Mf2_temp.resize(A_Mf2_p1.rows(), A_Mf2_p1.cols() + A_Mf2_p2.cols());
    A_Mf2_temp.middleCols(0, A_Mf2_p1.cols()) = A_Mf2_p1;
    A_Mf2_temp.middleCols(A_Mf2_p1.cols(), A_Mf2_p2.cols()) = A_Mf2_p2;

    // std::cout << "Dimensions temp: " << A_Mf2_temp.rows() << " x " << A_Mf2_temp.cols() << "\n";

    A_Mf2_final.resize(A_Mf2_temp.rows(), A_Mf2_temp.cols() + A_Mf2_p3.cols());
    A_Mf2_final.middleCols(0, A_Mf2_temp.cols()) = A_Mf2_temp;
    A_Mf2_final.middleCols(A_Mf2_temp.cols(), A_Mf2_p3.cols()) = A_Mf2_p3;

    // std::cout << "Dimensions A_Mf2_final: " << A_Mf2_final.rows() << " x " << A_Mf2_final.cols() << "\n";

    /*
    from zero to column of first pinned,*/
    // look at uv coord buisness

    // std::cout << "\n\nRemoved some stuff...A_Mf1:\n\n";
    // std::cout << A_Mf1 << "\n\n"
    //           << std::endl;

    // std::cout << "\n\nRemoved some stuff...A_Mf2:\n\n";
    // std::cout << A_Mf2 << "\n\n"
    //           << std::endl;

    // std::cout << "Dimensions A_Mf1_final: " << A_Mf1_final.rows() << " x " << A_Mf1_final.cols() << "\n";
    // std::cout << "Dimensions A_Mf2_final: " << A_Mf2_final.rows() << " x " << A_Mf2_final.cols() << "\n";
    // Form A, consists of 4 block matricies
    /*
    Mf1 -Mf2
    Mf2 Mf1
    */
    std::cout << "listofFaces size: " << listOfFaces.size() << "   listofpoint size: " << listOfPoints.size() << "\n";

    // Eigen::SparseMatrix<double> A_top(listOfFaces.size(), 2 * (listOfPoints.size() - 2));
    // Eigen::SparseMatrix<double> A_bottom(listOfFaces.size(), 2 * (listOfPoints.size() - 2));
    // Eigen::SparseMatrix<double> A(2 * listOfFaces.size(), 2 * (listOfPoints.size() - 2));
    // Eigen::SparseMatrix<double> A(listOfFaces.size(), 4 * (listOfPoints.size() - 2)); /// wrong, if concatting verticallly

    // for sparse matricies we can't just concat like before

    // A_top << A_Mf1, -A_Mf2;  //final
    A_top.resize(A_Mf1_final.rows(), A_Mf1_final.cols() + A_Mf2_final.cols());
    A_top.middleCols(0, A_Mf1_final.cols()) = A_Mf1_final;
    A_top.middleCols(A_Mf1_final.cols(), A_Mf2_final.cols()) = -A_Mf2_final;

    // A_bottom << A_Mf2, A_Mf1;
    A_bottom.resize(A_Mf2_final.rows(), A_Mf2_final.cols() + A_Mf1_final.cols());
    A_bottom.middleCols(0, A_Mf2_final.cols()) = A_Mf2_final;
    A_bottom.middleCols(A_Mf2_final.cols(), A_Mf1_final.cols()) = A_Mf1_final;

    // A << A_top, A_bottom;   //vertical concat

    // joining code is for vertically!!
    A.reserve(A_top.nonZeros() + A_bottom.nonZeros());
    for (Eigen::Index c = 0; c < A_top.cols(); ++c)
    {
        A.startVec(c); // Important: Must be called once for each column before inserting!
        for (Eigen::SparseMatrix<double>::InnerIterator it_A_top(A_top, c); it_A_top; ++it_A_top)
            A.insertBack(it_A_top.row(), c) = it_A_top.value();
        for (Eigen::SparseMatrix<double>::InnerIterator it_A_bottom(A_bottom, c); it_A_bottom; ++it_A_bottom)
            A.insertBack(it_A_bottom.row() + A_top.rows(), c) = (it_A_bottom.value());
    }
    A.finalize();

    std::cout << "#############  Final things   #########\n\n";
    std::cout << "Dimensions A: " << A.rows() << " x " << A.cols() << "\n";
    // std::cout << "A\n\n"
    //           << A << "\n\n"
    //           << std::endl;

    // Eigen::MatrixXd Bmat_top = Eigen::MatrixXd::Zero(listOfFaces.size(), 4); // 4 as two pinned verticeis
    // Eigen::MatrixXd Bmat_bottom = Eigen::MatrixXd::Zero(listOfFaces.size(), 4);
    // Eigen::MatrixXd Bmat = Eigen::MatrixXd::Zero(2 * listOfFaces.size(), 4);

    Bmat_top << b_Mp1, -b_Mp2;
    Bmat_bottom << b_Mp2, b_Mp1;

    Bmat << Bmat_top, Bmat_bottom;

    // b = - Bmat x PinnedVector          ??
    // do we assign values for Up1??

    // std::cout << "Bmat:\n"
    //           << Bmat << "\n\n"
    //           << std::endl;

    // Eigen::VectorXd pinnedUV(4, 1); // will always pin two coords, therefore 4 points
    pinnedUV << 0, 1, 0, 0; // choosing to pin in UV space, one coord at (0,0), (1,0)   //is this sensible for all shapes?
    std::cout << "Pinned UV:\n\n"
              << pinnedUV << "\n\n"
              << std::endl;

    RHS = -Bmat * pinnedUV; // should rhs be neg? Ax=b
    std::cout << "Dimensions RHS: " << RHS.rows() << " x " << RHS.cols() << "\n";
    return true;
}

// u_coords, v_coords, pinnedVerticies, solution, listOfPoints.size())
bool prepSolutionOutput(Eigen::VectorXd &u_coords,
                        Eigen::VectorXd &v_coords,
                        Eigen::VectorXi &pinnedVerticies,
                        Eigen::VectorXd &solution,
                        Eigen::VectorXd &pinnedUV,
                        int size_listOfPoints)
{
    // Now we have a list of u coordinates, followed by v coordinates
    /*We want to plot these on the UV plane
    However this list does not contain the pinned coordiantes
    First create a new list and insert the pinned coordiantes in the right place
    The results can then be written to a tri file*/

    // // efficency concerns of adding in the middle of a vector
    //
    // pre first pinned    //from zero to one before pinned
    // first pinned        //firt pinned
    // post first pinned   //from first pinned to one before second pinned
    // second  pinned      //second pinned
    // post second pinned  //from second pinned till the end

    // //but pinned may be first or last?? worry about this if it happens?*/

    for (int i = 0; i < pinnedVerticies(0); i++)
    {
        u_coords(i) = solution(i);
    }

    u_coords(pinnedVerticies(0)) = pinnedUV(0); // first pinned coord, u

    for (int i = pinnedVerticies(0) + 1; i < pinnedVerticies(1); i++)
    {
        u_coords(i) = solution(i - 1); // i-1 as solution index as at this point weve passed one pinned point
    }

    u_coords(pinnedVerticies(1)) = pinnedUV(1); // second pinned coord, u

    for (int i = pinnedVerticies(1) + 1; i < size_listOfPoints; i++)
    {
        u_coords(i) = solution(i - 2); // now weve passed two pinned points
    }

    // std::cout << "\nU coords:\n"
    //           << u_coords << "\n"
    //           << std::endl;

    //--------------v

    int j = 0;
    for (int i = solution.rows() / 2; i < pinnedVerticies(0) + solution.rows() / 2; i++)
    {
        v_coords(j) = solution(i);
        j++;
    }

    v_coords(pinnedVerticies(0)) = pinnedUV(2);
    j++;

    for (int i = pinnedVerticies(0) + 1 + solution.rows() / 2; i < pinnedVerticies(1) + solution.rows() / 2; i++)
    {
        v_coords(j) = solution(i - 1);
        j++;
    }

    v_coords(pinnedVerticies(1)) = pinnedUV(3);
    j++;

    for (int i = pinnedVerticies(1) + 1 + solution.rows() / 2; i < size_listOfPoints + solution.rows() / 2; i++)
    {
        v_coords(j) = solution(i - 2);
        j++;
    }

    // std::cout << "\nV coords:\n"
    //           << v_coords << "\n\n"
    //           << std::endl;
    return true;
}
