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