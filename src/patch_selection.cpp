    
        // we now need to preprocess before we can perform mapping
    // e.g. currently not topologically a disk

    
    //  double dome
    //  we want to be able to select a seed point, and grow a patch from there
    //  obvious choice is start at 0,0,0
    //  currently my data strucutre means triangles dont know anything about their neighbours

    // start at 0,0,0
    // add all triangles that contain that point
    // starting at first triangle added, choose one of its other points, add all triangles that contain that point
    // beware this is all doubles so comparison function needeed
    // std::vector<Point> patch_listOfPoints; // underscore just to check different
    // std::vector<Face> patch_listOfFaces;

    // std::vector<int> listOfIndicies(100000);

    // patch_listOfFaces.resize(100000); // guess, we can resize later
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