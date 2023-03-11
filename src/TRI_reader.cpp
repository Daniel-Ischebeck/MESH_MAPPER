

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

#include <chrono> //for timing execution

#include <Eigen/Dense>

#include <Eigen/Sparse>

// #include <Eigen/OrderingMethods>    //for part of sparseqrsolver?

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

bool outputUVfile(std::vector<Face> &listOfFaces,
                  Eigen::MatrixXi &faceMatrix,
                  Eigen::VectorXd &u_coords,
                  Eigen::VectorXd &v_coords,
                  std::string filePath); // testing face matrix and witing it to file

bool outputTRIfile(std::vector<Point> &listOfPoints,
                   std::vector<Face> &listOfFaces,
                   std::string filePath);

// briefly have a global varible for eigen matrix of faces
// igl boundary loop test

std::vector<std::vector<int>> faces;

int main()
{
    std::chrono::steady_clock::time_point overallBegin = std::chrono::steady_clock::now();
    std::vector<Point> listOfPoints; // underscore just to check different
    std::vector<Face> listOfFaces;
    // std::string filePath = "../files/indexed_straight_dome.tri";
    std::string filePath = "../files/part_sphere_low.tri";

    if (!readFile(listOfPoints, listOfFaces, filePath))
    {
        std::cout << "Exiting\n";
        return (-1);
    }

    // Done lower down afer removing some triangles
    //  Eigen::MatrixXi faceMatrix(listOfFaces.size(), 3);
    //  for (int i = 0; i < listOfFaces.size(); i++)
    //  {
    //      faceMatrix.row(i) = Eigen::VectorXi::Map(&faces[i][0], faces[i].size());
    //  }

    // we now need to preprocess before we can perform mapping
    // currently not topologically a disk

    /*
    we want to remove flat bottom of dome
    i.e. triangles that have all verticies in the x-y plane -> z coordinate of all three verticies is the same
    */
    // #########################triangle removel
    double a, b, c;
    int listOfFacesSizeBefore = listOfFaces.size();
    int numElementsRemoved = 0;
    int numElementsAdded = 0;
    int rowCounter = 0;

    std::vector<Face> newListOfFaces(listOfFacesSizeBefore);

    for (int i = 0; i < listOfFacesSizeBefore; i++)
    {
        a = listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_z();
        b = listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_z();
        c = listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_z();
        // std::cout << a << "\n";
        // std::cout << b << "\n";
        // std::cout << c << "\n";
        // std::cout << "\n\n";

        // copy elements that are NOT part of the base into a new list which will now use
        if (!((a == b) && (b == c)))
        {
            // std::cout << "Triangle: " << i << " is on x-y plane\n";
            //  listOfFaces.erase(listOfFaces.begin() + i);
            // numElementsRemoved++;
            numElementsAdded++;

            // newListOfFaces.at(rowCounter) = Face(i, listOfFaces.at(i).get_aIndex(),listOfFaces.at(i).get_bIndex(), listOfFaces.at(i).get_cIndex());
            listOfFaces.at(rowCounter) = Face(i, listOfFaces.at(i).get_aIndex(), listOfFaces.at(i).get_bIndex(), listOfFaces.at(i).get_cIndex());

            faces[rowCounter][0] = listOfFaces.at(i).get_aIndex();
            faces[rowCounter][1] = listOfFaces.at(i).get_bIndex();
            faces[rowCounter][2] = listOfFaces.at(i).get_cIndex();
            rowCounter++;
        }
    }

    // newListOfFaces.resize(numElementsAdded);
    // for(int i=0; i<newListOfFaces.size(); i++){
    //     std::cout << i << " " << newListOfFaces.at(i).get_aIndex() << " " << newListOfFaces.at(i).get_bIndex() << " " << newListOfFaces.at(i).get_cIndex() << "\n";
    // }

    listOfFaces.resize(numElementsAdded);

    // face matrix also needs updating!

    // faces.resize(numFaces, std::vector<int>(numVertPF));
    //             faces[faceIndexCounter][0] = _aIndex;
    //             faces[faceIndexCounter][1] = _bIndex;
    //             faces[faceIndexCounter][2] = _cIndex;

    faces.resize(numElementsAdded, std::vector<int>(3)); // always 3  verticies per row

    Eigen::MatrixXi faceMatrix(listOfFaces.size(), 3);
    for (int i = 0; i < listOfFaces.size(); i++)
    {
        faceMatrix.row(i) = Eigen::VectorXi::Map(&faces[i][0], faces[i].size());
    }

    // std::cout << "\nFace matrix:\n"
    //           << faceMatrix << "\n\n"
    //           << std::endl;

    std::vector<double> listOfAreas(listOfFaces.size());

    if (!calcTriangleAreas(listOfPoints, listOfFaces, listOfAreas))
    {
        std::cout << "Triangle calcualtion failed\n";
        return (-1);
    }

    outputTRIfile(listOfPoints, listOfFaces, "testOutputFile.tri");

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
    Eigen::SparseMatrix<double> M(listOfFaces.size(), listOfPoints.size());
    // Eigen::MatrixXd M = Eigen::MatrixXd::Zero(listOfFaces.size(), listOfPoints.size());
    // would be sparse in reality, dense for now

    Eigen::SparseMatrix<double> A_Mf1(listOfFaces.size(), listOfPoints.size());
    Eigen::SparseMatrix<double> A_Mf2(listOfFaces.size(), listOfPoints.size());
    // Eigen::MatrixXd A_Mf1 = Eigen::MatrixXd::Zero(listOfFaces.size(), listOfPoints.size());
    // Eigen::MatrixXd A_Mf2 = Eigen::MatrixXd::Zero(listOfFaces.size(), listOfPoints.size());

    for (int i = 0; i < listOfFaces.size(); i++)
    { // loop through the triangles

        for (int j = 0; j < listOfPoints.size(); j++)
        { // loop through the points

            if (listOfFaces.at(i).get_aIndex() == j) // Weight 1
            {
                // real part x3-x2
                // M(i, j) = 1;
                M.insert(i, j) = 1; // for sparse we need .insert()
                A_Mf1.insert(i, j) = (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x() -
                                      listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x()) /
                                     sqrt(abs(listOfAreas.at(i))); // W1/sqrt(dt)       //W1 = (x3-x2)+ i (y3-y2)
                                                                   // because dividing by sqrt of negative, should this be in complex part?

                // complex part y3-y2
                A_Mf2.insert(i, j) = (listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y() -
                                      listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y()) /
                                     sqrt(abs(listOfAreas.at(i))); // W1/sqrt(dt)       //W1 = (x3-x2)+ i (y3-y2)
                                                                   // because dividing by sqrt of negative, should this be in complex part?
            }

            if (listOfFaces.at(i).get_bIndex() == j) // Weight 2
            {
                // real part x1-x2
                M.insert(i, j) = 2;
                A_Mf1.insert(i, j) = (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x() -
                                      listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_x()) /
                                     sqrt(abs(listOfAreas.at(i)));

                // complex part y1-y3
                A_Mf2.insert(i, j) = (listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y() -
                                      listOfPoints.at(listOfFaces.at(i).get_cIndex()).get_y()) /
                                     sqrt(abs(listOfAreas.at(i)));
            }

            if (listOfFaces.at(i).get_cIndex() == j) // Weight 3
            {
                // real part x2-x1
                M.insert(i, j) = 3;
                A_Mf1.insert(i, j) = (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_x() -
                                      listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_x()) /
                                     sqrt(abs(listOfAreas.at(i)));

                // complex part y2-y1
                A_Mf2.insert(i, j) = (listOfPoints.at(listOfFaces.at(i).get_bIndex()).get_y() -
                                      listOfPoints.at(listOfFaces.at(i).get_aIndex()).get_y()) /
                                     sqrt(abs(listOfAreas.at(i)));
            }
        }
    }

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

    Eigen::MatrixXd b_Mp1 = Eigen::MatrixXd::Zero(listOfFaces.size(), 2); // dont think need these to be zero initalised, remove later, just working on something else rn
    Eigen::MatrixXd b_Mp2 = Eigen::MatrixXd::Zero(listOfFaces.size(), 2);
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

    Eigen::SparseMatrix<double> A_Mf1_p1, A_Mf1_p2, A_Mf1_p3, A_Mf1_temp, A_Mf1_final;

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
    Eigen::SparseMatrix<double> A_Mf2_p1, A_Mf2_p2, A_Mf2_p3, A_Mf2_temp, A_Mf2_final;

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

    Eigen::SparseMatrix<double> A_top(listOfFaces.size(), 2 * (listOfPoints.size() - 2));
    Eigen::SparseMatrix<double> A_bottom(listOfFaces.size(), 2 * (listOfPoints.size() - 2));
    Eigen::SparseMatrix<double> A(2 * listOfFaces.size(), 2 * (listOfPoints.size() - 2));
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

    // ########################this joining code is for vertically!!
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

    // Eigen::SparseMatrix<double> Bmat_top(listOfFaces.size(), 4);
    // Eigen::SparseMatrix<double> Bmat_bottom(listOfFaces.size(), 4);
    // Eigen::SparseMatrix<double> Bmat(2 * listOfFaces.size(), 4);

    Eigen::MatrixXd Bmat_top = Eigen::MatrixXd::Zero(listOfFaces.size(), 4); // 4 as two pinned verticeis
    Eigen::MatrixXd Bmat_bottom = Eigen::MatrixXd::Zero(listOfFaces.size(), 4);
    Eigen::MatrixXd Bmat = Eigen::MatrixXd::Zero(2 * listOfFaces.size(), 4);

    Bmat_top << b_Mp1, -b_Mp2;
    Bmat_bottom << b_Mp2, b_Mp1;

    Bmat << Bmat_top, Bmat_bottom;

    // b = - Bmat x PinnedVector          ??
    // do we assign values for Up1??

    // std::cout << "Bmat:\n"
    //           << Bmat << "\n\n"
    //           << std::endl;

    Eigen::VectorXd pinnedUV(4, 1); // will always pin two coords, therefore 4 points
    pinnedUV << 0, 2, 0, 0;         // choosing to pin in UV space, one coord at (0,0), (1,0)   //is this sensible for all shapes?
    std::cout << "Pinned UV:\n\n"
              << pinnedUV << "\n\n"
              << std::endl;

    Eigen::MatrixXd RHS = -Bmat * pinnedUV; // should rhs be neg? Ax=b
    std::cout << "Dimensions RHS: " << RHS.rows() << " x " << RHS.cols() << "\n";
    // std::cout << "RHS\n"
    //           << RHS << "\n\n"
    //           << std::endl;

    // std::cout << "The least-squares solution is:\n"
    //     //<< A.template bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(Bmat) << std::endl;
    //      << A.colPivHouseholderQr().solve(RHS);

    Eigen::VectorXd solution(2 * (listOfPoints.size() - 2), 1);

    std::cout << "Calculating...\n";

    // solution = A.colPivHouseholderQr().solve(RHS);

    // SOlver
    // Matrix needs to be compressed before it can be used with solver
    A.makeCompressed();
    std::cout << "Dimensions (post compression) A: " << A.rows() << " x " << A.cols() << "\n";
    // std::ofstream A_sparseMatrixfile;
    // A_sparseMatrixfile.open("A_sparse.txt");
    // A_sparseMatrixfile << Eigen::MatrixXd(A);

    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    std::chrono::steady_clock::time_point begin_compute= std::chrono::steady_clock::now();
    solver.compute(A);
    std::chrono::steady_clock::time_point end_compute = std::chrono::steady_clock::now();

    std::cout << "Time for compute (sec) = " << std::chrono::duration_cast<std::chrono::microseconds>(end_compute - begin_compute).count() / 1000000.0 << std::endl;
    if (solver.info() != Eigen::Success)
    {
        // decomposition failed
        return -1;
    }
    std::chrono::steady_clock::time_point begin= std::chrono::steady_clock::now();
    solution = solver.solve(RHS);
    if (solver.info() != Eigen::Success)
    {
        // solving failed
        return -1;
    }
    std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();

    // std::cout << "Least-Squares Solution (U coords, then V):\n\n"
    //           << solution << std::endl;

    std::cout << "Time for solver (sec) = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000000.0 << std::endl;
    //  std::cout << "\n\n\n\n\nThe solution using normal equations is:\n"
    //  << (A.transpose() * A).ldlt().solve(A.transpose() * RHS) << std::endl;

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

    Eigen::VectorXd u_coords(listOfPoints.size(), 1);
    Eigen::VectorXd v_coords(listOfPoints.size(), 1);

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

    for (int i = pinnedVerticies(1) + 1; i < listOfPoints.size(); i++)
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

    for (int i = pinnedVerticies(1) + 1 + solution.rows() / 2; i < listOfPoints.size() + solution.rows() / 2; i++)
    {
        v_coords(j) = solution(i - 2);
        j++;
    }

    // std::cout << "\nV coords:\n"
    //           << v_coords << "\n\n"
    //           << std::endl;

    outputUVfile(listOfFaces, faceMatrix, u_coords, v_coords, "output_UV.tri");

    std::chrono::steady_clock::time_point overallEnd = std::chrono::steady_clock::now();
    std::cout << "Time for overall execution (sec) = " << std::chrono::duration_cast<std::chrono::microseconds>(overallEnd - overallBegin).count() / 1000000.0 << std::endl;

    return 0;
}

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
        outputTRIfile << i << " " << listOfPoints.at(i).get_x() << " "
                      << listOfPoints.at(i).get_y() << " "
                      << listOfPoints.at(i).get_z() << "\n";
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

/*
    //Code for writing matrix to file, to then be able to compare it to sparse version
    std::ofstream denseMatrixfile;
    denseMatrixfile.open("dense_A_Mf1.txt");
    denseMatrixfile << A_Mf1;
*/