/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "calibration.h"
#include "matrix_algo.h"


using namespace easy3d;



/**
 * TODO: Finish this function for calibrating a camera from the corresponding 3D-2D point pairs.
 *       You may define a few functions for some sub-tasks.
 * @return True on success, otherwise false. On success, the camera parameters are returned by
 */
bool Calibration::calibration(
        const std::vector<Vector3D>& points_3d, /// input: An array of 3D points.
        const std::vector<Vector2D>& points_2d, /// input: An array of 2D image points.
        double& fx,    /// output: focal length (i.e., K[0][0], which is equal to 'alpha' in our slides).
        double& fy,    /// output: focal length (i.e., K[1][1], which is equal to 'beta/sin(theta)' in our slides).
        double& cx,    /// output: x component of the principal point (i.e., K[0][2], which is 'u0' in our slides).
        double& cy,    /// output: y component of the principal point (i.e., K[1][2], which is 'v0' in our slides).
        double& skew,  /// output: skew factor (i.e., K[0][1], which is equal to '-alpha * cot(theta)' in our slides).
        Matrix33& R,   /// output: the 3x3 rotation matrix encoding camera rotation.
        Vector3D& t)   /// output：a 3D vector encoding camera translation.
{
    std::cout << "\nTODO: I am going to implement the calibration() function in the following file:\n"
                 "\t    - calibration_method.cpp\n\n";

    std::cout << "[Liangliang]:\n"
                 "\tCamera calibration requires computing the SVD and inverse of matrices.\n"
                 "\tIn this assignment, I provide you with a 'Matrix' and a 'Vector' data structures for storing and\n"
                 "\tmanipulating matrices and vectors of arbitrary sizes. I also wrote some code to show you how to:\n"
                 "\t    - compute the SVD of a matrix;\n"
                 "\t    - compute the inverse of a matrix;\n"
                 "\t    - compute the transpose of a matrix.\n\n"
                 "\tFeel free to use any of the provided data structures and functions. The commonly used linear algebra\n"
                 "\tfunctions are provided in the following files:\n"
                 "\t    - Calibration/matrix.h  Matrices of arbitrary dimensions and related functions.\n"
                 "\t    - Calibration/vector.h  Vectors of arbitrary dimensions and related functions.\n"
                 "\t    - Calibration/matrix_algo.h  Determinant, inverse, SVD, linear least-squares...\n"
                 "\tPlease refer to the above files for a complete list of useful functions and their usage.\n\n"
                 "\tIn your final submission, please\n"
                 "\t    - delete ALL unrelated test or debug code and avoid unnecessary output.\n"
                 "\t    - include all the source code (and please do NOT modify the structure of the directories).\n"
                 "\t    - do NOT include the 'build' directory (which contains the intermediate files in a build step).\n"
                 "\t    - make sure your code compiles and can reproduce your results without ANY modification.\n\n" << std::flush;

    std::cout << "\n[Liangliang]:\n"
                 "\tThe input parameters of this function are:\n"
                 "\t\t- points_3d: An array of 3D points (input to this function)\n"
                 "\t\t- points_2d: An array of 2D image points (input to this function)\n"
                 "\tThis function must return either 'true' on success or 'false' otherwise. On success, the camera\n"
                 "\tparameters are returned by the following variables:\n"
                 "\t\t- fx and fy: the focal lengths (in our slides, we use 'alpha' and 'beta')\n"
                 "\t\t- cx and cy: the principal point (in our slides, we use 'u0' and 'v0')\n"
                 "\t\t- skew:      the skew factor ('-alpha * cot_theta')\n"
                 "\t\t- R:         the 3x3 rotation matrix encoding camera orientation\n"
                 "\t\t- t:         a 3D vector encoding camera location.\n"
                 "\tIMPORTANT: don't forget to write your recovered parameters to the above variables." << std::endl;

    // TODO: check if input is valid (e.g., number of correspondences >= 6, sizes of 2D/3D points must match)
    if (points_3d.size() != points_2d.size()) {
        std::cout << "Input invalid" << std::endl;
        return false;
    }
    if (points_3d.size() < 6){
        std::cout << " Not enough points!" << std::endl;
        return false;
    }

    // TODO: construct the P matrix (so P * m = 0).
    int size = points_3d.size();
    int nr_rows = points_2d.size() * 2;
    std::cout<< "number of points " << size <<std::endl;
    Matrix P_matrix(nr_rows, 12, 0.0);
    int ii = 0;
    for (int i = 0; i < 2 * points_3d.size(); i = i+2){
        Vector3D p3d = points_3d[ii];
        Vector2D p2d = points_2d[ii];
        Vector3D p3dm1 = p2d[0] * p3d;
        Vector3D p3dm2 = p2d[1] * p3d;
        P_matrix.set_row(i, {p3d[0], p3d[1], p3d[2], 1, 0, 0, 0, 0, -p3dm1[0], -p3dm1[1], -p3dm1[2], -p2d[0]});
        P_matrix.set_row(i + 1, {0, 0, 0, 0, p3d[0], p3d[1], p3d[2], 1, -p3dm2[0], -p3dm2[1], -p3dm2[2], -p2d[1]});
        ii++;
    }
    std::cout << "P matrix: \n" << P_matrix << std::endl;

    // TODO: solve for M (the whole projection matrix, i.e., M = K * [R, t]) using SVD decomposition.
    //   Optional: you can check if your M is correct by applying M on the 3D points. If correct, the projected point
    //             should be very close to your input images points.
    Matrix U_matrix(nr_rows, nr_rows, 0.0);   // initialized with 0s
    Matrix S_matrix(nr_rows, 12, 0.0);   // initialized with 0s
    Matrix V_matrix(12, 12, 0.0);   // initialized with 0s
    svd_decompose(P_matrix, U_matrix, S_matrix, V_matrix);

    // check the decomposition (CHECKED! all good)
    // Check 1: U is orthogonal, so U * U^T must be identity
//    std::cout << "U*U^T: \n" << U_matrix * transpose(U_matrix) << std::endl;

    // Check 2: V is orthogonal, so V * V^T must be identity
//    std::cout << "V*V^T: \n" << V_matrix * transpose(V_matrix) << std::endl;

    // Check 3: S must be a diagonal matrix
//    std::cout << "S: \n" << S_matrix << std::endl;

    // Check 4: according to the definition, A = U * S * V^T
//    std::cout << "M - U * S * V^T: \n" << P_matrix - U_matrix * S_matrix * transpose(V_matrix) << std::endl;

    // m_vec is equal to the last column of V
    Vector m_vec = V_matrix.get_column(V_matrix.cols() - 1);

    //reformat vector m into matrix M
    Matrix M_matrix(3, 4, 0.0);
    M_matrix.set_row(0, {m_vec[0], m_vec[1], m_vec[2], m_vec[3]});
    M_matrix.set_row(1, {m_vec[4], m_vec[5], m_vec[6], m_vec[7]});
    M_matrix.set_row(2, {m_vec[8], m_vec[9], m_vec[10], m_vec[11]});

    std::cout << "M matrix: \n" << M_matrix << std::endl;

    Vector3D a1 = {m_vec[0], m_vec[1], m_vec[2]};
    Vector3D a2 = {m_vec[4], m_vec[5], m_vec[6]};
    Vector3D a3 = {m_vec[8], m_vec[9], m_vec[10]};
    double b1 = m_vec[3];
    double b2 = m_vec[7];
    double b3 = m_vec[11];
    Matrix b_matrix(3, 1, 0.0);
    b_matrix.set_column(0, {b1, b2, b3});

    // TODO: extract intrinsic parameters from M.
    double rho1 = 1 / a3.norm();
    double rho2 = - 1 / a3.norm();
    std::cout << "+ rho  \n" << rho1 << std::endl;
    std::cout << "- rho \n" << rho2 << std::endl;

    double u0 = pow(rho1,2) * (dot(a1, a3));
    std::cout << "u_0 \n" << u0 << std::endl;
    double v0 = pow(rho1,2) * (dot(a2, a3));
    std::cout << "v_0 \n" << v0 << std::endl;

    double cos_theta = - (dot(cross(a1, a3), cross(a2, a3)) ) / ((cross(a1, a3)).length() * (cross(a2, a3)).length());
    std::cout << "cos theta \n" << cos_theta<< std::endl;

//    previous calculation of cos_theta before feedback:
//    double cos_theta2 = - (dot(cross(a1, a3), cross(a2, a3)) ) / dot((cross(a1, a3)).normalize(), (cross(a2, a3)).normalize());
//    std::cout << "old cost theta: " << cos_theta2 << std::endl;

    // get theta
    double theta = acos(cos_theta);
    std::cout << "teta: " << theta << std::endl;
    double alpha = pow(rho1, 2) * norm(cross(a1, a3)) * sin(theta);
    double beta = pow(rho1, 2) * norm(cross(a2, a3)) * sin(theta);
    std::cout << "alpha: " << alpha << "\nbeta: " << beta << std::endl;
    double cot_theta = cos(theta) / sin(theta);
    std::cout << "cot theta: \n" << cot_theta << std::endl;

    // TODO: extract extrinsic parameters from M.

    Vector3D r1, r2, r3;
    Matrix K_matrix(3, 3, 0.0);
    K_matrix.set_row(0, {alpha, (-alpha * cot_theta), u0});
    K_matrix.set_row(1, {0, (beta/sin(theta)), v0});
    K_matrix.set_row(2, {0, 0, 1});
    Matrix Kinv;
    inverse(K_matrix, Kinv);

    Matrix t_matrix = rho1 * Kinv * b_matrix;

    // select which rho to use, condition to make sure if tz is positive
    // compute t and R
    if (t_matrix(2, 0) < 0) {
        r1 = cross(a2, a3) / cross(a2, a3).norm();
        r3 = rho2 * a3;
        r2 = cross(r3, r1);
        Matrix t_matrix_1 = rho2 * Kinv * b_matrix;
        t[0] = t_matrix_1(0, 0);
        t[1] = t_matrix_1(1, 0);
        t[2] = t_matrix_1(2, 0);
        std::cout << "selected rho value: " << rho2 << std::endl;
    }
    else {
        r1 = cross(a2, a3) / cross(a2, a3).norm();
        r3 = rho1 * a3;
        r2 = cross(r3, r1);
        t[0] = t_matrix(0, 0);
        t[1] = t_matrix(1, 0);
        t[2] = t_matrix(2, 0);
        std::cout << "selected rho value: " << rho1 << std::endl;
    }

    std::cout << "t result: \n" << t << std::endl;
    R.set_row(0, r1);
    R.set_row(1, r2);
    R.set_row(2, r3);
    std::cout << "R result: \n" << R << std::endl;

    // set computed parameters
    fx = K_matrix(0,0);
    fy = K_matrix(1,1);
    cx = K_matrix(0, 2);
    cy = K_matrix(1, 2);
    skew = K_matrix(0,1);

    std::cout << "\n\tTODO: After you implement this function, please return 'true' - this will trigger the viewer to\n"
                 "\t\tupdate the rendering using your recovered camera parameters. This can help you to visually check\n"
                 "\t\tif your calibration is successful or not.\n\n" << std::flush;
    return true;
}

















