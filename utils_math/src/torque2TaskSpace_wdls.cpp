 // Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
 // Version: 1.0
 // Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
 // Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
 // URL: http://www.orocos.org/kdl
 
 // This library is free software; you can redistribute it and/or
 // modify it under the terms of the GNU Lesser General Public
 // License as published by the Free Software Foundation; either
 // version 2.1 of the License, or (at your option) any later version.
 
 // This library is distributed in the hope that it will be useful,
 // but WITHOUT ANY WARRANTY; without even the implied warranty of
 // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 // Lesser General Public License for more details.
 
 // You should have received a copy of the GNU Lesser General Public
 // License along with this library; if not, write to the Free Software
 // Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 #include "torque2TaskSpace_wdls.h"
 #include "eigen_conversions/eigen_kdl.h"
 #include "kdl/utilities/svd_eigen_HH.hpp"
 #include "iostream"
 
 namespace KDL {

 Torque2TaskSpace_wdls::Torque2TaskSpace_wdls(const Chain &_chain, double _eps,
                                              int _maxiter)
     : chain(_chain), jnt2jac(chain), nj(chain.getNrOfJoints()), jac(nj),
       U(MatrixXd::Zero(6, nj)), S(VectorXd::Zero(nj)),
       V(MatrixXd::Zero(nj, nj)), eps(_eps), maxiter(_maxiter),
       tmp(VectorXd::Zero(nj)), tmp_jac(MatrixXd::Zero(6, nj)),
       tmp_jac_weight1(MatrixXd::Zero(6, nj)),
       tmp_jac_weight2(MatrixXd::Zero(6, nj)), tmp_ts(MatrixXd::Zero(6, nj)),
       tmp_js(MatrixXd::Zero(nj, nj)), weight_ts(MatrixXd::Identity(6, 6)),
       weight_js(MatrixXd::Identity(nj, nj)), lambda(0.0), lambda_scaled(0.0),
       nrZeroSigmas(0), svdResult(0), sigmaMin(0),error(E_NOERROR) { 
         //std::cout<<tmp_ts.cols()<<" "<<tmp_ts.rows()<<" "<<std::endl;
       }
 

  
 Torque2TaskSpace_wdls::~Torque2TaskSpace_wdls() {}

 int Torque2TaskSpace_wdls::setWeightJS(const MatrixXd &Mq) {
   if (nj != chain.getNrOfJoints())
     return (error = -4);

   if (Mq.size() != weight_js.size())
     return (error = -8);
   weight_js = Mq;
   return (error = E_NOERROR);
 }

 int Torque2TaskSpace_wdls::setWeightTS(const MatrixXd &Mx) {
   if (Mx.size() != weight_ts.size())
     return (error = -8);
   weight_ts = Mx;
   return (error = E_NOERROR);
 }

 void Torque2TaskSpace_wdls::setLambda(const double lambda_in) {
   lambda = lambda_in;
 }

 void Torque2TaskSpace_wdls::setEps(const double eps_in) { eps = eps_in; }

 void Torque2TaskSpace_wdls::setMaxIter(const int maxiter_in) {
   maxiter = maxiter_in;
 }

 int Torque2TaskSpace_wdls::getSigma(Eigen::VectorXd &Sout) {
   if (Sout.size() != S.size())
     return (error = -8);
   Sout = S;
   return (error = E_NOERROR);
 }

 int Torque2TaskSpace_wdls::JntToCart(const JntArray &q_in,
                                         const JntArray &torque_in,
                                        Eigen::Matrix<double,6,1> &wrench_out) {
   
   // std::cout<<"debug_message"<<std::endl;
   if (nj != chain.getNrOfJoints())
     return (error = -4);

   if (nj != q_in.rows() || nj != torque_in.rows())
     return (error = -8);
   error = jnt2jac.JntToJac(q_in, jac);
   if (error < E_NOERROR)
     return error;

   double sum;
   unsigned int i, j;

   // Initialize (internal) return values
   nrZeroSigmas = 0;
   sigmaMin = 0.0;
   lambda_scaled = 0.;

  //  // Create the Weighted jacobian
    tmp_jac_weight1 = jac.data.lazyProduct(weight_js);
    tmp_jac_weight2 = weight_ts.lazyProduct(tmp_jac_weight1);

  //  // Compute the SVD of the weighted jacobian
  //  //svdResult = svd_eigen_HH(tmp_jac_weight2, U, S, V, tmp, maxiter);
    Eigen::JacobiSVD<MatrixXd> svd(tmp_jac_weight2, ComputeThinU | ComputeThinV);
    U = svd.matrixU(); V= svd.matrixV(); S=svd.singularValues();
   if (0 != svdResult) {
     wrench_out.setZero();
     return (error = E_SVD_FAILED);
   }

  //  // Pre-multiply U and V by the task space and joint space weighting matrix
  //  // respectively
  // //  tmp_ts = weight_ts.lazyProduct(U.topLeftCorner(6, 6));
  // //  tmp_js = weight_js.lazyProduct(V);
  
   unsigned int j_ = nj >= 6 ? 6 : nj;
  //  std::cout<<"debug_message1"<<std::endl;
   tmp_ts = weight_ts.lazyProduct(U.topLeftCorner(6,j_));
   tmp_js = V.adjoint().lazyProduct(weight_js);

   // Minimum of six largest singular values of J is S(5) if number of joints
   // >=6 and 0 for <6
   if (jac.columns() >= 6) {
     sigmaMin = S(5);
   } else {
     sigmaMin = 0.0;
   }

   // tmp = (Si*U'*Mx*x),
   // tmp_new = (Si'*V'*Mq'*torque),
   for (i = 0; i < tmp_js.rows(); i++) {
     sum = 0.0;
     for (j = 0; j < tmp_js.cols(); j++) {
       if (i < std::min((int)tmp_js.rows(),6))
         sum += tmp_js(i, j) * torque_in(j);
       else
        sum += 0.0;
     }
     // If sigmaMin > eps, then wdls is not active and lambda_scaled = 0
     // (default value)
     // If sigmaMin < eps, then wdls is active and lambda_scaled is scaled from
     // 0 to lambda
     // Note:  singular values are always positive so sigmaMin >=0
     if (sigmaMin < eps) {
       lambda_scaled = sqrt(1.0 - (sigmaMin / eps) * (sigmaMin / eps)) * lambda;
     }
     if (fabs(S(i)) < eps) {
       if (i < std::min((int)tmp_js.rows(),6)) {
         // Scale lambda to size of singular value sigmaMin
         tmp(i) =
             sum * ((S(i) / (S(i) * S(i) + lambda_scaled * lambda_scaled)));
       } else {
         tmp(i) = 0.0; // S(i)=0 for i>=6 due to cols>rows
       }
       //  Count number of singular values near zero
       ++nrZeroSigmas;
     } else {
       tmp(i) = sum / S(i);
     }
   }
  // std::cout<<tmp.rows()<<" "<<tmp.cols()<<" "<<std::endl;
  //  std::cout<<"debug_message2"<<std::endl;
  float temp_j = tmp_ts.cols() < nj ? tmp_ts.cols() : nj;
   wrench_out = tmp_ts.lazyProduct(tmp.segment(0,temp_j));

   // If number of near zero singular values is greater than the full rank
   // of jac, then wdls is active
   if (nrZeroSigmas > fabs((jac.columns() - jac.rows()))) {
     return (error = E_CONVERGE_PINV_SINGULAR); // converged but pinv singular
   } else {
     return (error = E_NOERROR); // have converged
   }
 }

 const char *Torque2TaskSpace_wdls::strError(const int error) const {
   if (E_CONVERGE_PINV_SINGULAR == error)
     return "Converged put pseudo inverse of jacobian is singular.";
   else
     return strErrorGen(error);
 }
 }

 // The original transformation of wdls is:

// J# = Mq * V * Db# * U' * Mx

// then I transpose:
// J#' = Mx' * U * Db#' * V' * Mq'

// since Db is a diagonal (hence symmetric) matrix, 
// The transpose of the pseudoinverse is the same as the original matrix and clipped to 6x6 Db#' = Db#

//Note that the original version of this file (only for the pinverse of the Jacobian), the U' * Mx is calculated as Mx * U and then transposed
//in the for loop (j,i) instead of (i,j)


