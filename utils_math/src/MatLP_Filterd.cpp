#include <MatLP_Filterd.h>
#include <iostream>

MatLP_Filterd::MatLP_Filterd(MatrixXd alphas)
{
  _output = Eigen::MatrixXd::Zero(alphas.rows(),alphas.cols());
  _old_output = Eigen::MatrixXd::Zero(alphas.rows(), alphas.cols());
  _alphas=alphas;
  _alphasComp = (1.0f - alphas.array()).matrix();
  _bias = Eigen::MatrixXd::Zero(alphas.rows(), alphas.cols());
}


MatrixXd MatLP_Filterd::update(MatrixXd raw_matrix){ 
  //raw_matrix = raw_matrix.array().isNaN().select(raw_matrix,_old_output);
  _output=_alphas.cwiseProduct(_old_output) + _alphasComp.cwiseProduct(raw_matrix);  
  _old_output=_output;
  return _output - _bias;
}

void MatLP_Filterd::setBias(MatrixXd bias_)
{
  _bias=bias_;
}

void MatLP_Filterd::reset()
{
  _output.setConstant(0.0f);
  _old_output.setConstant(0.0f);
}

void MatLP_Filterd::setAlphas(MatrixXd alphas)
{
  _alphas=alphas;
  _alphasComp = (1.0 - alphas.array()).matrix();
}