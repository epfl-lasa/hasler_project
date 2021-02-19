#ifndef __MATLP_FILTERD__
#define __MATLP_FILTERD__

#include "Eigen/Eigen"

using namespace Eigen;

class MatLP_Filterd
{
	public:
        MatLP_Filterd(MatrixXd alphas);
		MatLP_Filterd();
        MatrixXd update(MatrixXd raw_matrix);
        void setAlphas(MatrixXd alphas);
        void reset();
        void setBias(MatrixXd bias_);
	private:
		MatrixXd _alphas;
		MatrixXd _alphasComp;
		MatrixXd _bias;
		MatrixXd _old_output;
		MatrixXd _output;
		bool _initialized;
};

#endif /*__MATLP_FILTERD__*/
