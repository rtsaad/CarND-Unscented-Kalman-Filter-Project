#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include <string>
#include <list>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to compute NIS
  */
  static double CalculeNIS(const VectorXd &z_diff, const MatrixXd &S_inverse);

  /**
   * A helper function to print 2d graphs using gnuplot
   */
  static void PrintGraph(const std::list<double> &data, const std::string fileName,
			 const double reference, const std::string title,
			 const std::string xTitle, const std::string yTitle);
};

#endif /* TOOLS_H_ */
