#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
   //Initialise vector
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  //Check if data is valid
  if(estimations.size() != ground_truth.size() ||
     estimations.size() == 0){
    std:: cout << "ERROR: Invalid estimation or ground_truth data \n";
    return rmse;
  }

  for(unsigned int i=0; i<estimations.size(); ++i){
    VectorXd residual = estimations[i] - ground_truth[i];   
    residual = residual.array()*residual.array();
    rmse += residual;    
  }

  //Compute mean
  rmse = rmse/estimations.size();

  //compute square root
  rmse = rmse.array().sqrt();

  return rmse;
}


double Tools::CalculeNIS(const VectorXd &z_diff, const MatrixXd &S_inverse){
  //hold nis error
  double epislon = 0;
  //compute error
  epislon = z_diff.transpose() * S_inverse * z_diff;
  //return error
  return epislon;
}


void Tools::PrintGraph(const std::list<double> &data, const std::string fileName,
		       const double reference, const std::string title,
		       const std::string xTitle, const std::string yTitle){
  
  //open a pipe to gnuplot
  static FILE *gnuplotPipe = NULL;
  if (gnuplotPipe==NULL){
    gnuplotPipe= popen("gnuplot -persist", "w");  
  }  

  //print graph
  if(gnuplotPipe){
    //prepare graph
    fprintf(gnuplotPipe, "reset\n"); //gnuplot commands    
    fprintf(gnuplotPipe, "set term png #output terminal and file\n");
    fprintf(gnuplotPipe, "set output '%s'\n", fileName.c_str());
    //fprintf(gnuplotPipe, "set xrange [min:max]\n");
    //fprintf(gnuplotPipe, "set yrange [0:]\n");
    fprintf(gnuplotPipe, "set style fill solid 0.5\n");    
    fprintf(gnuplotPipe, "set xlabel '%s'\n", xTitle.c_str());
    fprintf(gnuplotPipe, "set ylabel '%s'\n", yTitle.c_str());
    fprintf(gnuplotPipe, "set title '%s'\n", title.c_str());
    //fprintf(gnuplotPipe, "plot(x, sin(x))\n");
    fprintf(gnuplotPipe, "plot '-' using 2:1 title 'NIS value' with linespoint, %f  title '95% Reference'\n", reference);
    unsigned int count = 0;
    for(double nis: data){
      //ignore first point 
      if(count < 1) {count++; continue;}
      fprintf(gnuplotPipe, "%f ", nis);
      fprintf(gnuplotPipe, "%d ", count);
      fprintf(gnuplotPipe, "\n");
      count++;
    }
    
    //flush pipe
    fflush(gnuplotPipe); 

    //close pipe
    fprintf(gnuplotPipe,"exit \n");   
    pclose(gnuplotPipe);    
    gnuplotPipe = NULL;
    std::cout << "Grap Plot finished\n";
  }
}
