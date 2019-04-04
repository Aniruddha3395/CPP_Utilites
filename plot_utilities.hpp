#ifndef PLOT_UTILITIES
#define PLOT_UTILITIES

#include <string>
#include <vector>
#include <Eigen/Eigen>
#include "gnuplot-iostream.h"

class plt
{
public:
    plt();
    ~plt();
    Gnuplot gp;
    void title(std::string);
    void grid();
    void xlabel(std::string);
    void ylabel(std::string);
    void zlabel(std::string);
    void xlimit(double,double); 
    void ylimit(double,double); 
    void zlimit(double,double); 
    void autoscale(std::string);
    void multiplot();
    void plot2d(std::vector<std::vector<double> >, std::string="black", double=1, std::string="");
    void plot2d(Eigen::MatrixXd, std::string="black", double=1, std::string="");
    void plot3d(std::vector<std::vector<double> >, std::string="black", double=1, std::string="");
    void plot3d(Eigen::MatrixXd, std::string="black", double=1, std::string="");
    void ActivePlot3d(std::vector<std::vector<double> >, std::string="black", double=1, std::string="");
    void ActivePlot3d(Eigen::MatrixXd, std::string="black", double=1, std::string="");
    void scatterplot2d(std::vector<std::vector<double> >, std::string="black", double=1, int=7);
    void scatterplot2d(Eigen::MatrixXd, std::string="black", double=1, int=7);
    void scatterplot3d(std::vector<std::vector<double> >, std::string="black", double=1, int=7);
    void scatterplot3d(Eigen::MatrixXd, std::string="black", double=1, int=7);
    void ActiveScatterplot3d(std::vector<std::vector<double> >, std::string="black", double=1, int=7);
    void ActiveScatterplot3d(Eigen::MatrixXd, std::string="black", double=1, int=7);
};

#endif