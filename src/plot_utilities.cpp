//****************************************************************************************
//
// Author : Aniruddha Shembekar, University of Southern California
//
//****************************************************************************************

#define GNUPLOT_ENABLE_PTY

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include "stdlib.h"
#include "plot_utilities.hpp"
#include <cstdlib>
#include <thread>
#include <boost/tuple/tuple.hpp>
#include "gnuplot-iostream.h"
#include "utilities.hpp"

plt::plt()
{	
};

plt::~plt()
{
};

void plt::title(std::string t)
{
	gp << "set title '" << t << "' \n";
}

void plt::grid()
{
	gp << "set grid \n";
}

void plt::xlabel(std::string str)
{
	gp << "set xlabel '" << str << "'\n";
}

void plt::ylabel(std::string str)
{
	gp << "set ylabel '" << str << "'\n";
}

void plt::zlabel(std::string str)
{
	gp << "set zlabel '" << str << "'\n";
}

void plt::xlimit(double ll, double ul)
{
	gp << "set xrange [" << ll << ":" << ul << "]\n";
}

void plt::ylimit(double ll, double ul)
{
	gp << "set yrange [" << ll << ":" << ul << "]\n";
}

void plt::zlimit(double ll, double ul)
{
	gp << "set zrange [" << ll << ":" << ul << "]\n";
}

void plt::autoscale(std::string axis)
{
	std::string x = "X";
	std::string y = "Y";
	std::string z = "Z";
	std::string all = "ALL";
	if (!std::strcmp(axis.c_str(),x.c_str()))
	{
		gp << "set autoscale x \n";
	}
	else if (!std::strcmp(axis.c_str(),y.c_str()))
	{
		gp << "set autoscale y \n";
	}
	else if (!std::strcmp(axis.c_str(),z.c_str()))
	{
		gp << "set autoscale z \n";
	}
	else if (!std::strcmp(axis.c_str(),all.c_str()))
	{
		gp << "set autoscale x \n";
		gp << "set autoscale y \n";
		gp << "set autoscale z \n";
	}
	else
	{
		std::cerr << "plot_utilities.cpp error: give any one of these (X or Y or Z or ALL) arguments" << std::endl;
		std::terminate();		
	}
}

void plt::multiplot()
{
	gp << "set multiplot\n";
}

void plt::plot2d(std::vector<std::vector<double> > data_vec, std::string color, double lw, std::string linetitle)
{
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin. 
	gp << "plot '-' with lines lt rgb '" << color << "' lw " << lw << " title '" << linetitle << "' \n";
	gp.send1d(data_vec);
	// NOTE:
	// some colors names:pink, black, blue, violet, yellow, red, green, brown
	// if any specific color is needed, then use Hex code e.g. #FF0000 for red (DO NOT NEGLECT #) 
}

void plt::plot2d(Eigen::MatrixXd data, std::string color, double lw, std::string linetitle)
{
	std::vector<std::vector<double> > data_vec = ut::mat_to_vec(data);
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin. 
	gp << "plot '-' with lines lt rgb '" << color << "' lw " << lw << " title '" << linetitle << "' \n";
	gp.send1d(data_vec);
	// NOTE:
	// some colors names:pink, black, blue, violet, yellow, red, green, brown
	// if any specific color is needed, then use Hex code e.g. #FF0000 for red (DO NOT NEGLECT #) 
}

void plt::plot3d(std::vector<std::vector<double> > data_vec, std::string color, double lw, std::string linetitle)
{
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin. 
	gp << "splot '-' with lines lt rgb '" << color << "' lw " << lw << " title '" << linetitle << "' \n";
	gp.send1d(data_vec);
	// NOTE:
	// some colors names:pink, black, blue, violet, yellow, red, green, brown
	// if any specific color is needed, then use Hex code e.g. #FF0000 for red (DO NOT NEGLECT #) 
}

void plt::plot3d(Eigen::MatrixXd data, std::string color, double lw, std::string linetitle)
{
	std::vector<std::vector<double> > data_vec = ut::mat_to_vec(data);
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin. 
	gp << "splot '-' with lines lt rgb '" << color << "' lw " << lw << " title '" << linetitle << "' \n";
	gp.send1d(data_vec);
	// NOTE:
	// some colors names:pink, black, blue, violet, yellow, red, green, brown
	// if any specific color is needed, then use Hex code e.g. #FF0000 for red (DO NOT NEGLECT #) 
}

void plt::ActivePlot3d(std::vector<std::vector<double> > data_vec, std::string color, double lw, std::string linetitle)
{
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin. 
	gp << "splot '-' with lines lt rgb '" << color << "' lw " << lw << " title '" << linetitle << "' \n";
	gp.send1d(data_vec);
	// NOTE:
	// some colors names:pink, black, blue, violet, yellow, red, green, brown
	// if any specific color is needed, then use Hex code e.g. #FF0000 for red (DO NOT NEGLECT #) 
	double mx=0.5, my=0.5;
	int mb=1;
	int start_idx = 1;
	while(mb != 3 && mb >= 0) 
	{
		if (start_idx==1)
		{
			gp.getMouse(mx, my, mb, "Left click to rotate plot, right click to exit.");
			++start_idx;
		}
		else
		{
			gp.getMouse(mx, my, mb, "");	
		}

		if(mb == 3 || mb < 0) 
			{
				printf("\nplot closed\n");
			}
	}
}

void plt::ActivePlot3d(Eigen::MatrixXd data, std::string color, double lw, std::string linetitle)
{
	std::vector<std::vector<double> > data_vec = ut::mat_to_vec(data);
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin. 
	gp << "splot '-' with lines lt rgb '" << color << "' lw " << lw << " title '" << linetitle << "' \n";
	gp.send1d(data_vec);
	// NOTE:
	// some colors names:pink, black, blue, violet, yellow, red, green, brown
	// if any specific color is needed, then use Hex code e.g. #FF0000 for red (DO NOT NEGLECT #) 
	double mx=0.5, my=0.5;
	int mb=1;
	int start_idx = 1;
	while(mb != 3 && mb >= 0) 
	{
		if (start_idx==1)
		{
			gp.getMouse(mx, my, mb, "Left click to rotate plot, right click to exit.");
			++start_idx;
		}
		else
		{
			gp.getMouse(mx, my, mb, "");	
		}

		if(mb == 3 || mb < 0) 
			{
				printf("\nplot closed\n");
			}
	}
}

void plt::scatterplot2d(std::vector<std::vector<double> > data_vec, std::string color, double ps, int pt)
{
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
	gp << "plot '-' pointtype "<< pt << " lc rgb '" << color << "' ps " << ps << " \n";
	gp.send1d(data_vec);
}

void plt::scatterplot2d(Eigen::MatrixXd data, std::string color, double ps, int pt)
{
	std::vector<std::vector<double> > data_vec = ut::mat_to_vec(data);
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
	gp << "plot '-' pointtype "<< pt << " lc rgb '" << color << "' ps " << ps << " \n";
	gp.send1d(data_vec);
}

void plt::scatterplot3d(std::vector<std::vector<double> > data_vec, std::string color, double ps, int pt)
{
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
	gp << "splot '-' pointtype "<< pt << " lc rgb '" << color << "' ps " << ps << " \n";
	gp.send1d(data_vec);
}

void plt::scatterplot3d(Eigen::MatrixXd data, std::string color, double ps, int pt)
{
	std::vector<std::vector<double> > data_vec = ut::mat_to_vec(data);
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
	gp << "splot '-' pointtype "<< pt << " lc rgb '" << color << "' ps " << ps << " \n";
	gp.send1d(data_vec);
}

void plt::ActiveScatterplot3d(std::vector<std::vector<double> > data_vec, std::string color, double ps, int pt)
{
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
	//NOTE: pt values for symbols = 1:+,2:x,3:*,4:square,5:filled square,6:hollow circle,7:filled circle
	gp << "splot '-' pointtype "<< pt << " lc rgb '" << color << "' ps " << ps << " \n";
	gp.send1d(data_vec);
	double mx=0.5, my=0.5;
	int mb=1;
	int start_idx = 1;
	while(mb != 3 && mb >= 0) 
	{
		if (start_idx==1)
		{
			gp.getMouse(mx, my, mb, "Left click to rotate plot, right click to exit.");
			++start_idx;
		}
		else
		{
			gp.getMouse(mx, my, mb, "");	
		}

		if(mb == 3 || mb < 0) 
		{
			printf("\nplot closed\n");
		}
	}
}

void plt::ActiveScatterplot3d(Eigen::MatrixXd data, std::string color, double ps, int pt)
{
	std::vector<std::vector<double> > data_vec = ut::mat_to_vec(data);
	// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
	//NOTE: pt values for symbols = 1:+,2:x,3:*,4:square,5:filled square,6:hollow circle,7:filled circle
	gp << "splot '-' pointtype "<< pt << " lc rgb '" << color << "' ps " << ps << " \n";
	gp.send1d(data_vec);
	double mx=0.5, my=0.5;
	int mb=1;
	int start_idx = 1;
	while(mb != 3 && mb >= 0) 
	{
		if (start_idx==1)
		{
			gp.getMouse(mx, my, mb, "Left click to rotate plot, right click to exit.");
			++start_idx;
		}
		else
		{
			gp.getMouse(mx, my, mb, "");	
		}

		if(mb == 3 || mb < 0) 
		{
			printf("\nplot closed\n");
		}
	}
}

