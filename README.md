# Some Useful CPP Codes

### lIBRARIES REQUIRED:
1. Eigen ([Link](http://eigen.tuxfamily.org/index.php?title=Main_Page))
2. nlopt ([Link](https://nlopt.readthedocs.io/en/latest/))
3. gnuplot-iostream ([Link](https://github.com/dstahlke/gnuplot-iostream)) for plot_utilities

### Classes Details

1. file_rw.cpp : *file_rw*
	* file reading methods
	``` static std::vector< std::vector<double> > file_read_vec(std::string ) ```
	``` static Eigen::MatrixXd file_read_mat(std::string); ```
	``` static void file_write(std::string, std::vector< std::vector<int> >); ```
	``` static void file_write(std::string, std::vector< std::vector<float> >); ```
	``` static void file_write(std::string, std::vector< std::vector<double> >); ```
	``` static void file_write(std::string, Eigen::MatrixXd); ```
	``` static void file_write(std::string, Eigen::MatrixXi); ```
	``` static void file_write(std::string, Eigen::MatrixXf); ```

2. NPAM_utilities.cpp : *NPAM*
	* static Eigen::MatrixXd rotate_pts(Eigen::MatrixXd, double, double, double);
	* static int number_of_layers(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, double);
	* static Eigen::MatrixXd generate_grid_points(double, double, double, double, double, double, double);
	* static Eigen::MatrixXd identify_bottom_layer(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&);
	* static Eigen::MatrixXd identify_top_layer(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&);
	* static Eigen::MatrixXd project_grid_points(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, double, double, double);
	* static Eigen::MatrixXd Infill_Path(const Eigen::MatrixXd&, bool, double, double, double, double, int);
	* static Eigen::MatrixXd Infill_Path_with_Normals(const Eigen::MatrixXd&, bool, double, double, double, double, int);
	* static Eigen::MatrixXd Infill_Path_with_bxbybz(const Eigen::MatrixXd&, bool, double, double, double, double, int);
	* static Eigen::MatrixXd Infill_Path_with_euler(const Eigen::MatrixXd&, bool, double, double, double, double, int);

3. plot_utilities.cpp : *plt*
	* plt();
    * ~plt();
    * Gnuplot gp;
    * void title(std::string);
   	* void grid();
    * void xlabel(std::string);
    * void ylabel(std::string);
    * void zlabel(std::string);
    * void xlimit(double,double); 
    * void ylimit(double,double); 
    * void zlimit(double,double); 
    * void autoscale(std::string);
    * void multiplot();
    * void plot2d(std::vector<std::vector<double> >, std::string="black", double=1, std::string="");
    * void plot2d(Eigen::MatrixXd, std::string="black", double=1, std::string="");
    * void plot3d(std::vector<std::vector<double> >, std::string="black", double=1, std::string="");
    * void plot3d(Eigen::MatrixXd, std::string="black", double=1, std::string="");
    * void ActivePlot3d(std::vector<std::vector<double> >, std::string="black", double=1, std::string="");
    * void ActivePlot3d(Eigen::MatrixXd, std::string="black", double=1, std::string="");
    * void scatterplot2d(std::vector<std::vector<double> >, std::string="black", double=1, int=7);
    * void scatterplot2d(Eigen::MatrixXd, std::string="black", double=1, int=7);
    * void scatterplot3d(std::vector<std::vector<double> >, std::string="black", double=1, int=7);
    * void scatterplot3d(Eigen::MatrixXd, std::string="black", double=1, int=7);
    * void ActiveScatterplot3d(std::vector<std::vector<double> >, std::string="black", double=1, int=7);
    * void ActiveScatterplot3d(Eigen::MatrixXd, std::string="black", double=1, int=7);

4. robot_comm.cpp : *robot_comm*
    * robot_comm(std::string Ip_address, int Port);
    * bool establish_comm();
    * void send_data(std::string str);
    * std::string receive_data();
    * void close_comm();

5. transformation_utilities.cpp : *rtf*
   	* static Eigen::Matrix4i hom_T(Eigen::Vector3i, Eigen::Matrix3i);
	* static Eigen::Matrix4f hom_T(Eigen::Vector3f, Eigen::Matrix3f);
	* static Eigen::Matrix4d hom_T(Eigen::Vector3d, Eigen::Matrix3d);
	* static Eigen::MatrixXd apply_transformation(Eigen::MatrixXd, Eigen::Matrix4d);
	* static Eigen::MatrixXd apply_transformation_to_waypoints(Eigen::MatrixXd, Eigen::Matrix4d);
	* static Eigen::MatrixXd apply_transformation(Eigen::MatrixXd, Eigen::MatrixXd);
	* static Eigen::MatrixXd apply_transformation_to_waypoints(Eigen::MatrixXd, Eigen::MatrixXd);
    * static std::string validate_seq(std::string="");
	* static Eigen::Matrix3d eul2rot(Eigen::MatrixXd, std::string="");
	* static Eigen::MatrixXd rot2eul(Eigen::Matrix3d, std::string="");
	* static Eigen::Matrix3d qt2rot(Eigen::MatrixXd);
	* static Eigen::MatrixXd rot2qt(Eigen::Matrix3d);
	* static Eigen::MatrixXd eul2qt(Eigen::MatrixXd, std::string="");
	* static Eigen::MatrixXd qt2eul(Eigen::MatrixXd, std::string="");
	* static Eigen::MatrixXd eul2bxbybz(Eigen::MatrixXd);
	* static Eigen::MatrixXd bxbybz2eul(Eigen::MatrixXd);
	* static Eigen::MatrixXd get_rob_T_part(Eigen::MatrixXd, Eigen::MatrixXd);
	* static Eigen::MatrixXd mean(Eigen::MatrixXi);
	* static Eigen::MatrixXd mean(Eigen::MatrixXf);
	* static Eigen::MatrixXd mean(Eigen::MatrixXd);
	* static Eigen::Matrix3d rot_x(double);
	* static Eigen::Matrix3d rot_y(double);
	* static Eigen::Matrix3d rot_z(double);
	* static Eigen::MatrixXd pose_to_hom_T(Eigen::MatrixXd);
	* static Eigen::MatrixXd hom_T_to_pose(Eigen::MatrixXd);
	* static Eigen::MatrixXd hom_T_to_pose(Eigen::Matrix4d);

6. utilities.cpp : *ut*
	* static std::vector<std::vector<int> > GetUniqueRows(std::vector<std::vector<int> >);
	* static std::vector<std::vector<float> > GetUniqueRows(std::vector<std::vector<float> >);
	* static std::vector<std::vector<double> > GetUniqueRows(std::vector<std::vector<double> >);
	* static Eigen::MatrixXi vec_to_mat(std::vector<std::vector<int> >);
	* static Eigen::MatrixXf vec_to_mat(std::vector<std::vector<float> >);
	* static Eigen::MatrixXd vec_to_mat(std::vector<std::vector<double> >);
	* static std::vector<std::vector<int> > mat_to_vec(Eigen::MatrixXi);
	* static std::vector<std::vector<float> > mat_to_vec(Eigen::MatrixXf);
	* static std::vector<std::vector<double> > mat_to_vec(Eigen::MatrixXd);
	* static void disp_vec(std::vector<std::vector<int> >);
	* static void disp_vec(std::vector<std::vector<float> >);
	* static void disp_vec(std::vector<std::vector<double> >);
	* static Eigen::MatrixXd compute_TCP(Eigen::MatrixXd, Eigen::MatrixXd);
	* static double get_pt_to_lsf_plane_dist(Eigen::MatrixXd, Eigen::MatrixXd);
	* static Eigen::MatrixXd get_traj_wrt_tcp(Eigen::Matrix4d, std::vector<std::vector<double> >);
	* static std::vector<std::vector<int> > SortRows(std::vector<std::vector<int> >, int);
	* static std::vector<std::vector<float> > SortRows(std::vector<std::vector<float> >, int);
	* static std::vector<std::vector<double> > SortRows(std::vector<std::vector<double> >, int);
	* static std::vector<int> ismember(std::vector<std::vector<int> >, std::vector<int>);
	* static std::vector<int> ismember(std::vector<std::vector<float> >, std::vector<float>);
	* static std::vector<int> ismember(std::vector<std::vector<double> >, std::vector<double>);
	* static std::vector<int> ismember(Eigen::MatrixXi, Eigen::MatrixXi);
	* static std::vector<int> ismember(Eigen::MatrixXf, Eigen::MatrixXf);
	* static std::vector<int> ismember(Eigen::MatrixXd, Eigen::MatrixXd);
	* static Eigen::MatrixXd mean(Eigen::MatrixXi);
	* static Eigen::MatrixXd mean(Eigen::MatrixXf);
	* static Eigen::MatrixXd mean(Eigen::MatrixXd);
	* static double median(std::vector<int>);
	* static double median(std::vector<float>);
	* static double median(std::vector<double>);
	* static Eigen::VectorXd linsp(double, double, double);
	* static Eigen::MatrixXd InPoly(Eigen::MatrixXd, Eigen::MatrixXd);
	* static void InPoly(const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::MatrixXd&);
	* static bool lines_intersect(double l1[2][2], double l2[2][2]);
	* static std::vector<int> find_idx(Eigen::VectorXi);
	* static std::vector<int> find_idx(Eigen::VectorXf);
	* static std::vector<int> find_idx(Eigen::VectorXd);
	* static std::vector<int> find_idx(Eigen::MatrixXd);
	* static std::vector<int> find_idx(std::vector<int>);
	* static std::vector<int> find_idx(std::vector<float>);
	* static std::vector<int> find_idx(std::vector<double>);
	* static double vec_norm(Eigen::MatrixXd);
	* static double vec_norm(std::vector<double>);
	* static double vec_norm(std::vector<int>);
	* static Eigen::MatrixXd generate_pointcloud(Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, double, double);
	* static Eigen::MatrixXd add_pts(Eigen::MatrixXd, Eigen::MatrixXd);
	* static Eigen::MatrixXd generate_grid_points(double, double, double, double, double, double);
	* static void timer_start();
	* static void timer_end(std::string time_unit="millisec");

