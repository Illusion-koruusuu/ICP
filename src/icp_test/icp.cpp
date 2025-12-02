#include <iostream>
#include <string>
#include <algorithm>
#include <sys/stat.h>
#include <ctime>
#include <iomanip>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

#define VISUALIZATION

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


bool next_iteration = false;

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void*)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
}

static bool ensureDirectoryExists(const std::string &dir)
{
  struct stat info;
  if (stat(dir.c_str(), &info) != 0)
  {
    if (mkdir(dir.c_str(), 0755) != 0)
    {
      PCL_ERROR("Could not create directory %s\n", dir.c_str());
      return false;
    }
  }
  else if (!S_ISDIR(info.st_mode))
  {
    PCL_ERROR("%s exists but is not a directory\n", dir.c_str());
    return false;
  }
  return true;
}

static std::string getFileBaseName(const std::string &path)
{
  size_t pos = path.find_last_of("/\\");
  std::string fname = (pos == std::string::npos) ? path : path.substr(pos + 1);
  size_t dot = fname.find_last_of('.');
  if (dot == std::string::npos) return fname;
  return fname.substr(0, dot);
}

static bool saveAlignedCloud(const PointCloudT::Ptr &cloud, const std::string &out_dir, const std::string &source_file, const std::string &target_file)
{
  if (!ensureDirectoryExists(out_dir)) return false;

  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  std::stringstream ss;
  ss << out_dir;
  if (!out_dir.empty() && out_dir.back() != '/') ss << '/';
  ss << "registered_" << getFileBaseName(source_file) << "_to_" << getFileBaseName(target_file) << "_";
  ss << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".pcd";
  std::string out_path = ss.str();

  if (pcl::io::savePCDFileBinary(out_path, *cloud) < 0)
  {
    PCL_ERROR("Failed to save PCD file %s\n", out_path.c_str());
    return false;
  }
  std::cout << "Saved aligned cloud to " << out_path << " (" << cloud->size() << " points)\n";
  return true;
}

static bool
loadFile (const std::string& filename, PointCloudT::Ptr& cloud)
{
  pcl::console::TicToc tictoc;   // 避免与全局/库函数 time 冲突
  tictoc.tic ();
  // get extension in lower case
  std::string::size_type idx = filename.rfind('.');
  if (idx == std::string::npos)
  {
    PCL_ERROR("No file extension found for %s\n", filename.c_str());
    return false;
  }
  std::string ext = filename.substr(idx + 1);
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

  int ret = -1;
  //同时支持ply和pcd格式
  if (ext == "ply")
    ret = pcl::io::loadPLYFile(filename, *cloud);
  else if (ext == "pcd")
    ret = pcl::io::loadPCDFile(filename, *cloud);
  else
  {
    PCL_ERROR("Unsupported file extension: %s\n", ext.c_str());
    return false;
  }

  if (ret < 0)
  {
    PCL_ERROR("Error loading cloud %s.\n", filename.c_str());
    return false;
  }
  std::cout << "\nLoaded file " << filename << " (" << cloud->size () << " points) in " << tictoc.toc () << " ms\n" << std::endl;
  return true;
}

int
main (int argc,
      char* argv[])
{
  // The point clouds we will be using
  PointCloudT::Ptr cloud_ori (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Input point cloud
  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

  // Checking program arguments
  if (argc < 3)
  {
    printf ("Usage :\n");
    printf ("\t\t%s file.ply/pcd number_of_ICP_iterations\n", argv[0]);
    PCL_ERROR ("Provide two ply/pcd files.\n");
    return (-1);
  }

  int iterations = 1;  // Default number of ICP iterations
  if (argc > 3)
  {
    // If the user passed the number of iteration as an argument
    iterations = atoi (argv[argc - 1]);
    if (iterations < 1)
    {
      PCL_ERROR ("Number of initial iterations must be >= 1\n");
      return (-1);
    }
  }

  
  // Load the first cloud from file [base cloud]
  if(!loadFile (argv[1], cloud_ori))
    return (-1);

  //Load the second cloud from file [target cloud]
  if(!loadFile (argv[2], cloud_in))
    return (-1);

  // The Iterative Closest Point algorithm
  //创建ICP对象并设置参数
  pcl::console::TicToc time;
  time.tic ();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations (iterations);
  icp.setInputSource (cloud_in);
  icp.setInputTarget (cloud_ori);
  icp.align (*cloud_icp);
  icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

  // Defining a rotation matrix
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  
  //检查是否收敛并输出结果
  if (icp.hasConverged ())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_ori" << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);
    // Save the aligned point cloud to ./output
    saveAlignedCloud(cloud_icp, "./output", argv[2], argv[1]);
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }

  #ifdef VISUALIZATION
  // Visualization
  pcl::visualization::PCLVisualizer viewer ("ICP demo");
  // Create two vertically separated viewports
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_ori_color_h (cloud_ori, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
  viewer.addPointCloud (cloud_ori, cloud_ori_color_h, "cloud_ori_v1", v1);
  viewer.addPointCloud (cloud_ori, cloud_ori_color_h, "cloud_ori_v2", v2);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, 20, 20, 180);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
  viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

  // Adding text descriptions in each viewport
  viewer.addText ("White: Original point cloud\nGreen: Input point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  std::stringstream ss;
  ss << iterations;
  std::string iterations_cnt = "ICP iterations = " + ss.str ();
  viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

  // Set background color
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize (1280, 1024);  // Visualiser window size

  // Register keyboard callback :
  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

  // Display the visualiser
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();

    // The user pressed "space" :
    if (next_iteration)
    {
      // The Iterative Closest Point algorithm
      time.tic ();
      icp.align (*cloud_icp);
      std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

      if (icp.hasConverged ())
      {
        printf ("\033[11A");  // Go up 11 lines in terminal output.
        printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
        std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_ori" << std::endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>() * transformation_matrix;  // WARNING /!\ This is not accurate! For "educational" purpose only!
        print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

        ss.str ("");
        ss << iterations;
        std::string iterations_cnt = "ICP iterations = " + ss.str ();
        viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
        viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
        // Save the aligned point cloud after this iteration
        saveAlignedCloud(cloud_icp, "./output", argv[2], argv[1]);
      }
      else
      {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
      }
    }
    next_iteration = false;
  }
  #endif
  return (0);
}