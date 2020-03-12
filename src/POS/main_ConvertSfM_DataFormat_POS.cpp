// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/tracks/tracks.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::cameras;

// Convert from a SfM_Data format to another
int main(int argc, char **argv)
{
  CmdLine cmd;
 /* std::string folder;
  cmd.add(make_option('i', folder, "input_folder"));
  cmd.process(argc, argv);*/
  std::string
    sSfM_Data_Filename_In,
    sSfM_Data_Filename_Out;
  /*sSfM_Data_Filename_In = folder+"/sfm.json/reconstruction_sequential/sfm_data_incre.bin";
  sSfM_Data_Filename_Out = folder+"/sfm.json/gesfm";*/

  cmd.add(make_option('i', sSfM_Data_Filename_In, "input_file"));
  cmd.add(make_switch('V', "VIEWS"));
  cmd.add(make_switch('I', "INTRINSICS"));
  cmd.add(make_switch('E', "EXTRINSICS"));
  cmd.add(make_switch('S', "STRUCTURE"));
  cmd.add(make_switch('C', "CONTROL_POINTS"));
  cmd.add(make_option('o', sSfM_Data_Filename_Out, "output_file"));

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch (const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
        << "[-i|--input_file] path to the input SfM_Data scene\n"
        << "[-o|--output_file] path to the output SfM_Data scene\n"
        << "\t .json, .bin, .xml, .ply, .baf\n"
        << "\n[Options to export partial data (by default all data are exported)]\n"
        << "\nUsable for json/bin/xml format\n"
        << "[-V|--VIEWS] export views\n"
        << "[-I|--INTRINSICS] export intrinsics\n"
        << "[-E|--EXTRINSICS] export extrinsics (view poses)\n"
        << "[-S|--STRUCTURE] export structure\n"
        << "[-C|--CONTROL_POINTS] export control points\n"
        << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  if (sSfM_Data_Filename_In.empty() || sSfM_Data_Filename_Out.empty())
  {
    std::cerr << "Invalid input or output filename." << std::endl;
    return EXIT_FAILURE;
  }
  if (!stlplus::folder_exists(sSfM_Data_Filename_Out))
  {
	  if (!stlplus::folder_create(sSfM_Data_Filename_Out))
	  {
		  std::cerr << "\nCannot create the output directory" << std::endl;
	  }
  }
  

  // Load input SfM_Data scene
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename_In, ESfM_Data(ALL)))
  {
    std::cerr << std::endl
      << "The input SfM_Data file \"" << sSfM_Data_Filename_In << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  std::ofstream osk(sSfM_Data_Filename_Out+"/cameras_intrinsic.txt");
  for (const auto & intrinsici :sfm_data.intrinsics)
  {
	  //cameras::IntrinsicBase ki = intrinsici.second.get;
	  //std::shared_ptr<IntrinsicBase> intrinsic = 
	  const IntrinsicBase * cam_I = intrinsici.second.get();
	  const Pinhole_Intrinsic * pinhole_cam = static_cast<const Pinhole_Intrinsic *>(cam_I);
	  double focal = pinhole_cam->focal();
	  const double ppX = pinhole_cam->principal_point()[0];
	  const double ppY = pinhole_cam->principal_point()[1];
	  osk << focal << " 0 " << ppX << std::endl
		  << "0 " << focal << " " << ppY << std::endl
		  << "0 0 1" << std::endl
		  << cam_I->w() << " " << cam_I->h() << std::endl;

  }
  std::ofstream osp(sSfM_Data_Filename_Out + "/cameras_poseCR.txt");
  for (size_t i = 0; i < sfm_data.views.size(); i++)
  {
	  const View  * view_I = sfm_data.views.at(i).get();
	  //
	  geometry::Pose3 RC = sfm_data.poses.at(view_I->id_pose);
	  //<< view_I->id_intrinsic<<" "
	 osp << RC.center()(0) << " "
		<< RC.center()(1) << " "
		<< RC.center()(2) << " "
		<< RC.rotation()(0, 0) << " "
		  << RC.rotation()(0, 1) << " "
		  << RC.rotation()(0, 2) << " "
		  << RC.rotation()(1, 0) << " "
		  << RC.rotation()(1, 1) << " "
		  << RC.rotation()(1, 2) << " "
		  << RC.rotation()(2, 0) << " "
		  << RC.rotation()(2, 1) << " "
		  << RC.rotation()(2, 2) << " "
		  
		  << std::endl;
  }
  osk.close();
  osp.close();

  //output structure
  std::ofstream oss(sSfM_Data_Filename_Out + "/structure_3Dpoints.txt");
  std::ofstream osm(sSfM_Data_Filename_Out + "/measurement_tracks.txt");
  uint32_t ncol = sfm_data.structure.size();
  uint32_t mrow = 2 * sfm_data.views.size();
  std::vector<std::vector<double>> measure(mrow, std::vector<double>(ncol, 0.0));
  std::unique_ptr<C_Progress> progress_status
  (new C_Progress_display(ncol,
	  std::cout, "\n- sfm_data.structure output -\n"));

  uint32_t j = 0;
  for (const auto &structurei:sfm_data.structure)
  {
	  ++(*progress_status);
	  Vec3 point3d = structurei.second.X;
	  oss << point3d(0) << " " << point3d(1) << " " << point3d(2) << std::endl;
	  //
	  Observations obss = structurei.second.obs;
	  for (const auto & obsi : obss)
	  {
		  IndexT idxI = obsi.first;	
		  Vec2  xi = obsi.second.x;
		  double ix = xi(0);
		  double iy = xi(1);

		  measure[idxI * 2][j] = ix;
		  measure[idxI * 2 + 1][j] = iy;
	  }
	  j = j + 1;
  }

  ///
  for (uint32_t i = 0; i < mrow; i++)
  {
	  for (uint32_t j = 0; j < ncol; j++)
	  {
		  osm << measure[i][j] << " ";

	  }
	  osm << std::endl;
  }
  osm.close();


  return EXIT_SUCCESS;
}
