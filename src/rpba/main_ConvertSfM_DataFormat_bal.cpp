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

void ComputeRodriguesAnglefromRotationMatrix(double &a, double &b, double &c,
	const Mat3 &Rabc)
{
	const double epsilon0 = 0.01;
	const double epsilon1 = 0.1;
	const double pi = 3.14159265358979323846;

	double trace1 = (Rabc(0, 0) + Rabc(1, 1) + Rabc(2, 2) - 1.) * 0.5;
	if (fabs(Rabc(0, 1) - Rabc(1, 0)) < epsilon0 &&
		fabs(Rabc(1, 2) - Rabc(2, 1)) < epsilon0 &&
		fabs(Rabc(0, 2) - Rabc(2, 0)) < epsilon0) {
		if (fabs(Rabc(0, 1) + Rabc(1, 0)) < epsilon1 &&
			fabs(Rabc(1, 2) + Rabc(2, 1)) < epsilon1 &&
			fabs(Rabc(0, 2) + Rabc(2, 0)) < epsilon1 && trace1 > 0.9) {
			a = b = c = 0.;
			return;
		}

		const double xx12 = (Rabc(0, 0) + 1.) * 0.5;
		const double yy12 = (Rabc(1, 1) + 1.) * 0.5;
		const double zz12 = (Rabc(2, 2) + 1.) * 0.5;
		const double xy4 = (Rabc(0, 1) + Rabc(1, 0)) * 0.25;
		const double xz4 = (Rabc(0, 2) + Rabc(2, 0)) * 0.25;
		const double yz4 = (Rabc(1, 2) + Rabc(2, 1)) * 0.25;

		if ((xx12 > yy12) && (xx12 > zz12)) {
			if (xx12 < epsilon0) {
				a = 0.;
				b = c = sqrt(0.5) * pi;
				return;
			} // if xx
			const double t = sqrt(xx12);
			a = t * pi;
			b = xy4 / t * pi;
			c = xz4 / t * pi;
			return;
		} // if xx
		if (yy12 > zz12) {
			if (yy12 < epsilon0) {
				a = c = sqrt(0.5) * pi;
				b = 0.;
				return;
			} // if yy
			const double t = sqrt(yy12);
			a = xy4 / t * pi;
			b = t * pi;
			c = yz4 / t * pi;
			return;
		} // if
		if (zz12 < epsilon0) {
			a = b = sqrt(0.5) * pi;
			c = 0.;
			return;
		} // if zz
		const double t = sqrt(zz12);
		a = xz4 / t * pi;
		b = yz4 / t * pi;
		c = t * pi;
		return;
	} // if fabs

	const double aa = acos(trace1);
	const double bb = 0.5 * aa / sin(aa);
	a = bb * (Rabc(2, 1) - Rabc(1, 2));
	b = bb * (Rabc(0, 2) - Rabc(2, 0));
	c = bb * (Rabc(1, 0) - Rabc(0, 1));

	return;
} // ComputeRodriguesAnglefromRotationMatrix


// Convert from a SfM_Data format to another
int main(int argc, char **argv)
{
  //CmdLine cmd;
 /* std::string folder;
  cmd.add(make_option('i', folder, "input_folder"));
  cmd.process(argc, argv);*/
  std::string
    sSfM_Data_Filename_In,
    sSfM_Data_Filename_Out;
  sSfM_Data_Filename_In = "E:/sfmtools-data/castle-P30/sfm.json/reconstruction_global/sfm_data_BA_initial.bin";
  sSfM_Data_Filename_Out = "E:/sfmtools-data/castle-P30/rpba";

  /*cmd.add(make_option('i', sSfM_Data_Filename_In, "input_file"));
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
  }*/

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
  //
  std::ofstream osbal(sSfM_Data_Filename_Out+"/sfm_data_BA_initial_bal.txt"); 

  uint32_t np = sfm_data.structure.size();//num points;
  uint32_t nc = sfm_data.poses.size();//num cameras
  uint32_t nobs = 0;
  for (const auto &structurei : sfm_data.structure)
  {
	  nobs = nobs + structurei.second.obs.size();
  }
  osbal << nc<< " " << np << " " << nobs << std::endl;
  //
  std::unique_ptr<C_Progress> progress_status
  (new C_Progress_display(np,
	  std::cout, "\n- sfm_data measure matrix -\n"));
  uint32_t idxp = 0;
  for (const auto &structurei : sfm_data.structure)
  {
	  ++(*progress_status);
	  //
	  Observations obss = structurei.second.obs;	   

	  for (const auto & obsi : obss)
	  {
		  uint32_t idxc = obsi.first;
		  const View  * view_I = sfm_data.views.at(idxc).get();
		  const IntrinsicBase * cam_I = sfm_data.intrinsics.at(view_I->id_intrinsic).get();
		  const Pinhole_Intrinsic * pinhole_cam = static_cast<const Pinhole_Intrinsic *>(cam_I);
		  
		  const double ppX = pinhole_cam->principal_point()[0];
		  const double ppY = pinhole_cam->principal_point()[1];
		  Vec2  xi = obsi.second.x;
		  double ix = xi(0);
		  double iy = xi(1);
		// convert to bal 
		  ix = ix - ppX;
		  iy = -(iy - ppY);
		  osbal << idxc << " " << idxp << " " << ix << " " << iy << std::endl;
	  }
	  idxp = idxp + 1;
  }
  //
  std::unique_ptr<C_Progress> progress_status1
  (new C_Progress_display(nc,
	  std::cout, "\n- sfm_data cameras -\n"));
  for (size_t i = 0; i < sfm_data.poses.size(); i++)
  {
	  ++(*progress_status1);
	  const View  * view_I = sfm_data.views.at(i).get();
	  const IntrinsicBase * cam_I = sfm_data.intrinsics.at(view_I->id_intrinsic).get();
	  const Pinhole_Intrinsic * pinhole_cam = static_cast<const Pinhole_Intrinsic *>(cam_I);
	  double focal = pinhole_cam->focal();
	  //
	  geometry::Pose3 RC = sfm_data.poses.at(view_I->id_pose);
	  //convert to bal
	  //rot cv to rot pho; and then to rodrigou vector
	  Mat3 rot = RC.rotation();
	  rot(1, 0) = -rot(1, 0);
	  rot(1, 1) = -rot(1, 1);
	  rot(1, 2) = -rot(1, 2);
	  rot(2, 0) = -rot(2, 0);
	  rot(2, 1) = -rot(2, 1);
	  rot(2, 2) = -rot(2, 2);
	  double v1, v2, v3;
	  ComputeRodriguesAnglefromRotationMatrix(v1, v2, v3, rot);
	  //
	  osbal << v1 << std::endl
		  << v2 << std::endl
		  << v3 << std::endl
		  << RC.translation()(0) << std::endl
		  << RC.translation()(1) << std::endl
		  << RC.translation()(2) << std::endl
		  << focal << std::endl
		  << 0.0 << std::endl
		  << 0.0 << std::endl;
  }
  //
  std::unique_ptr<C_Progress> progress_status2
  (new C_Progress_display(np,
	  std::cout, "\n- sfm_data points3d -\n"));
  for (const auto &structurei : sfm_data.structure)
  {
	  ++(*progress_status2);
	  Vec3 point3d = structurei.second.X;
	  osbal << point3d(0) << std::endl 
		  << point3d(1) << std::endl 
		  << point3d(2) << std::endl;
  }

  osbal.close();
  
  return EXIT_SUCCESS;
}
