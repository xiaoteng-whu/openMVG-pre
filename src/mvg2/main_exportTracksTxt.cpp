// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2013, 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching/svg_matches.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/tracks/tracks.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"

#include "software/SfM/SfMIOHelper.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/progress/progress_display.hpp"

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::sfm;
using namespace openMVG::tracks;

int main(int argc, char ** argv)
{
  CmdLine cmd;
  std::string sSfM_Data_Filename;
  std::string sMatchesDir;
  std::string sMatchFile;
  std::string sOutFile;

  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('m', sMatchesDir, "matchdir") );
  cmd.add( make_option('M', sMatchFile, "matchfile") );
  cmd.add( make_option('o', sOutFile, "outdir") );

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch (const std::string& s) {
      std::cerr << "Export pairwise tracks.\nUsage: " << argv[0] << "\n"
      << "[-i|--input_file file] path to a SfM_Data scene\n"
      << "[-m|--matchdir path]\n"
      << "[-M|--sMatchFile filename]\n"
      << "[-o|--outdir path]\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  if (sOutFile.empty())  {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Read SfM Scene (image view names)
  //---------------------------------------
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }
  //---------------------------------------
  // Load SfM Scene regions
  //---------------------------------------
  // Init the regions_type from the image describer file (used for image regions extraction)
  using namespace openMVG::features;
  const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
  std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
  if (!regions_type)
  {
    std::cerr << "Invalid: "
      << sImage_describer << " regions type file." << std::endl;
    return EXIT_FAILURE;
  }

  // Read the features
  std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
  if (!feats_provider->load(sfm_data, sMatchesDir, regions_type)) {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }
  // Read the matches
  std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
  if (!matches_provider->load(sfm_data, sMatchFile)) {
    std::cerr << "\nInvalid matches file." << std::endl;
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Compute tracks from matches
  //---------------------------------------
  tracks::STLMAPTracks map_tracks;
  {
    const openMVG::matching::PairWiseMatches & map_Matches = matches_provider->pairWise_matches_;
    tracks::TracksBuilder tracksBuilder;
    tracksBuilder.Build(map_Matches);
    tracksBuilder.Filter();
    tracksBuilder.ExportToSTL(map_tracks);
  }
  openMVG::tracks::SharedTrackVisibilityHelper track_visibility_helper(map_tracks);
  tracks::STLMAPTracks map_trackspoints;
  std::set<uint32_t> set_imageid;
  openMVG::tracks::TracksUtilsMap trackmap;
  trackmap.ImageIdInTracks(map_tracks,set_imageid);
  uint32_t ncol = map_tracks.size();
  uint32_t mrow = 2 * set_imageid.size();
  std::vector<std::vector<double>> measure(mrow, std::vector<double>(ncol,0.0));
  uint32_t j = 0;
  for (const auto & track : map_tracks)
  {
	 // uint32_t j = track.first;
	  submapTrack obss = track.second;
	  for (const auto & obs : obss)
	  {
		  uint32_t idxI = obs.first;
		  uint32_t featId = obs.second;
		  const View * view_I = sfm_data.views.at(idxI).get();
		  const openMVG::cameras::IntrinsicBase * cam_I = sfm_data.GetIntrinsics().at(view_I->id_intrinsic).get();
		  Vec2  xi = cam_I->get_ud_pixel(feats_provider->feats_per_view.at(idxI)[featId].coords().cast<double>());
		  double ix = xi(0);
		  double iy = xi(1);

		  measure[idxI * 2][ j] = ix;
		  measure[idxI * 2 +1] [j] = iy;
	  }
	  j = j + 1;
  }
  //output measurement
  std::ofstream out(sOutFile);
  for (uint32_t i = 0; i < mrow; i++)
  {
	  for (uint32_t j = 0; j < ncol; j++)
	  {
		  out << measure[i][j] << " ";

	  }
	  out << std::endl;
  }
  out.close();
  
  return EXIT_SUCCESS;
}
