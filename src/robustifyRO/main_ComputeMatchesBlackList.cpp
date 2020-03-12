// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2013, 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching/svg_matches.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/progress/progress_display.hpp"

#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <map>

using namespace openMVG;
using namespace openMVG::features;
using namespace openMVG::matching;
using namespace openMVG::sfm;

using namespace std;

int main(int argc, char ** argv)
{
	CmdLine cmd;

	std::string sSfM_Data_Filename;
	std::string sMatchesDir;
	std::string sMatchFile;
	std::string sBlackList;
	std::string sOutfile;

	cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
	cmd.add(make_option('m', sMatchesDir, "matchdir"));
	cmd.add(make_option('M', sMatchFile, "sMatchFile"));
	cmd.add(make_option('b', sBlackList, "blacklist.txt"));
	cmd.add(make_option('o', sOutfile, "sOutfile"));
	///
	/*std::string dataset = "E:/xiao.teng/sfmIPI/2viewgraph_selection/expriements/testloops/oats";
	sSfM_Data_Filename = dataset+ "/sfm.json/sfm_data.json";
	sMatchesDir = dataset + "/sfm.json";
	sBlackList = dataset + "/v3d/dev4-inc/black_edges_inc_node.txt";
	sOutDir = dataset + "/sfm.v3d.inc";
	std::string sMatchFile = sMatchesDir + "/matches.f.bin";*/
	try {
		if (argc == 1) throw std::string("Invalid command line parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string& s) {
		std::cerr << "Export pairwise matches.\nUsage: " << argv[0] << "\n"
			<< "[-i|--input_file file] path to a SfM_Data scene\n"
			<< "[-d|--matchdir path]\n"
			<< "[-m|--sMatchFile filename]\n"
			<< "[-o|--outdir path]\n"
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}

	if (sOutfile.empty()) {
		std::cerr << "\nIt is an invalid output directory" << std::endl;
		return EXIT_FAILURE;
	}
	if (sMatchesDir.empty()) {
		std::cerr << "\nmatchdir cannot be an empty option" << std::endl;
		return EXIT_FAILURE;
	}	

	//---------------------------------------
	// Read SfM Scene (image view names)
	//---------------------------------------
	SfM_Data sfm_data;
	if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS | INTRINSICS))) {
		std::cerr << std::endl
			<< "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
		return EXIT_FAILURE;
	}

	//---------------------------------------
	// Load SfM Scene regions
	//---------------------------------------
	// Init the regions_type from the image describer file (used for image regions extraction)
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
	std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
	if (!(matches_provider->load(sfm_data, sMatchFile)||
		matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.f.bin")))) {
		std::cerr << "\nxiaot-Invalid matches file." << std::endl;
		return EXIT_FAILURE;
	}

	// ------------
	// using blacklist to cerate matches.b.bin
	// ------------
	std::ifstream black(sBlackList);
	std::vector<Pair> blacklist;
	while (!black.eof())
	{
		IndexT i, j;
		black >> i >> j;
		blacklist.push_back(make_pair(i, j)); 
	}
	black.close();
	PairWiseMatches map_GeometricMatches = matches_provider->pairWise_matches_;
	PairWiseMatches map_GeometricMatchesBlackList;
	for (int i = 0; i < (int)map_GeometricMatches.size(); i++)
	{
		auto iter = map_GeometricMatches.begin();
		advance(iter, i);

		Pair current_pair = iter->first;
		matching::IndMatches vec_GeometricMatches = iter->second;
		vector<Pair>::const_iterator p = find(blacklist.begin(), blacklist.end(), current_pair);
		if (p == blacklist.end())
		{
			map_GeometricMatchesBlackList.insert(make_pair(current_pair, vec_GeometricMatches));
		}
	}
	if (!Save(map_GeometricMatchesBlackList,
		std::string(sOutfile)))
	{
		std::cerr
			<< "Cannot save computed matches in: "
			<< std::string(sOutfile);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
