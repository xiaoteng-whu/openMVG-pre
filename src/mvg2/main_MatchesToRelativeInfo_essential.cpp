#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include <cstdlib>
#include <memory>
#include <string>

using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::matching;

int main(int argc, char **argv)
{
	using namespace std;
	std::cout << std::endl
		<< "-----------------------------------------------------------\n"
		<< "Matches to relative information:\n"
		<< "------------------------------------------------------------"
		<< std::endl;

	CmdLine cmd;
	std::string dir_json ="";
	std::string sSfM_Data_Filename;
	std::string sMatchFilename2;
	std::string sOutFile;
	cmd.add(make_option('i', sSfM_Data_Filename, "sSfM_Data_Filename"));
	cmd.add(make_option('m', dir_json, "dir_json"));
	cmd.add(make_option('M', sMatchFilename2, "matches.f.bin"));
	cmd.add(make_option('o', sOutFile, "relativeInfo_essential.txt"));
	
	try {
		if (argc == 1) throw std::string("Invalid parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string& s) {
		std::cerr << "Usage: " << argv[0] << '\n'
			<< "[-i|--input_file] path to a SfM_Data scene\n"
			<< "[-m|--matchdir] path to the matches that corresponds to the provided SfM_Data scene\n"
			<< "[-o|--outdir] path where the output data will be stored\n"			
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}
	/////
	std::cerr << "Usage: " << argv[0] << '\n'
		<< argv[1]<< " "<<argv[2] <<'\n'
		<< argv[3] << " " << argv[4] << '\n'
		<< argv[5] << " " << argv[6] << '\n'
		<< argv[7] << " " << argv[8] << '\n'
		<< std::endl;
	//std::string folder = "E:/xiao.teng/sfmIPI/3gpsfm/dataset/castle-P30";
	//std::string sMatchesDir = folder+"/sfm.json";
	//std::string sOutFile = folder + "/mvg2/relativeInfo_essential.txt";
	//std::string sSfM_Data_Filename = folder +"/sfm.json/sfm_data.json";
	//std::string sMatchFilename2 = folder +"/matches.e.bin";

	// Load input SfM_Data scene
	SfM_Data sfm_data;
	if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS | INTRINSICS))) {
		std::cerr << std::endl
			<< "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
		return EXIT_FAILURE;
	}
	// Init the regions_type from the image describer file (used for image regions extraction)
	using namespace openMVG::features;
	const std::string sImage_describer = stlplus::create_filespec(dir_json, "image_describer", "json");
	std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
	if (!regions_type)
	{
		std::cerr << "Invalid: "
			<< sImage_describer << " regions type file." << std::endl;
		return EXIT_FAILURE;
	}
	// Features reading
	std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
	if (!feats_provider->load(sfm_data, dir_json, regions_type)) {
		std::cerr << std::endl
			<< "Invalid features." << std::endl;
		return EXIT_FAILURE;
	}

	std::shared_ptr<Matches_Provider> matches_provider2 = std::make_shared<Matches_Provider>();
	if // Try to read the provided match filename or the default one (matches.e.txt/bin)
		(!(matches_provider2->load(sfm_data, sMatchFilename2))
		)
	{
		std::cerr << std::endl
			<< "Invalid matches file." << std::endl;
		return EXIT_FAILURE;
	}

	if (sOutFile.empty()) {
		std::cerr << "\nIt is an invalid output directory" << std::endl;
		return EXIT_FAILURE;
	}	
	
	Pair_Set relative_pose_pairs;
	for (const auto & iterMatches : matches_provider2->pairWise_matches_)
	{
		const Pair pair = iterMatches.first;
		const View * v1 = sfm_data.GetViews().at(pair.first).get();
		const View * v2 = sfm_data.GetViews().at(pair.second).get();
		if (v1->id_pose != v2->id_pose)
			relative_pose_pairs.insert({ v1->id_pose, v2->id_pose });
	}
	//
	using PoseWiseMatches = Hash_Map<Pair, Pair_Set>;
	PoseWiseMatches posewise_matches;
	for (const auto & iterMatches : matches_provider2->pairWise_matches_)
	{
		const Pair pair = iterMatches.first;
		const View * v1 = sfm_data.GetViews().at(pair.first).get();
		const View * v2 = sfm_data.GetViews().at(pair.second).get();
		if (v1->id_pose != v2->id_pose
			&& relative_pose_pairs.count({ v1->id_pose, v2->id_pose }) == 1)
			posewise_matches[{v1->id_pose, v2->id_pose}].insert(pair);
	}
	std::unique_ptr<C_Progress> progress_status
	(new C_Progress_display(posewise_matches.size(),
		std::cout, "\n- Relative info essential output -\n"));

	std::ofstream outrp(sOutFile);
	// Compute the relative pose from pairwise point matches:
//#ifdef OPENMVG_USE_OPENMP
//#pragma omp parallel for schedule(dynamic)
//#endif
	for (int i = 0; i < static_cast<int>(posewise_matches.size()); ++i)
	{
		++(*progress_status);
		{
			PoseWiseMatches::const_iterator iter(posewise_matches.begin());
			std::advance(iter, i);
			const auto & relative_pose_iterator(*iter);
			const Pair relative_pose_pair = relative_pose_iterator.first;
			const Pair_Set & match_pairs = relative_pose_iterator.second;

			// If a pair has the same ID, discard it
			if (relative_pose_pair.first == relative_pose_pair.second)
			{
				continue;
			}
			// Select common bearing vectors
			if (match_pairs.size() > 1)
			{
				std::cerr << "Compute relative pose between more than two view is not supported" << std::endl;
				continue;
			}
			const Pair current_pair(*std::begin(match_pairs));
			const IndexT
				I = current_pair.first,
				J = current_pair.second;
			const View
				* view_I = sfm_data.views.at(I).get(),
				* view_J = sfm_data.views.at(J).get();
			// Check that valid cameras exist for the view pair
			if (sfm_data.GetIntrinsics().count(view_I->id_intrinsic) == 0 ||
				sfm_data.GetIntrinsics().count(view_J->id_intrinsic) == 0)
				continue;
			const IntrinsicBase
				* cam_I = sfm_data.GetIntrinsics().at(view_I->id_intrinsic).get(),
				* cam_J = sfm_data.GetIntrinsics().at(view_J->id_intrinsic).get();

			// Compute for each feature the un-distorted camera coordinates
			const matching::IndMatches & matches = matches_provider2->pairWise_matches_.at(current_pair);
			//const matching::IndMatches & matches1 = matches_provider1->pairWise_matches_.at(current_pair);
			size_t number_matches = matches.size();
			Mat2X x1(2, number_matches), x2(2, number_matches);
			number_matches = 0;				
			size_t Ngf = matches.size();
			for (const auto & match : matches)
			{
				x1.col(number_matches) = cam_I->get_ud_pixel(
					feats_provider->feats_per_view.at(I)[match.i_].coords().cast<double>());
				x2.col(number_matches++) = cam_J->get_ud_pixel(
					feats_provider->feats_per_view.at(J)[match.j_].coords().cast<double>());				
			}

			RelativePose_Info relativePose_info;
			relativePose_info.initial_residual_tolerance = Square(2.5);
			if (robustRelativePose(cam_I, cam_J,
				x1, x2, relativePose_info,
				{ cam_I->w(), cam_I->h() },
				{ cam_J->w(), cam_J->h() },
				256))
			{
//#ifdef OPENMVG_USE_OPENMP
//#pragma omp critical
//#endif
				//{
					outrp << I << " " << J << " " << Ngf << " "
						<< relativePose_info.essential_matrix(0) << " "
						<< relativePose_info.essential_matrix(1) << " "
						<< relativePose_info.essential_matrix(2) << " "
						<< relativePose_info.essential_matrix(3) << " "
						<< relativePose_info.essential_matrix(4) << " "
						<< relativePose_info.essential_matrix(5) << " "
						<< relativePose_info.essential_matrix(6) << " "
						<< relativePose_info.essential_matrix(7) << " "
						<< relativePose_info.essential_matrix(8) << " "
						<< relativePose_info.relativePose.rotation()(0, 0) << " "
						<< relativePose_info.relativePose.rotation()(0, 1) << " "
						<< relativePose_info.relativePose.rotation()(0, 2) << " "
						<< relativePose_info.relativePose.rotation()(1, 0) << " "
						<< relativePose_info.relativePose.rotation()(1, 1) << " "
						<< relativePose_info.relativePose.rotation()(1, 2) << " "
						<< relativePose_info.relativePose.rotation()(2, 0) << " "
						<< relativePose_info.relativePose.rotation()(2, 1) << " "
						<< relativePose_info.relativePose.rotation()(2, 2) << " "
						<< relativePose_info.relativePose.translation()(0) << " "
						<< relativePose_info.relativePose.translation()(1) << " "
						<< relativePose_info.relativePose.translation()(2) << " "
						<< std::endl;
/*				}	*/			
			}
			else {
				continue;
			}
		}
	}

	return EXIT_SUCCESS;
}