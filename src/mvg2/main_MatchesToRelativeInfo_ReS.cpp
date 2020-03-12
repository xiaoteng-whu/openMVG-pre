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
	//CmdLine cmd;
	//std::string folder = "";
	//std::string sMatchFilename2;
	//cmd.add(make_option('i', folder, "input_folder"));
	//cmd.add(make_option('m', sMatchFilename2, "matchdir"));
	//cmd.process(argc, argv);
	CmdLine cmd;
	std::string sSfM_Data_Filename;
	std::string sMatchesDir, sMatchFilename1, sMatchFilename2;
	std::string sOutFile = "";
	cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
	cmd.add(make_option('m', sMatchesDir, "matchdir"));
	cmd.add(make_option('p', sMatchFilename1, "match_file"));
	cmd.add(make_option('g', sMatchFilename2, "match_file"));
	cmd.add(make_option('o', sOutFile, "outdir"));
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
	/*std::string folder = "E:/xiao.teng/sfmIPI/2viewgraph_selection/expriements/test_CAB";
	std::string sMatchesDir = folder +"/sfm.json";
	std::string sOutFile = folder + "/mvg2/relativeInfoReS.txt";
	std::string sSfM_Data_Filename = sMatchesDir+"/sfm_data.json";	
	std::string sMatchFilename1 = sMatchesDir+"/matches.putative.bin";
	std::string sMatchFilename2 = sMatchesDir+"/matches.f.bin";*/
	

	// Load input SfM_Data scene
	SfM_Data sfm_data;
	if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS | INTRINSICS))) {
		std::cerr << std::endl
			<< "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
		return EXIT_FAILURE;
	}
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
	// Features reading
	std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
	if (!feats_provider->load(sfm_data, sMatchesDir, regions_type)) {
		std::cerr << std::endl
			<< "Invalid features." << std::endl;
		return EXIT_FAILURE;
	}
	// Matches reading
	std::shared_ptr<Matches_Provider> matches_provider1 = std::make_shared<Matches_Provider>();
	if // Try to read the provided match filename or the default one (matches.e.txt/bin)
		( !matches_provider1->load(sfm_data, sMatchFilename1) )
	{
		std::cerr << std::endl
			<< "Invalid matches file." << std::endl;
		return EXIT_FAILURE;
	}
	std::shared_ptr<Matches_Provider> matches_provider2 = std::make_shared<Matches_Provider>();
	if // Try to read the provided match filename or the default one (matches.e.txt/bin)
		(!matches_provider2->load(sfm_data, sMatchFilename2))
	{
		std::cerr << std::endl
			<< "Invalid matches file." << std::endl;
		return EXIT_FAILURE;
	}

	if (sOutFile.empty()) {
		std::cerr << "\nIt is an invalid output directory" << std::endl;
		return EXIT_FAILURE;
	}
	
	/*
	id1
	id2
	Nf1:num of features in image id1
	Nf2:num of features in image id2
	Npm:num of correspondences between image id1 and id2 on the putative matches 
	Ngf:num of correspondences between image id1 and id2 on the geometric filter 
	Sr: score on ratio of inliers
	Sd1: score on distribution of inliers
	Sd2: score on distribution of inliers
	A1: aera of inliers in image id1
	A2: aera of inliers in image id2
	S.: the density of inliers,i.e. inliers/aera
    G.: the degree of the node in graph
	R12:9个旋转元素
	T12：3个平移元素
	*/
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
	

	std::ofstream outrp(sOutFile);
	// Compute the relative pose from pairwise point matches:
	for (int i = 0; i < static_cast<int>(posewise_matches.size()); ++i)
	{
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
			const matching::IndMatches & matches1 = matches_provider1->pairWise_matches_.at(current_pair);
			size_t number_matches = matches.size();
			Mat2X x1(2, number_matches), x2(2, number_matches);
			number_matches = 0;
			// score of distribution Sd1,Sd2
			size_t Sd1 = 0;
			size_t Sd2 = 0;
			size_t num_levels = 3;
			size_t width = view_I->ui_width;
			size_t height = view_I->ui_height;
			std::vector<Eigen::MatrixXi> pyramid_;
			pyramid_.resize(num_levels);
			for (size_t level = 0; level < num_levels; ++level)
			{
				const size_t level_plus_one = level + 1;
				const int dim = 1 << level_plus_one;
				pyramid_[level].setZero(dim, dim);

			}
			std::vector<Eigen::MatrixXi> pyramid_2;
			pyramid_2.resize(num_levels);
			for (size_t level = 0; level < num_levels; ++level)
			{
				const size_t level_plus_one = level + 1;
				const int dim = 1 << level_plus_one;
				pyramid_2[level].setZero(dim, dim);

			}
			const int max_dim = 1 << pyramid_.size();
			for (const auto & match : matches)
			{
				x1.col(number_matches) = cam_I->get_ud_pixel(
					feats_provider->feats_per_view.at(I)[match.i_].coords().cast<double>());
				x2.col(number_matches++) = cam_J->get_ud_pixel(
					feats_provider->feats_per_view.at(J)[match.j_].coords().cast<double>());
				const Vec2 xd1 = feats_provider->feats_per_view.at(I)[match.i_].coords().cast<double>();
				const Vec2 xd2 = feats_provider->feats_per_view.at(J)[match.j_].coords().cast<double>();
				size_t cx = 0;
				size_t cy = 0;
				if (xd1(0) < 0 || xd1(1) < 0)
				{
					std::cout << " feature error";
				}
				cx = static_cast<size_t>(max_dim* xd1(0) / width);
				cy = static_cast<size_t>(max_dim* xd1(1) / height);
				for (int i = static_cast<int>(pyramid_.size() - 1); i >= 0; --i)
					{
					    //auto& level = pyramid_[i];
						pyramid_[i](cy, cx) += 1;
						if (pyramid_[i](cy, cx) == 1)
						{
							Sd1 += pyramid_[i].size();
						}
						cx = cx >> 1;
						cy = cy >> 1;
					}
				size_t cx2 = 0;
				size_t cy2 = 0;
				cx2 = static_cast<size_t>(max_dim* xd2(0) / width);
				cy2 = static_cast<size_t>(max_dim* xd2(1) / height);
				for (int i = static_cast<int>(pyramid_2.size() - 1); i >= 0; --i)
				{
					//auto& level2 = pyramid_2[i];
					pyramid_2[i](cy2, cx2) += 1;
					if (pyramid_2[i](cy2, cx2) == 1)
					{
						Sd2 += pyramid_2[i].size();
					}
					cx2 = cx2 >> 1;
					cy2 = cy2 >> 1;
				}
			}	
			//aera of image id1 and id2
			size_t ndim = 6;
			size_t max_gird = 1 << 5;
			size_t A1 = 0;
			size_t A2 = 0;
			Eigen::MatrixXi aera1;
			aera1.setZero(max_gird, max_gird);
			Eigen::MatrixXi aera2;
			aera2.setZero(max_gird, max_gird);
			for (int i = 0; i < number_matches; i++)
			{
				size_t ax1 = static_cast<size_t>(max_gird* x1(0,i) / width);
				size_t ay1 = static_cast<size_t>(max_gird* x1(1,i) / height);
				size_t ax2 = static_cast<size_t>(max_gird* x2(0, i) / width);
				size_t ay2 = static_cast<size_t>(max_gird* x2(1, i) / height);
				if (aera1(ax1, ay1) == 0)
				{
					aera1(ax1, ay1) += 1;
					A1 += 1;
				}
				if (aera2(ax2, ay2) == 0)
				{
					aera2(ax2, ay2) += 1;
					A2 += 1;
				}	
			}			
			
			// Nf1, Nf2，
			size_t Nf1 = feats_provider->feats_per_view.at(I).size();
			size_t Nf2 = feats_provider->feats_per_view.at(J).size();
			size_t Npm = matches1.size();
			size_t Ngf = matches.size();
			float Sr = float(Ngf) / float(Npm);

			RelativePose_Info relativePose_info;
			relativePose_info.initial_residual_tolerance = Square(2.5);
			if (!robustRelativePose(cam_I, cam_J,
				x1, x2, relativePose_info,
				{ cam_I->w(), cam_I->h() },
				{ cam_J->w(), cam_J->h() },
				256))
			{
				continue;
			}
			else
			{
			outrp << I << " " << J << " " 
			<< Nf1 << " "<< Nf2<<" "
			<< Npm <<" "<< Ngf <<" "
			<< Sr <<" "<< Sd1 <<" "<< Sd2 <<" "
			<< A1 <<" "<<A2 <<" "
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
			}
			
		}
	}

	return EXIT_FAILURE;
}