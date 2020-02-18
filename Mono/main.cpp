/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define USE_FREE
//#define USE_IMAGES_SET

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <boost/log/core.hpp>

// ADD MODULES TRAITS HEADERS HERE

// ADD XPCF HEADERS HERE
#include "xpcf/xpcf.h"
#include "core/Log.h"
// ADD COMPONENTS HEADERS HERE
#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapper.h"
#include "api/solver/map/IKeyframeSelector.h"
#include "api/solver/map/IMapFilter.h"
#include "api/solver/map/IBundler.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/features/IMatchesFilter.h"
#include "api/display/I2DOverlay.h"
#include "api/display/IMatchesOverlay.h"
#include "api/display/I3DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/geom/IProject.h"
#include "core/Log.h"

#include "api/input/files/IMarker2DSquaredBinary.h"
#include "api/image/IImageFilter.h"
#include "api/image/IImageConvertor.h"
#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/image/IPerspectiveController.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/features/ISBPatternReIndexer.h"
#include "api/geom/IImage2WorldMapper.h"

#define MIN_THRESHOLD -1
#define MAX_THRESHOLD 220
#define NB_THRESHOLD 8


using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf = org::bcom::xpcf;

int main(int argc, char **argv) {

#if NDEBUG
	boost::log::core::get()->set_logging_enabled(false);
#endif

	LOG_ADD_LOG_TO_CONSOLE();

	try {

		/* instantiate component manager*/
		/* this is needed in dynamic mode */
		SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

		std::string configxml = std::string("conf_SLAM.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);
		if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file {}", configxml.c_str())
				return -1;
		}

		// declare and create components
		LOG_INFO("Start creating components");

		// component creation
#ifdef USE_IMAGES_SET
        auto camera = xpcfComponentManager->resolve<input::devices::ICamera>("ImagesAsCamera");
#else
        auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
#endif
        auto  keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
        auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
        SRef<features::IDescriptorMatcher> matcher = xpcfComponentManager->resolve<features::IDescriptorMatcher>();
        SRef<solver::pose::I3DTransformFinderFrom2D2D> poseFinderFrom2D2D = xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom2D2D>();
        SRef<solver::map::ITriangulator> triangulator = xpcfComponentManager->resolve<solver::map::ITriangulator>();
        SRef<features::IMatchesFilter> matchesFilter = xpcfComponentManager->resolve<features::IMatchesFilter>();
        SRef<solver::pose::I3DTransformSACFinderFrom2D3D> pnpRansac = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>();
        SRef<solver::pose::I3DTransformFinderFrom2D3D> pnp = xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom2D3D>();
        SRef<solver::pose::I2D3DCorrespondencesFinder> corr2D3DFinder = xpcfComponentManager->resolve<solver::pose::I2D3DCorrespondencesFinder>();
        SRef<solver::map::IMapFilter> mapFilter = xpcfComponentManager->resolve<solver::map::IMapFilter>();
        SRef<solver::map::IMapper> mapper = xpcfComponentManager->resolve<solver::map::IMapper>();
        SRef<solver::map::IKeyframeSelector> keyframeSelector = xpcfComponentManager->resolve<solver::map::IKeyframeSelector>();
        SRef<display::IMatchesOverlay> matchesOverlay = xpcfComponentManager->resolve<display::IMatchesOverlay>();
        SRef<display::IMatchesOverlay> matchesOverlayBlue = xpcfComponentManager->resolve<display::IMatchesOverlay>("matchesBlue");
        SRef<display::IMatchesOverlay> matchesOverlayRed = xpcfComponentManager->resolve<display::IMatchesOverlay>("matchesRed");
        SRef<display::IImageViewer> imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
        SRef<display::I3DPointsViewer> viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();
        SRef<reloc::IKeyframeRetriever> kfRetriever = xpcfComponentManager->resolve<reloc::IKeyframeRetriever>();
        SRef<geom::IProject> projector = xpcfComponentManager->resolve<geom::IProject>();
        SRef <solver::map::IBundler> bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
		// marker fiducial
        auto binaryMarker = xpcfComponentManager->resolve<input::files::IMarker2DSquaredBinary>();
        auto imageFilterBinary = xpcfComponentManager->resolve<image::IImageFilter>();
        auto imageConvertor = xpcfComponentManager->resolve<image::IImageConvertor>();
        auto contoursExtractor = xpcfComponentManager->resolve<features::IContoursExtractor>();
        auto contoursFilter = xpcfComponentManager->resolve<features::IContoursFilter>();
        auto perspectiveController = xpcfComponentManager->resolve<image::IPerspectiveController>();
        auto patternDescriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractorSBPattern>();
        auto patternMatcher = xpcfComponentManager->resolve<features::IDescriptorMatcher>();
        auto patternReIndexer = xpcfComponentManager->resolve<features::ISBPatternReIndexer>();
        auto img2worldMapper = xpcfComponentManager->resolve<geom::IImage2WorldMapper>();
        auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();
        auto overlay2D = xpcfComponentManager->resolve<display::I2DOverlay>();


		// declarations
		SRef<Image>                                         view1, view2, view;
		SRef<Keyframe>                                      keyframe1;
		SRef<Keyframe>                                      keyframe2;
		std::vector<Keypoint>								keypointsView1, keypointsView2, keypoints;
		SRef<DescriptorBuffer>                              descriptorsView1, descriptorsView2, descriptors;
		std::vector<DescriptorMatch>                        matches;

		Transform3Df                                        poseFrame1 = Transform3Df::Identity();
		Transform3Df                                        poseFrame2;
		Transform3Df                                        newFramePose;
		Transform3Df                                        lastPose;

		std::vector<CloudPoint>								cloud, filteredCloud;

		std::vector<Transform3Df>                           keyframePoses;
		std::vector<Transform3Df>                           framePoses;

		SRef<Frame>                                         newFrame;
		SRef<Frame>											frameToTrack;

		SRef<Keyframe>                                      referenceKeyframe, updatedRefKf;

		SRef<Image>                                         imageMatches, imageMatches2;
		SRef<Map>                                           map;
		std::vector<CloudPoint>								localMap;
		std::vector<unsigned int>							idxLocalMap;

		bool												isLostTrack = false;
		bool												bundling = true;
		double												bundleReprojError;		


		auto localBundleAdjuster = [&bundler, &camera, &mapper](std::vector<int>&framesIdxToBundle, double& reprojError) {
			std::vector<Transform3Df>correctedPoses;
			std::vector<CloudPoint>correctedCloud;
			CamCalibration correctedCalib;
			CamDistortion correctedDist;
			reprojError = bundler->solve(mapper->getKeyframes(),
										 mapper->getGlobalMap()->getPointCloud(),
										 camera->getIntrinsicsParameters(),
										 camera->getDistorsionParameters(),
										 framesIdxToBundle,
										 correctedPoses,
										 correctedCloud,
										 correctedCalib,
										 correctedDist);
			mapper->update(correctedPoses, correctedCloud);				
		};


		// components initialisation for marker detection
		SRef<DescriptorBuffer> markerPatternDescriptor;
		binaryMarker->loadMarker();
		patternDescriptorExtractor->extract(binaryMarker->getPattern(), markerPatternDescriptor);
		LOG_DEBUG("Marker pattern:\n {}", binaryMarker->getPattern().getPatternMatrix())
		// Set the size of the box to display according to the marker size in world unit
		overlay3D->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(binaryMarker->getSize().width, 0);
		overlay3D->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(binaryMarker->getSize().height, 1);
		overlay3D->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(binaryMarker->getSize().height / 2.0f, 2);
		int patternSize = binaryMarker->getPattern().getSize();
		patternDescriptorExtractor->bindTo<xpcf::IConfigurable>()->getProperty("patternSize")->setIntegerValue(patternSize);
		patternReIndexer->bindTo<xpcf::IConfigurable>()->getProperty("sbPatternSize")->setIntegerValue(patternSize);
		img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalWidth")->setIntegerValue(patternSize);
		img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalHeight")->setIntegerValue(patternSize);
		img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("worldWidth")->setFloatingValue(binaryMarker->getSize().width);
		img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("worldHeight")->setFloatingValue(binaryMarker->getSize().height);
		overlay3D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());



		// initialize pose estimation with the camera intrinsic parameters (please refeer to the use of intrinsec parameters file)
		pnpRansac->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
		pnp->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
		poseFinderFrom2D2D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
		triangulator->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
		projector->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
		LOG_DEBUG("Intrincic parameters : \n {}", camera->getIntrinsicsParameters());

		if (camera->start() != FrameworkReturnCode::_SUCCESS)
		{
			LOG_ERROR("Camera cannot start");
			return -1;
		}

		auto detectFiducialMarker = [&imageConvertor, &imageFilterBinary, &contoursExtractor, &contoursFilter, &perspectiveController,
			&patternDescriptorExtractor, &patternMatcher, &markerPatternDescriptor, &patternReIndexer, &img2worldMapper, &pnp, &overlay3D](SRef<Image>& image, Transform3Df &pose){
			SRef<Image>                     greyImage, binaryImage;
			std::vector<Contour2Df>			contours;
			std::vector<Contour2Df>			filtered_contours;
			std::vector<SRef<Image>>        patches;
			std::vector<Contour2Df>			recognizedContours;
			SRef<DescriptorBuffer>          recognizedPatternsDescriptors;
			std::vector<DescriptorMatch>    patternMatches;
			std::vector<Point2Df>			pattern2DPoints;
			std::vector<Point2Df>			img2DPoints;
			std::vector<Point3Df>			pattern3DPoints;

			bool marker_found = false;
			// Convert Image from RGB to grey
			imageConvertor->convert(image, greyImage, Image::ImageLayout::LAYOUT_GREY);
			for (int num_threshold = 0; !marker_found && num_threshold < NB_THRESHOLD; num_threshold++)
			{
				// Compute the current Threshold valu for image binarization
				int threshold = MIN_THRESHOLD + (MAX_THRESHOLD - MIN_THRESHOLD)*((float)num_threshold / (float)(NB_THRESHOLD - 1));
				// Convert Image from grey to black and white
				imageFilterBinary->bindTo<xpcf::IConfigurable>()->getProperty("min")->setIntegerValue(threshold);
				imageFilterBinary->bindTo<xpcf::IConfigurable>()->getProperty("max")->setIntegerValue(255);
				// Convert Image from grey to black and white
				imageFilterBinary->filter(greyImage, binaryImage);
				// Extract contours from binary image
				contoursExtractor->extract(binaryImage, contours);
				// Filter 4 edges contours to find those candidate for marker contours
				contoursFilter->filter(contours, filtered_contours);
				// Create one warpped and cropped image by contour
				perspectiveController->correct(binaryImage, filtered_contours, patches);
				// test if this last image is really a squared binary marker, and if it is the case, extract its descriptor
				if (patternDescriptorExtractor->extract(patches, filtered_contours, recognizedPatternsDescriptors, recognizedContours) != FrameworkReturnCode::_ERROR_)
				{
					// From extracted squared binary pattern, match the one corresponding to the squared binary marker
					if (patternMatcher->match(markerPatternDescriptor, recognizedPatternsDescriptors, patternMatches) == features::IDescriptorMatcher::DESCRIPTORS_MATCHER_OK)
					{
						// Reindex the pattern to create two vector of points, the first one corresponding to marker corner, the second one corresponding to the poitsn of the contour
						patternReIndexer->reindex(recognizedContours, patternMatches, pattern2DPoints, img2DPoints);
						// Compute the 3D position of each corner of the marker
						img2worldMapper->map(pattern2DPoints, pattern3DPoints);
						// Compute the pose of the camera using a Perspective n Points algorithm using only the 4 corners of the marker
						if (pnp->estimate(img2DPoints, pattern3DPoints, pose) == FrameworkReturnCode::_SUCCESS)
						{														
							marker_found = true;
						}
					}
				}
			}

			return marker_found;

		};

		// Here, Capture the two first keyframe view1, view2
		bool imageCaptured = false;
		bool fiducialDetectStart = false;
		while (!imageCaptured)
		{
			if (camera->getNextImage(view1) == SolAR::FrameworkReturnCode::_ERROR_)
				break;
#ifdef USE_IMAGES_SET
			fiducialDetectStart = true;
			if (imageViewer->display(view1) == SolAR::FrameworkReturnCode::_STOP)
				return 1;
#else
			if (imageViewer->display(view1) == SolAR::FrameworkReturnCode::_STOP)
				if (!fiducialDetectStart)
					fiducialDetectStart = true;
				else
					return 1;
#endif
			if (fiducialDetectStart && detectFiducialMarker(view1, poseFrame1))
			{
				keypointsDetector->detect(view1, keypointsView1);
				descriptorExtractor->extract(view1, keypointsView1, descriptorsView1);
				keyframe1 = xpcf::utils::make_shared<Keyframe>(keypointsView1, descriptorsView1, view1, poseFrame1);
				mapper->update(map, keyframe1, {}, {}, {});
				keyframePoses.push_back(poseFrame1); // used for display
				kfRetriever->addKeyframe(keyframe1); // add keyframe for reloc
				imageCaptured = true;
				LOG_INFO("Pose of keyframe 1: \n {}", poseFrame1.matrix());
			}
		}

		bool bootstrapOk = false;
		while (!bootstrapOk)
		{
			if (camera->getNextImage(view2) == SolAR::FrameworkReturnCode::_ERROR_)
				break;

			if (!detectFiducialMarker(view2, poseFrame2)) {
				if (imageViewer->display(view2) == SolAR::FrameworkReturnCode::_STOP)
					return 1;
				continue;
			}			
			float disTwoKeyframes = std::sqrtf(std::powf(poseFrame1(0, 3) - poseFrame2(0, 3), 2.f) + std::powf(poseFrame1(1, 3) - poseFrame2(1, 3), 2.f) + 
				std::powf(poseFrame1(2, 3) - poseFrame2(2, 3), 2.f));

			if (disTwoKeyframes < 0.1) {
				if (imageViewer->display(view2) == SolAR::FrameworkReturnCode::_STOP)
					return 1;
				continue;
			}

			keypointsDetector->detect(view2, keypointsView2);
			descriptorExtractor->extract(view2, keypointsView2, descriptorsView2);
			SRef<Frame> frame2 = xpcf::utils::make_shared<Frame>(keypointsView2, descriptorsView2, view2, keyframe1);
			matcher->match(descriptorsView1, descriptorsView2, matches);
			int nbOriginalMatches = matches.size();
			matchesFilter->filter(matches, matches, keypointsView1, keypointsView2);

			matchesOverlay->draw(view2, imageMatches, keypointsView1, keypointsView2, matches);
			if (imageViewer->display(imageMatches) == SolAR::FrameworkReturnCode::_STOP)
				return 1;

			if (keyframeSelector->select(frame2, matches)){				
				LOG_INFO("Pose of keyframe 2: \n {}", poseFrame2.matrix());
				frame2->setPose(poseFrame2);
				LOG_INFO("Nb matches for triangulation: {}\\{}", matches.size(), nbOriginalMatches);
				// Triangulate
				keyframe2 = xpcf::utils::make_shared<Keyframe>(frame2);
				triangulator->triangulate(keyframe2, matches, cloud);
				//double reproj_error = triangulator->triangulate(keypointsView1, keypointsView2, matches, std::make_pair(0, 1), poseFrame1, poseFrame2, cloud);
				mapFilter->filter(poseFrame1, poseFrame2, cloud, filteredCloud);
				keyframePoses.push_back(poseFrame2); // used for display
				mapper->update(map, keyframe2, filteredCloud, matches, {});
				kfRetriever->addKeyframe(keyframe2); // add keyframe for reloc
				if (bundling) {
					std::vector<int>firstIdxKFs = { 0,1 };
					localBundleAdjuster(firstIdxKFs, bundleReprojError);
				}
				bootstrapOk = true;
			}
		}

		LOG_INFO("Number of initial point cloud: {}", filteredCloud.size());

		auto updateData = [&mapper](const SRef<Keyframe> refKf, std::vector<CloudPoint> &localMap, std::vector<unsigned int> &idxLocalMap, SRef<Keyframe> & referenceKeyframe, SRef<Frame> &frameToTrack)
		{
			referenceKeyframe = refKf;
			LOG_DEBUG("Update new reference keyframe with id {}", referenceKeyframe->m_idx);
			frameToTrack = xpcf::utils::make_shared<Frame>(referenceKeyframe);
			frameToTrack->setReferenceKeyframe(referenceKeyframe);
			idxLocalMap.clear();
			localMap.clear();
			mapper->getLocalMapIndex(referenceKeyframe, idxLocalMap);
			for (auto it : idxLocalMap)
				localMap.push_back(mapper->getGlobalMap()->getAPoint(it));
		};			

		// check need to make a new keyframe based on all existed keyframes
		std::function<bool(const SRef<Frame>&)> checkNeedNewKfWithAllKfs = [&referenceKeyframe, &mapper, &kfRetriever, &updatedRefKf](const SRef<Frame>& newFrame) -> bool {
			std::vector < SRef <Keyframe>> ret_keyframes;
			if (kfRetriever->retrieve(newFrame, ret_keyframes) == FrameworkReturnCode::_SUCCESS) {
				if (ret_keyframes[0]->m_idx != referenceKeyframe->m_idx) {
					updatedRefKf = ret_keyframes[0];
					LOG_DEBUG("Update new reference keyframe with id {}", referenceKeyframe->m_idx);
					return true;
				}
				LOG_DEBUG("Find same reference keyframe, need make new keyframe");
				return false;
			}
			else
				return false;
		};

		// checkDisparityDistance
		std::function<bool(const SRef<Frame>&)> checkDisparityDistance = [&referenceKeyframe, &mapper, &map, &projector](const SRef<Frame>& newFrame) -> bool {
			const std::vector<CloudPoint> &cloudPoint = map->getPointCloud();
			const std::vector<Keypoint> &refKeypoints = referenceKeyframe->getKeypoints();
			const std::map<unsigned int, unsigned int> &refMapVisibility = referenceKeyframe->getVisibleMapPoints();
			std::vector<CloudPoint> cpRef;
			std::vector<Point2Df> projected2DPts, ref2DPts;

			for (auto it = refMapVisibility.begin(); it != refMapVisibility.end(); it++) {
				cpRef.push_back(cloudPoint[it->second]);
				ref2DPts.push_back(Point2Df(refKeypoints[it->first].getX(), refKeypoints[it->first].getY()));
			}
			projector->project(cpRef, projected2DPts, newFrame->getPose());

			unsigned int imageWidth = newFrame->getView()->getWidth();
			double totalMatchesDist = 0.0;
			for (int i = 0; i < projected2DPts.size(); i++)
			{
				Point2Df pt1 = ref2DPts[i];
				Point2Df pt2 = projected2DPts[i];
                totalMatchesDist += (pt1 - pt2).norm() / imageWidth;
			}
			double meanMatchesDist = totalMatchesDist / projected2DPts.size();
			LOG_DEBUG("Keyframe Selector Mean Matches Dist: {}", meanMatchesDist);
			return (meanMatchesDist > 0.07);
		};

		// Update keypoint visibility, descriptor in cloud point
		auto updateAssociateCloudPoint = [&map](SRef<Keyframe> & newKf) {
			std::map<unsigned int, unsigned int> newkf_mapVisibility = newKf->getVisibleMapPoints();
			std::map<unsigned int, int> kfCounter;
			for (auto it = newkf_mapVisibility.begin(); it != newkf_mapVisibility.end(); it++) {
				CloudPoint &cp = map->getAPoint(it->second);
				// calculate the number of connections to other keyframes
				std::map<unsigned int, unsigned int> cpKfVisibility = cp.getVisibility();
				for (auto it_kf = cpKfVisibility.begin(); it_kf != cpKfVisibility.end(); it_kf++)
					kfCounter[it_kf->first]++;
				///// update descriptor of cp: des_cp = ((des_cp * cp.getVisibility().size()) + des_buf) / (cp.getVisibility().size() + 1)
				//// TO DO
				cp.visibilityAddKeypoint(newKf->m_idx, it->first);
			}

			for (auto it = kfCounter.begin(); it != kfCounter.end(); it++)
				if ((it->first != newKf->m_idx) && (it->second > 20)) {
					newKf->addNeighborKeyframe(it->first, it->second);
				}
		};

		// find matches between unmatching keypoints in the new keyframe and the best neighboring keyframes
		auto findMatchesAndTriangulation = [&matcher, &matchesFilter, &triangulator, &mapper, &mapFilter, &kfRetriever, &camera](SRef<Keyframe> & newKf, std::vector<unsigned int> &idxBestNeighborKfs, std::vector<std::tuple<unsigned int, int, unsigned int>> &infoMatches, std::vector<CloudPoint> &cloudPoint) {
			const std::map<unsigned int, unsigned int> &newKf_mapVisibility = newKf->getVisibleMapPoints();
			const SRef<DescriptorBuffer> &newKf_des = newKf->getDescriptors();
			const std::vector<Keypoint> & newKf_kp = newKf->getKeypoints();
			Transform3Df newKf_pose = newKf->getPose();

			std::vector<bool> checkMatches(newKf_kp.size(), false);

			for (int i = 0; i < newKf_kp.size(); ++i)
				if (newKf_mapVisibility.find(i) != newKf_mapVisibility.end()) {
					checkMatches[i] = true;
				}
			
			for (int i = 0; i < idxBestNeighborKfs.size(); ++i) {				
				std::vector<int> newKf_indexKeypoints;
				for (int j = 0; j < checkMatches.size(); ++j)
					if (!checkMatches[j])
						newKf_indexKeypoints.push_back(j);

				// get neighbor keyframe i
				SRef<Keyframe> &tmpKf = mapper->getKeyframe(idxBestNeighborKfs[i]);
				Transform3Df tmpPose = tmpKf->getPose();

				// check distance between two keyframes
				float distPose = std::sqrtf(std::powf(newKf_pose(0, 3) - tmpPose(0, 3), 2.f) + std::powf(newKf_pose(1, 3) - tmpPose(1, 3), 2.f) +
					std::powf(newKf_pose(2, 3) - tmpPose(2, 3), 2.f));
				if (distPose < 0.05)
					continue;

				// Matching based on BoW
				std::vector < DescriptorMatch> tmpMatches, goodMatches;
				kfRetriever->match(newKf_indexKeypoints, newKf_des, idxBestNeighborKfs[i], tmpMatches);

				// matches filter based epipolar lines
				matchesFilter->filter(tmpMatches, tmpMatches, newKf_kp, tmpKf->getKeypoints(), newKf->getPose(), tmpKf->getPose(), camera->getIntrinsicsParameters());

				// find info to triangulate				
				std::vector<std::tuple<unsigned int, int, unsigned int>> tmpInfoMatches;
				const std::map<unsigned int, unsigned int> & tmpMapVisibility = tmpKf->getVisibleMapPoints();
				for (int j = 0; j < tmpMatches.size(); ++j) {
					unsigned int idx_newKf = tmpMatches[j].getIndexInDescriptorA();
					unsigned int idx_tmpKf = tmpMatches[j].getIndexInDescriptorB();
					if ((!checkMatches[idx_newKf]) && (tmpMapVisibility.find(idx_tmpKf) == tmpMapVisibility.end())) {
						tmpInfoMatches.push_back(std::make_tuple(idx_newKf, idxBestNeighborKfs[i], idx_tmpKf));
						goodMatches.push_back(tmpMatches[j]);
					}
				}

				// triangulation
				std::vector<CloudPoint> tmpCloudPoint, tmpFilteredCloudPoint;
				std::vector<int> indexFiltered;
				if (goodMatches.size() > 0)
					triangulator->triangulate(newKf_kp, tmpKf->getKeypoints(), newKf_des, tmpKf->getDescriptors(), goodMatches, 
						std::make_pair(newKf->m_idx, idxBestNeighborKfs[i]), newKf->getPose(), tmpKf->getPose(), tmpCloudPoint);

				// filter cloud points
				if (tmpCloudPoint.size() > 0)
					mapFilter->filter(newKf->getPose(), tmpKf->getPose(), tmpCloudPoint, tmpFilteredCloudPoint, indexFiltered);
				for (int i = 0; i < indexFiltered.size(); ++i) {
					checkMatches[std::get<0>(tmpInfoMatches[indexFiltered[i]])] = true;
					infoMatches.push_back(tmpInfoMatches[indexFiltered[i]]);
					cloudPoint.push_back(tmpFilteredCloudPoint[i]);
				}
			}

		};

		// check and fuse cloud point
		auto fuseCloudPoint = [&mapper, &projector, &matcher, &map](SRef<Keyframe> &newKeyframe, std::vector<unsigned int> &idxNeigborKfs, std::vector<std::tuple<unsigned int, int, unsigned int>> &infoMatches, std::vector<CloudPoint> &newCloudPoint) {
			std::vector<bool> checkMatches(newCloudPoint.size(), true);
			std::vector<SRef<DescriptorBuffer>> desNewCloudPoint;
			for (auto &it_cp : newCloudPoint) {
				desNewCloudPoint.push_back(it_cp.getDescriptor());
			}			

			for (int i = 0; i < idxNeigborKfs.size(); ++i) {
				// get a neighbor
				SRef<Keyframe> &neighborKf = mapper->getKeyframe(idxNeigborKfs[i]);
				const std::map<unsigned int, unsigned int> mapVisibilitiesNeighbor = neighborKf->getVisibleMapPoints();

				//  projection points
				std::vector< Point2Df > projected2DPts;
				projector->project(newCloudPoint, projected2DPts, neighborKf->getPose());

				std::vector<DescriptorMatch> allMatches;
				matcher->matchInRegion(projected2DPts, desNewCloudPoint, neighborKf, allMatches, 5.f);

				for (int j = 0; j < allMatches.size(); ++j) {
					int idxNewCloudPoint = allMatches[j].getIndexInDescriptorA();
					int idxKpNeighbor = allMatches[j].getIndexInDescriptorB();
					if (!checkMatches[idxNewCloudPoint])
						continue;
					std::tuple<unsigned int, int, unsigned int> infoMatch = infoMatches[idxNewCloudPoint];

					// check this cloud point is created from the same neighbor keyframe
					if (std::get<1>(infoMatch) == idxNeigborKfs[i])
						continue;
					
					// check if have a cloud point in the neighbor keyframe is coincide with this cloud point.
					auto it_cp = mapVisibilitiesNeighbor.find(idxKpNeighbor);
					if (it_cp != mapVisibilitiesNeighbor.end()) {
						// fuse
						CloudPoint &old_cp = map->getAPoint(it_cp->second);
						old_cp.visibilityAddKeypoint(newKeyframe->m_idx, std::get<0>(infoMatch));
						old_cp.visibilityAddKeypoint(std::get<1>(infoMatch), std::get<2>(infoMatch));

						newKeyframe->addVisibleMapPoint(std::get<0>(infoMatch), it_cp->second);
						mapper->getKeyframe(std::get<1>(infoMatch))->addVisibleMapPoint(std::get<2>(infoMatch), it_cp->second);

						checkMatches[idxNewCloudPoint] = false;
					}
				}
			}

			std::vector<std::tuple<unsigned int, int, unsigned int>> tmpInfoMatches;
			std::vector<CloudPoint> tmpNewCloudPoint;
			for (int i = 0; i < checkMatches.size(); ++i)
				if (checkMatches[i]) {
					tmpInfoMatches.push_back(infoMatches[i]);
					tmpNewCloudPoint.push_back(newCloudPoint[i]);
				}
			tmpInfoMatches.swap(infoMatches);
			tmpNewCloudPoint.swap(newCloudPoint);
		};

		// process to add a new keyframe
		auto processNewKeyframe = [&findMatchesAndTriangulation, &updateAssociateCloudPoint, &fuseCloudPoint, &map, &kfRetriever, &mapper, &referenceKeyframe, &projector, &matcher](SRef<Frame> &newFrame) -> SRef<Keyframe> {
			// create a new keyframe from the current frame
			SRef<Keyframe> newKeyframe = xpcf::utils::make_shared<Keyframe>(newFrame);
			// Add to BOW retrieval			
			kfRetriever->addKeyframe(newKeyframe);
			// Update keypoint visibility, descriptor in cloud point and connections between new keyframe with other keyframes
			updateAssociateCloudPoint(newKeyframe);
			// get best neighbor keyframes
			std::vector<unsigned int> idxBestNeighborKfs = newKeyframe->getBestNeighborKeyframes(4);
			// find matches between unmatching keypoints in the new keyframe and the best neighboring keyframes
			std::vector<std::tuple<unsigned int, int, unsigned int>> infoMatches; // first: index of kp in newKf, second: index of Kf, third: index of kp in Kf.
			std::vector<CloudPoint> newCloudPoint;
			findMatchesAndTriangulation(newKeyframe, idxBestNeighborKfs, infoMatches, newCloudPoint);
			LOG_DEBUG("Number of new 3D points before fusing: {}", newCloudPoint.size());
			if (newCloudPoint.size() > 0) {
				// fuse duplicate points
				std::vector<unsigned int> idxNeigborKfs = newKeyframe->getBestNeighborKeyframes(10);
				fuseCloudPoint(newKeyframe, idxNeigborKfs, infoMatches, newCloudPoint);				
			}
			LOG_INFO("Keyframe: {} -> Number of new 3D points: {}", newKeyframe->m_idx, newCloudPoint.size());
			// mapper update
			mapper->update(map, newKeyframe, newCloudPoint, infoMatches);
			return newKeyframe;
		};		

		// Prepare for tracking
		lastPose = poseFrame2;
		updateData(keyframe2, localMap, idxLocalMap, referenceKeyframe, frameToTrack);

		// Start tracking
		clock_t start, end;
		int count = 0;
		start = clock();
		while (true)
		{
			// Get current image
			camera->getNextImage(view);
			keypointsDetector->detect(view, keypoints);
			LOG_DEBUG("Number of keypoints: {}", keypoints.size());
			descriptorExtractor->extract(view, keypoints, descriptors);		
			newFrame = xpcf::utils::make_shared<Frame>(keypoints, descriptors, view, referenceKeyframe);
			// match current keypoints with the keypoints of the Keyframe
			matcher->match(frameToTrack->getDescriptors(), descriptors, matches);			
			matchesFilter->filter(matches, matches, frameToTrack->getKeypoints(), keypoints);			

			std::vector<Point2Df> pt2d;
			std::vector<Point3Df> pt3d;
			std::vector<CloudPoint> foundPoints;
			std::vector<DescriptorMatch> foundMatches;
			std::vector<DescriptorMatch> remainingMatches;
			corr2D3DFinder->find(frameToTrack, newFrame, matches, map, pt3d, pt2d, foundMatches, remainingMatches);						
			// display matches
			imageMatches = view->copy();

			std::vector<Point2Df> imagePoints_inliers;
			std::vector<Point3Df> worldPoints_inliers;
			if (pnpRansac->estimate(pt2d, pt3d, imagePoints_inliers, worldPoints_inliers, newFramePose, lastPose) == FrameworkReturnCode::_SUCCESS) {
				LOG_DEBUG(" pnp inliers size: {} / {}", worldPoints_inliers.size(), pt3d.size());
				LOG_DEBUG("Estimated pose: \n {}", newFramePose.matrix());
				// Set the pose of the new frame
				newFrame->setPose(newFramePose);

				// refine pose and update map visibility of frame
				{
					// get all keypoints of the new frame
					const std::vector<Keypoint> &keypoints = newFrame->getKeypoints();

					//  projection points
					std::vector< Point2Df > projected2DPts;
					projector->project(localMap, projected2DPts, newFrame->getPose());
					
					std::vector<SRef<DescriptorBuffer>> desAllLocalMap;
					for (auto &it_cp : localMap) {
						desAllLocalMap.push_back(it_cp.getDescriptor());
					}

					// matches feature in region
					std::vector<DescriptorMatch> allMatches;
					matcher->matchInRegion(projected2DPts, desAllLocalMap, newFrame, allMatches, 5.f);

					std::vector<Point2Df> pt2d;
					std::vector<Point3Df> pt3d;
					std::map<unsigned int, unsigned int> newMapVisibility;


					for (auto &it_match : allMatches) {
						int idx_2d = it_match.getIndexInDescriptorB();
						int idx_3d = it_match.getIndexInDescriptorA();
						pt2d.push_back(Point2Df(keypoints[idx_2d].getX(), keypoints[idx_2d].getY()));
						pt3d.push_back(Point3Df(localMap[idx_3d].getX(), localMap[idx_3d].getY(), localMap[idx_3d].getZ()));
						newMapVisibility[idx_2d] = idxLocalMap[idx_3d];
					}
					
					// pnp optimization
					Transform3Df refinedPose;
					pnp->estimate(pt2d, pt3d, refinedPose, newFrame->getPose());
					newFrame->setPose(refinedPose);

					// update map visibility of current frame
					newFrame->addVisibleMapPoints(newMapVisibility);
					LOG_DEBUG("Number of map visibility of frame to track: {}", newMapVisibility.size());
					overlay2D->drawCircles(pt2d, imageMatches);
					overlay3D->draw(refinedPose, imageMatches);
				}
				LOG_DEBUG("Refined pose: \n {}", newFrame->getPose().matrix());
				lastPose = newFrame->getPose();

				// check need new keyframe
				if (keyframeSelector->select(newFrame, checkDisparityDistance))
				{
					if (keyframeSelector->select(newFrame, checkNeedNewKfWithAllKfs)) {
						updateData(updatedRefKf, localMap, idxLocalMap, referenceKeyframe, frameToTrack);
					}
					else {
						SRef<Keyframe> newKeyframe = processNewKeyframe(newFrame);
						if (bundling) {
							// Local bundle adjustment
							std::vector<unsigned int>bestIdx = newKeyframe->getBestNeighborKeyframes(10);
							std::vector<int> windowIdxBundling;
							for (auto it_best : bestIdx)
								windowIdxBundling.push_back(it_best);
							windowIdxBundling.push_back(newKeyframe->m_idx);
							localBundleAdjuster(windowIdxBundling, bundleReprojError);
						}
						// update data
						updateData(newKeyframe, localMap, idxLocalMap, referenceKeyframe, frameToTrack);
						// add keyframe pose to display
						keyframePoses.push_back(newKeyframe->getPose());
						LOG_INFO("Number of keyframe: {} -> cloud current size: {} \n", mapper->getKeyframes().size(), map->getPointCloud().size());
					}
				}
				else
				{
					// update frame to track
					frameToTrack = newFrame;
				}

				framePoses.push_back(newFrame->getPose()); // used for display

				isLostTrack = false;	// tracking is good

			}
			else {
				LOG_DEBUG("Pose estimation has failed");
				isLostTrack = true;		// lost tracking
				// reloc
				std::vector < SRef <Keyframe>> ret_keyframes;
				if (kfRetriever->retrieve(newFrame, ret_keyframes) == FrameworkReturnCode::_SUCCESS) {
					// update data
					updateData(ret_keyframes[0], localMap, idxLocalMap, referenceKeyframe, frameToTrack);
					lastPose = referenceKeyframe->getPose();
					LOG_DEBUG("Retrieval Success");
				}
				else
					LOG_DEBUG("Retrieval Failed");
			}

			LOG_DEBUG("Nb of Local Map / World Map: {} / {}", localMap.size(), map->getPointCloud().size());
			LOG_DEBUG("Index of current reference keyframe: {}", referenceKeyframe->m_idx);

			// display matches and a cube on the fiducial marker
			if (imageViewer->display(imageMatches) == FrameworkReturnCode::_STOP)
				break;

			// display point cloud
			if (viewer3DPoints->display(map->getPointCloud(), lastPose, keyframePoses, framePoses, localMap) == FrameworkReturnCode::_STOP)
				break;

			count++;
		}

		// display stats on frame rate
		end = clock();
		double duration = double(end - start) / CLOCKS_PER_SEC;
		printf("\n\nElasped time is %.2lf seconds.\n", duration);
		printf("Number of processed frame per second : %8.2f\n", count / duration);

	}
	catch (xpcf::Exception &e)
	{
		LOG_DEBUG("{}", e.what());
		return -1;
	}

	return 0;
}
