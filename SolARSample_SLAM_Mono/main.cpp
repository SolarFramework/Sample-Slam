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

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <cmath>
#include <boost/log/core.hpp>

// ADD XPCF HEADERS HERE
#include "xpcf/xpcf.h"
#include "core/Log.h"
// ADD COMPONENTS HEADERS HERE
#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/storage/IMapManager.h"
#include "api/display/I3DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/ICovisibilityGraphManager.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
#include "api/input/files/ITrackableLoader.h"
#include "api/solver/pose/ITrackablePose.h"
#include "api/solver/map/IBundler.h"
#include "api/geom/IUndistortPoints.h"
#include "api/slam/IBootstrapper.h"
#include "api/slam/ITracking.h"
#include "api/slam/IMapping.h"

#define NB_NEWKEYFRAMES_LOOP 10
#define NB_LOCALKEYFRAMES 10

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::api::storage;
using namespace SolAR::api::reloc;

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

		std::string configxml = std::string("SolARSample_SLAM_Mono_conf.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);
		if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file {}", configxml.c_str())
				return -1;
		}
		// declare and create components
		LOG_INFO("Start creating components");
		LOG_INFO("Resolving camera ");
		auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
		LOG_INFO("Resolving point cloud manager");
		auto pointCloudManager = xpcfComponentManager->resolve<IPointCloudManager>();
		LOG_INFO("Resolving key frames manager");
		auto keyframesManager = xpcfComponentManager->resolve<IKeyframesManager>();
		LOG_INFO("Resolving covisibility graph");
		auto covisibilityGraphManager = xpcfComponentManager->resolve<ICovisibilityGraphManager>();
		LOG_INFO("Resolving key frame retriever");
		auto keyframeRetriever = xpcfComponentManager->resolve<IKeyframeRetriever>();
		LOG_INFO("Resolving key mapper");
		auto mapManager = xpcfComponentManager->resolve<IMapManager>();
		LOG_INFO("Resolving key points detector");
		auto  keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
		LOG_INFO("Resolving descriptor extractor");
		auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
		LOG_INFO("Resolving image viewer");
		auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
		LOG_INFO("Resolving viewer3D points");
		auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		LOG_INFO("Resolving loop detector");
		auto loopDetector = xpcfComponentManager->resolve<loop::ILoopClosureDetector>();
		LOG_INFO("Resolving loop corrector");
		auto loopCorrector = xpcfComponentManager->resolve<loop::ILoopCorrector>();
		LOG_INFO("Resolving 3D overlay");
		auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();
        LOG_INFO("Resolving Trackable Loader");
        auto trackableLoader = xpcfComponentManager->resolve<input::files::ITrackableLoader>();
		LOG_INFO("Resolving Fiducial marker pose");
        auto fiducialMarkerPoseEstimator = xpcfComponentManager->resolve<solver::pose::ITrackablePose>();
		LOG_INFO("Resolving bundle adjustment");
		auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
		LOG_INFO("Resolving undistort points");
		auto undistortKeypoints = xpcfComponentManager->resolve<api::geom::IUndistortPoints>();
		LOG_INFO("Resolving bootstrapper");
		auto bootstrapper = xpcfComponentManager->resolve<slam::IBootstrapper>();
		LOG_INFO("Resolving tracking");
		auto tracking = xpcfComponentManager->resolve<slam::ITracking>();
		LOG_INFO("Resolving mapping");
		auto mapping = xpcfComponentManager->resolve<slam::IMapping>();
		LOG_INFO("Loaded all components");

		// initialize pose estimation with the camera intrinsic parameters (please refer to the use of intrinsic parameters file)
		CamCalibration calibration = camera->getIntrinsicsParameters();
		CamDistortion distortion = camera->getDistortionParameters();
		overlay3D->setCameraParameters(calibration, distortion);
		loopDetector->setCameraParameters(calibration, distortion);
		loopCorrector->setCameraParameters(calibration, distortion);
		fiducialMarkerPoseEstimator->setCameraParameters(calibration, distortion);
		bootstrapper->setCameraParameters(calibration, distortion);
		tracking->setCameraParameters(calibration, distortion);
		mapping->setCameraParameters(calibration, distortion);
		undistortKeypoints->setCameraParameters(calibration, distortion);
		LOG_DEBUG("Intrincic parameters : \n {}", calibration);
		// get properties
		float minWeightNeighbor = mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
		float reprojErrorThreshold = mapManager->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();

		if (camera->start() != FrameworkReturnCode::_SUCCESS)
		{
			LOG_ERROR("Camera cannot start");
			return -1;
		}

        // Load map from file
        if (mapManager->loadFromFile() == FrameworkReturnCode::_SUCCESS)
        {
            LOG_INFO("Load map done!");
        }
        else
        {
            LOG_WARNING("Failed to load map from file");
        }

        SRef<Keyframe>	keyframe2;
        if (pointCloudManager->getNbPoints() > 0)
        {
            // Map loaded from file and not empty
            if (keyframesManager->getKeyframe(0, keyframe2) != FrameworkReturnCode::_SUCCESS)
            {
                return -1; //FrameworkReturnCode::_ERROR_;
            }
        }
        else
        {
            // Either no or empty map files
            LOG_INFO("Initialization from scratch");
            SRef<Trackable> trackable;
            if (trackableLoader->loadTrackable(trackable) != FrameworkReturnCode::_SUCCESS)
            {
                LOG_ERROR("cannot load fiducial marker");
                return -1;
            }
            else
            {
                if (fiducialMarkerPoseEstimator->setTrackable(trackable)!= FrameworkReturnCode::_SUCCESS)
                {
                    LOG_ERROR("cannot set fiducial marker as a trackable ofr fiducial marker pose estimator");
                    return -1;
                }
            }
            bool bootstrapOk = false;
            while (!bootstrapOk) {
                SRef<Image> image, view;
                camera->getNextImage(image);
                Transform3Df pose = Transform3Df::Identity();
                fiducialMarkerPoseEstimator->estimate(image, pose);
                if (bootstrapper->process(image, view, pose) == FrameworkReturnCode::_SUCCESS) {
                    double bundleReprojError = bundler->bundleAdjustment(calibration, distortion);
                    bootstrapOk = true;
                }
                if (!pose.isApprox(Transform3Df::Identity()))
                    overlay3D->draw(pose, view);
                if (imageViewer->display(view) == SolAR::FrameworkReturnCode::_STOP)
                    return 1;
            }
            keyframesManager->getKeyframe(1, keyframe2);
        }

		LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
		LOG_INFO("Number of initial keyframes: {}", keyframesManager->getNbKeyframes());

		// display point cloud function
		auto fnDisplay = [&keyframesManager, &pointCloudManager, &viewer3DPoints](const std::vector<Transform3Df>& framePoses) {
			// get all keyframes and point cloud
			std::vector<Transform3Df>   keyframePoses;
			std::vector<SRef<Keyframe>> allKeyframes;
			keyframesManager->getAllKeyframes(allKeyframes);
			for (auto const &it : allKeyframes)
				keyframePoses.push_back(it->getPose());
			std::vector<SRef<CloudPoint>> pointCloud;
			pointCloudManager->getAllPoints(pointCloud);
			// display point cloud 
			if (framePoses.size() == 0
				|| viewer3DPoints->display(pointCloud, framePoses.back(), keyframePoses, framePoses) == FrameworkReturnCode::_STOP)
				return false;
			else
				return true;
		};

		// Prepare for tracking
		tracking->updateReferenceKeyframe(keyframe2);

		// init display point cloud
		fnDisplay({ keyframe2->getPose() });

		// Start tracking
		clock_t start, end;
		int count = 0;
		int countNewKeyframes = 0;
		std::vector<Transform3Df>   framePoses{ keyframe2->getPose() };
		start = clock();
		while (true)
		{
			SRef<Image>											view, displayImage;
			std::vector<Keypoint>								keypoints, undistortedKeypoints;
			SRef<DescriptorBuffer>                              descriptors;
			SRef<Frame>                                         frame;
			SRef<Keyframe>										keyframe;
			// Get current image
			if (camera->getNextImage(view) != FrameworkReturnCode::_SUCCESS)
				break;
			// feature extraction
			keypointsDetector->detect(view, keypoints);
			if (keypoints.size() == 0)
				continue;
			descriptorExtractor->extract(view, keypoints, descriptors);
			// undistort keypoints
			undistortKeypoints->undistort(keypoints, undistortedKeypoints);
			frame = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, view);
			// tracking
			if (tracking->process(frame, displayImage) == FrameworkReturnCode::_SUCCESS) {
				// used for display
				framePoses.push_back(frame->getPose());
				// draw cube
				overlay3D->draw(frame->getPose(), displayImage);
				// mapping
				if (mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
					LOG_DEBUG("New keyframe id: {}", keyframe->getId());
					// Local bundle adjustment
					std::vector<uint32_t> bestIdx, bestIdxToOptimize;
					covisibilityGraphManager->getNeighbors(keyframe->getId(), minWeightNeighbor, bestIdx);
					if (bestIdx.size() < NB_LOCALKEYFRAMES)
						bestIdxToOptimize = bestIdx;
					else
						bestIdxToOptimize.insert(bestIdxToOptimize.begin(), bestIdx.begin(), bestIdx.begin() + NB_LOCALKEYFRAMES);
					bestIdxToOptimize.push_back(keyframe->getId());
					LOG_DEBUG("Nb keyframe to local bundle: {}", bestIdxToOptimize.size());
					double bundleReprojError = bundler->bundleAdjustment(calibration, distortion, bestIdxToOptimize);
					// local map pruning
					std::vector<SRef<CloudPoint>> localPointCloud;
					mapManager->getLocalPointCloud(keyframe, 1.0, localPointCloud);
					int nbRemovedCP = mapManager->pointCloudPruning(localPointCloud);
					std::vector<SRef<Keyframe>> localKeyframes;
					keyframesManager->getKeyframes(bestIdx, localKeyframes);
					int nbRemovedKf = mapManager->keyframePruning(localKeyframes);
					LOG_DEBUG("Nb of pruning cloud points / keyframes: {} / {}", nbRemovedCP, nbRemovedKf);
					// loop closure
					countNewKeyframes++;
					if (countNewKeyframes >= NB_NEWKEYFRAMES_LOOP) {
						SRef<Keyframe> detectedLoopKeyframe;
						Transform3Df sim3Transform;
						std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
						if (loopDetector->detect(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
							// detected loop keyframe
							LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
							LOG_INFO("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
							LOG_INFO("sim3Transform: \n{}", sim3Transform.matrix());
							// performs loop correction 
							countNewKeyframes = 0;
							loopCorrector->correct(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);
							// Loop optimisation
							bundler->bundleAdjustment(calibration, distortion);
							// map pruning
							mapManager->pointCloudPruning();
							mapManager->keyframePruning();
						}
					}					
				}
				// update reference keyframe
				if (keyframe) {
					tracking->updateReferenceKeyframe(keyframe);
				}
			}
			count++;

			// display matches and a cube on the origin of coordinate system
			if (imageViewer->display(displayImage) == FrameworkReturnCode::_STOP)
				break;

			// display point cloud
			if (!fnDisplay(framePoses))
				break;
		}

		// display stats on frame rate
		end = clock();
		double duration = double(end - start) / CLOCKS_PER_SEC;
		printf("\n\nElasped time is %.2lf seconds.\n", duration);
		printf("Number of processed frame per second : %8.2f\n", count / duration);

		// run global BA before exit
		bundler->bundleAdjustment(calibration, distortion);
		mapManager->pointCloudPruning();
		mapManager->keyframePruning();
		LOG_INFO("Nb keyframes of map: {}", keyframesManager->getNbKeyframes());
		LOG_INFO("Nb cloud points of map: {}", pointCloudManager->getNbPoints());

		// visualize final map	
		while (fnDisplay(framePoses)) {}

		// Save map
		mapManager->saveToFile();
	}
	catch (xpcf::Exception e)
	{
		LOG_ERROR("The following exception has been catch : {}", e.what());
		return -1;
	}

	return 0;
}
