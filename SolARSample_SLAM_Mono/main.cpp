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

#include <boost/log/core.hpp>

// ADD XPCF HEADERS HERE
#include "xpcf/xpcf.h"
#include "core/Log.h"
// ADD COMPONENTS HEADERS HERE
#include "api/input/devices/ICamera.h"
#include "api/input/devices/IARDevice.h"
#include "api/features/IDescriptorsExtractorFromImage.h"
#include "api/storage/IMapManager.h"
#include "api/display/I3DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/ICovisibilityGraphManager.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/ICameraParametersManager.h"
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

// if macro defined will launch semantic segmentation also  
//#define SEMANTIC_ID

#ifdef SEMANTIC_ID
#include "api/segm/ISemanticSegmentation.h"
#include "api/display/IMaskOverlay.h"
#endif

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
		auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
		auto pointCloudManager = xpcfComponentManager->resolve<IPointCloudManager>();
		auto keyframesManager = xpcfComponentManager->resolve<IKeyframesManager>();
        auto cameraParametersManager = xpcfComponentManager->resolve<ICameraParametersManager>();
		auto covisibilityGraphManager = xpcfComponentManager->resolve<ICovisibilityGraphManager>();
		auto keyframeRetriever = xpcfComponentManager->resolve<IKeyframeRetriever>();
		auto mapManager = xpcfComponentManager->resolve<IMapManager>();      
        auto descriptorExtractorFromImage = xpcfComponentManager->resolve<features::IDescriptorsExtractorFromImage>();
		auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>("slam");
		auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		auto loopDetector = xpcfComponentManager->resolve<loop::ILoopClosureDetector>();
		auto loopCorrector = xpcfComponentManager->resolve<loop::ILoopCorrector>();
		auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();
        auto trackableLoader = xpcfComponentManager->resolve<input::files::ITrackableLoader>();
        auto fiducialMarkerPoseEstimator = xpcfComponentManager->resolve<solver::pose::ITrackablePose>();
		auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
		auto undistortKeypoints = xpcfComponentManager->resolve<api::geom::IUndistortPoints>();
		auto bootstrapper = xpcfComponentManager->resolve<slam::IBootstrapper>();
		auto tracking = xpcfComponentManager->resolve<slam::ITracking>();
		auto mapping = xpcfComponentManager->resolve<slam::IMapping>();
		auto gArDevice = xpcfComponentManager->resolve<input::devices::IARDevice>();
#ifdef SEMANTIC_ID
		auto segmentor = xpcfComponentManager->resolve<segm::ISemanticSegmentation>();
		auto maskOverlay = xpcfComponentManager->resolve<display::IMaskOverlay>();
		auto imageViewer2 = xpcfComponentManager->resolve<display::IImageViewer>("segmentation");
		auto fnSegment = [&segmentor, &maskOverlay, &imageViewer2](SRef<Frame> frame) {
			SRef<Image> mask;
			if (segmentor->segment(frame->getView(), mask) != FrameworkReturnCode::_SUCCESS)
				return false;
			for (int i = 0; i < static_cast<int>(frame->getKeypoints().size()); i++) {
				int classId = static_cast<int>(mask->getPixel<uint8_t>(static_cast<int>(frame->getKeypoint(i).getY()),
					static_cast<int>(frame->getKeypoint(i).getX())));
				frame->updateKeypointClassId(i, classId);
			}
			SRef<Image> view2 = frame->getView()->copy();
			maskOverlay->draw(view2, mask);
			if (imageViewer2->display(view2) == FrameworkReturnCode::_STOP)
				return false;
			return true;
		};
#endif 
		LOG_INFO("Loaded all components");

		// get camera parameters
		CameraRigParameters camRigParams = gArDevice->getCameraParameters();
		CameraParameters camParams = camRigParams.cameraParams[0];
		LOG_DEBUG("Intrincic\Distortion parameters : \n{}\n\n{}", camParams.intrinsic, camParams.distortion);
		// get properties
		float minWeightNeighbor = mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
		float reprojErrorThreshold = mapManager->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();
		int hasPose = bootstrapper->bindTo<xpcf::IConfigurable>()->getProperty("hasPose")->getIntegerValue();

		// start camera
		if (camera->start() != FrameworkReturnCode::_SUCCESS)
		{
			LOG_ERROR("Camera cannot start");
			return -1;
		}

        // Load map from file
		bool bootstrapOk = false;
		SRef<Keyframe> keyframe2;
		std::vector<Transform3Df> framePoses;
        if (mapManager->loadFromFile() == FrameworkReturnCode::_SUCCESS)
        {
            LOG_INFO("Load map done!");
			if (keyframesManager->getKeyframe(0, keyframe2) != FrameworkReturnCode::_SUCCESS)
				return -1;
			// Prepare for tracking
			tracking->setNewKeyframe(keyframe2);
			framePoses.push_back(keyframe2->getPose());
			bootstrapOk = true;
			LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
			LOG_INFO("Number of initial keyframes: {}", keyframesManager->getNbKeyframes());
        }
        else
        {
            LOG_WARNING("Failed to load map from file");
			LOG_INFO("Initialization from scratch");
			if (hasPose) {
				SRef<Trackable> trackable;
				if (trackableLoader->loadTrackable(trackable) != FrameworkReturnCode::_SUCCESS){
					LOG_ERROR("cannot load fiducial marker");
					return -1;
				}
				if (fiducialMarkerPoseEstimator->setTrackable(trackable) != FrameworkReturnCode::_SUCCESS)
				{
					LOG_ERROR("cannot set fiducial marker as a trackable ofr fiducial marker pose estimator");
					return -1;
				}
			}
        }          	

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

		// Start tracking
		clock_t start, end;
		int count = 0;
		int countNewKeyframes = 0;		
		start = clock();

        cameraParametersManager->addCameraParameters(camParams);
        uint32_t cameraID = camParams.id;

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
			if (descriptorExtractorFromImage->extract(view, keypoints, descriptors) != FrameworkReturnCode::_SUCCESS)
				continue;
			// set keypoints' class id to -1 which means no semantic information
			std::for_each(keypoints.begin(), keypoints.end(), [](auto& p) { p.setClassId(-1); });
			// undistort keypoints
			undistortKeypoints->undistort(keypoints, camParams, undistortedKeypoints);
            frame = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, view, cameraID);
			// check bootstrap
			if (!bootstrapOk) {
				Transform3Df pose;
				if (hasPose && (fiducialMarkerPoseEstimator->estimate(view, camParams, pose) == FrameworkReturnCode::_SUCCESS))
					frame->setPose(pose);
				if (bootstrapper->process(frame, displayImage) == FrameworkReturnCode::_SUCCESS) {
					keyframesManager->getKeyframe(1, keyframe2);
					tracking->setNewKeyframe(keyframe2);
					framePoses.push_back(keyframe2->getPose());
#ifdef SEMANTIC_ID
					SRef<Keyframe> keyframe0, keyframe1;
					keyframesManager->getKeyframe(0, keyframe0);
					keyframesManager->getKeyframe(1, keyframe1);

					if (!fnSegment(boost::static_pointer_cast<Frame>(keyframe0)))
						break;
					std::vector<SRef<CloudPoint>> pointCloud;
					pointCloudManager->getAllPoints(pointCloud);
					for (auto& pt : pointCloud) {
						auto visibility = pt->getVisibility();
						int classId = keyframe0->getUndistortedKeypoint(visibility[0]).getClassId();
						pt->setSemanticId(classId);
						keyframe1->updateKeypointClassId(visibility[1], classId);
					}	
#endif
					bootstrapOk = true;
					LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
					LOG_INFO("Number of initial keyframes: {}", keyframesManager->getNbKeyframes());
				}
			}
			else {
				// tracking
				if (tracking->process(frame, displayImage) == FrameworkReturnCode::_SUCCESS) {
					// used for display
					framePoses.push_back(frame->getPose());	
					// mapping
					if (tracking->checkNeedNewKeyframe()) {
#ifdef SEMANTIC_ID
						if (!fnSegment(frame))
							break;
#endif 
						if (mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
							LOG_DEBUG("New keyframe id: {}", keyframe->getId());
							// Local bundle adjustment
							std::vector<uint32_t> bestIdx;
							covisibilityGraphManager->getNeighbors(keyframe->getId(), minWeightNeighbor, bestIdx, NB_LOCALKEYFRAMES);
							bestIdx.push_back(keyframe->getId());
							LOG_DEBUG("Nb keyframe to local bundle: {}", bestIdx.size());
							double bundleReprojError = bundler->bundleAdjustment(bestIdx);
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
									bundler->bundleAdjustment();
									// map pruning
									mapManager->pointCloudPruning();
									mapManager->keyframePruning();
								}
							}

						// update reference keyframe to tracking
						tracking->setNewKeyframe(keyframe);
						}
					}
				}				
			}
			
			// draw cube
			if (!frame->getPose().isApprox(Transform3Df::Identity()))
                overlay3D->draw(frame->getPose(), camParams, displayImage);
			// display matches and a cube on the origin of coordinate system
			if (imageViewer->display(displayImage) == FrameworkReturnCode::_STOP)
				break;	
			// display point cloud
			if (bootstrapOk && !fnDisplay(framePoses))
				break;
			count++;
		}

		// display stats on frame rate
		end = clock();
		double duration = double(end - start) / CLOCKS_PER_SEC;
		printf("\n\nElasped time is %.2lf seconds.\n", duration);
		printf("Number of processed frame per second : %8.2f\n", count / duration);

		// run global BA before exit
		bundler->bundleAdjustment();
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
