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
#include "xpcf/threading/BaseTask.h"
#include "xpcf/threading/DropBuffer.h"
#include "core/Log.h"
// ADD COMPONENTS HEADERS HERE
#include "api/input/devices/ICamera.h"
#include "api/features/IDescriptorsExtractorFromImage.h"
#include "api/storage/IMapManager.h"
#include "api/display/I3DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/ICovisibilityGraphManager.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/input/files/ITrackableLoader.h"
#include "api/solver/pose/ITrackablePose.h"
#include "api/solver/map/IBundler.h"
#include "api/geom/IUndistortPoints.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
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

		std::string configxml = std::string("SolARSample_SLAM_Multi_conf.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);
		if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file {}", configxml.c_str())
				return -1;
		}

		// declare and create components
		LOG_INFO("Start creating components");
		auto pointCloudManager = xpcfComponentManager->resolve<IPointCloudManager>();
		auto keyframesManager = xpcfComponentManager->resolve<IKeyframesManager>();
		auto covisibilityGraphManager = xpcfComponentManager->resolve<ICovisibilityGraphManager>();
		auto keyframeRetriever = xpcfComponentManager->resolve<IKeyframeRetriever>();
		auto mapManager = xpcfComponentManager->resolve<IMapManager>();
		auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
		auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractorFromImage>();
		auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
		auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();
        auto trackableLoader = xpcfComponentManager->resolve<input::files::ITrackableLoader>();
        auto fiducialMarkerPoseEstimator = xpcfComponentManager->resolve<solver::pose::ITrackablePose>();
        auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
		auto globalBundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
		auto undistortKeypoints = xpcfComponentManager->resolve<api::geom::IUndistortPoints>();
		auto loopDetector = xpcfComponentManager->resolve<loop::ILoopClosureDetector>();
		auto loopCorrector = xpcfComponentManager->resolve<loop::ILoopCorrector>();
		auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();
		auto bootstrapper = xpcfComponentManager->resolve<slam::IBootstrapper>();
		auto tracking = xpcfComponentManager->resolve<slam::ITracking>();
		auto mapping = xpcfComponentManager->resolve<slam::IMapping>();
		LOG_INFO("Loaded all components");

		// get camera parameters
		CameraParameters camParams = camera->getParameters();
		LOG_DEBUG("Intrincic\Distortion parameters : \n{}\n\n{}", camParams.intrinsic, camParams.distortion);

		// get properties
		float minWeightNeighbor = mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
		float reprojErrorThreshold = mapManager->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();

		if (camera->start() != FrameworkReturnCode::_SUCCESS)
		{
			LOG_ERROR("Camera cannot start");
			return -1;
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

		xpcf::DropBuffer<SRef<Image>>												m_dropBufferCamImageCapture;		
		xpcf::DropBuffer<SRef<Frame>>												m_dropBufferAddKeyframe;
		xpcf::DropBuffer<SRef<Keyframe>>											m_dropBufferNewKeyframeLoop;
		xpcf::DropBuffer<SRef<Image>>												m_dropBufferDisplay;
		xpcf::DropBuffer<SRef<Frame>>												m_dropBufferFrameTracking;
		xpcf::DropBuffer<SRef<Frame>>												m_dropBufferFrameBootstrap;
		bool stop = false;
		bool bootstrapOk = false;
		SRef<Keyframe> keyframe2;
		std::vector<Transform3Df> framePoses;
		bool isMappingIdle = true;
		bool isLoopIdle = true;
		int countNewKeyframes = 0;

		// Load map from file
		if (mapManager->loadFromFile() == FrameworkReturnCode::_SUCCESS)
		{
			LOG_INFO("Load map done!");
		}
		else
		{
			LOG_WARNING("Failed to load map from file");
		}

		if (pointCloudManager->getNbPoints() > 0)
		{
			// Map loaded from file and not empty
			if (keyframesManager->getKeyframe(0, keyframe2) != FrameworkReturnCode::_SUCCESS)
			{
				return -1; //FrameworkReturnCode::_ERROR_;
			}
			tracking->setNewKeyframe(keyframe2);
			framePoses.push_back(keyframe2->getPose());
			LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
			LOG_INFO("Number of initial keyframes: {}", keyframesManager->getNbKeyframes());
			bootstrapOk = true;
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
        }

		// Camera image capture task
		auto fnCamImageCapture = [&]()
		{
			SRef<Image> view;
			if (camera->getNextImage(view) != SolAR::FrameworkReturnCode::_SUCCESS)
			{
				stop = true;
				return;
			}
			m_dropBufferCamImageCapture.push(view);			
		};

		// Feature extraction task
		auto fnExtraction = [&]()
		{
			SRef<Image> image;
			if (!m_dropBufferCamImageCapture.tryPop(image)) {
				xpcf::DelegateTask::yield();
				return;
			}
			std::vector<Keypoint> keypoints, undistortedKeypoints;
			SRef<DescriptorBuffer> descriptors;
			if (descriptorExtractor->extract(image, keypoints, descriptors) == FrameworkReturnCode::_SUCCESS) {
				undistortKeypoints->undistort(keypoints, camParams, undistortedKeypoints);
				SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, image);
				frame->setCameraParameters(camParams);
				if (bootstrapOk)
					m_dropBufferFrameTracking.push(frame);
				else
					m_dropBufferFrameBootstrap.push(frame);
			}						
		};

		// Bootstrap task
		auto fnBootstrap = [&]()
		{
			SRef<Frame> frame;
			if (bootstrapOk || !m_dropBufferFrameBootstrap.tryPop(frame)) {
				xpcf::DelegateTask::yield();
				return;
			}
			SRef<Image> view;
			Transform3Df pose = Transform3Df::Identity();
			fiducialMarkerPoseEstimator->estimate(frame->getView(), frame->getCameraParameters(), pose);
			frame->setPose(pose);
			if (bootstrapper->process(frame, view) == FrameworkReturnCode::_SUCCESS) {
				double bundleReprojError = bundler->bundleAdjustment();
				keyframesManager->getKeyframe(1, keyframe2);
				tracking->setNewKeyframe(keyframe2);
				framePoses.push_back(keyframe2->getPose());
				LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
				LOG_INFO("Number of initial keyframes: {}", keyframesManager->getNbKeyframes());
				bootstrapOk = true;
			}
			if (!pose.isApprox(Transform3Df::Identity()))
				overlay3D->draw(pose, frame->getCameraParameters(), view);
			m_dropBufferDisplay.push(view);
		};

		// Tracking task		
		auto fnTracking = [&]()
		{
			SRef<Frame> frame;
			if (!bootstrapOk || !m_dropBufferFrameTracking.tryPop(frame)) {
				xpcf::DelegateTask::yield();
				return;
			}
			// tracking
			SRef<Image>	displayImage;
			if (tracking->process(frame, displayImage) == FrameworkReturnCode::_SUCCESS) {
				// used for display
				framePoses.push_back(frame->getPose());
				// draw cube
				overlay3D->draw(frame->getPose(), frame->getCameraParameters(), displayImage);
				// send frame to mapping task
				if (isMappingIdle && isLoopIdle && tracking->checkNeedNewKeyframe())
					m_dropBufferAddKeyframe.push(frame);
			}

			m_dropBufferDisplay.push(displayImage);
		};

		// check need new keyframe	
		auto fnMapping = [&]()
		{
			SRef<Frame> frame;
			if (!m_dropBufferAddKeyframe.tryPop(frame)) {
				xpcf::DelegateTask::yield();
				return;
			}
			isMappingIdle = false;
			SRef<Keyframe> keyframe;
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
				countNewKeyframes++;
				m_dropBufferNewKeyframeLoop.push(keyframe);
				// send new keyframe to tracking
				tracking->setNewKeyframe(keyframe);
			}
			isMappingIdle = true;
		};

		// loop closure task
		auto fnLoopClosure = [&]() {
			SRef<Keyframe> lastKeyframe;
			if ((countNewKeyframes < NB_NEWKEYFRAMES_LOOP) || (!m_dropBufferNewKeyframeLoop.tryPop(lastKeyframe))) {
				xpcf::DelegateTask::yield();
				return;
			}
			uint32_t lastKeyframeId = lastKeyframe->getId();
			SRef<Keyframe> detectedLoopKeyframe;
			Transform3Df sim3Transform;
			std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
			if (loopDetector->detect(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
				// stop mapping process
				isLoopIdle = false;
				// detected loop keyframe
				LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
				LOG_INFO("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
				LOG_INFO("sim3Transform: \n{}", sim3Transform.matrix());
				// performs loop correction 			
				loopCorrector->correct(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);
				countNewKeyframes = 0;
				// Loop optimisation
				globalBundler->bundleAdjustment();
				LOG_DEBUG("Global BA done");
				// map pruning
				mapManager->pointCloudPruning();
				mapManager->keyframePruning();
				// free mapping
				isLoopIdle = true;
			}
		};

		// instantiate and start tasks
		xpcf::DelegateTask taskCamImageCapture(fnCamImageCapture);
		xpcf::DelegateTask taskBootstrap(fnBootstrap);
		xpcf::DelegateTask taskExtraction(fnExtraction);
		xpcf::DelegateTask taskTracking(fnTracking);
		xpcf::DelegateTask taskMapping(fnMapping);
		xpcf::DelegateTask taskLoopClosure(fnLoopClosure);

		taskCamImageCapture.start();
		taskBootstrap.start();
		taskExtraction.start();
		taskTracking.start();
		taskMapping.start();
		taskLoopClosure.start();

		// Start tracking
		clock_t start, end;
		start = clock();
		int count(0);
		while (!stop)
		{
			SRef<Image> displayImage;
			if (!m_dropBufferDisplay.tryPop(displayImage)) {
				xpcf::DelegateTask::yield();
				continue;
			}
			if (imageViewer->display(displayImage) == SolAR::FrameworkReturnCode::_STOP)
				stop = true;
			if (bootstrapOk) {
				if (!fnDisplay(framePoses))
					stop = true;
			}
			++count;
		}

		// Stop tasks
		taskCamImageCapture.stop();
		taskBootstrap.stop();
		taskExtraction.stop();
		taskTracking.stop();
		taskMapping.stop();
		taskLoopClosure.stop();

		// display stats on frame rate
		end = clock();
		double duration = double(end - start) / CLOCKS_PER_SEC;
		printf("\n\nElasped time is %.2lf seconds.\n", duration);
		printf("Number of processed frame per second : %8.2f\n", count / duration);

		// run global BA before exit
		bundler->bundleAdjustment();
		// map pruning
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
