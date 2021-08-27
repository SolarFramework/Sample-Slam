#include "xpcf/module/ModuleFactory.h"
#include "pipelineSlam.h"
#include "core/Log.h"
#include "boost/log/core/core.hpp"
#include <cmath>

#define NB_NEWKEYFRAMES_LOOP 10
#define NB_LOCALKEYFRAMES 10

// The pipeline component for the fiducial marker

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::PIPELINES::PipelineSlam)

namespace SolAR {
using namespace datastructure;
using namespace api;
using namespace api::pipeline;
using namespace api::source;
using namespace api::sink;
using namespace api::storage;
namespace PIPELINES {

PipelineSlam::PipelineSlam():ConfigurableBase(xpcf::toUUID<PipelineSlam>())
{
    declareInterface<api::pipeline::IPoseEstimationPipeline>(this);
    declareInjectable<input::devices::ICamera>(m_camera);
	declareInjectable<IPointCloudManager>(m_pointCloudManager);
	declareInjectable<IKeyframesManager>(m_keyframesManager);
	declareInjectable<ICovisibilityGraphManager>(m_covisibilityGraphManager);
	declareInjectable<reloc::IKeyframeRetriever>(m_kfRetriever);
	declareInjectable<IMapManager>(m_mapManager);
	declareInjectable<solver::map::IBundler>(m_bundler);
	declareInjectable<solver::map::IBundler>(m_globalBundler, "GlobalBA");
	declareInjectable<features::IKeypointDetector>(m_keypointsDetector);
	declareInjectable<features::IDescriptorsExtractor>(m_descriptorExtractor);
    declareInjectable<input::files::ITrackableLoader>(m_trackableLoader);
    declareInjectable<solver::pose::ITrackablePose>(m_fiducialMarkerPoseEstimator);
	declareInjectable<image::IImageConvertor>(m_imageConvertorUnity);
	declareInjectable<loop::ILoopClosureDetector>(m_loopDetector);
	declareInjectable<loop::ILoopCorrector>(m_loopCorrector);
	declareInjectable<geom::IUndistortPoints>(m_undistortKeypoints);
	declareInjectable<slam::IBootstrapper>(m_bootstrapper);
	declareInjectable<slam::ITracking>(m_tracking);
	declareInjectable<slam::IMapping>(m_mapping);
    declareInjectable<sink::ISinkPoseImage>(m_sink);
    declareInjectable<source::ISourceImage>(m_source);    

    m_bootstrapOk=false;
	m_stopFlag = false;
	m_startedOK = false;
	m_isStopMapping = false;

    LOG_DEBUG(" Pipeline constructor");	
}


PipelineSlam::~PipelineSlam()
{
     LOG_DEBUG(" Pipeline destructor")
}

FrameworkReturnCode PipelineSlam::init()
{    
    // component creation
    try {
        // initialize components requiring the camera intrinsic and distortion parameters
        m_calibration = m_camera->getIntrinsicsParameters();
        m_distortion = m_camera->getDistortionParameters();
		m_loopDetector->setCameraParameters(m_calibration, m_distortion);
		m_loopCorrector->setCameraParameters(m_calibration, m_distortion);
		m_fiducialMarkerPoseEstimator->setCameraParameters(m_calibration, m_distortion);
		m_undistortKeypoints->setCameraParameters(m_calibration, m_distortion);
		m_bootstrapper->setCameraParameters(m_calibration, m_distortion);
		m_tracking->setCameraParameters(m_calibration, m_distortion);
		m_mapping->setCameraParameters(m_calibration, m_distortion);

		// get properties
		m_minWeightNeighbor = m_mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
		m_reprojErrorThreshold = m_mapManager->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();

        m_initOK = true;	
		m_haveToBeFlip = false;
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR("Exception catched: {}", e.what());
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode PipelineSlam::start(void* imageDataBuffer)
{
    if (m_initOK==false)
    {
        LOG_WARNING("Try to start the Fiducial marker pipeline without initializing it");
        return FrameworkReturnCode::_ERROR_;
    }
    m_stopFlag=false;

    m_sink->setImageBuffer((unsigned char*)imageDataBuffer);

	if (!m_haveToBeFlip) {
		if (m_camera->start() != FrameworkReturnCode::_SUCCESS)
		{
			LOG_ERROR("Camera cannot start")
            return FrameworkReturnCode::_ERROR_;
		}
	}

	// Load map from file
	if (m_mapManager->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
		LOG_INFO("Load map done!");
	}
	else
	{
		LOG_WARNING("Failed to load map from file");
	}

	if (m_pointCloudManager->getNbPoints() > 0)
	{
		// Map loaded from file and not empty
		if (m_keyframesManager->getKeyframe(0, m_keyframe2) != FrameworkReturnCode::_SUCCESS)
		{
			return FrameworkReturnCode::_ERROR_;
		}
		m_bootstrapOk = true;
		m_tracking->updateReferenceKeyframe(m_keyframe2);
	}
	else
	{
		// Either no or empty map files
		LOG_INFO("Initialization from scratch");
        SRef<Trackable> trackable;
        if (m_trackableLoader->loadTrackable(trackable) != FrameworkReturnCode::_SUCCESS)
        {
            LOG_ERROR("cannot load fiducial marker");
            return FrameworkReturnCode::_ERROR_;
        }
        else
        {
            if (m_fiducialMarkerPoseEstimator->setTrackable(trackable)!= FrameworkReturnCode::_SUCCESS)
            {
                LOG_ERROR("cannot set fiducial marker as a trackable ofr fiducial marker pose estimator");
                return FrameworkReturnCode::_ERROR_;
            }
        }
	}

    // create and start threads
    auto getCameraImagesThread = [this](){getCameraImages();};
    auto doBootStrapThread = [this](){doBootStrap();};
    auto getKeyPointsThread = [this](){getKeyPoints();};
    auto getDescriptorsThread = [this](){getDescriptors();};
    auto getTrackingThread = [this](){tracking();};
    auto getMappingThread = [this](){mapping();};
    auto getLoopClosureThread = [this](){loopClosure();};

    m_taskGetCameraImages = new xpcf::DelegateTask(getCameraImagesThread);
    m_taskDoBootStrap = new xpcf::DelegateTask(doBootStrapThread);
    m_taskGetKeyPoints = new xpcf::DelegateTask(getKeyPointsThread);
    m_taskGetDescriptors = new xpcf::DelegateTask(getDescriptorsThread);
    m_taskTracking = new xpcf::DelegateTask(getTrackingThread);
    m_taskMapping = new xpcf::DelegateTask(getMappingThread);
	m_taskLoopClosure = new xpcf::DelegateTask(getLoopClosureThread);

    m_taskGetCameraImages->start();
    m_taskDoBootStrap ->start();
    m_taskGetKeyPoints->start();
    m_taskGetDescriptors->start();
	m_taskTracking->start();
	m_taskMapping->start();
	m_taskLoopClosure->start();

    LOG_INFO("Threads have started");
    m_startedOK = true;	

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode PipelineSlam::stop()
{
    m_stopFlag=true;
    m_camera->stop();

    if (m_taskGetCameraImages != nullptr)
        m_taskGetCameraImages->stop();
    if (m_taskDoBootStrap != nullptr)
        m_taskDoBootStrap->stop();
    if (m_taskGetKeyPoints != nullptr)
        m_taskGetKeyPoints->stop();
    if (m_taskGetDescriptors != nullptr)
        m_taskGetDescriptors->stop();
    if (m_taskTracking != nullptr)
		m_taskTracking->stop();
    if (m_taskMapping != nullptr)
		m_taskMapping->stop();
	if (m_taskLoopClosure != nullptr)
		m_taskLoopClosure->stop();

    if(!m_initOK)
    {
        LOG_WARNING("Try to stop a pipeline that has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }
    if (!m_startedOK)
    {
        LOG_WARNING("Try to stop a pipeline that has not been started");
        return FrameworkReturnCode::_ERROR_;
    }
    LOG_INFO("Pipeline has stopped: \n");
	// run global BA before exit
	m_globalBundler->bundleAdjustment(m_calibration, m_distortion);
	// map pruning
	m_mapManager->pointCloudPruning();
	m_mapManager->keyframePruning();
	// Save map
	m_mapManager->saveToFile();

    return FrameworkReturnCode::_SUCCESS;
}

SourceReturnCode PipelineSlam::loadSourceImage(void* sourceTextureHandle, int width, int height)
{
	m_haveToBeFlip = true;
	return m_source->setInputTexture((unsigned char *)sourceTextureHandle, width, height);
}

SinkReturnCode PipelineSlam::update(Transform3Df& pose)
{
    if(m_stopFlag)
        return SinkReturnCode::_ERROR;
    else
        return m_sink->tryGet(pose);
}

CameraParameters PipelineSlam::getCameraParameters() const
{
    CameraParameters camParam;
    if (m_camera)
    {
        camParam = m_camera->getParameters();
    }
    return camParam;
}

void PipelineSlam::getCameraImages() {

	SRef<Image> view;
	if (m_stopFlag || !m_initOK || !m_startedOK)
		return;
	if (m_haveToBeFlip){
		m_source->getNextImage(view);
		m_imageConvertorUnity->convert(view, view, Image::ImageLayout::LAYOUT_RGB);
	}
	else if (m_camera->getNextImage(view) != FrameworkReturnCode::_SUCCESS) {
		m_stopFlag = true;
		return;
	}
	if (m_bootstrapOk)
		m_CameraImagesBuffer.push(view);
	else
		m_CameraImagesBootstrapBuffer.push(view);

#ifdef USE_IMAGES_SET
	std::this_thread::sleep_for(std::chrono::milliseconds(40));
#endif
};

void PipelineSlam::doBootStrap()
{
	if (m_stopFlag || !m_initOK || !m_startedOK || m_bootstrapOk || !m_CameraImagesBootstrapBuffer.tryPop(m_camImage)) {
		xpcf::DelegateTask::yield();
		return;
	}

	SRef<Image> view;
	m_poseFrame = Transform3Df::Identity();
	m_fiducialMarkerPoseEstimator->estimate(m_camImage, m_poseFrame);
	if (m_bootstrapper->process(m_camImage, view, m_poseFrame) == FrameworkReturnCode::_SUCCESS) {
		double bundleReprojError = m_bundler->bundleAdjustment(m_calibration, m_distortion);
		m_keyframesManager->getKeyframe(0, m_keyframe2);
		m_tracking->updateReferenceKeyframe(m_keyframe2);
		m_bootstrapOk = true;
	}
	if (!m_poseFrame.isApprox(Transform3Df::Identity())){
		m_sink->set(m_poseFrame, view);
	}
	else
		m_sink->set(view);
}

void PipelineSlam::getKeyPoints() {
	SRef<Image>  camImage;

	if (m_stopFlag || !m_initOK || !m_startedOK || !m_bootstrapOk || !m_CameraImagesBuffer.tryPop(camImage)) {
		xpcf::DelegateTask::yield();
		return;
	}
	m_keypointsDetector->detect(camImage, m_keypoints);
	m_keypointsBuffer.push(std::make_pair(camImage, m_keypoints));
};

void PipelineSlam::getDescriptors()
{
	std::pair<SRef<Image>, std::vector<Keypoint>> frameKeypoints;
	if (m_stopFlag || !m_initOK || !m_startedOK || !m_keypointsBuffer.tryPop(frameKeypoints)) {
		xpcf::DelegateTask::yield();
		return;
	}
	
	std::vector<Keypoint> undistortedKeypoints;
	SRef<datastructure::DescriptorBuffer> descriptors;
	if (frameKeypoints.second.size() > 0) {		
		m_undistortKeypoints->undistort(frameKeypoints.second, undistortedKeypoints);
		m_descriptorExtractor->extract(frameKeypoints.first, frameKeypoints.second, descriptors);
	}
	SRef<Frame> frame = xpcf::utils::make_shared<Frame>(frameKeypoints.second, undistortedKeypoints, descriptors, frameKeypoints.first);
	m_descriptorsBuffer.push(frame);
};

void PipelineSlam::tracking()
{
	SRef<Frame> newFrame;
	if (m_stopFlag || !m_initOK || !m_startedOK || !m_descriptorsBuffer.tryPop(newFrame)) {
		xpcf::DelegateTask::yield();
		return;
	}

	LOG_DEBUG("Number of keypoints: {}", newFrame->getKeypoints().size());
	SRef<Keyframe> newKeyframe;
	if (m_newKeyframeBuffer.tryPop(newKeyframe))
	{
		m_tracking->updateReferenceKeyframe(newKeyframe);
		SRef<Frame> tmpFrame;
		m_addKeyframeBuffer.tryPop(tmpFrame);
		m_isStopMapping = false;
	}
	// tracking
	SRef<Image>	displayImage;
	if (m_tracking->process(newFrame, displayImage) == FrameworkReturnCode::_SUCCESS) {
		m_sink->set(newFrame->getPose(), displayImage);
		// send frame to mapping task
		m_addKeyframeBuffer.push(newFrame);
	}
	else
		m_sink->set(displayImage);
}

void PipelineSlam::mapping()
{
	std::unique_lock<std::mutex> lock(m_mutexMapping);
	SRef<Frame> newFrame;
	if (m_stopFlag || !m_initOK || !m_startedOK || m_isStopMapping || !m_addKeyframeBuffer.tryPop(newFrame)) {
		xpcf::DelegateTask::yield();
		return;
	}

	SRef<Keyframe> keyframe;
	if (m_mapping->process(newFrame, keyframe) == FrameworkReturnCode::_SUCCESS) {
		LOG_DEBUG("New keyframe id: {}", keyframe->getId());
		// Local bundle adjustment
		std::vector<uint32_t> bestIdx, bestIdxToOptimize;
		m_covisibilityGraphManager->getNeighbors(keyframe->getId(), m_minWeightNeighbor, bestIdx);
		if (bestIdx.size() < NB_LOCALKEYFRAMES)
			bestIdxToOptimize = bestIdx;
		else
			bestIdxToOptimize.insert(bestIdxToOptimize.begin(), bestIdx.begin(), bestIdx.begin() + NB_LOCALKEYFRAMES);
		bestIdxToOptimize.push_back(keyframe->getId());
		LOG_DEBUG("Nb keyframe to local bundle: {}", bestIdx.size());
		double bundleReprojError = m_bundler->bundleAdjustment(m_calibration, m_distortion, bestIdxToOptimize);
		// local map pruning
		std::vector<SRef<CloudPoint>> localPointCloud;
		m_mapManager->getLocalPointCloud(keyframe, 1.0, localPointCloud);
		int nbRemovedCP = m_mapManager->pointCloudPruning(localPointCloud);
		std::vector<SRef<Keyframe>> localKeyframes;
		m_keyframesManager->getKeyframes(bestIdx, localKeyframes);
		int nbRemovedKf = m_mapManager->keyframePruning(localKeyframes);
		LOG_DEBUG("Nb of pruning cloud points / keyframes: {} / {}", nbRemovedCP, nbRemovedKf);
		m_countNewKeyframes++;
		m_newKeyframeLoopBuffer.push(keyframe);
	}

	if (keyframe) {
		m_isStopMapping = true;
		m_newKeyframeBuffer.push(keyframe);
	}
}

void PipelineSlam::loopClosure()
{		
	SRef<Keyframe> lastKeyframe;
	if (m_stopFlag || !m_initOK || !m_startedOK || (m_countNewKeyframes < NB_NEWKEYFRAMES_LOOP) || (!m_newKeyframeLoopBuffer.tryPop(lastKeyframe))) {
		xpcf::DelegateTask::yield();
		return;
	}
	uint32_t lastKeyframeId = lastKeyframe->getId();
	SRef<Keyframe> detectedLoopKeyframe;
	Transform3Df sim3Transform;
	std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
	if (m_loopDetector->detect(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
		// detected loop keyframe
		LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
		LOG_INFO("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
		LOG_INFO("sim3Transform: \n{}", sim3Transform.matrix());
		// performs loop correction 			
		std::unique_lock<std::mutex> lock(m_mutexMapping);
		m_countNewKeyframes = 0;
		m_loopCorrector->correct(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);		
		// Loop optimisation
		m_globalBundler->bundleAdjustment(m_calibration, m_distortion);
		// map pruning
		m_mapManager->pointCloudPruning();
		m_mapManager->keyframePruning();
	}
}

}//namespace PIPELINES
}//namespace SolAR
