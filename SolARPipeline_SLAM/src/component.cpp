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
	declareInjectable<IMapManager>(m_mapManager);
	declareInjectable<solver::map::IBundler>(m_bundler);
	declareInjectable<solver::map::IBundler>(m_globalBundler, "GlobalBA");
	declareInjectable<features::IDescriptorsExtractorFromImage>(m_descriptorExtractor);
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
        // get camera parameters
        m_camParams = m_camera->getParameters();

        // add current camera parameters to the map manager
        SRef<CameraParameters> camParams = xpcf::utils::make_shared<CameraParameters>(m_camParams);
        m_mapManager->addCameraParameters(camParams);
        m_camParamsID = camParams->id;

		// get properties
		m_minWeightNeighbor = m_mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
		m_reprojErrorThreshold = m_mapManager->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();

		m_bootstrapOk = false;
		m_stopFlag = false;
		m_startedOK = false;
		m_isLoopIdle = true;
		m_isMappingIdle = true;        	
		m_haveToBeFlip = false;
		m_initOK = true;
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
		m_tracking->setNewKeyframe(m_keyframe2);
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
    auto getDescriptorsThread = [this](){getDescriptors();};
    auto getTrackingThread = [this](){tracking();};
    auto getMappingThread = [this](){mapping();};
    auto getLoopClosureThread = [this](){loopClosure();};

    m_taskGetCameraImages = new xpcf::DelegateTask(getCameraImagesThread);
    m_taskDoBootStrap = new xpcf::DelegateTask(doBootStrapThread);
    m_taskGetDescriptors = new xpcf::DelegateTask(getDescriptorsThread);
    m_taskTracking = new xpcf::DelegateTask(getTrackingThread);
    m_taskMapping = new xpcf::DelegateTask(getMappingThread);
	m_taskLoopClosure = new xpcf::DelegateTask(getLoopClosureThread);

    m_taskGetCameraImages->start();
    m_taskDoBootStrap ->start();
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
    m_globalBundler->bundleAdjustment();
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
	m_CameraImagesBuffer.push(view);	
#ifdef USE_IMAGES_SET
	std::this_thread::sleep_for(std::chrono::milliseconds(40));
#endif
};

void PipelineSlam::doBootStrap()
{
	if (m_stopFlag || !m_initOK || !m_startedOK || m_bootstrapOk || !m_frameBootstrapBuffer.tryPop(m_frame)) {
		xpcf::DelegateTask::yield();
		return;
	}
	SRef<Image> view;
	m_poseFrame = Transform3Df::Identity();
    m_fiducialMarkerPoseEstimator->estimate(m_frame->getView(), m_camParams, m_poseFrame);
	m_frame->setPose(m_poseFrame);
	if (m_bootstrapper->process(m_frame, view) == FrameworkReturnCode::_SUCCESS) {
        double bundleReprojError = m_bundler->bundleAdjustment();
		m_keyframesManager->getKeyframe(0, m_keyframe2);
		m_tracking->setNewKeyframe(m_keyframe2);
		m_bootstrapOk = true;
	}
	if (!m_poseFrame.isApprox(Transform3Df::Identity())){
		m_sink->set(m_poseFrame, view);
	}
	else
		m_sink->set(view);
}

void PipelineSlam::getDescriptors()
{
	SRef<Image> image;
	if (m_stopFlag || !m_initOK || !m_startedOK || !m_CameraImagesBuffer.tryPop(image)) {
		xpcf::DelegateTask::yield();
		return;
	}
	std::vector<Keypoint> keypoints, undistortedKeypoints;
	SRef<datastructure::DescriptorBuffer> descriptors;
	if (m_descriptorExtractor->extract(image, keypoints, descriptors) == FrameworkReturnCode::_SUCCESS) {
        m_undistortKeypoints->undistort(keypoints, m_camParams, undistortedKeypoints);
        SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, image, m_camParamsID);
		if (m_bootstrapOk)
			m_frameBuffer.push(frame);
		else
			m_frameBootstrapBuffer.push(frame);
	}
};

void PipelineSlam::tracking()
{
	SRef<Frame> newFrame;
	if (m_stopFlag || !m_initOK || !m_startedOK || !m_frameBuffer.tryPop(newFrame)) {
		xpcf::DelegateTask::yield();
		return;
	}

	// tracking
	SRef<Image>	displayImage;
	if (m_tracking->process(newFrame, displayImage) == FrameworkReturnCode::_SUCCESS) {
		m_sink->set(newFrame->getPose(), displayImage);
		// send frame to mapping task
		if (m_isMappingIdle && m_isLoopIdle && m_tracking->checkNeedNewKeyframe())
			m_addKeyframeBuffer.push(newFrame);
	}
	else
		m_sink->set(displayImage);
}

void PipelineSlam::mapping()
{
	SRef<Frame> newFrame;
	if (m_stopFlag || !m_initOK || !m_startedOK || !m_addKeyframeBuffer.tryPop(newFrame)) {
		xpcf::DelegateTask::yield();
		return;
	}
	m_isMappingIdle = false;
	SRef<Keyframe> keyframe;
	if (m_mapping->process(newFrame, keyframe) == FrameworkReturnCode::_SUCCESS) {
		LOG_DEBUG("New keyframe id: {}", keyframe->getId());
		// Local bundle adjustment
		std::vector<uint32_t> bestIdx;
		m_covisibilityGraphManager->getNeighbors(keyframe->getId(), m_minWeightNeighbor, bestIdx, NB_LOCALKEYFRAMES);
		bestIdx.push_back(keyframe->getId());
		LOG_DEBUG("Nb keyframe to local bundle: {}", bestIdx.size());
        double bundleReprojError = m_bundler->bundleAdjustment(bestIdx);
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
		// send new keyframe to tracking
		m_tracking->setNewKeyframe(keyframe);
	}
	m_isMappingIdle = true;
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
		// stop mapping process
		m_isLoopIdle = false;
		// detected loop keyframe
		LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
		LOG_INFO("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
		LOG_INFO("sim3Transform: \n{}", sim3Transform.matrix());
		// performs loop correction 			
		m_countNewKeyframes = 0;
		m_loopCorrector->correct(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);		
		// Loop optimisation
        m_globalBundler->bundleAdjustment();
		// map pruning
		m_mapManager->pointCloudPruning();
		m_mapManager->keyframePruning();
		// free mapping
		m_isLoopIdle = true;
	}
}

}//namespace PIPELINES
}//namespace SolAR
