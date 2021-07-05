#ifndef PIPELINESLAM_H
#define PIPELINESLAM_H

#if _WIN32
#ifdef SolARPipeline_SLAM_API_DLLEXPORT
#define SOLARPIPELINESLAM_EXPORT_API __declspec(dllexport)
#else //SOLARPIPELINEFIDUCIALMARKER_API_DLLEXPORT
#define SOLARPIPELINESLAM_EXPORT_API __declspec(dllimport)
#endif //SOLARPIPELINEFIDUCIALMARKER_API_DLLEXPORT
#else //_WIN32
#define SOLARPIPELINESLAM_EXPORT_API
#endif //_WIN32

#include "xpcf/core/traits.h"
#include "xpcf/component/ConfigurableBase.h"
#include "api/pipeline/IPoseEstimationPipeline.h"

// Add the headers to datastructures and component interfaces used by the pipeline

#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/solver/map/IMapper.h"
#include "api/display/I2DOverlay.h"
#include "api/display/IMatchesOverlay.h"
#include "api/display/I3DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/solver/pose/IFiducialMarkerPose.h"
#include "api/solver/map/IBundler.h"
#include "api/geom/IUndistortPoints.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
#include "api/slam/IBootstrapper.h"
#include "api/slam/ITracking.h"
#include "api/slam/IMapping.h"
#include "core/Log.h"
#include "api/image/IImageConvertor.h"

#ifdef USE_OPENGL
    #include "api/sink/ISinkPoseTextureBuffer.h"
#else
    #include "api/sink/ISinkPoseImage.h"
#endif
#include "api/source/ISourceImage.h"

#include "xpcf/threading/SharedBuffer.h"
#include "xpcf/threading/DropBuffer.h"
#include "xpcf/threading/BaseTask.h"

//#define USE_IMAGES_SET
//#define VIDEO_INPUT

namespace xpcf = org::bcom::xpcf;

namespace SolAR {
namespace PIPELINES {

/**
 * @class PipelineSlam
 * @brief A pipeline for SLAM.
 *
 * @SolARComponentInjectablesBegin
 * @SolARComponentInjectable{SolAR::api::input::devices::ICamera}
 * @SolARComponentInjectable{SolAR::api::storage::IPointCloudManager}
 * @SolARComponentInjectable{SolAR::api::storage::IKeyframesManager}
 * @SolARComponentInjectable{SolAR::api::storage::ICovisibilityGraph}
 * @SolARComponentInjectable{SolAR::api::reloc::IKeyframeRetriever}
 * @SolARComponentInjectable{SolAR::api::solver::map::IMapper}
 * @SolARComponentInjectable{SolAR::api::solver::map::IBundler}
 * @SolARComponentInjectable{SolAR::api::solver::map::IBundler}
 * @SolARComponentInjectable{SolAR::api::features::IKeypointDetector}
 * @SolARComponentInjectable{SolAR::api::features::IDescriptorsExtractor}
 * @SolARComponentInjectable{SolAR::api::solver::pose::IFiducialMarkerPose}
 * @SolARComponentInjectable{SolAR::api::image::IImageConvertor}
 * @SolARComponentInjectable{SolAR::api::loop::ILoopClosureDetector}
 * @SolARComponentInjectable{SolAR::api::loop::ILoopCorrector}
 * @SolARComponentInjectable{SolAR::api::slam::IBootstrapper}
 * @SolARComponentInjectable{SolAR::api::slam::ITracking}
 * @SolARComponentInjectable{SolAR::api::slam::IMapping}
 * @SolARComponentInjectable{SolAR::api::sink::ISinkPoseImage}
 * @SolARComponentInjectable{SolAR::api::source::ISourceImage}
 * @SolARComponentInjectablesEnd
 *
 */

class SOLARPIPELINESLAM_EXPORT_API PipelineSlam : public org::bcom::xpcf::ConfigurableBase,
    public api::pipeline::IPoseEstimationPipeline
{
public:
    PipelineSlam();
    ~PipelineSlam();

    //// @brief Initialization of the pipeline
    /// Initialize the pipeline by providing a reference to the component manager loaded by the PipelineManager.
    /// @param[in] componentManager a shared reference to the component manager which has loaded the components and configuration in the pipleine manager
    FrameworkReturnCode init(SRef<xpcf::IComponentManager> xpcfComponentManager) override;

    /// @brief Provide the camera parameters
    /// @return the camera parameters (its resolution and its focal)
    datastructure::CameraParameters getCameraParameters() const override;

    /// @brief Starts the pipeline and provides a texture buffer which will be updated when required.
    /// @param[in] textureHandle a pointer to the texture buffer which will be updated at each call of the update method.

    /// @brief Start the pipeline
    /// @return FrameworkReturnCode::_ERROR_ by default as the pipeline needs to be construct with an imageDataBuffer as parameter
    FrameworkReturnCode start() override { return FrameworkReturnCode::_ERROR_; }

#ifdef USE_OPENGL
    FrameworkReturnCode start(void* textureHandle) override;
#else
    FrameworkReturnCode start(void* imageDataBuffer) override;
#endif

    /// @brief Stop the pipeline.
    FrameworkReturnCode stop() override;

    /// @brief update the pipeline
    /// Get the new pose and update the texture buffer with the image that has to be displayed
    api::sink::SinkReturnCode update(datastructure::Transform3Df& pose) override;

    api::source::SourceReturnCode loadSourceImage(void* sourceTextureHandle, int width, int height) override;

    void unloadComponent () override final;

private:	
	// Image capture task
	void getCameraImages();

	// Bootstrap task
	void doBootStrap();

	// Keypoint detection task
	void getKeyPoints();

	// Feature extraction task
	void getDescriptors();

	// tracking  task
	void tracking();

	// mapping task
	void mapping();	

	// global bundle adjustment task
	void loopClosure();
private:

	// State flag of the pipeline
	bool m_stopFlag, m_initOK, m_startedOK, m_isStopMapping;

	// mutex
	std::mutex globalVarsMutex;

	// storage components
	SRef<api::storage::IPointCloudManager>				m_pointCloudManager;
	SRef<api::storage::IKeyframesManager>				m_keyframesManager;
	SRef<api::storage::ICovisibilityGraph>				m_covisibilityGraph;
	SRef<api::reloc::IKeyframeRetriever>				m_kfRetriever;
	SRef<api::solver::map::IMapper>						m_mapper;

	// components
    SRef<api::input::devices::ICamera>					m_camera;
	SRef<api::image::IImageConvertor>					m_imageConvertorUnity;
    SRef<api::features::IKeypointDetector>				m_keypointsDetector;
    SRef<api::features::IDescriptorsExtractor>			m_descriptorExtractor;
	SRef<api::solver::map::IBundler>					m_bundler;
	SRef<api::solver::map::IBundler>					m_globalBundler;
	SRef<api::solver::pose::IFiducialMarkerPose>		m_fiducialMarkerPoseEstimator;
	SRef<api::loop::ILoopClosureDetector>				m_loopDetector;
	SRef<api::loop::ILoopCorrector>						m_loopCorrector;
	SRef<api::geom::IUndistortPoints>					m_undistortKeypoints;
	SRef<api::slam::IBootstrapper>						m_bootstrapper;
	SRef<api::slam::ITracking>							m_tracking;
	SRef<api::slam::IMapping>							m_mapping;

    // display stuff
    SRef<api::display::I2DOverlay>						m_i2DOverlay;


#ifdef USE_OPENGL
    SRef<sink::ISinkPoseTextureBuffer>					m_sink;
#else
    SRef<api::sink::ISinkPoseImage>						m_sink;
#endif   
	SRef<api::source::ISourceImage>						m_source;

	// SLAM variables
	datastructure::Transform3Df							m_pose;
	SRef<datastructure::Image>							m_camImage;
	datastructure::Transform3Df                         m_poseFrame;
    SRef<datastructure::Keyframe>                       m_keyframe1, m_keyframe2;
	bool												m_bootstrapOk = false;
	bool												m_haveToBeFlip;
	int													m_countNewKeyframes = 0;
	float												m_minWeightNeighbor;
	float												m_reprojErrorThreshold;
	datastructure::CamCalibration                       m_calibration;
	datastructure::CamDistortion                        m_distortion;
	std::vector<datastructure::Keypoint>				m_keypoints;
	SRef<datastructure::DescriptorBuffer>				m_descriptors;
	double												m_bundleReprojError;
	std::mutex											m_mutexMapping;

	xpcf::DropBuffer< SRef<datastructure::Image>>		m_CameraImagesBuffer;
	xpcf::DropBuffer< std::pair< SRef<datastructure::Image>, std::vector<datastructure::Keypoint> >> m_keypointsBuffer;
	xpcf::DropBuffer< SRef<datastructure::Frame >>		m_descriptorsBuffer;
	xpcf::DropBuffer<SRef<datastructure::Frame>>		m_addKeyframeBuffer;
	xpcf::DropBuffer<SRef<datastructure::Keyframe>>		m_newKeyframeBuffer;
	xpcf::DropBuffer<SRef<datastructure::Keyframe>>		m_newKeyframeLoopBuffer;

	// tasks
    xpcf::DelegateTask*									m_taskGetCameraImages;
    xpcf::DelegateTask*									m_taskDoBootStrap;
    xpcf::DelegateTask*									m_taskGetKeyPoints;
    xpcf::DelegateTask*									m_taskGetDescriptors;
    xpcf::DelegateTask*									m_taskTracking;
    xpcf::DelegateTask*									m_taskMapping;        
    xpcf::DelegateTask*									m_taskLoopClosure;

};

}//namespace PIPELINES
}//namespace SolAR

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::PIPELINES::PipelineSlam,
                             "577ccd2c-de1b-402a-8829-496747598588",
                             "PipelineSlam",
                             "A pipeline to estimate the pose based on a Slam");
#endif // PIPELINESLAM_H
