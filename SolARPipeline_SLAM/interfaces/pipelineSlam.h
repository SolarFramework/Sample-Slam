#ifndef PIPELINESLAM_H
#define PIPELINESLAM_H

#if _WIN32
#ifdef SolARPipelineSLAM_API_DLLEXPORT
#define SOLARPIPELINESLAM_EXPORT_API __declspec(dllexport)
#else //SolARPipelineSLAM_API_DLLEXPORT
#define SOLARPIPELINESLAM_EXPORT_API __declspec(dllimport)
#endif //SolARPipelineSLAM_API_DLLEXPORT
#else //_WIN32
#define SOLARPIPELINESLAM_EXPORT_API
#endif //_WIN32

#include "xpcf/core/traits.h"
#include "xpcf/component/ConfigurableBase.h"
#include "api/pipeline/IPoseEstimationPipeline.h"

// Add the headers to datastructures and component interfaces used by the pipeline

#include "api/input/devices/ICamera.h"
#include "api/features/IDescriptorsExtractorFromImage.h"
#include "api/storage/IMapManager.h"
#include "api/display/I2DOverlay.h"
#include "api/display/IMatchesOverlay.h"
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
#include "api/image/IImageConvertor.h"
#include "core/Log.h"

#ifdef USE_OPENGL
    #include "api/sink/ISinkPoseTextureBuffer.h"
#else
    #include "api/sink/ISinkPoseImage.h"
#endif
#include "api/source/ISourceImage.h"

#include "xpcf/threading/SharedBuffer.h"
#include "xpcf/threading/DropBuffer.h"
#include "xpcf/threading/BaseTask.h"

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
 * @SolARComponentInjectable{SolAR::api::storage::ICovisibilityGraphManager}
 * @SolARComponentInjectable{SolAR::api::reloc::IKeyframeRetriever}
 * @SolARComponentInjectable{SolAR::api::storage::IMapManager}
 * @SolARComponentInjectable{SolAR::api::solver::map::IBundler}
 * @SolARComponentInjectable{SolAR::api::solver::map::IBundler}
 * @SolARComponentInjectable{SolAR::api::features::IDescriptorsExtractorFromImage}
 * @SolARComponentInjectable{SolAR::input::files::ITrackableLoader}
 * @SolARComponentInjectable{SolAR::api::solver::pose::ITrackablePose}
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
    /// @return FrameworkReturnCode::_SUCCESS if the init succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode init() override;

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
	SRef<api::storage::ICovisibilityGraphManager>		m_covisibilityGraphManager;
	SRef<api::reloc::IKeyframeRetriever>				m_kfRetriever;
	SRef<api::storage::IMapManager>						m_mapManager;

	// components
    SRef<api::input::devices::ICamera>					m_camera;
	SRef<api::image::IImageConvertor>					m_imageConvertorUnity;
    SRef<api::features::IDescriptorsExtractorFromImage>	m_descriptorExtractor;
	SRef<api::solver::map::IBundler>					m_bundler;
	SRef<api::solver::map::IBundler>					m_globalBundler;
    SRef<api::input::files::ITrackableLoader>           m_trackableLoader;
    SRef<api::solver::pose::ITrackablePose>             m_fiducialMarkerPoseEstimator;
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
	SRef<datastructure::Frame>							m_frame;
	datastructure::Transform3Df                         m_poseFrame;
    SRef<datastructure::Keyframe>                       m_keyframe1, m_keyframe2;
	bool												m_bootstrapOk = false;
	bool												m_haveToBeFlip;
	int													m_countNewKeyframes = 0;
	float												m_minWeightNeighbor;
	float												m_reprojErrorThreshold;
	datastructure::CamCalibration                       m_calibration;
	datastructure::CamDistortion                        m_distortion;
	double												m_bundleReprojError;
	std::mutex											m_mutexMapping;

	xpcf::DropBuffer< SRef<datastructure::Image>>		m_CameraImagesBuffer;	
	xpcf::DropBuffer< SRef<datastructure::Frame >>		m_frameBuffer;
	xpcf::DropBuffer< SRef<datastructure::Frame>>		m_frameBootstrapBuffer;
	xpcf::DropBuffer<SRef<datastructure::Frame>>		m_addKeyframeBuffer;
	xpcf::DropBuffer<SRef<datastructure::Keyframe>>		m_newKeyframeBuffer;
	xpcf::DropBuffer<SRef<datastructure::Keyframe>>		m_newKeyframeLoopBuffer;

	// tasks
    xpcf::DelegateTask*									m_taskGetCameraImages;
    xpcf::DelegateTask*									m_taskDoBootStrap;
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
