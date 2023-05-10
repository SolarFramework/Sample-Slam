#include "xpcf/module/ModuleFactory.h"
#include "pipelineSlam.h"
#include "core/Log.h"

// Declaration of the module embedding the Slam pipeline
XPCF_DECLARE_MODULE("da89a6eb-3233-4dea-afdc-9d918be0bd74", "SolARPipelineSLAM", "The module embedding a pipeline to estimate the pose based on a multithreaded Slam")

extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID,SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
    xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
    errCode = xpcf::tryCreateComponent<SolAR::PIPELINES::PipelineSlam>(componentUUID,interfaceRef);

    return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::PIPELINES::PipelineSlam)
XPCF_END_COMPONENTS_DECLARATION
