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
#include "core/Log.h"
#include "xpcf/xpcf.h"


// ADD COMPONENTS HEADERS HERE, e.g #include "SolarComponent.h"
#include "api/pipeline/IPipeline.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DOverlay.h"


#define MARKER_CONFIGFILE "FiducialMarker.yml"

namespace xpcf  = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::api;

int main(){
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();
    try{
        SRef<xpcf::IComponentManager> componentMgr = xpcf::getComponentManagerInstance();
        xpcf::XPCFErrorCode errorLoad = componentMgr->load("PipelineSlam.xml");
        if (errorLoad != xpcf::_SUCCESS)
        {
            LOG_ERROR("The file PipelineSlam.xml has an error");
        }

        auto pipeline = componentMgr->resolve<pipeline::IPipeline>();

        if (pipeline->init(componentMgr) == FrameworkReturnCode::_SUCCESS )
        {
            auto imageViewerResult = componentMgr->resolve<display::IImageViewer>();
            auto overlay3DComponent = componentMgr->resolve<display::I3DOverlay>();

            // Set camera parameters
            CameraParameters camParam = pipeline->getCameraParameters();
            overlay3DComponent->setCameraParameters(camParam.intrinsic, camParam.distorsion);

            unsigned char* r_imageData=new unsigned char[camParam.resolution.width * camParam.resolution.height * 3];
            SRef<Image> camImage=xpcf::utils::make_shared<Image>(r_imageData,camParam.resolution.width,camParam.resolution.height,SolAR::Image::LAYOUT_BGR,SolAR::Image::INTERLEAVED,SolAR::Image::TYPE_8U);

            Transform3Df s_pose;

            clock_t start, end;
            start = clock();
            int count = 0;
            if (pipeline->start(camImage->data()) == FrameworkReturnCode::_SUCCESS)
            {
                while (true)
                {
                    Transform3Df pose;

                    sink::SinkReturnCode returnCode = pipeline->update(pose);

                    if (returnCode == SinkReturnCode::_ERROR)
                        break;

                    if (returnCode == sink::SinkReturnCode::_NOTHING)
                        continue;

                    if ((returnCode == sink::SinkReturnCode::_NEW_POSE) || (returnCode == sink::SinkReturnCode::_NEW_POSE_AND_IMAGE))
                    {
                        for(int i=0;i<3;i++)
                             for(int j=0;j<3;j++)
                                 s_pose(i,j)=pose(i,j);
                        for(int i=0;i<3;i++)
                                 s_pose(i,3)=pose(i,3);
                        for(int j=0;j<3;j++)
                            s_pose(3,j)=0;
                        s_pose(3,3)=1;
                        //LOG_INFO("pose.matrix():\n {} \n",s_pose.matrix())
                        overlay3DComponent->draw(s_pose, camImage);
                    }

                    if (imageViewerResult->display(camImage) == SolAR::FrameworkReturnCode::_STOP){
                        pipeline->stop();
                        break;
                    }

                    count++;
                 }
            }
            end = clock();
            double duration = double(end - start) / CLOCKS_PER_SEC;
            printf("\n\nElasped time is %.2lf seconds.\n", duration);
            printf("Number of processed frame per second : %8.2f\n", count / duration);
            delete[] r_imageData;
        }
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catched: {}", e.what());
        return -1;
    }
}





