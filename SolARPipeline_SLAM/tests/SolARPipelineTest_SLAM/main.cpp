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
#include "api/pipeline/IPoseEstimationPipeline.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DOverlay.h"

namespace xpcf  = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::api::sink;
using namespace SolAR::datastructure;

int main(int argc, char **argv){
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();
    try{
		std::string configxml = std::string("SolARPipelineTest_SLAM_conf.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);
        SRef<xpcf::IComponentManager> componentMgr = xpcf::getComponentManagerInstance();
        xpcf::XPCFErrorCode errorLoad = componentMgr->load(configxml.c_str());
        if (errorLoad != xpcf::_SUCCESS)
        {
            LOG_ERROR("The file PipelineSlam.xml has an error");
        }

        auto pipeline = componentMgr->resolve<pipeline::IPoseEstimationPipeline>();

        if (pipeline->init() == FrameworkReturnCode::_SUCCESS )
        {
            auto imageViewerResult = componentMgr->resolve<display::IImageViewer>();
            auto overlay3DComponent = componentMgr->resolve<display::I3DOverlay>();

            // Set camera parameters
            CameraParameters camParam = pipeline->getCameraParameters();
            overlay3DComponent->setCameraParameters(camParam.intrinsic, camParam.distortion);

            unsigned char* r_imageData=new unsigned char[camParam.resolution.width * camParam.resolution.height * 3];
            SRef<Image> camImage=xpcf::utils::make_shared<Image>(r_imageData,camParam.resolution.width,camParam.resolution.height, Image::LAYOUT_BGR, Image::INTERLEAVED, Image::TYPE_8U);

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

                    if (returnCode == SinkReturnCode::_ERROR) {
						pipeline->stop();
						break;
					}

                    if (returnCode == sink::SinkReturnCode::_NOTHING)
                        continue;

                    if ((returnCode == sink::SinkReturnCode::_NEW_POSE) || (returnCode == sink::SinkReturnCode::_NEW_POSE_AND_IMAGE))
                    {
						s_pose = pose;
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
    catch (xpcf::InjectableNotFoundException e)
    {
        LOG_ERROR ("The following exception in relation to a unfound injectable has been catched: {}", e.what());
        return -1;
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catched: {}", e.what());
        return -1;
    }

}





