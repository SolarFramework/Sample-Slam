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

#include "opencv2/core.hpp" // TO REMOVE
#include "opencv2/calib3d.hpp"

#include <boost/log/core.hpp>

// ADD COMPONENTS HEADERS HERE

#include "SolARModuleOpencv_traits.h"
#include "SolARModuleOpengl_traits.h"

#include "xpcf/xpcf.h"

#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapper.h"
#include "api/solver/map/IMapFilter.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/solver/pose/I3DTransformFinderfrom2D3D.h"
#include "api/features/IMatchesFilter.h"
#include "api/display/ISideBySideOverlay.h"
#include "api/display/I2DOverlay.h"
#include "api/display/I3DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::OPENGL;

namespace xpcf = org::bcom::xpcf;

int main(int argc, char **argv){

//#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
//#endif

    LOG_ADD_LOG_TO_CONSOLE();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("conf_SLAM.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_SLAM.xml")
        return -1;
    }

    // declare and create components
    LOG_INFO("Start creating components");

 // component creation

    auto camera =xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    auto keypointsDetector =xpcfComponentManager->create<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
    auto descriptorExtractorAKAZE2 =xpcfComponentManager->create<SolARDescriptorsExtractorAKAZE2Opencv>()->bindTo<features::IDescriptorsExtractor>();
    auto descriptorExtractorORB =xpcfComponentManager->create<SolARDescriptorsExtractorORBOpencv>()->bindTo<features::IDescriptorsExtractor>();
    auto matcherKNN =xpcfComponentManager->create<SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto matcherBF =xpcfComponentManager->create<SolARDescriptorMatcherHammingBruteForceOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto poseFinderFrom2D2D =xpcfComponentManager->create<SolARPoseFinderFrom2D2DOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D2D>();
    auto mapper =xpcfComponentManager->create<SolARSVDTriangulationOpencv>()->bindTo<solver::map::ITriangulator>();
    auto mapFilter =xpcfComponentManager->create<SolARMapFilterOpencv>()->bindTo<solver::map::IMapFilter>();
    auto poseGraph =xpcfComponentManager->create<SolARMapperOpencv>()->bindTo<solver::map::IMapper>();
    auto matchesFilter =xpcfComponentManager->create<SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
    auto PnP =xpcfComponentManager->create<SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
    auto corr2D3DFinder =xpcfComponentManager->create<SolAR2D3DCorrespondencesFinderOpencv>()->bindTo<solver::pose::I2D3DCorrespondencesFinder>();

    auto overlaySBS =xpcfComponentManager->create<SolARSideBySideOverlayOpencv>()->bindTo<display::ISideBySideOverlay>();
    auto imageViewerFrame1 =xpcfComponentManager->create<SolARImageViewerOpencv>("frame1")->bindTo<display::IImageViewer>();
    auto imageViewerFrame2 =xpcfComponentManager->create<SolARImageViewerOpencv>("frame2")->bindTo<display::IImageViewer>();
    auto imageViewerMatches =xpcfComponentManager->create<SolARImageViewerOpencv>("matches")->bindTo<display::IImageViewer>();
    auto viewer3DPoints =xpcfComponentManager->create<SolAR3DPointsViewerOpengl>()->bindTo<display::I3DPointsViewer>();

    /* in dynamic mode, we need to check that components are well created*/
    /* this is needed in dynamic mode */
    if ( !camera || !keypointsDetector || !descriptorExtractorORB || !descriptorExtractorAKAZE2 || !matcherKNN || !matcherBF ||
         !poseFinderFrom2D2D || !mapper || !mapFilter || !poseGraph || !PnP || !corr2D3DFinder || !matchesFilter ||
         !overlaySBS || !imageViewerFrame1 || !imageViewerFrame2 || !imageViewerMatches || !viewer3DPoints)
    {
        LOG_ERROR("One or more component creations have failed");
        return -1;
    }

    // declarations
    SRef<Image>                                         view1, view2, view;
    SRef<Keyframe>                                      keyframe1;
    SRef<Keyframe>                                      keyframe2;
    std::vector<SRef<Keypoint>>                         keypointsView1, keypointsView2, keypoints;
    SRef<DescriptorBuffer>                              descriptorsView1, descriptorsView2, descriptors;
    std::vector<DescriptorMatch>                        matches;

    Transform3Df                                        poseFrame1 = Transform3Df::Identity();
    Transform3Df                                        poseFrame2;

    std::vector<SRef<CloudPoint>>                       cloud;

    std::vector<Transform3Df>                           keyframePoses;
    std::vector<Transform3Df>                           framePoses;

    SRef<Image>                                         view_current;
    std::vector<SRef<Image>>                            views;
    SRef<Image>                                         currentMatchImage;
    SRef<Image>                                         projected_image;

    SRef<Image>                                         imageSBSMatches;

    // initialize pose estimation with the camera intrinsic parameters (please refeer to the use of intrinsec parameters file)
    PnP->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    poseFinderFrom2D2D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    mapper->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

    LOG_DEBUG("Intrincic parameters : \n {}", camera->getIntrinsicsParameters());

    if (camera->start() != FrameworkReturnCode::_SUCCESS) // videoFile
    {
        LOG_ERROR("Camera cannot start");
        return -1;
    }

    // Here, Capture the two first keyframe view1, view2
    bool imageCaptured = false;
    while (!imageCaptured)
    {
        if (camera->getNextImage(view1) == SolAR::FrameworkReturnCode::_ERROR_)
            break;
        if (imageViewerFrame1->display(view1) == SolAR::FrameworkReturnCode::_STOP)
            imageCaptured = true;
    }

    imageCaptured = false;
    while (!imageCaptured)
    {
        if (camera->getNextImage(view2) == SolAR::FrameworkReturnCode::_ERROR_)
            break;
        if (imageViewerFrame2->display(view2) == SolAR::FrameworkReturnCode::_STOP)
            imageCaptured = true;
    }

    // Detect keypoints, extract descriptors and create frame for the first two view
    keypointsDetector->detect(view1, keypointsView1);
    descriptorExtractorAKAZE2->extract(view1, keypointsView1, descriptorsView1);


    keypointsDetector->detect(view2, keypointsView2);
    descriptorExtractorAKAZE2->extract(view2, keypointsView2, descriptorsView2);

    // Match keypoint between the two first frame, filtered and display them
    matcherKNN->match(descriptorsView1, descriptorsView2, matches);
    int nbOriginalMatches = matches.size();
    // Estimate the pose of of the second frame (the first frame being the reference of our coordinate system)
    poseFinderFrom2D2D->estimate(keypointsView1, keypointsView2, poseFrame1, poseFrame2, matches);
    overlaySBS->drawMatchesLines(view1, view2, imageSBSMatches, keypointsView1, keypointsView2, matches);
    LOG_INFO("Nb matches for triangulation: {}\\{}", matches.size(), nbOriginalMatches);
    LOG_INFO("Estimate pose of the camera for the frame 2: \n {}", poseFrame2.matrix());

    // Triangulate
    double reproj_error = mapper->triangulate(keypointsView1,keypointsView2, matches, std::make_pair(0, 1),poseFrame1, poseFrame2,cloud);


    // Initialize the mapper with the frist to frames
    keyframe1 = xpcf::utils::make_shared<Keyframe>(view1, descriptorsView1, 0, poseFrame1, keypointsView1);
    keyframe2 = xpcf::utils::make_shared<Keyframe>(view2, descriptorsView2, 1, poseFrame2, keypointsView2);
    poseGraph->initMap(keyframe1, keyframe2, cloud, matches);
    keyframePoses.push_back(poseFrame1);
    keyframePoses.push_back(poseFrame2);

    if ((imageViewerMatches->display(imageSBSMatches) == FrameworkReturnCode::_STOP) ||
       (viewer3DPoints->display(cloud, poseFrame2) == FrameworkReturnCode::_STOP))
               return 0;

    // Start tracking
    int nbFrameSinceKeyFrame = 0;
    while (true)
    {
        nbFrameSinceKeyFrame++;

        // Get current image
        camera->getNextImage(view);

        // Create a new fram
        SRef<Frame> newFrame = xpcf::utils::make_shared<Frame>();

        // Detect keypoints, extract their corresponding descriptors and add them to the frame
        keypointsDetector->detect(view, keypoints);
        descriptorExtractorAKAZE2->extract(view, keypoints, descriptors);
        newFrame->InitKeyPointsAndDescriptors(keypoints, descriptors);

        // Add this new frame to the Keyframe of reference
        poseGraph->associateReferenceKeyFrameToFrame(newFrame);
        newFrame->setNumberOfFramesSinceLastKeyFrame(nbFrameSinceKeyFrame);

        SRef<Keyframe> referenceKeyFrame = newFrame->getReferenceKeyFrame();

        // match current keypoints with the keypoints of the Keyframe
        SRef<DescriptorBuffer> keyframeDescriptors = referenceKeyFrame->getDescriptors();
        matcherKNN->match(keyframeDescriptors, descriptors, matches);
        matchesFilter->filter(matches, matches, referenceKeyFrame->getKeyPoints(), keypoints);

        // display matches
        overlaySBS->drawMatchesLines(referenceKeyFrame->m_view, view, imageSBSMatches, referenceKeyFrame->getKeyPoints(), keypoints, matches);
        if (imageViewerMatches->display(imageSBSMatches) == FrameworkReturnCode::_STOP)
            return 0;

        std::vector<SRef<Point2Df>> pt2d;
        std::vector<SRef<Point3Df>> pt3d;
        std::vector<SRef<CloudPoint>> foundPoints;
        std::vector<DescriptorMatch> foundMatches;
        std::vector<DescriptorMatch> remainingMatches;

        corr2D3DFinder->find(referenceKeyFrame->getVisibleMapPoints(), referenceKeyFrame->m_idx, matches, keypoints, foundPoints, pt3d, pt2d, foundMatches, remainingMatches);

        std::vector<SRef<Point2Df>> imagePoints_inliers;
        std::vector<SRef<Point3Df>> worldPoints_inliers;

        Transform3Df pose_current;

        bool updating_map = true;
        if (PnP->estimate(pt2d, pt3d, imagePoints_inliers, worldPoints_inliers, pose_current) == FrameworkReturnCode::_SUCCESS){
            LOG_INFO(" pnp inliers size: {} / {}",worldPoints_inliers.size(), pt3d.size());

            newFrame->m_pose = pose_current.inverse();
            framePoses.push_back(newFrame->m_pose);

            // triangulate with the first keyframe !
            std::vector<SRef<CloudPoint>>cloud_t;
            int referenceKeyFrameId = referenceKeyFrame->m_idx;
            std::cout<<"    ->reference keyframe: "<<referenceKeyFrame->m_idx<<std::endl;
            if(updating_map){
                mapper->triangulate(referenceKeyFrame->getKeyPoints(), keypoints, matches,std::make_pair<int,int>((int)referenceKeyFrameId,(int)(referenceKeyFrameId+1)),
                                    referenceKeyFrame->m_pose, pose_current, cloud_t);

                cloud.insert(cloud.end(), cloud_t.begin()  , cloud_t.end());
                if (viewer3DPoints->display(cloud, pose_current, keyframePoses, framePoses) == FrameworkReturnCode::_STOP)
                           return 0;
            }

            LOG_INFO(" cloud current size: {}", cloud.size());

            //return true;

            }else{
               // std::cout<<"new keyframe creation.."<<std::endl;
                return false;
        }
    }
    return 0;
}



