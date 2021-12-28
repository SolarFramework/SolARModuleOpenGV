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

#include "Triangulation.h"
#include "SolARModuleOpengv_traits.h"
#include "core/Log.h"

#include <opengv/triangulation/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>


namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENGV::Triangulation);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENGV {

Triangulation::Triangulation():ComponentBase(xpcf::toUUID<Triangulation>())
{
   addInterface<api::solver::map::ITriangulator>(this);
   LOG_DEBUG(" Triangulation constructor");
}

Triangulation::~Triangulation(){

}

double Triangulation::triangulate(const std::vector<Point2Df> & pointsView1,
                                                const std::vector<Point2Df> & pointsView2,
                                                const std::vector<DescriptorMatch> &matches,
                                                const std::pair<unsigned int,unsigned int>&working_views,
                                                const Transform3Df& poseView1,
                                                const Transform3Df& poseView2,
                                                std::vector<SRef<CloudPoint>> & pcloud ){
    pcloud.clear();

    opengv::translation_t position1 = Eigen::Vector3d(poseView1(0,3), poseView1(1,3) ,poseView1(2,3));
    opengv::rotation_t rotation1;// = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>();

    rotation1(0,0)  = poseView1(0,0); rotation1(0,1)  = poseView1(0,1); rotation1(0,2)  = poseView1(0,2);
    rotation1(1,0)  = poseView1(1,0); rotation1(1,1)  = poseView1(1,1); rotation1(1,2)  = poseView1(1,2);
    rotation1(2,0)  = poseView1(2,0); rotation1(2,1)  = poseView1(2,1); rotation1(2,2)  = poseView1(2,2);

    opengv::translation_t position2  = Eigen::Vector3d(poseView2(0,3), poseView2(1,3) ,poseView2(2,3));
    opengv::rotation_t rotation2;// = Eigen::Matrix<double, 3,3, Eigen::ColMajor>();

    rotation2(0,0)  = poseView2(0,0); rotation2(0,1)  = poseView2(0,1); rotation2(0,2)  = poseView2(0,2);
    rotation2(1,0)  = poseView2(1,0); rotation2(1,1)  = poseView2(1,1); rotation2(1,2)  = poseView2(1,2);
    rotation2(2,0)  = poseView2(2,0); rotation2(2,1)  = poseView2(2,1); rotation2(2,2)  = poseView2(2,2);

#if NDEBUG

    LOG_INFO("Pose 1 Position: \n {}", position1.matrix());
    LOG_INFO("Pose 1 rotation: \n {}", rotation1.matrix());

    LOG_INFO("Pose 2 position: \n {}", position2.matrix());
    LOG_INFO("Pose 2 rotation2: \n {}", rotation2.matrix());
#endif

    //Compute the relative pose
    opengv::translation_t relativePosition;
    opengv::rotation_t relativeRotation;
    relativeRotation = rotation1.transpose() * rotation2;
    relativePosition = rotation1.transpose() * (position2 - position1);

#if NDEBUG
    LOG_INFO("Estimated pose of the camera Relative Rotation: \n {}", relativeRotation.matrix());
    LOG_INFO("Estimated pose of the camera Relative Translation: \n {}", relativePosition.matrix());
#endif

    opengv::bearingVectors_t bearingVectors1;
    opengv::bearingVectors_t bearingVectors2;

    size_t numberPoints = matches.size();

    //we also need to multiply the point
    Eigen::Matrix<float,3,3> k_invert =  m_intrinsicParams.inverse();
    std::vector<Eigen::Vector3f> buffer_vector_A,buffer_vector_B;
    buffer_vector_A.resize(numberPoints);
    buffer_vector_B.resize(numberPoints);

     for (size_t i = 0; i<numberPoints; i++) {

        buffer_vector_A[i] = k_invert*Vector3f( (float)pointsView1[matches[i].getIndexInDescriptorA()].getX(), (float)pointsView1[matches[i].getIndexInDescriptorA()].getY(), 1.0);
        bearingVectors1.push_back(opengv::point_t( buffer_vector_A[i][0],buffer_vector_A[i][1],buffer_vector_A[i][2]));
        bearingVectors1[i] /= bearingVectors1[i].norm();

        buffer_vector_B[i] = k_invert*Vector3f( (float)pointsView2[matches[i].getIndexInDescriptorB()].getX(), (float)pointsView2[matches[i].getIndexInDescriptorB()].getY(), 1.0);
        bearingVectors2.push_back(opengv::point_t( buffer_vector_B[i][0],buffer_vector_B[i][1],buffer_vector_B[i][2]));
        bearingVectors2[i] /= bearingVectors1[i].norm();
     }

    //create a central relative adapter and pass the relative pose
    opengv::relative_pose::CentralRelativeAdapter adapter = opengv::relative_pose::CentralRelativeAdapter( bearingVectors1,
      bearingVectors2,
      relativePosition,
      relativeRotation);

    // size_t numberPoints = matches.size()
    Eigen::MatrixXd triangulate_results(3, numberPoints);

    //triangulate points
    for(size_t j = 0; j < numberPoints; j++){
        triangulate_results.block<3,1>(0,j) = opengv::triangulation::triangulate2(adapter,j);
    }

    double reproj_error = 0.0;

    for(size_t k = 0; k < numberPoints; k++){

#if NDEBUG   
    LOG_INFO(" FOR NOW ITS REPROJECTION ERROR IS SET TO 0...:" );
#endif
        std::map<unsigned int, unsigned int> visibility;

        visibility[working_views.first]  = matches[k].getIndexInDescriptorA();
        visibility[working_views.second] = matches[k].getIndexInDescriptorB();

        Eigen::Vector4f tmp_vec_to_reproj= Eigen::Vector4f(  triangulate_results.col(k)[0], triangulate_results.col(k)[1], triangulate_results.col(k)[2],1.0f);

        Eigen::Matrix<float,4,4> tmp_transform;
        tmp_transform(0,0) = poseView1(0,0);
        tmp_transform(0,1) = poseView1(0,1);
        tmp_transform(0,2) = poseView1(0,2);
        tmp_transform(0,3) = poseView1(0,3);

        tmp_transform(1,0) = poseView1(1,0);
        tmp_transform(1,1) = poseView1(1,1);
        tmp_transform(1,2) = poseView1(1,2);
        tmp_transform(1,3) = poseView1(1,3);

        tmp_transform(2,0) = poseView1(2,0);
        tmp_transform(2,1) = poseView1(2,1);
        tmp_transform(2,2) = poseView1(2,2);
        tmp_transform(2,3) = poseView1(2,3);

        tmp_transform(3,0) = poseView1(3,0);
        tmp_transform(3,1) = poseView1(3,1);
        tmp_transform(3,2) = poseView1(3,2);
        tmp_transform(3,3) = poseView1(3,3);

        Eigen::Vector4f  reprojected_point_3d = tmp_transform *  tmp_vec_to_reproj ;

        Eigen::Vector2d  reprojected_point_2d = Eigen::Vector2d(reprojected_point_3d[0]/reprojected_point_3d[2], reprojected_point_3d[1]/reprojected_point_3d[2]);

        Eigen::Vector3f undistort_point =  k_invert* Eigen::Vector3f((float)pointsView1[matches[k].getIndexInDescriptorA()].getX(),(float)pointsView1[matches[k].getIndexInDescriptorA()].getY(),1.0f);
#if NDEBUG          
        //TO DO : Apply undirstorsion
        LOG_INFO("  Apply undirstorsion ");
#endif
        Eigen::Vector2d initial_point_2d = Eigen::Vector2d(    undistort_point[0]/undistort_point.norm(),  undistort_point[1]/undistort_point.norm()   );
       
        double reprj_err =  Eigen::Vector2d(reprojected_point_2d - initial_point_2d).norm();

        reproj_error += reprj_err;

        Eigen::Vector3d tmp_vec = triangulate_results.col(k);
        SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(tmp_vec(0), tmp_vec(1), tmp_vec(2) ,0.0f,0.0f,0.0f, reprj_err, visibility);
        pcloud.push_back(cp);

    }
    
    return (double)reproj_error/(double)numberPoints;
}

double Triangulation::triangulate(const std::vector<Keypoint>& keypointsView1,
                                                const std::vector<Keypoint>& keypointsView2,
                                                const SRef<DescriptorBuffer>& descriptor1,
                                                const SRef<DescriptorBuffer>& descriptor2,
                                                const std::vector<DescriptorMatch>& matches,
                                                const std::pair<unsigned int, unsigned int>& working_views,
                                                const Transform3Df & poseView1,
                                                const Transform3Df & poseView2,
                                                std::vector<SRef<CloudPoint>>& pcloud)
{
    pcloud.clear();
    opengv::translation_t position1 = Eigen::Vector3d(poseView1(0,3), poseView1(1,3) ,poseView1(2,3));
    opengv::rotation_t rotation1;// = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>();

    rotation1(0,0)  = poseView1(0,0); rotation1(0,1)  = poseView1(0,1); rotation1(0,2)  = poseView1(0,2);
    rotation1(1,0)  = poseView1(1,0); rotation1(1,1)  = poseView1(1,1); rotation1(1,2)  = poseView1(1,2);
    rotation1(2,0)  = poseView1(2,0); rotation1(2,1)  = poseView1(2,1); rotation1(2,2)  = poseView1(2,2);

    opengv::translation_t position2  = Eigen::Vector3d(poseView2(0,3), poseView2(1,3) ,poseView2(2,3));
    opengv::rotation_t rotation2;// = Eigen::Matrix<double, 3,3, Eigen::ColMajor>();

    rotation2(0,0)  = poseView2(0,0); rotation2(0,1)  = poseView2(0,1); rotation2(0,2)  = poseView2(0,2);
    rotation2(1,0)  = poseView2(1,0); rotation2(1,1)  = poseView2(1,1); rotation2(1,2)  = poseView2(1,2);
    rotation2(2,0)  = poseView2(2,0); rotation2(2,1)  = poseView2(2,1); rotation2(2,2)  = poseView2(2,2);

#if NDEBUG

    LOG_INFO("Pose 1 Position: \n {}", position1.matrix());
    LOG_INFO("Pose 1 rotation: \n {}", rotation1.matrix());

    LOG_INFO("Pose 2 position: \n {}", position2.matrix());
    LOG_INFO("Pose 2 rotation2: \n {}", rotation2.matrix());
#endif

    //Compute the relative pose
    opengv::translation_t relativePosition;
    opengv::rotation_t relativeRotation;
    relativeRotation = rotation1.transpose() * rotation2;
    relativePosition = rotation1.transpose() * (position2 - position1);

#if NDEBUG
    LOG_INFO("Estimated pose of the camera Relative Rotation: \n {}", relativeRotation.matrix());
    LOG_INFO("Estimated pose of the camera Relative Translation: \n {}", relativePosition.matrix());
#endif

    opengv::bearingVectors_t bearingVectors1;
    opengv::bearingVectors_t bearingVectors2;

    size_t numberPoints = matches.size();

    //we also need to multiply the point
    Eigen::Matrix<float,3,3> k_invert =  m_intrinsicParams.inverse();
    std::vector<Eigen::Vector3f> buffer_vector_A,buffer_vector_B;
    buffer_vector_A.resize(numberPoints);
    buffer_vector_B.resize(numberPoints);

     for (size_t i = 0; i<numberPoints; i++) {

        buffer_vector_A[i] = k_invert*Vector3f( (float)keypointsView1[matches[i].getIndexInDescriptorA()].getX(), (float)keypointsView1[matches[i].getIndexInDescriptorA()].getY(), 1.0);
        bearingVectors1.push_back(opengv::point_t( buffer_vector_A[i][0],buffer_vector_A[i][1],buffer_vector_A[i][2]));
        bearingVectors1[i] /= bearingVectors1[i].norm();

        buffer_vector_B[i] = k_invert*Vector3f( (float)keypointsView2[matches[i].getIndexInDescriptorB()].getX(), (float)keypointsView2[matches[i].getIndexInDescriptorB()].getY(), 1.0);
        bearingVectors2.push_back(opengv::point_t( buffer_vector_B[i][0],buffer_vector_B[i][1],buffer_vector_B[i][2]));
        bearingVectors2[i] /= bearingVectors1[i].norm();
     }

    //create a central relative adapter and pass the relative pose
    opengv::relative_pose::CentralRelativeAdapter adapter = opengv::relative_pose::CentralRelativeAdapter( bearingVectors1,
      bearingVectors2,
      relativePosition,
      relativeRotation);

    // size_t numberPoints = matches.size()
    Eigen::MatrixXd triangulate_results(3, numberPoints);

    //triangulate points
    for(size_t j = 0; j < numberPoints; j++){
        triangulate_results.block<3,1>(0,j) = opengv::triangulation::triangulate2(adapter,j);
    }

    double reproj_error = 0.0;

    for(size_t k = 0; k < numberPoints; k++){

#if NDEBUG
    LOG_INFO(" FOR NOW ITS REPROJECTION ERROR IS SET TO 0...:" );
#endif
        std::map<unsigned int, unsigned int> visibility;

        visibility[working_views.first]  = matches[k].getIndexInDescriptorA();
        visibility[working_views.second] = matches[k].getIndexInDescriptorB();

        Eigen::Vector4f tmp_vec_to_reproj= Eigen::Vector4f(  triangulate_results.col(k)[0], triangulate_results.col(k)[1], triangulate_results.col(k)[2],1.0f);

        Eigen::Matrix<float,4,4> tmp_transform;
        tmp_transform(0,0) = poseView1(0,0);
        tmp_transform(0,1) = poseView1(0,1);
        tmp_transform(0,2) = poseView1(0,2);
        tmp_transform(0,3) = poseView1(0,3);

        tmp_transform(1,0) = poseView1(1,0);
        tmp_transform(1,1) = poseView1(1,1);
        tmp_transform(1,2) = poseView1(1,2);
        tmp_transform(1,3) = poseView1(1,3);

        tmp_transform(2,0) = poseView1(2,0);
        tmp_transform(2,1) = poseView1(2,1);
        tmp_transform(2,2) = poseView1(2,2);
        tmp_transform(2,3) = poseView1(2,3);

        tmp_transform(3,0) = poseView1(3,0);
        tmp_transform(3,1) = poseView1(3,1);
        tmp_transform(3,2) = poseView1(3,2);
        tmp_transform(3,3) = poseView1(3,3);

        Eigen::Vector4f  reprojected_point_3d = tmp_transform *  tmp_vec_to_reproj ;

        Eigen::Vector2d  reprojected_point_2d = Eigen::Vector2d(reprojected_point_3d[0]/reprojected_point_3d[2], reprojected_point_3d[1]/reprojected_point_3d[2]);

        Eigen::Vector3f undistort_point =  k_invert* Eigen::Vector3f((float)keypointsView1[matches[k].getIndexInDescriptorA()].getX(),(float)keypointsView1[matches[k].getIndexInDescriptorA()].getY(),1.0f);
#if NDEBUG
        //TO DO : Apply undirstorsion
        LOG_INFO("  Apply undirstorsion ");
#endif
        Eigen::Vector2d initial_point_2d = Eigen::Vector2d(    undistort_point[0]/undistort_point.norm(),  undistort_point[1]/undistort_point.norm()   );

        double reprj_err =  Eigen::Vector2d(reprojected_point_2d - initial_point_2d).norm();

        reproj_error += reprj_err;

        Eigen::Vector3d tmp_vec = triangulate_results.col(k);


        // calculate the mean of two features
        SRef<DescriptorBuffer> descBufferMean;
        if (descriptor1->getDescriptorDataType() == DescriptorDataType::TYPE_8U){

            DescriptorView8U desc_1 = descriptor1->getDescriptor<DescriptorDataType::TYPE_8U>(matches[k].getIndexInDescriptorA());
            DescriptorView8U desc_2 = descriptor1->getDescriptor<DescriptorDataType::TYPE_8U>(matches[k].getIndexInDescriptorB());

            Maths::Matrix<unsigned char, 1, Maths::Dynamic> desc1 = Maths::Map<Maths::Matrix<unsigned char, 1, Maths::Dynamic>>((unsigned char*)desc_1.data(), 1, desc_1.length());
            Maths::Matrix<unsigned char, 1, Maths::Dynamic> desc2 = Maths::Map<Maths::Matrix<unsigned char, 1, Maths::Dynamic>>((unsigned char*)desc_2.data(), 1, desc_2.length());
            // Cast to unsigned int to avoid overflow during mean computation
            Maths::Matrix<unsigned int, 1, Maths::Dynamic> descMeanInt = (desc1.cast<unsigned int>() + desc2.cast<unsigned int>())/2;
            Maths::Matrix<unsigned char, 1, Maths::Dynamic> descMean = descMeanInt.cast<unsigned char>();

            descBufferMean = xpcf::utils::make_shared<DescriptorBuffer>((unsigned char*)descMean.data(), descriptor1->getDescriptorType(), descriptor1->getDescriptorDataType(), descriptor1->getNbElements(), 1);
        }
        else {

            DescriptorView32F desc_1 = descriptor1->getDescriptor<DescriptorDataType::TYPE_32F>(matches[k].getIndexInDescriptorA());
            DescriptorView32F desc_2 = descriptor1->getDescriptor<DescriptorDataType::TYPE_32F>(matches[k].getIndexInDescriptorB());

            // calculate the mean of two features
            Maths::MatrixXf desc1 = Maths::Map<Maths::MatrixXf>((float*)desc_1.data(), 1, desc_1.length());
            Maths::MatrixXf desc2 = Maths::Map<Maths::MatrixXf>((float*)desc_2.data(), 1, desc_2.length());

            Maths::MatrixXf descMean = (desc1 + desc2) / 2;
            descBufferMean = xpcf::utils::make_shared<DescriptorBuffer>((unsigned char*)descMean.data(), descriptor1->getDescriptorType(), descriptor1->getDescriptorDataType(), descriptor1->getNbElements(), 1);
        }
        SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(tmp_vec(0), tmp_vec(1), tmp_vec(2) ,0.0f,0.0f,0.0f, reprj_err, visibility, descBufferMean);
        pcloud.push_back(cp);
    }

    return (double)reproj_error/(double)numberPoints;
}


double Triangulation::triangulate(const std::vector<Keypoint> & pointsView1,
                                                const std::vector<Keypoint> & pointsView2,
                                                const std::vector<DescriptorMatch> &matches,
                                                const std::pair<unsigned int,unsigned int>&working_views,
                                                const Transform3Df& poseView1,
                                                const Transform3Df& poseView2,
                                                std::vector<SRef<CloudPoint>> & pcloud)
{

    pcloud.clear();

    opengv::translation_t position1 = Eigen::Vector3d(poseView1(0,3), poseView1(1,3) ,poseView1(2,3));
    opengv::rotation_t rotation1;// = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>();

    rotation1(0,0)  = poseView1(0,0); rotation1(0,1)  = poseView1(0,1); rotation1(0,2)  = poseView1(0,2);
    rotation1(1,0)  = poseView1(1,0); rotation1(1,1)  = poseView1(1,1); rotation1(1,2)  = poseView1(1,2);
    rotation1(2,0)  = poseView1(2,0); rotation1(2,1)  = poseView1(2,1); rotation1(2,2)  = poseView1(2,2);

    opengv::translation_t position2  = Eigen::Vector3d(poseView2(0,3), poseView2(1,3) ,poseView2(2,3));
    opengv::rotation_t rotation2;// = Eigen::Matrix<double, 3,3, Eigen::ColMajor>();

    rotation2(0,0)  = poseView2(0,0); rotation2(0,1)  = poseView2(0,1); rotation2(0,2)  = poseView2(0,2);
    rotation2(1,0)  = poseView2(1,0); rotation2(1,1)  = poseView2(1,1); rotation2(1,2)  = poseView2(1,2);
    rotation2(2,0)  = poseView2(2,0); rotation2(2,1)  = poseView2(2,1); rotation2(2,2)  = poseView2(2,2);

#if NDEBUG

    LOG_INFO("Pose 1 Position: \n {}", position1.matrix());
    LOG_INFO("Pose 1 rotation: \n {}", rotation1.matrix());

    LOG_INFO("Pose 2 position: \n {}", position2.matrix());
    LOG_INFO("Pose 2 rotation2: \n {}", rotation2.matrix());
#endif

    //Compute the relative pose
    opengv::translation_t relativePosition;
    opengv::rotation_t relativeRotation;
    relativeRotation = rotation1.transpose() * rotation2;
    relativePosition = rotation1.transpose() * (position2 - position1);

#if NDEBUG
    LOG_INFO("Estimated pose of the camera Relative Rotation: \n {}", relativeRotation.matrix());
    LOG_INFO("Estimated pose of the camera Relative Translation: \n {}", relativePosition.matrix());
#endif

    opengv::bearingVectors_t bearingVectors1;
    opengv::bearingVectors_t bearingVectors2;

    size_t numberPoints = matches.size();

    //we also need to multiply the point
    Eigen::Matrix<float,3,3> k_invert =  m_intrinsicParams.inverse();
    std::vector<Eigen::Vector3f> buffer_vector_A,buffer_vector_B;
    buffer_vector_A.resize(numberPoints);
    buffer_vector_B.resize(numberPoints);

     for (size_t i = 0; i<numberPoints; i++) {

        buffer_vector_A[i] = k_invert*Vector3f( (float)pointsView1[matches[i].getIndexInDescriptorA()].getX(), (float)pointsView1[matches[i].getIndexInDescriptorA()].getY(), 1.0);
        bearingVectors1.push_back(opengv::point_t( buffer_vector_A[i][0],buffer_vector_A[i][1],buffer_vector_A[i][2]));
        bearingVectors1[i] /= bearingVectors1[i].norm();

        buffer_vector_B[i] = k_invert*Vector3f( (float)pointsView2[matches[i].getIndexInDescriptorB()].getX(), (float)pointsView2[matches[i].getIndexInDescriptorB()].getY(), 1.0);
        bearingVectors2.push_back(opengv::point_t( buffer_vector_B[i][0],buffer_vector_B[i][1],buffer_vector_B[i][2]));
        bearingVectors2[i] /= bearingVectors1[i].norm();
     }

    //create a central relative adapter and pass the relative pose
    opengv::relative_pose::CentralRelativeAdapter adapter = opengv::relative_pose::CentralRelativeAdapter( bearingVectors1,
      bearingVectors2,
      relativePosition,
      relativeRotation);

    // size_t numberPoints = matches.size()
    Eigen::MatrixXd triangulate_results(3, numberPoints);

    //triangulate points
    for(size_t j = 0; j < numberPoints; j++){
        triangulate_results.block<3,1>(0,j) = opengv::triangulation::triangulate2(adapter,j);
    }

    double reproj_error = 0.0;

    for(size_t k = 0; k < numberPoints; k++){

        LOG_DEBUG(" FOR NOW ITS REPROJECTION ERROR IS SET TO 0...:" );

        std::map<unsigned int, unsigned int> visibility;

        visibility[working_views.first]  = matches[k].getIndexInDescriptorA();
        visibility[working_views.second] = matches[k].getIndexInDescriptorB();

        Eigen::Vector4f tmp_vec_to_reproj= Eigen::Vector4f(  triangulate_results.col(k)[0], triangulate_results.col(k)[1], triangulate_results.col(k)[2],1.0f);

        Eigen::Matrix<float,4,4> tmp_transform;
        tmp_transform(0,0) = poseView1(0,0);
        tmp_transform(0,1) = poseView1(0,1);
        tmp_transform(0,2) = poseView1(0,2);
        tmp_transform(0,3) = poseView1(0,3);

        tmp_transform(1,0) = poseView1(1,0);
        tmp_transform(1,1) = poseView1(1,1);
        tmp_transform(1,2) = poseView1(1,2);
        tmp_transform(1,3) = poseView1(1,3);

        tmp_transform(2,0) = poseView1(2,0);
        tmp_transform(2,1) = poseView1(2,1);
        tmp_transform(2,2) = poseView1(2,2);
        tmp_transform(2,3) = poseView1(2,3);

        tmp_transform(3,0) = poseView1(3,0);
        tmp_transform(3,1) = poseView1(3,1);
        tmp_transform(3,2) = poseView1(3,2);
        tmp_transform(3,3) = poseView1(3,3);

        Eigen::Vector4f  reprojected_point_3d = tmp_transform *  tmp_vec_to_reproj ;

        Eigen::Vector2d  reprojected_point_2d = Eigen::Vector2d(reprojected_point_3d[0]/reprojected_point_3d[2], reprojected_point_3d[1]/reprojected_point_3d[2]);

        Eigen::Vector3f undistort_point =  k_invert* Eigen::Vector3f((float)pointsView1[matches[k].getIndexInDescriptorA()].getX(),(float)pointsView1[matches[k].getIndexInDescriptorA()].getY(),1.0f);

        //TO DO : Apply undirstorsion
        LOG_DEBUG("  Apply undirstorsion ");

        Eigen::Vector2d initial_point_2d = Eigen::Vector2d(    undistort_point[0]/undistort_point.norm(),  undistort_point[1]/undistort_point.norm()   );
       
        double reprj_err =  Eigen::Vector2d(reprojected_point_2d - initial_point_2d).norm();

        reproj_error += reprj_err;

        Eigen::Vector3d tmp_vec = triangulate_results.col(k);
        SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(tmp_vec(0), tmp_vec(1), tmp_vec(2) ,0.0f,0.0f,0.0f, reprj_err, visibility);
        pcloud.push_back(cp);

    }
    
    return (double)reproj_error/(double)numberPoints;
}

double Triangulation::triangulate(SRef<SolAR::datastructure::Frame> frame1,
                                  SRef<SolAR::datastructure::Frame> frame2,
                                  const std::vector<SolAR::datastructure::DescriptorMatch> &matches,
                                  const std::pair<uint32_t, uint32_t> & working_views,
                                  std::vector<SRef<SolAR::datastructure::CloudPoint>> & pcloud,
                                  const bool& onlyDepth)
{
    LOG_WARNING("Not implemented");
    return 0.0;
}

void Triangulation::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {

   m_intrinsicParams  = intrinsicParams;
   m_distorsionParams = distorsionParams;

}


}
}
}
