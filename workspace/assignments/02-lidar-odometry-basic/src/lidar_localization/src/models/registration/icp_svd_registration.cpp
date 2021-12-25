/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */

#include <pcl/common/transforms.h>

#include <cmath>
#include <vector>
#include <numeric>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "glog/logging.h"

#include "lidar_localization/models/registration/icp_svd_registration.hpp"

namespace lidar_localization {

ICPSVDRegistration::ICPSVDRegistration(
    const YAML::Node& node
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    // parse params:
    float max_corr_dist = node["max_corr_dist"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

ICPSVDRegistration::ICPSVDRegistration(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

bool ICPSVDRegistration::SetRegistrationParam(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) {
    // set params:
    max_corr_dist_ = max_corr_dist;
    trans_eps_ = trans_eps;
    euc_fitness_eps_ = euc_fitness_eps;
    max_iter_ = max_iter;

    LOG(INFO) << "ICP SVD params:" << std::endl
              << "max_corr_dist: " << max_corr_dist_ << ", "
              << "trans_eps: " << trans_eps_ << ", "
              << "euc_fitness_eps: " << euc_fitness_eps_ << ", "
              << "max_iter: " << max_iter_ 
              << std::endl << std::endl;

    return true;
}

bool ICPSVDRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    input_target_ = input_target;
    input_target_kdtree_->setInputCloud(input_target_);

    return true;
}

bool ICPSVDRegistration::ScanMatch(
    const CloudData::CLOUD_PTR& input_source, 
    const Eigen::Matrix4f& predict_pose, 
    CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose
) {
    input_source_ = input_source;

    // pre-process input source:
    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

    // init estimation:
    transformation_.setIdentity();
    
    //
    // TODO: first option -- implement all computing logic on your own
    //
    // do estimation:
    Eigen::Matrix4f current_transform;
    current_transform.setIdentity();
    int curr_iter = 0;
    bool is_significant = true;
    while ((curr_iter < max_iter_) && is_significant) {
        LOG(INFO) << "curr_iter:" << curr_iter << std::endl;
        // TODO: apply current estimation:
        pcl::transformPointCloud(*transformed_input_source, *transformed_input_source, current_transform);
        // TODO: get correspondence:
        std::vector<Eigen::Vector3f> points_target;
        std::vector<Eigen::Vector3f> points_input;
        auto num = GetCorrespondence(transformed_input_source, points_target, points_input);
        // TODO: do not have enough correspondence -- break:
        if (num < 500) {
            LOG(INFO) << "quit corr num" << num << std::endl;
            break;
        }

        float correspondences_cur_mse = 0;        
        for (int i=0; i<num; i++) {
            correspondences_cur_mse += (points_input[i] - points_target[i]).topRows(2).norm();
        }
        correspondences_cur_mse /= num;
        LOG(INFO) << " correspondences_cur_mse: " << correspondences_cur_mse << std::endl;

        // TODO: update current transform:
        GetTransform(points_target, points_input, current_transform);

        // TODO: whether the transformation update is significant:
        is_significant = IsSignificant(current_transform, trans_eps_);
        LOG(INFO) << "is_significant: " << is_significant << std::endl;
        // TODO: update transformation:
        transformation_ = current_transform * transformation_;

        ++curr_iter;
    }
    if (curr_iter >= max_iter_) {
        LOG(INFO) << "quit exceed max iter" << std::endl;
    }
    
    // set output:
    result_pose = transformation_ * predict_pose;
    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);
    
    return true;
}

size_t ICPSVDRegistration::GetCorrespondence(
    const CloudData::CLOUD_PTR &input_source, 
    std::vector<Eigen::Vector3f> &xs,
    std::vector<Eigen::Vector3f> &ys
) {
    const float MAX_CORR_DIST_SQR = max_corr_dist_ * max_corr_dist_;

    size_t num_corr = 0;

    // TODO: set up point correspondence
    const auto cloud_size = input_source->size();
    std::vector<int> k_indices(1);
    std::vector<float> k_sqr_distances(1);

    for (size_t i=0; i<cloud_size; ++i) {
        int num = input_target_kdtree_->nearestKSearch((*input_source)[i], 1, k_indices, k_sqr_distances);
        if (num <= 0) continue;
        if (k_sqr_distances[0] > MAX_CORR_DIST_SQR) continue;
        auto corr_pt = (*input_target_)[k_indices[0]];
        xs.emplace_back(Eigen::Vector3f{corr_pt.x, corr_pt.y, corr_pt.z});
        ys.emplace_back(Eigen::Vector3f{(*input_source)[i].x, (*input_source)[i].y, (*input_source)[i].z});
    }
    num_corr = ys.size(); 
    return num_corr;
}

void ICPSVDRegistration::GetTransform(
    const std::vector<Eigen::Vector3f> &xs,
    const std::vector<Eigen::Vector3f> &ys,
    Eigen::Matrix4f &transformation
) {
    const size_t N = xs.size();

    // TODO: find centroids of mu_x and mu_y:
    Eigen::Vector3f mu_x = std::accumulate(xs.begin(), xs.end(), Eigen::Vector3f{0, 0, 0}) / xs.size();
    Eigen::Vector3f mu_y = std::accumulate(ys.begin(), ys.end(), Eigen::Vector3f{0, 0, 0}) / ys.size();
    // TODO: build H:
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (size_t i=0; i<N; ++i) {
        H += (ys[i]-mu_y) * (xs[i]-mu_x).transpose();
    }
    // TODO: solve R:
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV );
	Eigen::Matrix3f V = svd.matrixV(), U = svd.matrixU();
    Eigen::Quaternionf q(V*U.transpose());
    Eigen::Matrix3f R = q.normalized().toRotationMatrix();
    LOG(INFO) << "R.determinant:" << R.determinant() << std::endl;

    // TODO: solve t:
    Eigen::Vector3f t = mu_x - R * mu_y;
    // TODO: set output:
    transformation = Eigen::Matrix4f::Identity();

    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = t;
}

bool ICPSVDRegistration::IsSignificant(
    const Eigen::Matrix4f &transformation,
    const float trans_eps
) {
    // a. translation magnitude -- norm:
    float translation_magnitude = transformation.block<3, 1>(0, 3).norm();
    // b. rotation magnitude -- angle:
    float rotation_magnitude = fabs(
        acos(
            (transformation.block<3, 3>(0, 0).trace() - 1.0f) / 2.0f
        )
    );
    LOG(INFO) << "translation_magnitude: " << translation_magnitude <<
                 " rotation_magnitude:" << rotation_magnitude << std::endl;

    return (
        (translation_magnitude > trans_eps) || 
        (rotation_magnitude > trans_eps)
    );
}

} // namespace lidar_localization