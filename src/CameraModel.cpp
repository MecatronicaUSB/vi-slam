/**
* Copyright 2018.
* Developed by Fabio Morales,
* Email: fabmoraleshidalgo@gmail.com; GitHub: @fmoralesh

*/

#include "../include/CameraModel.hpp"

namespace vi
{

CameraModel::~CameraModel() {}

// TODO(GitHub:fmoralesh, fabmoraleshidalgo@gmail.com) 02-13-2018 - Implement other camera models to UWSLAM (FOV)
void CameraModel::GetCameraModel(string _calibration_path) {
    valid_ = true;

    // Reading intrinsic parameters and distortion coefficients from file
    Mat calibration_values = Mat(1, 4, CV_32FC1, Scalar(0.0));
    Mat distortion_values  = Mat(1, 4, CV_32FC1, Scalar(0.0));
    FileStorage opencv_file(_calibration_path, FileStorage::READ);
    if (opencv_file.isOpened()) {
        cout << " ... found" << endl;
        opencv_file["in_width"] >> in_width_;
        opencv_file["in_height"] >> in_height_;
        opencv_file["out_width"] >> out_width_;
        opencv_file["out_height"] >> out_height_;
        opencv_file["calibration_values"] >> calibration_values;
        opencv_file["rectification"] >> distortion_values;
        opencv_file["imu2cam0Transformation"]>> imu2cam0Transformation;
        opencv_file["camera_frecuency"]>> camera_frecuency;
        opencv_file["imu_frecuency"]>> imu_frecuency;
        opencv_file["min_features"]>>min_features;
        opencv_file["num_max_keyframes"]>> num_max_keyframes;
        opencv_file["start_index"]>> start_index;
        opencv_file["use_gt"]>> use_gt;
        opencv_file["use_ros"]>> use_ros;
        opencv_file["num_cells"]>> num_cells;
        opencv_file["length_patch"]>> length_patch;
        opencv_file["detector"]>> detector;
        opencv_file["matcher"]>> matcher;
        
        
        opencv_file.release();
    } else {
        cout << " ... not found" << endl;
        cout << "Cannot operate without calibration" << endl;
        cout << "Exiting..." << endl;
        valid_ = false;
        exit(0);
    }

    // Saving parameters and distCoeffs
    for (int i = 0; i < 4; i++) {
        input_calibration_[i] = calibration_values.at<float>(0,i);
		dist_coeffs_.at<float>(i,0) = distortion_values.at<float>(0,i);
    }

    // Checking if the intrinsic parameters needs rescaling
    if (input_calibration_[2] < 1 && input_calibration_[3] < 1) {
        cout << "WARNING: cx = " << input_calibration_[2] << " < 1, which should not be the case for normal cameras" << endl;
        // Rescale. (Maybe will need to delete -0.5 offset)      
        input_calibration_[0] = input_calibration_[0] * in_width_;
        input_calibration_[1] = input_calibration_[1] * in_height_;
        input_calibration_[2] = input_calibration_[2] * in_width_;
        input_calibration_[3] = input_calibration_[3] * in_height_;
    }

    // Saving parameters in original_intrinsic_camera_
    original_intrinsic_camera_.at<float>(0,0) = input_calibration_[0];
    original_intrinsic_camera_.at<float>(1,1) = input_calibration_[1];
    original_intrinsic_camera_.at<float>(0,2) = input_calibration_[2];
    original_intrinsic_camera_.at<float>(1,2) = input_calibration_[3];
    original_intrinsic_camera_.at<float>(2,2) = 1;

    // If distCoeff are 0, dont apply rectification
    if (dist_coeffs_.at<float>(0,0) == 0) {
        cout << "Distortion coefficients not found ... not rectifying" << endl;
        valid_ = false;
        // K_
        output_intrinsic_camera_ = original_intrinsic_camera_;        
    }
    if (valid_) {
        cout << "Distortion coefficients found ... rectifying" << endl;
        // Obtaining new intrinsic camera matrix with undistorted images

        
        output_intrinsic_camera_ = getOptimalNewCameraMatrix(original_intrinsic_camera_, dist_coeffs_, Size(in_width_, in_height_), 1.0, Size(out_width_, out_height_), nullptr, false);
        initUndistortRectifyMap(original_intrinsic_camera_, dist_coeffs_, Mat(), output_intrinsic_camera_, Size(out_width_, out_height_), CV_16SC2, map1_, map2_);

        // K_
        original_intrinsic_camera_.at<float>(0, 0) /= in_width_;
		original_intrinsic_camera_.at<float>(0, 2) /= in_width_;
		original_intrinsic_camera_.at<float>(1, 1) /= in_height_;
		original_intrinsic_camera_.at<float>(1, 2) /= in_height_;
        
    }
}

void CameraModel::Undistort(const Mat& _image, OutputArray _result) const {
	remap(_image, _result, map1_, map2_, INTER_LINEAR);
}

const Mat& CameraModel::GetMap1() const {
    return map1_;
}

const Mat& CameraModel::GetMap2() const {
    return map2_;
}

const Mat& CameraModel::GetK() const {
	return output_intrinsic_camera_;
}

const Mat& CameraModel::GetOriginalK() const {
	return original_intrinsic_camera_;
}

int CameraModel::GetOutputWidth() const {
	return out_width_;
}

int CameraModel::GetOutputHeight() const {
	return out_height_;
}

int CameraModel::GetInputWidth() const {
	return in_width_;
}

int CameraModel::GetInputHeight() const {
	return in_height_;
}

bool CameraModel::IsValid() const {
	return valid_;
}

}