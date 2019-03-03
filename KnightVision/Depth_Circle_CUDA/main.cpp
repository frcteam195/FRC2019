#include <sl_zed/Camera.hpp>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

#include "opencv2/core.hpp"
#include "opencv2/core/cuda_types.hpp"
#include "opencv2/core/cuda.inl.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudaarithm.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <iterator>

using namespace sl;

cv::Mat slMat2cvMat(Mat& input);
void processDepthCV(Mat &depthMat, cv::cuda::GpuMat &grSc);

int main(int argc, char **argv) {

    const char *window_name = "Canny";
    cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);

    int lowThresh = 20;
    int highThres = 3 * lowThresh;
    int centerThresh = 30;
    int paramHigh = 170;

    cv::createTrackbar("Low Radius", window_name, &lowThresh, 1000);
    cv::createTrackbar("High Radius", window_name, &highThres, 10000);
    cv::createTrackbar("Center Threshold", window_name, &centerThresh, 200);
    cv::createTrackbar("Canny High Threshold", window_name, &paramHigh, 500);

	// Create a ZED camera object
	Camera zed;

	// Set configuration parameters
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_HD720;
	init_params.depth_mode = DEPTH_MODE_QUALITY;
	init_params.coordinate_units = UNIT_INCH;
	init_params.camera_fps = 30;
	//if (argc > 1) init_params.svo_input_filename.set(argv[1]);

	// Open the camera
	ERROR_CODE err = zed.open(init_params);
	if(err != SUCCESS) {
		printf("%s\n", toString(err).c_str());
		zed.close();
		return 1; // Quit if an error occurred
	}

	//zed.setCameraSettings(CAMERA_SETTINGS_EXPOSURE, 10, false);

	// Set runtime parameters after opening the camera
	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE_FILL;

	// Prepare new image size to retrieve half-resolution images
	Resolution image_size = zed.getResolution();
	int new_width = image_size.width / 2;
	int new_height = image_size.height / 2;

	Mat image_zed(new_width, new_height, MAT_TYPE_8U_C4);
	cv::Mat src_host = slMat2cvMat(image_zed);
	Mat depth_map(new_width, new_height, MAT_TYPE_8U_C4);
	cv::Mat depth_cv = slMat2cvMat(depth_map);


    //cv::cuda::GpuMat cvCirc(new_height, new_width, CV_8UC3);
    cv::Mat blurMat, cvCirc;

	cv::Ptr<cv::cuda::HoughCirclesDetector> circleDetector = cv::cuda::createHoughCirclesDetector(1, new_height/3, 60, 49, 20, 1182, 1);
    cv::Ptr<cv::cuda::Filter> gauss = cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, cv::Size(3, 3), 1, 1);

    cv:Mat out;

	while(1) {
        try {
            if(zed.grab(runtime_parameters) == SUCCESS) {
                zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, new_width, new_height);
                zed.retrieveMeasure(depth_map, MEASURE_DEPTH, MEM_CPU, new_width, new_height);

                //grSc.upload(src_host);
                //grSc.convertTo(grSc, CV_BGR2GRAY);

    //			grSc = cv::cuda::GpuMat::zeros(depth_map.getHeight(), depth_map.getWidth(), CV_8UC1);
    //			grScBlur = cv::Mat::zeros(depth_map.getHeight(), depth_map.getWidth(), CV_8UC1);
    //			cvCirc = cv::Mat::zeros(depth_map.getHeight(), depth_map.getWidth(), CV_8UC3);
                cv::cuda::GpuMat grSc(new_height, new_width, CV_8UC1);
                cv::cuda::GpuMat grScBlur(new_height, new_width, CV_8UC1);
                cv::cuda::GpuMat circlesMat;

                processDepthCV(depth_map, grSc);

                gauss->apply(grSc, grScBlur);

                //cv::HoughCircles(grScBlur, circles, cv::HOUGH_GRADIENT, 1, grSc.rows/3, paramHigh, centerThresh, lowThresh, highThres);
                circleDetector->detect(grScBlur, circlesMat);

                std::vector<cv::Vec3f> circles;

                circles.resize(circlesMat.size().width);
                if(!circles.empty()) {
                    circlesMat.row(0).download(cv::Mat(circles).reshape(3, 1));
                }

                cvCirc = cv::Mat::zeros(new_height, new_width, CV_8UC3);

                for (size_t i = 0; i < circles.size(); i++) {
                    cv::Vec3i c = circles[i];
                    cv::Point center = cv::Point(c[0], c[1]);
                    cv::circle(cvCirc, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
                    cv::circle(cvCirc, center, c[2], cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
                }

                blurMat = cv::Mat::zeros(new_height, new_width, CV_8UC1);
                grScBlur.download(blurMat);

                cv::imshow("gr", blurMat);
                cv::imshow(window_name, cvCirc);

                cv::waitKey(10);
            }
		}
		catch (cv::Exception e) {
            std::cout << e.err << std::endl;
           // std::cout << e.func << std::endl;
           // std::cout << e.msg << std::endl;
		}
	}

	zed.close();
	return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
	// Mapping between MAT_TYPE and CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
		case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
		case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
		case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
		case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
		case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
		case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
		case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
		case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
		default: break;
	}

	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

void processDepthCV(sl::Mat &depthMat, cv::cuda::GpuMat &grSc) {
	const float minDepth = 2 * 12;
	const float maxDepth = 6 * 12;
//	float uMax = 0;
//	float uMin = 10000000;
	float currVal = 0;
//	for (size_t i = 0; i < depthMat.getWidth(); i++) {
//		for (size_t j = 0; j < depthMat.getHeight(); j++) {
//			depthMat.getValue(i, j, &currVal);
//			//std::cout << "I,J: "<< i << ","<< j << std::endl;
//			if (std::isfinite(currVal)) {
//				uMax = std::fmaxf(uMax, currVal);
//				uMin = std::fminf(uMin, currVal);
//            }
//		}
//	}

    cv::Mat tmp = cv::Mat::zeros(grSc.rows, grSc.cols, CV_8UC1);
    grSc.download(tmp);

	for (int i = 0; i < depthMat.getWidth(); i++) {
		for (int j = 0; j < depthMat.getHeight(); j++) {
			depthMat.getValue(i, j, &currVal);
			if (std::isfinite(currVal) && currVal <= maxDepth)
				tmp.at<uchar>(j, i, 0) = ((uchar)(std::fmaxf(currVal-minDepth,0) / maxDepth * 255));
			else
				tmp.at<uchar>(j, i, 0) = 0;
		}
	}

    grSc.upload(tmp);
}
