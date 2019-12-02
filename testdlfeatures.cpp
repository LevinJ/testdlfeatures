#include <iostream>
using namespace std;
#include<opencv2/core/core.hpp>
#include "SPextractor.h"
using namespace ORB_SLAM2;
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
#include "tic_toc.h"



int main(int argc, char **argv) {
	cout << "Hello world";
	int nFeatures = 150;
	float fScaleFactor = 1.2;
	int nLevels = 4;
	float fIniThFAST = 0.015;
	float fMinThFAST =0.007;

	SPextractor*  mpORBextractorLeft = new SPextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
	cv::Mat im;
	std::vector<cv::KeyPoint> mvKeys;
	cv::Mat mDescriptors;

	string img_filename = "/home/levin/workspace/data/kitti/sequences/04/image_0/000250.png";
	im = cv::imread(img_filename,CV_LOAD_IMAGE_GRAYSCALE);

	TicToc t_m;
	(*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
	cout<<"DL time =" <<t_m.toc()<<" ms"<<endl;
	//draw the result
	cv::Mat imTrack;
	imTrack = im.clone();
	cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

	for (size_t i = 0; i < mvKeys.size(); i++)
	{
		cv::Point2f rightPt = mvKeys[i].pt;
		cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
	}
	std::cout<<"dl feature size = "<<mvKeys.size()<<endl;

	//extract it with Tosimas
	vector<cv::Point2f> n_pts;
	int MIN_DIST = 30;
	TicToc t_m2;
	cv::goodFeaturesToTrack(im, n_pts, nFeatures, 0.01, MIN_DIST);
	cout<<"Tosimas time =" <<t_m2.toc()<<" ms"<<endl;
	//draw the result
	cv::Mat imTrack2;
	imTrack2 = im.clone();
	cv::cvtColor(imTrack2, imTrack2, CV_GRAY2RGB);

	for (size_t i = 0; i < n_pts.size(); i++)
	{
		cv::Point2f rightPt = n_pts[i];
		cv::circle(imTrack2, rightPt, 2, cv::Scalar(0, 255, 0), 2);
	}
	std::cout<<"Tosimas feature size = "<<n_pts.size()<<endl;


//	cv::namedWindow( "Display window", WINDOW_AUTOSIZE );
	cv::Mat matArray[] = { imTrack,imTrack2};
	cv::Mat out;
	cv::vconcat( matArray, 2, out );
	cv::imshow("Display window", out);
	cv::waitKey(0);
	cout << "\n Done...";

	return 0;
}
