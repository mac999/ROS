// package: reschy
// module: robot_sensor_opencv
// revision history
// date       | author      | description
// 2015.10.24 | T.W. Kang   | draft version. compile success.
//
//
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "ImageWindow";
static const std::string OPENCV_COLOR_REGION_WINDOW = "ColorRegionWindow";

// subscriber
typedef enum ColorCode
{
	NoColor = 0,
	WhiteColor,	// wall and floor color
	RedColor,			// ladder color and valve color
	GreenColor,			// stair top and big obstacle object color
	BlueColor,			// stair bottom and small obstacle object color
	AuburnColor			// door color
} ColorCodeDef;
ColorCodeDef _colorCode = RedColor; // NoColor;

void subscribeActiveOpenCV(const std_msgs::StringPtr& input)
{
	printf("sub=%s", input->data.c_str());
	
	if(input->data.size() == 0)
		return;
	if(input->data == "on=white")
		_colorCode = WhiteColor;
	else if(input->data == "on=red")
		_colorCode = RedColor;
	else if(input->data == "on=green")
		_colorCode = GreenColor;
	else if(input->data == "on=blue")
		_colorCode = BlueColor;
	else if(input->data == "on=auburn")
		_colorCode = RedColor;
	else if(input->data == "off")
		_colorCode = NoColor;
}

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Publisher image_info_;
	
	int lowHSV[3]; // = {150, 130, 50}; // {170, 150, 60};
	int highHSV[3]; // = {179, 255, 255}; // {179, 255, 255};

public:
	ImageConverter()
	: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/image_raw", 1, 
		&ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/reschy/sensor/vision/image_raw", 1);
		image_info_ = nh_.advertise<std_msgs::String> ("/reschy/sensor/vision/image_info", 1);	
		
		ros::Subscriber subActiveOpenCV = nh_.subscribe ("/reschy/control/active/vision", 1, subscribeActiveOpenCV);
		// rostopic pub -1 reschy/control/active/vision std_msgs/String "on=red"
		
		ResetColor(NoColor);

		cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}
	
	void ResetColor(ColorCodeDef code)
	{
		if(code == NoColor)
			return;
		
		if(code == RedColor) // for red
		{
			lowHSV[0] = 150, lowHSV[1] = 100, lowHSV[2] = 50;
			highHSV[0] = 179, highHSV[1] = 255, highHSV[2] = 255;
		}
		else if(code == GreenColor)	// for green
		{
			lowHSV[0] = 35, lowHSV[1] = 100, lowHSV[2] = 50;
			highHSV[0] = 80, highHSV[1] = 255, highHSV[2] = 255;
		}
		else if(code == BlueColor)	// for blue
		{
			lowHSV[0] = 90, lowHSV[1] = 100, lowHSV[2] = 50;
			highHSV[0] = 140, highHSV[1] = 255, highHSV[2] = 255;
		}
		else if(code == AuburnColor)	// for auburn
		{
			lowHSV[0] = 0, lowHSV[1] = 0, lowHSV[2] = 75;
			highHSV[0] = 100, highHSV[1] = 32, highHSV[2] = 115;
		}
		else if(code == WhiteColor)	// for white
		{
			lowHSV[0] = 30, lowHSV[1] = 0, lowHSV[2] = 160;
			highHSV[0] = 100, highHSV[1] = 40, highHSV[2] = 255;	
		}
	}
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		if(_colorCode == NoColor)
			return;
			
		cv_bridge::CvImagePtr cv_ptr; 
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			printf("\nError toCvCopy()");
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// Draw an example circle on the video stream
		cv_bridge::CvImage out_img;
		out_img.header = msg->header;
		out_img.encoding = sensor_msgs::image_encodings::BGR8;	// TBD: rviz image can't be seen.

		// cv::Mat imgThresholded;
		float area, posX, posY;
		area = posX = posY = 0.0;
		ResetColor(_colorCode);
		detectColorRegion(cv_ptr->image, area, posX, posY, out_img.image); // imgThresholded);
				
		char szImageInfo[512];
		memset(szImageInfo, sizeof(szImageInfo), 0);
		sprintf(szImageInfo, "%.2f,%.1f,%.1f", area, posX, posY);
		std_msgs::String imageInfo;  
		imageInfo.data = szImageInfo;
		image_info_.publish(imageInfo); 

		image_pub_.publish(out_img.toImageMsg());

		// if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
		// 	cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
		// cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		// cv::waitKey(3);
		//image_pub_.publish(cv_ptr->toImageMsg());
	}

	double detectColorRegion(cv::Mat& matImage, float& area, float& posX, float& posY, cv::Mat& imgThresholded)
	{
		// cv::Mat imgLines = cv::Mat::zeros(matImage.size() cv::CV_8UC3);
		
		cv::Mat imgHSV;
		cv::cvtColor(matImage, imgHSV, cv::COLOR_BGR2HSV);
		
		cv::inRange(imgHSV, cv::Scalar(lowHSV[0], lowHSV[1], lowHSV[2]), cv::Scalar(highHSV[0], highHSV[1], highHSV[2]), imgThresholded);
		
		cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		
		cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		
		cv::Moments moments = cv::moments(imgThresholded);
		float m01 = (float)moments.m01;
		float m10 = (float)moments.m10;
		area = (float)moments.m00;
		
		if(area > 10000)
		{
			posX = m10 / area;
			posY = m01 / area;
			cv::circle(imgThresholded, cv::Point(posX, posY), 30, CV_RGB(150, 130, 50), 3);
			printf("Color area = %.2f, Pos = (%.1f, %.1f)\n", area, posX, posY);
		}
		
		/* cv::imshow(OPENCV_COLOR_REGION_WINDOW, imgThresholded); 
		
		static bool InitGUI = false;
		if(InitGUI == false)
		{
			cv::createTrackbar("LowH", OPENCV_COLOR_REGION_WINDOW, &(lowHSV[0]), 179);
			cv::createTrackbar("HighH", OPENCV_COLOR_REGION_WINDOW, &(highHSV[0]), 179);

			cv::createTrackbar("LowS", OPENCV_COLOR_REGION_WINDOW, &(lowHSV[1]), 255);
			cv::createTrackbar("HighS", OPENCV_COLOR_REGION_WINDOW, &(highHSV[1]), 255);

			cv::createTrackbar("LowV", OPENCV_COLOR_REGION_WINDOW, &(lowHSV[2]), 255);
			cv::createTrackbar("HighV", OPENCV_COLOR_REGION_WINDOW, &(highHSV[2]), 255);

			InitGUI = true;
		} */

		return area;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_sensor_opencv");
	ImageConverter ic;
	ros::spin();
	return 0;
}
