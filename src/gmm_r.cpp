#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int64MultiArray.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <bits/stdc++.h>
#include <GMM/GMMAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/server_goal_handle_imp.h>
using namespace cv;
using namespace std;
using namespace actionlib;
typedef actionlib::SimpleActionServer<GMM::GMMAction> Server;
  ros::Publisher pub_n;
int val_crl = 0;
int val_crl_max= 255;
int val_cbl = 0;
int val_cbl_max= 255;
int val_cru = 0;
int val_cru_max= 255;
int val_cbu = 0;
int val_cbu_max= 255;
Mat fgMaskMOG2; //fg mask fg mask generated by MOG2 method
static Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
int goal_=0;
  
 /* @function processVideo
 */
void processVideo(Mat img) {

        std_msgs::Int64MultiArray s;
        Mat img_hsv;
      //  imshow("FG Mask MOG 2", fgMaskMOG2);

        cvtColor(img,img_hsv,CV_BGR2HSV);
        //TEST BLUE GOOD IF IT GIVES result
        Mat blue_tub(img.rows,img.cols,CV_8UC1);
        Mat blue_ycrcb(img.rows,img.cols,CV_8UC1);
        for (int i = 0; i < img.rows; i++)
	      {
            for (int j = 0; j < img.cols; j++)
            {
              //  h = get_h((int)img_hsv.at<Vec3b>(i,j)[0],(int)img_hsv.at<Vec3b>(i,j)[1],(int)img_hsv.at<Vec3b>(i,j)[2]);
              if((int)fgMaskMOG2.at<uchar>(i,j)==255 && (int)img_hsv.at<Vec3b>(i,j)[0] > 150)
              {
                  blue_tub.at<uchar>(i,j)=255;
              }
              else  blue_tub.at<uchar>(i,j)= 0 ;


            }
        }

        imshow("blue_tub",blue_tub);

        GaussianBlur(blue_tub,blue_tub,Size(45,45),2,2);

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        findContours(blue_tub, contours, hierarchy, RETR_CCOMP,CHAIN_APPROX_SIMPLE);

        if(contours.size() != 0)
        {
            /// Draw contours

            Mat drawing = Mat::zeros( blue_tub.size(), CV_8UC3 );
            Mat dst = Mat::zeros( blue_tub.size(), CV_8UC3 );

            int idx = 0, largestComp = 0;
            double maxArea = 0;

            vector<Point2f>center( contours.size() );
            vector<float>radius( contours.size() );
            vector<vector<Point> > contours_poly( contours.size() );
            Point2f vertex;
            Point2f vertex_opp;

            for( ; idx >= 0; idx = hierarchy[idx][0] )
            {
                const vector<Point>& c = contours[idx];
                double area = fabs(contourArea(Mat(c)));
                if( area > maxArea )
                {
                    maxArea = area;
                    largestComp = idx;
                }

                approxPolyDP( Mat(contours[idx]), contours_poly[idx], 3, true );
                minEnclosingCircle( (Mat)contours_poly[idx], center[idx], radius[idx] );
            }

            Scalar color( 0, 0, 255 );

            vertex.y = center[largestComp].y - radius[largestComp];
            vertex.x = center[largestComp].x - radius[largestComp];
            vertex_opp.y = center[largestComp].y + radius[largestComp];
            vertex_opp.x = center[largestComp].x + radius[largestComp];
            drawContours( dst, contours, largestComp, color, FILLED, LINE_8, hierarchy );
            rectangle( drawing, vertex, vertex_opp , color, 2, 8, 0 );
            circle( drawing, center[largestComp], (int)radius[largestComp], color, 2, 8, 0 );
           imshow("contours",drawing);

          s.data.push_back(center[largestComp].y);
         	s.data.push_back(center[largestComp].x);
         	s.data.push_back(radius[largestComp]);

           ROS_INFO("Values sent to topic-print\n");
           pub_n.publish(s);
        }

        else return;
}


/**re
  * @function call
  */
void call(const sensor_msgs::ImageConstPtr& msg){

  ROS_INFO("IMAGE RECIEVED\n");
  cv_bridge::CvImagePtr cv_ptr;
  ros::NodeHandle n;
 ros::Publisher pub_n;

  //create Background Subtractor objects
  //pMOG2 = createBackgroundSubtractorMOG2(); //MOG2 approach

  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  Mat img = cv_ptr->image;
  //update the background model
  pMOG2->apply(img, fgMaskMOG2);

  processVideo(img);
cout<<"hellp";
  imshow("FG Mask MOG 2", fgMaskMOG2);
  imshow("Frame", img);
  waitKey(10);

}

void execute(const GMM::GMMGoalConstPtr& goal)  // Note: "Action" is not appended to DoDishes here
	{
  // Do lots of awesome groundbreaking robot stuff here
  
 goal_=goal->order;


 
  //change this to 2 points bas utna kaafi hoga ..uska baad publish only when confidence he high ..like iske saath kf jhod sakte hai aur vo parameter bases pe template nikalna hai *sab kaam til sun

  
  

  /*if(goal_==3)*/
  
  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Mision_planner");
  ros::NodeHandle n;
   ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  Server server(n, "Mission_planner", boost::bind(&execute, _1, &server), false);
  server.start();
 	if (goal_==1)
 	 {
  	
  
  	pub_n = nh.advertise<std_msgs::Int64MultiArray>("state", 1);
  	image_transport::Subscriber image_sub_ = it.subscribe("input", 1, call);
	}
  
  ros::spin();
  
return 0;
}
