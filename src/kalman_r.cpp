#include <iostream>
#include <vector>
#include <sys/time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int64MultiArray.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;


//------------------------------------------------ convenience method for 
//                                                 using kalman filter with 
//                                                 Point objects
cv::KalmanFilter KF;
cv::Mat_<float> measurement(2,1); 
Mat_<float> state(4, 1); // (x, y, Vx, Vy)
ros::Publisher pub_n;
int flag=0;
Point S,p;
int dt=1;
void initKalman(float x, float y)
{
    // Initialize Kalman Filter with
    // 4 dynamic parameters and 2 measurement parameters,
    // where my measurement is: 2D location of object,
    // and dynamic is: 2D location and 2D velocity.
    KF.init(4, 2, 0);

    measurement = Mat_<float>::zeros(2,1);
    measurement.at<float>(0, 0) = x;
    measurement.at<float>(0, 0) = y;


    KF.statePre.setTo(0);
    KF.statePre.at<float>(0, 0) = x;
    KF.statePre.at<float>(1, 0) = y;

    KF.statePost.setTo(0);
    KF.statePost.at<float>(0, 0) = x;
    KF.statePost.at<float>(1, 0) = y; 

    KF.transitionMatrix = (Mat_<float>(4, 4) << 1,0,dt,0,   0,1,0,dt,  0,0,1,0,  0,0,0,1);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(.5)); //adjust this for faster convergence - but higher noise
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(100));
}
//Pretdiction function
Point kalmanPredict() 
{

  Mat prediction = KF.predict();
    Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

    KF.statePre.copyTo(KF.statePost);
    KF.errorCovPre.copyTo(KF.errorCovPost);

    return predictPt;
}

//Correction Function

Point kalmanCorrect(float x, float y)
{
    measurement(0) = x;
    measurement(1) = y;
    Mat estimated = KF.correct(measurement);
    Point statePt(estimated.at<float>(0),estimated.at<float>(1));
    return statePt;
}
void kfilter(const std_msgs::Int64MultiArray::ConstPtr& num) {

  

  std_msgs::Int64MultiArray s;


  
    p = kalmanPredict();
    S = kalmanCorrect(num->data[0],num->data[1]);

      Point measPt(measurement(0), measurement(1));
       cout<<"error matrix"<<KF.errorCovPost.at<float>(0)<<" "<<KF.errorCovPost.at<float>(1)<<endl;
      cout<<KF.processNoiseCov.at<float>(0);
      cout << "MEASUREMENT :  " << p << endl;

     
      
     cout<<"ESTIMATED :  "<<S<<endl;

    if(abs((S.x-p.x)*(S.y-p.y)) < 100){
      flag++;
     
      cout << flag << endl ;
    }
    else{
      flag = 0;
    }
    if (flag==50)   
    {
      s.data.push_back(S.x);
      s.data.push_back(S.y);
      s.data.push_back(num->data[2]);
      pub_n.publish(s);
     
      ros::shutdown();

    }

  ROS_INFO("Values sent to topic-print\n");


 return ;
}


//------------------------------------------------ main

int main (int argc, char** argv) 
{
    

    initKalman(0, 0);
    ros::init(argc, argv, "measurement");
    ros::NodeHandle nh;

    pub_n = nh.advertise<std_msgs::Int64MultiArray>("measurement", 1);

    ros::Rate loop_rate(10);

    ros::Subscriber sub_n = nh.subscribe("state", 1, kfilter);

    ros::spin();
 
    return 0;
}