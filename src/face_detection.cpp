#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

#include "boost/format.hpp"

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/opencv.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace dlib;
using namespace std;

class face_position_publisher {
public:
    face_position_publisher ()
        :it_(nh_),
         save_counter(0) {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera_node/image_raw", 1,
                                   &face_position_publisher::face_position_callback, this);
        image_pub_ = it_.advertise("/face_recognized", 1);

        continuous_time_pub_ = nh_.advertise<std_msgs::Float32>("/continuous_time", 1);
        save_sub_  = nh_.subscribe("/save_trigger", 1, &face_position_publisher::save_callback_, this);

        continuous_time.data = 0.0;
        pre_time = ros::Time::now();

        detector = get_frontal_face_detector();
    };
    ~face_position_publisher() {
    };

    void face_position_callback(const sensor_msgs::ImageConstPtr& msg) {

        cv_bridge::CvImagePtr cv_ptr;
        std::vector<cv::Rect> faces;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        pub_image_ = cv_ptr->image.clone();
        cvtColor(cv_ptr->image, frame_gray_, CV_BGR2GRAY);
        equalizeHist(frame_gray_, frame_gray_);

        cv_image<bgr_pixel> org_img(cv_ptr->image);
        dlib::cv_image<uchar> img(frame_gray_);
        std::vector<rectangle> dets = detector(img);
        // cout << "Number of faces detected: " << dets.size() << endl;
        win.clear_overlay();
        win.set_image(org_img);
        win.add_overlay(dets, rgb_pixel(255,0,0));

        // for(int i=0;i<faces.size();++i) {
        //     cv::rectangle(cv_ptr->image,
        //                   cvPoint(faces[i].x, faces[i].y),
        //                   cvPoint(faces[i].x+faces[i].width, faces[i].y+faces[i].height),
        //                   cv::Scalar(0, 0, 255));
        // }
        if(dets.size() == 0) {
            continuous_time.data = 0.0;
        } else {
            continuous_time.data += (ros::Time::now() - pre_time).toSec();
        }
        continuous_time_pub_.publish(continuous_time);
        // ROS_INFO("%lf", continuous_time.data);
        pre_time = ros::Time::now();

        // Output modified video stream
        // image_pub_.publish(cv_ptr->toImageMsg());
    }
    void save_callback_(const std_msgs::Empty::ConstPtr& msg) {
        continuous_time.data = 0.0;
        pre_time = ros::Time::now();
        cv::imwrite(boost::str(boost::format("/tmp/face_%d.png") % save_counter++), pub_image_);
        if(save_counter > 1000) {
            save_counter = 0;
        }
    }
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber save_sub_;
    geometry_msgs::Point face_position_pub_;
    cv::Mat frame_gray_;
    cv::Mat pub_image_;
    cv::CascadeClassifier face_cascade_;
    std_msgs::Float32 continuous_time;
    ros::Publisher continuous_time_pub_;
    ros::Time pre_time;

    image_window win;
    frontal_face_detector detector;

    int save_counter;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "face_position_publisher");
    face_position_publisher fp;
    ros::spin();
}
