#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"

const std::string OPENCV_WINDOW = "EYE_SEE";
const std::string face_cascade_name = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";

class face_position_publisher {
public:
    face_position_publisher ()
        :it_(nh_) {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera_node/image_raw", 1,
                                   &face_position_publisher::face_position_callback, this);
        image_pub_ = it_.advertise("/face_recognized", 1);

        if(!face_cascade_.load(face_cascade_name)) {
            std::cerr << "[Error]: could not load cascaded file" << std::endl;
        }
        continuous_time_pub_ = nh_.advertise<std_msgs::Float32>("/continuous_time", 1);

        continuous_time.data = 0.0;
        pre_time = ros::Time::now();

        cv::namedWindow(OPENCV_WINDOW);
    };
    ~face_position_publisher() {
        cv::destroyWindow(OPENCV_WINDOW);
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
        cvtColor(cv_ptr->image, frame_gray_, CV_BGR2GRAY);
        equalizeHist(frame_gray_, frame_gray_);

        face_cascade_.detectMultiScale(frame_gray_, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(40, 40));
        for(int i=0;i<faces.size();++i) {
            cv::rectangle(cv_ptr->image,
                          cvPoint(faces[i].x, faces[i].y),
                          cvPoint(faces[i].x+faces[i].width, faces[i].y+faces[i].height),
                          cv::Scalar(0, 0, 255));
        }
        if(faces.size() == 0) {
            continuous_time.data = 0.0;
        } else {
            continuous_time.data += (ros::Time::now() - pre_time).toSec();
        }
        continuous_time_pub_.publish(continuous_time);
        // ROS_INFO("%lf", continuous_time.data);
        pre_time = ros::Time::now();


        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        // cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    geometry_msgs::Point face_position_pub_;
    cv::Mat frame_gray_;
    cv::CascadeClassifier face_cascade_;
    std_msgs::Float32 continuous_time;
    ros::Publisher continuous_time_pub_;
    ros::Time pre_time;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "face_position_publisher");
    face_position_publisher fp;
    ros::spin();
    return 0;
}
