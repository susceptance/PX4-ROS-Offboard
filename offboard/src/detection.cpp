#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <detection_msgs/BoundingBoxes.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <librealsense2/rs.hpp>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "offboard/coordinate.h"
#include "offboard/coordinate_array.h"


class ImageConverter
{
    ros::NodeHandle nh;
    cv_bridge::CvImagePtr cv_ptr_prev;
    image_transport::ImageTransport it;
    //image_transport::Subscriber depth_sub;
    image_transport::CameraSubscriber depth_sub;
    detection_msgs::BoundingBoxes boxes;
    ros::Publisher coordinates_pub, coordinate_pub;
    ros::Subscriber box_sub;

    rs2_intrinsics intrinsic;
    rs2_extrinsics extrinsic;

    offboard::coordinate comsg;
    offboard::coordinate_array comsgs;

    float distance = 7, prev_dist, obj_pr;
    int center_x, center_y;
    bool prev_dist_flag = true, detection_flag = false;
    std::string obj_name = "No detected";


public:
    ImageConverter(): it(nh) // it = nh 
    {
        //depth_sub = it.subscribe("/camera/depth/image_raw", 1, &ImageConverter::depth_cb, this);
        //cam_sub = it.subscribeCamera("/camera/depth/image_raw", 1, boost::bind(&ImageConverter::depth_cb, this, _1, _2 ,cv_ptr_prev));
        depth_sub = it.subscribeCamera("/camera/aligned_depth_to_color/image_raw", 1024, &ImageConverter::depth_cb, this);
        box_sub = nh.subscribe("/yolov5/detections", 256, &ImageConverter::BoundingBoxes_cb, this);
        coordinates_pub = nh.advertise<offboard::coordinate_array>("coordinates", 1);
        coordinate_pub = nh.advertise<offboard::coordinate>("coordinate", 1);
    }

    ~ImageConverter()
    {
        //std::cout << "ImageConverter class was destructed" << std::endl;
    }


    void BoundingBoxes_cb(const detection_msgs::BoundingBoxes::ConstPtr& msg)
    {   
        boxes = *msg;
    }

    //void depth_cb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg, cv_bridge::CvImagePtr cv_ptr_prev)
    void depth_cb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        if(boxes.bounding_boxes.size() == 0)
        {
            detection_flag = false;
            comsg.detection_flag = detection_flag;
            coordinate_pub.publish(comsg);
            //std::cout << "No detected" << std::endl;
            return;
        }

		cv::Mat image;
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
			image = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //    [fx  0 cx]
        //K = [ 0 fy cy]
        //    [ 0  0  1]
        intrinsic.width = info_msg->width;
        intrinsic.height = info_msg->height;
        intrinsic.ppx = info_msg->K[2];
        intrinsic.ppy = info_msg->K[5];
        intrinsic.fx = info_msg->K[0];
        intrinsic.fy = info_msg->K[4];
        intrinsic.model = RS2_DISTORTION_NONE;
        for (int i = 0; i < info_msg->D.size(); i++)
        {
            intrinsic.coeffs[i] = info_msg->D[i];
        }

        //std::cout << boxes.bounding_boxes.size() << std::endl;

        for(int i = 0; i < boxes.bounding_boxes.size(); i++)
        {
            if (boxes.bounding_boxes[i].probability > 0.5)
            {
                center_x = (boxes.bounding_boxes[i].xmax + boxes.bounding_boxes[i].xmin) / 2;
                center_y = (boxes.bounding_boxes[i].ymax + boxes.bounding_boxes[i].ymin) / 2;
                obj_name = boxes.bounding_boxes[i].Class;
                //obj_pr = boxes.bounding_boxes[i].probability;

                //if(prev_dist_flag) { prev_dist = distance; }  
                distance = 0.001*image.at<short int>(cv::Point(center_x, center_y));
                //this->distance = 0.001*cv_ptr->image.at<u_int16_t>(this->center_x, this->center_y);
                //std::cout << "Distance : " << distance << std::endl;

                /*if(distance == 0 || distance > prev_dist + 3)
                {
                    prev_dist_flag = false;
                    detection_flag = false;
                    comsg.detection_flag = detection_flag;
                    coordinate_pub.publish(comsg);
                    //std::cout << "The object has disappeared." << std::endl;
                    return;
                }   
                else if(distance < prev_dist + 3)
                    prev_dist_flag = true;*/

                float cam_point[3];
                float pixel[2] = { (float)center_x, (float)center_y };

                rs2_deproject_pixel_to_point(cam_point, &intrinsic, pixel, distance);
                std::cout << i << ".BoundingBox - " << "x = " << cam_point[2] << ", y = " << -cam_point[0] << ", z = " << -cam_point[1] << std::endl;

            
                comsg.cam_x = cam_point[2];
                comsg.cam_y = -cam_point[0];
                comsg.cam_z = -cam_point[1];
                comsg.bbox_class = obj_name;

                comsgs.coordinates.push_back(comsg);
            }
        }

        detection_flag = true;
        comsg.detection_flag = detection_flag;
        coordinate_pub.publish(comsg);

        coordinates_pub.publish(comsgs);
        comsgs.coordinates.clear();
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObjectCoordinate");
    ImageConverter ic;
    ros::spin();
    return 0;
}








