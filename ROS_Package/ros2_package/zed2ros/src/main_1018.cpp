// ///////////////////////////////////////////////////////////////////////////
// //
// // Copyright (c) 2020, STEREOLABS.
// //
// // All rights reserved.
// //
// // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// // "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// // LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// // A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// // OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// // SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// // LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// // DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// // THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// // (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// // OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// //
// ///////////////////////////////////////////////////////////////////////////

 // Standard includes
#include <stdio.h>
#include <string.h>
// #include <iostream>
// #include <fstream>

#include <pthread.h>

#include<time.h>

#include <sl/Camera.hpp>

#include "rclcpp/rclcpp.hpp"
// OpenCV include (for display)
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

// Using std and sl namespaces
using namespace std;
using namespace sl;



using std::placeholders::_1;
// init global parameter
sl::Camera zed;
sl::InitParameters init_parameters;
sl::ObjectDetectionParameters detection_parameters;
sl::PositionalTrackingParameters positional_tracking_parameters;
sl::RuntimeParameters runtime_parameters;
sl::Objects objects;
sl::ObjectDetectionRuntimeParameters detection_parameters_rt;
sl::Pose camera_path;
sl::POSITIONAL_TRACKING_STATE tracking_state;
sl::SensorsData sensors_data;

cv::Mat cvImg;
sl::Mat image;
sl::Mat pointCloud;
char key = ' ';
cv::Point center;
cv::Mat cvMap;
cv::Mat cvFloatMap;
int flagMsg = 101;
clock_t startTime,startTime1,endTime;

bool frozenFrame = false;
bool trackTarget = false;


class flagLoop : public rclcpp::Node
{
public:
    flagLoop()
    : Node("flagLoop"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/yf_camera/flag", 1);
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
          "/yf_camera/flag", 1, std::bind(&flagLoop::topic_callback, this, _1));
        timer_ = this->create_wall_timer(
        10ms, std::bind(&flagLoop::timer_callback, this));
            
        startTime1 = clock();
        startTime = clock();
    }
  
    int idx = 0;

private:
    void timer_callback()
    {        
        auto message = std_msgs::msg::Int32();
        message.data = flagMsg;
        publisher_->publish(message);
        
        
        // cout << "flag pub FPS : " <<(double) CLOCKS_PER_SEC/(clock() - startTime1)  << " " << endl;        
        // startTime1 = clock();
    }
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
    {
        flagMsg = msg->data;
        
        // cout << "flag sub FPS : " <<(double) CLOCKS_PER_SEC/(clock() - startTime)  << " " << endl;        
        // startTime = clock();
    }

    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    size_t count_;
};

class zedPublisherFake : public rclcpp::Node
{
public:
    zedPublisherFake()
    : Node("fakePublisher"), count_(0)
    {
        // publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/yf_camera/Pose", 1);
        // subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        //   "/yf_camera/flag", 1, std::bind(&PostPublisher::topic_callback, this, _1));
        timer_ = this->create_wall_timer(
        10ms, std::bind(&zedPublisherFake::timer_callback, this));
            
        startTime = clock();
    }
  
    int idx = 0;

private:
    void timer_callback()
    {
        if (zed.grab() == ERROR_CODE::SUCCESS){   

            tracking_state = zed.getPosition(camera_path, REFERENCE_FRAME::WORLD);
            zed.retrieveImage(image, VIEW::LEFT);                 
            zed.retrieveMeasure(pointCloud, MEASURE::XYZ,MEM::CPU);
            zed.retrieveObjects(objects, detection_parameters_rt);
            // cout << "zed FPS : " <<(double) CLOCKS_PER_SEC/(clock() - startTime)  << " " << endl;        
            // startTime = clock();
        }
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    size_t count_;
};

class PostPublisher : public rclcpp::Node
{
public:
  PostPublisher()
  : Node("PostPublisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/yf_camera/Pose", 1);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&PostPublisher::timer_callback, this));
         
    startTime = clock();
  }

private:
    void timer_callback()
    {
        // if (zed.grab() == ERROR_CODE::SUCCESS){
            auto message = geometry_msgs::msg::PoseStamped();
            
            if (tracking_state == POSITIONAL_TRACKING_STATE::OK) {
                    // Get rotation and translation and displays it
                    sl::float3 rotate = camera_path.getEulerAngles();
                    sl::float3 trans = camera_path.getTranslation();

                    message.pose.orientation.x = rotate.x;
                    message.pose.orientation.y = rotate.y;
                    message.pose.orientation.z = rotate.z;
                    message.pose.position.x = trans.x;
                    message.pose.position.y = trans.y;
                    message.pose.position.z = trans.z;

            }
            publisher_->publish(message);
            // cout << "pose FPS : " <<(double) CLOCKS_PER_SEC/(clock() - startTime)  << " " << endl;        
            // startTime = clock();
        // }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    size_t count_;
};

class LabelImage : public rclcpp::Node
{
public:
    LabelImage()
    : Node("LiveVideo"), count_(0)
    {
        publisherImg_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/yf_camera/LiveVideo/compressed", 1);
        publisherGoal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/yf_camera/goal", 1);
        timer_ = this->create_wall_timer(
        10ms, std::bind(&LabelImage::timer_callback, this));
        
    }    
    
    bool init = false;
    bool findTarget = false;
    clock_t startTime = clock();
    sl::ObjectData target;

private:
    cv_bridge::CvImage img_bridge;
    void timer_callback()
    {
        
        if (zed.grab() == ERROR_CODE::SUCCESS){  
            // zed.retrieveObjects(objects, detection_parameters_rt);
            cvImg = cv::Mat((int) image.getHeight(), (int) image.getWidth(), CV_8UC4, image.getPtr<sl::uchar1>(sl::MEM::CPU)); 
            
            
            auto message2 = geometry_msgs::msg::PoseStamped();
            if (objects.is_new) {
                int objects_Num = objects.object_list.size();
                if (!objects.object_list.empty()) {
                    if (init){
                        target = objects.object_list.front();
                        cv::Rect bbox_target = cv::Rect(target.bounding_box_2d[0][0], // x
                                    target.bounding_box_2d[0][1],// y
                                    target.bounding_box_2d[2][0]-target.bounding_box_2d[0][0], // w
                                    target.bounding_box_2d[2][1]-target.bounding_box_2d[0][1]); // h
                        cv::Point center;
                        center.x = (int) (target.bounding_box_2d[2][0]+target.bounding_box_2d[0][0])>>1;
                        center.y = (int) (target.bounding_box_2d[2][1]+target.bounding_box_2d[0][1])>>1;
                        cv::rectangle(cvImg,bbox_target, cv::Scalar(0, 0, 255), 3, 3,0);
                        cv::circle(cvImg,  center , 2, cv::Scalar(0, 0, 255), 3, 8, 0);
                        message2.pose.position.x = target.position.x;
                        message2.pose.position.y = target.position.y;
                        message2.pose.position.z = target.position.z;
                        publisherGoal_->publish(message2);  
                        init = false;
                    }else{
                        findTarget = false;
                        for(int i = 0; i < objects_Num; ++i){
                        
                            auto object = objects.object_list[i];
                            if (target.id == object.id){
                                target = object;
                                findTarget = true;
                            }
                        }
                        if (!findTarget){
                            float dist = 0.5;                     
                            auto object_cache = objects.object_list.front();
                            for(int i = 0; i < objects_Num; ++i){
                                auto object = objects.object_list[i];
                                float curr_dist = pow(pow(object.position.x-target.position.x,2 )
                                                    + pow(object.position.y-target.position.y,2)
                                                    + pow(object.position.z-target.position.z,2),0.5);
                                if (dist>curr_dist){
                                    dist = curr_dist;
                                    object_cache = object;
                                }         
                            }
                            target = object_cache;
                        }
                        
                        cv::Rect bbox_target = cv::Rect(target.bounding_box_2d[0][0], // x
                                    target.bounding_box_2d[0][1],// y
                                    target.bounding_box_2d[2][0]-target.bounding_box_2d[0][0], // w
                                    target.bounding_box_2d[2][1]-target.bounding_box_2d[0][1]); // h
                        cv::Point center;
                        center.x = (int) (target.bounding_box_2d[2][0]+target.bounding_box_2d[0][0])>>1;
                        center.y = (int) (target.bounding_box_2d[2][1]+target.bounding_box_2d[0][1])>>1;
                        cv::rectangle(cvImg,bbox_target, cv::Scalar(0, 0, 255), 3, 3,0);
                        cv::circle(cvImg,  center , 2, cv::Scalar(0, 0, 255), 3, 8, 0);
                        message2.pose.position.x = target.position.x;
                        message2.pose.position.y = target.position.y;
                        message2.pose.position.z = target.position.z;
                        publisherGoal_->publish(message2);  

                    }
                }
            }


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // // if (zed.grab() == ERROR_CODE::SUCCESS){
        //     auto message2 = geometry_msgs::msg::PoseStamped();
        //     // zed.retrieveImage(image, VIEW::LEFT);     
        //     // Convert sl::Mat to cv::Mat (share buffer)
        //     cvImg = cv::Mat((int) image.getHeight(), (int) image.getWidth(), CV_8UC4, image.getPtr<sl::uchar1>(sl::MEM::CPU)); 
        //     // Get the object array
        //     // zed.retrieveObjects(objects, detection_parameters_rt); 
        //     if (objects.is_new) {
        //         bool findTarget = false;
        //         // fake tracking
        //         if (!objects.object_list.empty()) {
                    

        //             // func 1
        //             int objects_Num = objects.object_list.size();
        //             auto target = objects.object_list.front();
        //             if (init){
        //                 for(int i = 0; i < objects_Num; ++i){
        //                     auto object = objects.object_list[i];
        //                     cv::Rect bbox_iter = cv::Rect(object.bounding_box_2d[0][0], // x
        //                             object.bounding_box_2d[0][1],// y
        //                             object.bounding_box_2d[2][0]-object.bounding_box_2d[0][0], // w
        //                             object.bounding_box_2d[2][1]-object.bounding_box_2d[0][1]); // h
        //                     center.x = (int) (object.bounding_box_2d[2][0]+object.bounding_box_2d[0][0])>>1;
        //                     center.y = (int) (object.bounding_box_2d[2][1]+object.bounding_box_2d[0][1])>>1;
        //                     cv::rectangle(cvImg,bbox_iter, cv::Scalar(255, 0, 255), 3, 3,0);
        //                     cv::putText(cvImg,  to_string(i+1) , center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 255), 2, 8, 0);

        //                 }      
        //                 auto target = objects.object_list.front();
        //                 cv::Rect bbox_target = cv::Rect(target.bounding_box_2d[0][0], // x
        //                             target.bounding_box_2d[0][1],// y
        //                             target.bounding_box_2d[2][0]-target.bounding_box_2d[0][0], // w
        //                             target.bounding_box_2d[2][1]-target.bounding_box_2d[0][1]); // h
        //                 cv::Point center;
        //                 center.x = (int) (target.bounding_box_2d[2][0]+target.bounding_box_2d[0][0])>>1;
        //                 center.y = (int) (target.bounding_box_2d[2][1]+target.bounding_box_2d[0][1])>>1;
        //                 cv::rectangle(cvImg,bbox_target, cv::Scalar(0, 0, 255), 3, 3,0);
        //                 cv::circle(cvImg,  center , 2, cv::Scalar(0, 0, 255), 3, 8, 0);
        //                 // goal.pubMsg(target.position.x,target.position.y,target.position.z); 
        //                 message2.pose.position.x = target.position.x;
        //                 message2.pose.position.y = target.position.y;
        //                 message2.pose.position.z = target.position.z;
        //                 publisherGoal_->publish(message2);  
        //                 init = false;
        //             }
        //             // func 2
        //             for(int i = 0; i < objects_Num; ++i){
        //                 auto object = objects.object_list[i];
        //                 if (target.id == object.id){                            
        //                     target = object;
        //                     findTarget = true;
        //                 }
        //                 cv::Rect bbox_iter = cv::Rect(object.bounding_box_2d[0][0], // x
        //                         object.bounding_box_2d[0][1],// y
        //                         object.bounding_box_2d[2][0]-object.bounding_box_2d[0][0], // w
        //                         object.bounding_box_2d[2][1]-object.bounding_box_2d[0][1]); // h
        //                 center.x = (int) (object.bounding_box_2d[2][0]+object.bounding_box_2d[0][0])>>1;
        //                 center.y = (int) (object.bounding_box_2d[2][1]+object.bounding_box_2d[0][1])>>1;
        //                 cv::rectangle(cvImg,bbox_iter, cv::Scalar(255, 0, 255), 3, 3,0);
        //                 cv::putText(cvImg,  to_string(i+1) , center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 255), 2, 8, 0);
                        
        //                 // if (detection_parameters.enable_tracking)
        //                 //     cout << object.tracking_state ;
        //             }

        //             if (!findTarget){
        //                 float dist = 2.0;
        //                 for(int i = 0; i < objects_Num; ++i){
        //                     auto object = objects.object_list[i];
        //                     float curr_dist =     object.position.x*object.position.x 
        //                                 + object.position.y*object.position.y
        //                                 + object.position.z*object.position.z;
        //                     if (dist>curr_dist){
        //                         target = object;
        //                         // new target flag
        //                         init = true;
        //                     }                                
        //                 }
        //             }
        //             cv::Rect bbox_target = cv::Rect(target.bounding_box_2d[0][0], // x
        //                         target.bounding_box_2d[0][1],// y
        //                         target.bounding_box_2d[2][0]-target.bounding_box_2d[0][0], // w
        //                         target.bounding_box_2d[2][1]-target.bounding_box_2d[0][1]); // h
        //             cv::Point center;
        //             center.x = (int) (target.bounding_box_2d[2][0]+target.bounding_box_2d[0][0])>>1;
        //             center.y = (int) (target.bounding_box_2d[2][1]+target.bounding_box_2d[0][1])>>1;
        //             cv::rectangle(cvImg,bbox_target, cv::Scalar(0, 0, 255), 3, 3,0);
        //             cv::circle(cvImg,  center , 2, cv::Scalar(0, 0, 255), 3, 8, 0);
        //             // goal.pubMsg(target.position.x,target.position.y,target.position.z);                 
        //             message2.pose.position.x = target.position.x;
        //             message2.pose.position.y = target.position.y;
        //             message2.pose.position.z = target.position.z;
        //             publisherGoal_->publish(message2);  
        //         }

        //     //     if (!objects.object_list.empty()) {
        //     //         // func 1
        //     //         for(int i = 0; i < objects_Num; ++i){
        //     //             auto object = objects.object_list[i];
        //     //             cv::Rect bbox_iter = cv::Rect(object.bounding_box_2d[0][0], // x
        //     //                     object.bounding_box_2d[0][1],// y
        //     //                     object.bounding_box_2d[2][0]-object.bounding_box_2d[0][0], // w
        //     //                     object.bounding_box_2d[2][1]-object.bounding_box_2d[0][1]); // h
        //     //             center.x = (int) (object.bounding_box_2d[2][0]+object.bounding_box_2d[0][0])>>1;
        //     //             center.y = (int) (object.bounding_box_2d[2][1]+object.bounding_box_2d[0][1])>>1;
        //     //             cv::rectangle(cvImg,bbox_iter, cv::Scalar(255, 0, 255), 3, 3,0);
        //     //             cv::putText(cvImg,  to_string(i+1) , center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 255), 2, 8, 0);

        //     //         }      
        //     //         // auto target = objects.object_list.front();
        //     //         // cv::Rect bbox_target = cv::Rect(target.bounding_box_2d[0][0], // x
        //     //         //             target.bounding_box_2d[0][1],// y
        //     //         //             target.bounding_box_2d[2][0]-target.bounding_box_2d[0][0], // w
        //     //         //             target.bounding_box_2d[2][1]-target.bounding_box_2d[0][1]); // h
        //     //         // cv::Point center;
        //     //         // center.x = (int) (target.bounding_box_2d[2][0]+target.bounding_box_2d[0][0])>>1;
        //     //         // center.y = (int) (target.bounding_box_2d[2][1]+target.bounding_box_2d[0][1])>>1;
        //     //         // cv::rectangle(cvImg,bbox_target, cv::Scalar(0, 0, 255), 3, 3,0);
        //     //         // cv::circle(cvImg,  center , 2, cv::Scalar(0, 0, 255), 3, 8, 0);
        //     //         // // goal.pubMsg(target.position.x,target.position.y,target.position.z); 
        //     //         // message2.pose.position.x = target.position.x;
        //     //         // message2.pose.position.y = target.position.y;
        //     //         // message2.pose.position.z = target.position.z;
        //     //         // publisherGoal_->publish(message2);  
        //     //     }
        //     // }
        //     // if (flagMsg == 101){
        //     //     if (objects.is_new) {
        //     //         int objects_Num = objects.object_list.size();
        //     //         if (!objects.object_list.empty()) {
        //     //             // func 1
        //     //             for(int i = 0; i < objects_Num; ++i){
        //     //                 auto object = objects.object_list[i];

        //     //                 cv::Rect bbox_iter = cv::Rect(object.bounding_box_2d[0][0], // x
        //     //                         object.bounding_box_2d[0][1],// y
        //     //                         object.bounding_box_2d[2][0]-object.bounding_box_2d[0][0], // w
        //     //                         object.bounding_box_2d[2][1]-object.bounding_box_2d[0][1]); // h
        //     //                 center.x = (int) (object.bounding_box_2d[2][0]+object.bounding_box_2d[0][0])>>1;
        //     //                 center.y = (int) (object.bounding_box_2d[2][1]+object.bounding_box_2d[0][1])>>1;
        //     //                 cv::rectangle(cvImg,bbox_iter, cv::Scalar(255, 0, 255), 3, 3,0);
        //     //                 cv::putText(cvImg,  to_string(i+1) , center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 255), 2, 8, 0);

        //     //             }        
        //     //         }
        //     //     }
        //     // } else if( flagMsg == 0 && !frozenFrame){
        //     //     int objects_Num = objects.object_list.size();
        //     //     if (objects_Num == 0){
        //     //         flagMsg = 101;
        //     //     }
        //     //     if (!objects.object_list.empty()) {
        //     //         // func 1
        //     //         for(int i = 0; i < objects_Num; ++i){
        //     //             auto object = objects.object_list[i];
        //     //             cv::Rect bbox_iter = cv::Rect(object.bounding_box_2d[0][0], // x
        //     //                     object.bounding_box_2d[0][1],// y
        //     //                     object.bounding_box_2d[2][0]-object.bounding_box_2d[0][0], // w
        //     //                     object.bounding_box_2d[2][1]-object.bounding_box_2d[0][1]); // h
        //     //             center.x = (int) (object.bounding_box_2d[2][0]+object.bounding_box_2d[0][0])>>1;
        //     //             center.y = (int) (object.bounding_box_2d[2][1]+object.bounding_box_2d[0][1])>>1;
        //     //             cv::rectangle(cvImg,bbox_iter, cv::Scalar(255, 0, 255), 3, 3,0);
        //     //             cv::putText(cvImg,  to_string(i+1) , center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 255), 2, 8, 0);
        //     //         }
                    
        //     //         flagMsg = -1*objects_Num;
        //     //         frozenFrame = true;
        //     //         trackTarget = false;
        //     //     }

        //     // } else if( (flagMsg>0) && (flagMsg>=100) && (!trackTarget) ){
        //     //     frozenFrame = false;            
        //     //     trackTarget = true;
        //     //     int objIdx = flagMsg - 1;
        //     //     if (objects.is_new) {
        //     //         int objects_Num = objects.object_list.size();
        //     //         if (!objects.object_list.empty()) {
        //     //             auto target = objects.object_list.front();
        //     //             // func 1
        //     //             for(int i = 0; i < objects_Num; ++i){
        //     //                 auto object = objects.object_list[i];
        //     //                 if (i == objIdx){                            
        //     //                     target = object;
        //     //                 }
        //     //                 cv::Rect bbox_iter = cv::Rect(object.bounding_box_2d[0][0], // x
        //     //                         object.bounding_box_2d[0][1],// y
        //     //                         object.bounding_box_2d[2][0]-object.bounding_box_2d[0][0], // w
        //     //                         object.bounding_box_2d[2][1]-object.bounding_box_2d[0][1]); // h
        //     //                 center.x = (int) (object.bounding_box_2d[2][0]+object.bounding_box_2d[0][0])>>1;
        //     //                 center.y = (int) (object.bounding_box_2d[2][1]+object.bounding_box_2d[0][1])>>1;
        //     //                 cv::rectangle(cvImg,bbox_iter, cv::Scalar(255, 0, 255), 3, 3,0);
        //     //                 cv::putText(cvImg,  to_string(i+1) , center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 255), 2, 8, 0);
                            
        //     //                 // if (detection_parameters.enable_tracking)
        //     //                 //     cout << object.tracking_state ;
        //     //             }
        //     //             cv::Rect bbox_target = cv::Rect(target.bounding_box_2d[0][0], // x
        //     //                         target.bounding_box_2d[0][1],// y
        //     //                         target.bounding_box_2d[2][0]-target.bounding_box_2d[0][0], // w
        //     //                         target.bounding_box_2d[2][1]-target.bounding_box_2d[0][1]); // h
        //     //             cv::Point center;
        //     //             center.x = (int) (target.bounding_box_2d[2][0]+target.bounding_box_2d[0][0])>>1;
        //     //             center.y = (int) (target.bounding_box_2d[2][1]+target.bounding_box_2d[0][1])>>1;
        //     //             cv::rectangle(cvImg,bbox_target, cv::Scalar(0, 0, 255), 3, 3,0);
        //     //             cv::circle(cvImg,  center , 2, cv::Scalar(0, 0, 255), 3, 8, 0);
        //     //             // goal.pubMsg(target.position.x,target.position.y,target.position.z);                         
        //     //             message2.pose.position.x = target.position.x;
        //     //             message2.pose.position.y = target.position.y;
        //     //             message2.pose.position.z = target.position.z;
        //     //             publisherGoal_->publish(message2);  
        //     //         }
        //     //     }
        //     // } else if((flagMsg>0) && (flagMsg>=100) && (trackTarget)){
        //     //     if (objects.is_new) {
        //     //         int objects_Num = objects.object_list.size();
        //     //         if (!objects.object_list.empty()) {
        //     //             auto target = objects.object_list.front();
        //     //             bool findTarget = false;
        //     //             // func 1
        //     //             for(int i = 0; i < objects_Num; ++i){
        //     //                 auto object = objects.object_list[i];
        //     //                 if (target.id == object.id){                            
        //     //                     target = object;
        //     //                     findTarget = true;
        //     //                 }
        //     //                 cv::Rect bbox_iter = cv::Rect(object.bounding_box_2d[0][0], // x
        //     //                         object.bounding_box_2d[0][1],// y
        //     //                         object.bounding_box_2d[2][0]-object.bounding_box_2d[0][0], // w
        //     //                         object.bounding_box_2d[2][1]-object.bounding_box_2d[0][1]); // h
        //     //                 center.x = (int) (object.bounding_box_2d[2][0]+object.bounding_box_2d[0][0])>>1;
        //     //                 center.y = (int) (object.bounding_box_2d[2][1]+object.bounding_box_2d[0][1])>>1;
        //     //                 cv::rectangle(cvImg,bbox_iter, cv::Scalar(255, 0, 255), 3, 3,0);
        //     //                 cv::putText(cvImg,  to_string(i+1) , center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 255), 2, 8, 0);
                            
        //     //                 // if (detection_parameters.enable_tracking)
        //     //                 //     cout << object.tracking_state ;
        //     //             }
        //     //             if (!findTarget){
        //     //                 float dist = 10000.0;
        //     //                 for(int i = 0; i < objects_Num; ++i){
        //     //                     auto object = objects.object_list[i];
        //     //                     float curr_dist =     object.position.x*object.position.x 
        //     //                              + object.position.y*object.position.y
        //     //                              + object.position.z*object.position.z;
        //     //                     if (dist>curr_dist){
        //     //                         target = object;
        //     //                     }                                
        //     //                 }
        //     //             }
        //     //             cv::Rect bbox_target = cv::Rect(target.bounding_box_2d[0][0], // x
        //     //                         target.bounding_box_2d[0][1],// y
        //     //                         target.bounding_box_2d[2][0]-target.bounding_box_2d[0][0], // w
        //     //                         target.bounding_box_2d[2][1]-target.bounding_box_2d[0][1]); // h
        //     //             cv::Point center;
        //     //             center.x = (int) (target.bounding_box_2d[2][0]+target.bounding_box_2d[0][0])>>1;
        //     //             center.y = (int) (target.bounding_box_2d[2][1]+target.bounding_box_2d[0][1])>>1;
        //     //             cv::rectangle(cvImg,bbox_target, cv::Scalar(0, 0, 255), 3, 3,0);
        //     //             cv::circle(cvImg,  center , 2, cv::Scalar(0, 0, 255), 3, 8, 0);
        //     //             // goal.pubMsg(target.position.x,target.position.y,target.position.z);                 
        //     //             message2.pose.position.x = target.position.x;
        //     //             message2.pose.position.y = target.position.y;
        //     //             message2.pose.position.z = target.position.z;
        //     //             publisherGoal_->publish(message2);  
        //     //         }
        //     //     }



            // }

            cv::Mat cvShowMap;
            cv::resize(cvImg, cvShowMap, cv::Size(192, 144), 0, 0, cv::INTER_AREA);
            auto message = sensor_msgs::msg::CompressedImage ();
            std_msgs::msg::Header header; // empty header
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, cvShowMap);
            img_bridge.toCompressedImageMsg(message); // from cv_bridge to sensor_msgs::Image
            publisherImg_->publish(message);
            
            cout << "video FPS : " <<(double) CLOCKS_PER_SEC/(clock() - startTime)  << " " << endl;                
            startTime = clock();   
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage >::SharedPtr publisherImg_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisherGoal_;
    size_t count_;
};

class MapPublisher : public rclcpp::Node
{
public:
    MapPublisher()
    : Node("MapPublisher"), count_(0)
    {
        // publisherCost_ = this->create_publisher<sensor_msgs::msg::Image>("/yf_camera/costmap", 1);
        // publisherOb_ = this->create_publisher<sensor_msgs::msg::Image>("/yf_camera/obmap", 1);
        publisherArray_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/yf_camera/obList", 1);
        timer_ = this->create_wall_timer(
        10ms, std::bind(&MapPublisher::timer_callback, this));
          
        startTime = clock();
    }

private:
    cv_bridge::CvImage img_bridge;
    cv_bridge::CvImage img_bridge2;
    void timer_callback()
    {        
        // if (zed.grab() == ERROR_CODE::SUCCESS){
            auto oblist = std_msgs::msg::Float32MultiArray();
            // zed.retrieveMeasure(pointCloud, MEASURE::XYZ,MEM::CPU);
            cv::Mat cvPointCloud = cv::Mat((int) pointCloud.getHeight(), (int) pointCloud.getWidth(), CV_32FC4, pointCloud.getPtr<sl::uchar1>(sl::MEM::CPU));
            
            sl::float4 point3D;
            // Get the 3D point cloud values for pixel (i,j)
            cvMap = cv::Mat::zeros(cv::Size(500,500),CV_8UC4);
            cvFloatMap = cv::Mat::zeros(cv::Size(50,50),CV_8UC4);
           
            vector<vector<float>> array(2);
            
            // vector<signed char> arrayZ;
            for(int m = 0; m<(int) pointCloud.getHeight(); ++m){
                for(int n = 0; n<(int) pointCloud.getWidth(); ++n){                
                    sl::float4 originPoint3D;                
                    sl::float4 mapPoint3D;
                    pointCloud.getValue( n,m, &originPoint3D);
                    // x l&r y u&d z b&f
                    if (point3D.x==NULL || point3D.y==NULL || point3D.z==NULL){
                        continue;
                    }
                    mapPoint3D.x = (int) 100 * (originPoint3D.x + 2.5);
                    mapPoint3D.z = (int) 100 * (originPoint3D.z);
                    mapPoint3D.y = (int) 100 * (originPoint3D.y + 0.25);
                    if ((mapPoint3D.x < 500) && (mapPoint3D.x > 0)){
                        if ((mapPoint3D.z<500) && (mapPoint3D.z>0)){
                            // TODO: 计算y方向高度 
                            if ((mapPoint3D.y<50) && (mapPoint3D.y>15)){ //|| (point3D.y<-5)
                                
                                int x = (int) mapPoint3D.x;
                                int z = (int) mapPoint3D.z;                           
                                cvMap.at<cv::Vec4b>(500-z,x) = { 255,  255 , 255 } ;


                                int x50 = originPoint3D.x*10;
                                int z50 = originPoint3D.z*10;
                                vector<float> point;
                                point.push_back(x50);
                                point.push_back(z50);
                                array.push_back(point);
                                                     
                            }

                        }
                    }
                }
            }
            
            cv::Mat cvCostMap;
            cv::resize(cvMap, cvCostMap, cv::Size(50, 50), 0, 0, cv::INTER_AREA);

            vector<float> sendArray;
            std::sort(array.begin(),array.end());
            cout<<"origin"<<array.size()<<endl;
            array.erase(unique(array.begin(),array.end()),array.end());
            cout<<"remove"<<array.size()<<endl;
            for (auto points: array){
                for (auto i: points){
                    sendArray.push_back(i/10);
                }
            }
            
            cout<<"send"<<sendArray.size()<<endl;
             oblist.data = sendArray;
             publisherArray_->publish(oblist);
    
            // auto message = sensor_msgs::msg::Image();
            // std_msgs::msg::Header header; // empty header
            // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, cvCostMap);
            // img_bridge.toImageMsg(message); // from cv_bridge to sensor_msgs::Image
            // publisherCost_->publish(message);
    
            // auto message2 = sensor_msgs::msg::Image();
            // // std_msgs::msg::Header header; // empty header
            // img_bridge2 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, cvCostMap);
            // img_bridge2.toImageMsg(message2); // from cv_bridge to sensor_msgs::Image
            // publisherOb_->publish(message2);

            // cv::imshow("live video", cvCostMap);
            
            // cout << "map FPS : " <<(double) CLOCKS_PER_SEC/(clock() - startTime)  << " " << endl;        
            // startTime = clock();
        // }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisherCost_;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisherOb_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisherArray_;
    size_t count_;
};

int main(int argc, char **argv) {
    // YAML::Node config = YAML::LoadFile("/home/yf/yifan/config.yaml");
    // cout<<config["ZED2"].as<bool>()<<endl;

    //  init rclcpp
    rclcpp::init(argc, argv);
    // Create a ZED camera object
    init_parameters.camera_resolution = RESOLUTION::VGA;
    init_parameters.camera_fps = 100;
    init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.sdk_verbose = true;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::LEFT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed 
    init_parameters.depth_minimum_distance = 0.2;
    init_parameters.depth_maximum_distance = 5.0;

    
    // Open the camera
    cout << "ZED2: Opening..." << endl;  
    ERROR_CODE err = zed.open(init_parameters);
    if (err != ERROR_CODE::SUCCESS) {
        cout << "Error " << err << ", exit program." << endl;
        return EXIT_FAILURE;
    }

    cout << "Positional Tracking: Loading Module..." << endl;  
    // positional_tracking_parameters.enable_area_memory = true;  
    auto returned_state = zed.enablePositionalTracking(positional_tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        cout<<"Enabling positionnal tracking failed: "<< returned_state<<endl;
        zed.close();
        return EXIT_FAILURE;
    }

    // Define the Objects detection module parameters
    // run detection for every Camera grab
    detection_parameters.image_sync = true;
    // track detects object accross time and space
    detection_parameters.enable_tracking = true;    
    // compute a binary mask for each object aligned on the left image
    detection_parameters.enable_mask_output = false;
    

    cout << "Object Detection: Loading Module..." << endl;
    err = zed.enableObjectDetection(detection_parameters);
    if (err != ERROR_CODE::SUCCESS) {
        zed.close();
        return EXIT_FAILURE;
    }
    

    // Set runtime parameters after opening the camera
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD; // Use STANDARD sensing mode
    runtime_parameters.confidence_threshold = 30;


    // detection runtime parameters
    // detection_parameters_rt.object_class_filter.clear();
    detection_parameters_rt.object_class_filter.push_back(sl::OBJECT_CLASS::PERSON);
    // detection output
    cout << setprecision(3);


    // make multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    // make nodehandle
    auto nh_zed = std::make_shared<zedPublisherFake>();
    auto nh_flag = std::make_shared<flagLoop>();
    
    auto nh_video = std::make_shared<LabelImage>();
    auto nh_maps = std::make_shared<MapPublisher>();
    auto nh_pose = std::make_shared<PostPublisher>();
    // add to the executor
    executor.add_node(nh_zed);
    executor.add_node(nh_flag);
    executor.add_node(nh_video);
    executor.add_node(nh_maps);
    executor.add_node(nh_pose);

    // wait for shutdown and then clean up
    cout << "Ros2: Start Spin..." << endl;
    executor.spin();

    //Display the image
    // cv::imshow("showMap", cvMap);
    // cv::imshow("live video", cvImg);
    // cv::imshow("sendMap", cvCostMap);
    // key = cv::waitKey(1);


    // Close the camera
    zed.disablePositionalTracking();
    zed.close();    
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
