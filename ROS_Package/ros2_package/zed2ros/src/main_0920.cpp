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
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <std_msgs/msg/int8_multi_array.hpp>

// Using std and sl namespaces
using namespace std;
using namespace sl;


class ImagePub
{
public:
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image_<std::allocator<void> >, std::allocator<void> > > publisher;
    ImagePub(cv::String nodeName,cv::String topicName)
    {
        node = rclcpp::Node::make_shared(nodeName);
        publisher = node->create_publisher<sensor_msgs::msg::Image>(topicName, 1);
    }
    void pubMsg(cv::Mat income){
        img = income;
        doPub();
    }
    cv::Mat img;

private:
    cv_bridge::CvImage img_bridge;
    void doPub()
    {
        auto img_msg = sensor_msgs::msg::Image();
        std_msgs::msg::Header header; // empty header
        // RCLCPP_INFO(node->get_logger(), "Publishing");
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, this->img);
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        publisher->publish(img_msg);
    }
};

class GoalPub
{
public:
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped_<std::allocator<void> >, std::allocator<void> > > publisher;
    GoalPub(cv::String nodeName,cv::String topicName)
    {
        node = rclcpp::Node::make_shared(nodeName);
        publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(topicName, 1);
    }
    
    void pubMsg(float x_in,float y_in,float z_in){
        x = x_in;
        y = y_in;
        z = z_in;
        doPub();
    }
    float x;
    float y;
    float z;

private:
  void doPub()
  {
    auto message = geometry_msgs::msg::PoseStamped();
    message.pose.position.x = x;
    message.pose.position.y = y;
    message.pose.position.z = z;
    publisher->publish(message);
  }
};

class SignalPub
{
public:
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32_<std::allocator<void> >, std::allocator<void> > > publisher;
    SignalPub(cv::String nodeName,cv::String topicName)
    {
        node = rclcpp::Node::make_shared(nodeName);
        publisher = node->create_publisher<std_msgs::msg::Int32>(topicName, 1);
    }
    
    void pubMsg(int incomeData){
        data = incomeData;
        doPub();
    }
    int data;

private:
  void doPub()
  {
    auto message = std_msgs::msg::Int32();
    message.data = data;
    publisher->publish(message);
  }
};

int flag;
void signal_topic_callback(const std_msgs::msg::Int32::SharedPtr msg) 
    {
        // std::cout<<msg->data<<std::endl;
        flag = (int) msg->data;
    }

int main(int argc, char **argv) {
    // YAML::Node config = YAML::LoadFile("/home/yf/yifan/config.yaml");
    // cout<<config["ZED2"].as<bool>()<<endl;


    //  init rclcpp
    rclcpp::init(argc, argv);
    // init all node class: total 5, 4 pub, 1 sub->signal
    ImagePub video("LiveVideo","/yf_camera/LiveVideo");
    ImagePub showmap("ShowMap","/yf_camera/costmap");
    ImagePub obmap("CostMap","/yf_camera/obmap");
    GoalPub goal("goal","/move_base_simple/goal");
    SignalPub signal("flagPub","/yf_camera/flag");
    
    auto signal_node = rclcpp::Node::make_shared("flag");
    auto subscription =
      signal_node->create_subscription<std_msgs::msg::Int32>("/yf_camera/flag", 1, signal_topic_callback);
    // Create a ZED camera object
    sl::Camera zed;
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::VGA;
    init_parameters.camera_fps = 30;
    init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.sdk_verbose = true;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::LEFT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed    
    
    // Define the Objects detection module parameters
    ObjectDetectionParameters detection_parameters;
    // run detection for every Camera grab
    detection_parameters.image_sync = true;
    // track detects object accross time and space
    detection_parameters.enable_tracking = false;   
    PositionalTrackingParameters positional_tracking_parameters;
    // If you want to have object tracking you need to enable positional tracking first
    if (detection_parameters.enable_tracking)
        zed.enablePositionalTracking(positional_tracking_parameters);
  
    // compute a binary mask for each object aligned on the left image
    detection_parameters.enable_mask_output = false;
    
    // Open the camera
    ERROR_CODE err = zed.open(init_parameters);
    if (err != ERROR_CODE::SUCCESS) {
        cout << "Error " << err << ", exit program." << endl;
        return EXIT_FAILURE;
    }

    cout << "Object Detection: Loading Module..." << endl;
    err = zed.enableObjectDetection(detection_parameters);
    if (err != ERROR_CODE::SUCCESS) {
        zed.close();
        return EXIT_FAILURE;
    }

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD; // Use STANDARD sensing mode

    // detection runtime parameters
    ObjectDetectionRuntimeParameters detection_parameters_rt;
    // detection_parameters_rt.object_class_filter.clear();
    detection_parameters_rt.object_class_filter.push_back(sl::OBJECT_CLASS::PERSON);
    // detection output
    Objects objects;
    cout << setprecision(3);

    // cv::String win_name = "Zed2 Live Video";
    // cv::namedWindow(win_name);

    Mat image;
    Mat pointCloud;
    char key = ' ';
    cv::Point center;
    cv::Mat cvMap;
    clock_t startTime,endTime,timeVE,timeVS;

    bool frozenFrame = false;
    bool trackTarget = false;
    startTime = clock();
    __gnu_cxx::__alloc_traits<std::allocator<sl::ObjectData> >::value_type target;
    while ((zed.grab() == ERROR_CODE::SUCCESS) && key != 'q') {
        // spin every thing
        rclcpp::spin_some(video.node);
        rclcpp::spin_some(showmap.node);
        rclcpp::spin_some(goal.node);
        rclcpp::spin_some(obmap.node);
        rclcpp::spin_some(signal.node);
        rclcpp::spin_some(signal_node);
        // std::cout<<flag<<std::endl;
        // Retrieve colored point cloud. Point cloud is aligned on the left image.
        zed.retrieveMeasure(pointCloud, MEASURE::XYZ,MEM::CPU);
        // Get the object array
        zed.retrieveObjects(objects, detection_parameters_rt);
        // Get the left image
        zed.retrieveImage(image, VIEW::LEFT);        
        // Convert sl::Mat to cv::Mat (share buffer)
        cv::Mat cvImage = cv::Mat((int) image.getHeight(), (int) image.getWidth(), CV_8UC4, image.getPtr<sl::uchar1>(sl::MEM::CPU));
        cv::Mat cvPointCloud = cv::Mat((int) pointCloud.getHeight(), (int) pointCloud.getWidth(), CV_32FC4, pointCloud.getPtr<sl::uchar1>(sl::MEM::CPU));
        if (objects.is_new) {
            int objects_Num = objects.object_list.size();
            if (!objects.object_list.empty()) {
                // func 1
                // for(int i = 0; i < objects_Num; ++i){
                //     auto object = objects.object_list[i];
                //     cv::Rect bbox_iter = cv::Rect(object.bounding_box_2d[0][0], // x
                //             object.bounding_box_2d[0][1],// y
                //             object.bounding_box_2d[2][0]-object.bounding_box_2d[0][0], // w
                //             object.bounding_box_2d[2][1]-object.bounding_box_2d[0][1]); // h
                //     center.x = (int) (object.bounding_box_2d[2][0]+object.bounding_box_2d[0][0])>>1;
                //     center.y = (int) (object.bounding_box_2d[2][1]+object.bounding_box_2d[0][1])>>1;
                //     cv::rectangle(cvImage,bbox_iter, cv::Scalar(255, 0, 255), 3, 3,0);
                //     cv::putText(cvImage,  to_string(i+1) , center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 255), 2, 8, 0);

                // }      
                auto target = objects.object_list.front();
                cv::Rect bbox_target = cv::Rect(target.bounding_box_2d[0][0], // x
                            target.bounding_box_2d[0][1],// y
                            target.bounding_box_2d[2][0]-target.bounding_box_2d[0][0], // w
                            target.bounding_box_2d[2][1]-target.bounding_box_2d[0][1]); // h
                cv::Point center;
                center.x = (int) (target.bounding_box_2d[2][0]+target.bounding_box_2d[0][0])>>1;
                center.y = (int) (target.bounding_box_2d[2][1]+target.bounding_box_2d[0][1])>>1;
                cv::rectangle(cvImage,bbox_target, cv::Scalar(0, 0, 255), 3, 3,0);
                cv::circle(cvImage,  center , 2, cv::Scalar(0, 0, 255), 3, 8, 0);
                goal.pubMsg(target.position.x,target.position.y,target.position.z);   
            }
        }
        // if (flag == 101){
        //     if (objects.is_new) {
        //         int objects_Num = objects.object_list.size();
        //         if (!objects.object_list.empty()) {
        //             // func 1
        //             for(int i = 0; i < objects_Num; ++i){
        //                 auto object = objects.object_list[i];

        //                 cv::Rect bbox_iter = cv::Rect(object.bounding_box_2d[0][0], // x
        //                         object.bounding_box_2d[0][1],// y
        //                         object.bounding_box_2d[2][0]-object.bounding_box_2d[0][0], // w
        //                         object.bounding_box_2d[2][1]-object.bounding_box_2d[0][1]); // h
        //                 center.x = (int) (object.bounding_box_2d[2][0]+object.bounding_box_2d[0][0])>>1;
        //                 center.y = (int) (object.bounding_box_2d[2][1]+object.bounding_box_2d[0][1])>>1;
        //                 cv::rectangle(cvImage,bbox_iter, cv::Scalar(255, 0, 255), 3, 3,0);
        //                 cv::putText(cvImage,  to_string(i+1) , center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 255), 2, 8, 0);

        //             }        
        //         }
        //     }
        // } else if( flag == 0 && !frozenFrame){
        //     int objects_Num = objects.object_list.size();
        //     if (objects_Num == 0){
        //         signal.pubMsg(101);
        //         rclcpp::spin_some(signal.node);
        //     }
        //     if (!objects.object_list.empty()) {
        //         // func 1
        //         for(int i = 0; i < objects_Num; ++i){
        //             auto object = objects.object_list[i];
        //             cv::Rect bbox_iter = cv::Rect(object.bounding_box_2d[0][0], // x
        //                     object.bounding_box_2d[0][1],// y
        //                     object.bounding_box_2d[2][0]-object.bounding_box_2d[0][0], // w
        //                     object.bounding_box_2d[2][1]-object.bounding_box_2d[0][1]); // h
        //             center.x = (int) (object.bounding_box_2d[2][0]+object.bounding_box_2d[0][0])>>1;
        //             center.y = (int) (object.bounding_box_2d[2][1]+object.bounding_box_2d[0][1])>>1;
        //             cv::rectangle(cvImage,bbox_iter, cv::Scalar(255, 0, 255), 3, 3,0);
        //             cv::putText(cvImage,  to_string(i+1) , center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 255), 2, 8, 0);
        //         }
                
        //         signal.pubMsg(-1*objects_Num);
        //         rclcpp::spin_some(signal.node);
        //         frozenFrame = true;
        //         trackTarget = false;
        //     }

        // } else if( (flag<0) && (flag>=100) && (!trackTarget) ){
        //     frozenFrame = false;            
        //     trackTarget = true;
        //     int objIdx = flag - 1;
        //     if (objects.is_new) {
        //         int objects_Num = objects.object_list.size();
        //         if (!objects.object_list.empty()) {
        //             target = objects.object_list.front();
        //             // func 1
        //             for(int i = 0; i < objects_Num; ++i){
        //                 auto object = objects.object_list[i];
        //                 if (i == objIdx){                            
        //                     target = object;
        //                 }
        //                 cv::Rect bbox_iter = cv::Rect(object.bounding_box_2d[0][0], // x
        //                         object.bounding_box_2d[0][1],// y
        //                         object.bounding_box_2d[2][0]-object.bounding_box_2d[0][0], // w
        //                         object.bounding_box_2d[2][1]-object.bounding_box_2d[0][1]); // h
        //                 center.x = (int) (object.bounding_box_2d[2][0]+object.bounding_box_2d[0][0])>>1;
        //                 center.y = (int) (object.bounding_box_2d[2][1]+object.bounding_box_2d[0][1])>>1;
        //                 cv::rectangle(cvImage,bbox_iter, cv::Scalar(255, 0, 255), 3, 3,0);
        //                 cv::putText(cvImage,  to_string(i+1) , center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 255), 2, 8, 0);
                        
        //                 // if (detection_parameters.enable_tracking)
        //                 //     cout << object.tracking_state ;
        //             }
        //             cv::Rect bbox_target = cv::Rect(target.bounding_box_2d[0][0], // x
        //                         target.bounding_box_2d[0][1],// y
        //                         target.bounding_box_2d[2][0]-target.bounding_box_2d[0][0], // w
        //                         target.bounding_box_2d[2][1]-target.bounding_box_2d[0][1]); // h
        //             cv::Point center;
        //             center.x = (int) (target.bounding_box_2d[2][0]+target.bounding_box_2d[0][0])>>1;
        //             center.y = (int) (target.bounding_box_2d[2][1]+target.bounding_box_2d[0][1])>>1;
        //             cv::rectangle(cvImage,bbox_target, cv::Scalar(0, 0, 255), 3, 3,0);
        //             cv::circle(cvImage,  center , 2, cv::Scalar(0, 0, 255), 3, 8, 0);
        //             goal.pubMsg(target.position.x,target.position.y,target.position.z); 
        //         }
        //     }
        // } else if((flag<0) && (flag>=100) && (trackTarget)){
        //     if (objects.is_new) {
        //         int objects_Num = objects.object_list.size();
        //         if (!objects.object_list.empty()) {
        //             target = objects.object_list.front();
        //             bool findTarget = false;
        //             // func 1
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
        //                 cv::rectangle(cvImage,bbox_iter, cv::Scalar(255, 0, 255), 3, 3,0);
        //                 cv::putText(cvImage,  to_string(i+1) , center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 255), 2, 8, 0);
                        
        //                 // if (detection_parameters.enable_tracking)
        //                 //     cout << object.tracking_state ;
        //             }
        //             if (!findTarget){
        //                 float dist = 10000.0;
        //                 for(int i = 0; i < objects_Num; ++i){
        //                     auto object = objects.object_list[i];
        //                     float curr_dist =     object.position.x*object.position.x 
        //                              + object.position.y*object.position.y
        //                              + object.position.z*object.position.z;
        //                     if (dist>curr_dist){
        //                         target = object;
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
        //             cv::rectangle(cvImage,bbox_target, cv::Scalar(0, 0, 255), 3, 3,0);
        //             cv::circle(cvImage,  center , 2, cv::Scalar(0, 0, 255), 3, 8, 0);
        //             goal.pubMsg(target.position.x,target.position.y,target.position.z); 
        //         }
        //     }



        // }

        sl::float4 point3D;
        // Get the 3D point cloud values for pixel (i,j)
        cvMap = cv::Mat::zeros(cv::Size(500,500),CV_8UC4);
        for(int m = 0; m<(int) pointCloud.getHeight(); ++m){
            for(int n = 0; n<(int) pointCloud.getWidth(); ++n){                
                sl::float4 point3D;
                pointCloud.getValue( n,m, &point3D);
                // x l&r y u&d z b&f
                if (point3D.x==NULL || point3D.y==NULL || point3D.z==NULL){
                    continue;
                }
                point3D.x = (int) 100 * (point3D.x + 2.5);
                point3D.z = (int) 100 * (point3D.z);
                point3D.y = (int) 100 * (point3D.y + 0.25);
                if ((point3D.x < 500) && (point3D.x > 0)){
                    if ((point3D.z<500) && (point3D.z>0)){
                        // TODO: 计算y方向高度 
                        if ((point3D.y<200) && (point3D.y>15)){
                            
                            int x = (int) point3D.x;
                            int z = (int) point3D.z;                           
                            cvMap.at<cv::Vec4b>(500-z,x) = { 255,  255 , 255 } ;
                        }

                    }
                }
            }
        }
        // float distance = sqrt(point3D.x*point3D.x + point3D.y*point3D.y + point3D.z*point3D.z);
        // cout <<"x: "<<point3D.x<<",y: "<<point3D.y<<",z: "<<point3D.z<<endl;
        cv::Mat cvCostMap;
        cv::resize(cvMap, cvCostMap, cv::Size(50, 50), 0, 0, cv::INTER_AREA);
        video.pubMsg(cvImage);
        showmap.pubMsg(cvMap);
        obmap.pubMsg(cvCostMap);


        //Display the image
        // cv::imshow("showMap", cvMap);
        // cv::imshow("live video", cvImage);
        // cv::imshow("sendMap", cvCostMap);
        key = cv::waitKey(1);

        cout << "FPS : " <<(double) CLOCKS_PER_SEC/(clock() - startTime)  << " " << endl;        
        startTime = clock();
    }

    // Close the camera
    zed.close();    
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
