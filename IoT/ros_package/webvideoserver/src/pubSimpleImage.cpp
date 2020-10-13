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


#include "rclcpp/rclcpp.hpp"
// OpenCV include (for display)
#include <opencv2/opencv.hpp>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

// Using std and sl namespaces
using namespace std;




using std::placeholders::_1;
// init global parameter


cv::Mat cvImg;
cv::Mat cvImgCompressed;


clock_t startTime,startTime1,endTime;

class LabelImage : public rclcpp::Node
{
public:
    LabelImage()
    : Node("LiveVideo"), count_(0)
    {
        publisherCompressedImg_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/webserver/TestVideo1/compressed", 1);
        publisherImg_ = this->create_publisher<sensor_msgs::msg::Image>("/webserver/TestVideo2", 1);
        timer_ = this->create_wall_timer(
        50ms, std::bind(&LabelImage::timer_callback, this));
        
    }    
    
private:
    cv_bridge::CvImage img_bridge;
    void timer_callback()
    {
        
        // cv::Mat cvShowMap = cv::Mat::zeros(600,600,3);
        // cv::resize(cvImg, cvShowMap, cv::Size(192, 144), 0, 0, cv::INTER_AREA);


        auto message = sensor_msgs::msg::Image ();
        std_msgs::msg::Header header; // empty header
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, cvImg);
        img_bridge.toImageMsg(message); // from cv_bridge to sensor_msgs::Image


        auto Compressedmessage = sensor_msgs::msg::CompressedImage ();
        std_msgs::msg::Header Compressedheader; // empty header
        img_bridge = cv_bridge::CvImage(Compressedheader, sensor_msgs::image_encodings::BGRA8, cvImgCompressed);
        img_bridge.toCompressedImageMsg(Compressedmessage); // from cv_bridge to sensor_msgs::Image

       


        publisherCompressedImg_->publish(Compressedmessage);
        publisherImg_->publish(message);
        //Display the image
        // c  
        
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage >::SharedPtr publisherCompressedImg_;
    rclcpp::Publisher<sensor_msgs::msg::Image >::SharedPtr publisherImg_;
    size_t count_;
};


int main(int argc, char **argv) {

    cvImg = cv::imread("test/test.png",cv::IMREAD_COLOR);
    cv::cvtColor(cvImg, cvImg, cv::COLOR_RGBA2BGRA);

    cvImgCompressed = cv::imread("test/testCompressed.jpg",cv::IMREAD_COLOR);
    cv::cvtColor(cvImgCompressed, cvImgCompressed, cv::COLOR_RGBA2BGRA);
    //cv::imshow("Test video", cvImg);
    //cv::waitKey(5000);
    //  init rclcpp
    rclcpp::init(argc, argv);
    // make multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    // make nodehandle    
    auto nh_video = std::make_shared<LabelImage>();

    // add to the executor
    executor.add_node(nh_video);

    // wait for shutdown and then clean up
    cout << "Ros2: Start Spin..." << endl;
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}

