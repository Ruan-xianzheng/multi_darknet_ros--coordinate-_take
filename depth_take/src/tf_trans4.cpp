#include <ros/ros.h>
#include<darknet_ros_msgs/BoundingBoxes.h>
#include<darknet_ros_msgs/BoundingBox.h>
#include<std_msgs/String.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
// 坐标转换头文件
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
//坐标发送给QT包含的头文件
#include <depth_take/Object.h>
using namespace std;


int x_center[10];//定义标记框中心点坐标
int y_center[10];
string class_name[10];//识别物体的类别
float probability[10];//识别的成功率
float x_world[10],y_world[10]; //相机坐标系下的坐标
int length;
float depth[10]; 
//标记框坐标信息处理
void domsg(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
        int xmin[10],ymin[10],xmax[10],ymax[10]; //标记框坐标 
        length=msg->bounding_boxes.size();

        for(int i=0;i<length;i++){
            xmax[i]=msg->bounding_boxes[i].xmax;
            xmin[i]=msg->bounding_boxes[i].xmin;
            ymax[i]=msg->bounding_boxes[i].ymax;
            ymin[i]=msg->bounding_boxes[i].ymin;
            x_center[i]=(msg->bounding_boxes[i].xmax+msg->bounding_boxes[i].xmin)/2;
            y_center[i]=(msg->bounding_boxes[i].ymax+msg->bounding_boxes[i].ymin)/2;
            class_name[i]=msg->bounding_boxes[i].Class;
            probability[i] = msg->bounding_boxes[i].probability;
            
    }
}
 
//深度图像信息处理
void depthcallback(const sensor_msgs::Image::ConstPtr& msg)
{

    float de[10];//用于存储10个深度数据
    float dep_sum=0;
    float fx=554.27389,cx=320.5,fy=554.27389,cy=240.5; //定义相机内参
    cv_bridge::CvImagePtr img_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

    for(int j=0;j<length;j++){ 
        for(int n=0;n<10;n++){
        de[n] = img_cv_ptr->image.at<float>(y_center[j], x_center[j]);
        dep_sum=dep_sum+de[n];
        }       
        depth[j]=dep_sum/10;
    x_world[j]=(x_center[j]-cx)*depth[j]/fx;
    y_world[j]=(y_center[j]-cy)*depth[j]/fy; 
    }
   
    
}
int main(int argc, char *argv[])
{
    //防止中文乱码
    setlocale(LC_ALL,"");
    //ros初始化
    ros::init(argc,argv,"tf_trans4");
    //句柄初始化
    ros::NodeHandle nh4;
    tf2_ros::Buffer tfbuffer;//创造一个存储tf变换信息
    tf2_ros::TransformListener tfListener(tfbuffer);//监听坐标变化信息,并将信息传给tfbuffer
    geometry_msgs::TransformStamped tfs;//创建一个转换器对象
    geometry_msgs::TransformStamped transformStamped;//ros坐标变换的对象，包含变换的信息
    tf2::Transform tf_transform;
    geometry_msgs::PointStamped camera_point;
    //创建订阅者对象
    ros::Subscriber sub=nh4.subscribe("/darknet_ros/bounding_boxes4",10,domsg);
    ros::Subscriber image_sub=nh4.subscribe<sensor_msgs::Image>("robot4/camera_link/depth/image_raw",1,depthcallback);      
    //创建发布者对象 用于发布给QT
    ros::Publisher pub = nh4.advertise<depth_take::Object>("/robot4/yolo_object",1000);

    ros::Rate r(2);
    while (ros::ok)
    {
        //得到相机坐标系相对于odom的坐标偏移量
        transformStamped=tfbuffer.lookupTransform("robot4/odom","robot4/camera_link",ros::Time(0),ros::Duration(10.0));
        //相机坐标系和基座坐标系的转换信息设置
        camera_point.header.frame_id="robot4/camera_link";
        camera_point.header.stamp=ros::Time();

        for(int i=0;i<length;i++){
            //将获得的三维坐标换到相机坐标系
            camera_point.point.x=depth[i];
            camera_point.point.y=-x_world[i];
            camera_point.point.z=-y_world[i]; 
            //当图像中没有检测到物体或者检测到的物体距离小车较远时都不输出坐标信息
            if(depth[i]<10.0){
                try
                {
                    geometry_msgs::PointStamped base_footprint_point;
                    //转换坐标点（相对于odom坐标系）
                    base_footprint_point = tfbuffer.transform(camera_point,"robot4/odom");
                    ROS_INFO("类别:%s,坐标:x=%.3f米,y=%.3f米,z=%.3f米",class_name[i].c_str(),base_footprint_point.point.x,base_footprint_point.point.y,0.0);

                }
                catch(tf2::TransformException &ex)
                {
                    ROS_WARN("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
            }
        }
        r.sleep();
        ros::spinOnce();
    }
    
 
    return 0;
}