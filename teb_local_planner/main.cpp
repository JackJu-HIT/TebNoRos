#include <iostream>
#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
#include "inc/optimal_planner.h"
#include<boost/smart_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
//#include "matplotlibcpp.h"

using namespace teb_local_planner;
using namespace std;

int main()
{
    // 参数配置
    TebConfig config;
    PoseSE2 start(-2,0,0);
    PoseSE2 end(10,0,0);
    std::vector<ObstaclePtr> obst_vector;
    obst_vector.emplace_back(boost::make_shared<PointObstacle>(5,0.1));
    ViaPointContainer via_points;

    //add wayPoints
    for(int i = -2;i < 11;i++)
    via_points.push_back( Eigen::Vector2d( i, 1 ) );
    // Setup robot shape model
    RobotFootprintModelPtr robot_model = boost::make_shared<CircularRobotFootprint>(0.4);
    auto visual = TebVisualizationPtr(new TebVisualization(config));
    auto planner = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points);
    cv::Mat show_map = cv::Mat::zeros(cv::Size(500,500),CV_8UC3);

    // param
    int start_theta = 0;
    int end_theta = 0;
    float v_x = 0;
    float v_y = 0;
    float w   = 0;
    int look_ahead_poses = 1;


    while (true)
    {
        memset(show_map.data,0,500*500*3);
        try
        {
            start.theta() = start_theta * 0.1;
            end.theta() = end_theta * 0.1;

            planner->plan(start,end);
            // vi
            std::vector<Eigen::Vector3f> path;
            planner->getFullTrajectory(path);
            planner->getVelocityCommand(v_x,v_y,w,look_ahead_poses);
            cout<<"速度指令v_ｘ:"<<v_x<<" "<<"速度指令ｖ_y:"<<v_y<<" " <<"w指令"<<w<<endl;
            cout<<"路徑長度："<<path.size()<<endl;
            for(int i = 0;i < path.size() - 1;i ++)
            {
                int x = (int)(path.at(i)[0] * 100.f + 250);
                int y = (int)(path.at(i)[1] * 100.f + 250);
                int next_x = (int)(path.at(i+1)[0] * 100.f + 250);
                int next_y = (int)(path.at(i+1)[1] * 100.f + 250);
                cv::line(show_map,cv::Point(x,y),cv::Point(next_x,next_y),cv::Scalar(255,255,255));
                cout<<"("<<path.at(i)[0]<<","<<path.at(i)[1]<<")"<<endl;
            }
            cv::namedWindow("myWindowName");//创建窗口
            cv::createTrackbar("start theta","path",&start_theta,100);
            cv::createTrackbar("end theta","path",&end_theta,100);
            cv::imshow("path",show_map);
        }
        catch (...)
        {
            break;
        }
        cv::waitKey(10);
    }
    return 0;
}
