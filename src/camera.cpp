#include <cstdio>
#include <iostream>
//#include <random>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <photo_odometry/camera.h>
#include <photo_odometry/cam_operator.h>

#define GET_RANGE 300
#define X_ORIGIN 16
#define Y_ORIGIN 0
#define X_SIZE 500
#define Y_SIZE 300
#define FRAME_X_SIZE 500
#define FRAME_Y_SIZE 300

using namespace cv;
using namespace std;

short c_range[6] = {10, 90, 130, 35, 250, 210};

int x_max_area = 0;

bool color_range_select(void);

bool send_response(photo_odometry::cam_operator::Request &cam_req, photo_odometry::cam_operator::Response &cam_res)
{
    cam_res.x_diff = x_max_area;
    ROS_INFO("request: %d   response: %d \n", cam_req.sign, cam_res.x_diff);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_info");
    ros::NodeHandle nh;

    ros::Publisher ros_camera_pub = nh.advertise<photo_odometry::camera>("cam_msg", 1000);
    ros::ServiceServer ros_camera_srv = nh.advertiseService("cam_srv", send_response);
    ros::Rate loop_rate(30);

    photo_odometry::camera msg;
    photo_odometry::cam_operator srv;

    bool range_flag = false;
    bool flag = false;

    int av_count = 0;
    long int sum_A = 0;
    long int sum_B = 0;
    long int sum_C = 0;
    int average[3] = {};

    bool mode_sel = 1;

    Mat rec;

    cout << "adjust color 0, not 1" << endl;
    cin >> mode_sel;

    if (mode_sel == 0)
        color_range_select();

    VideoCapture cap(1);
    if (!cap.isOpened())
    {
        cout << "Not Opened" << endl;
        return -1;
    }
    else
    {
        cout << "Open Successful" << endl;
    }

    cout << "*---------Main Start---------*" << endl;

    while (true)
    {
        Mat mainframe;
        Mat mainhsv;
        int stopkey = waitKey(50);

        if (stopkey == 97)
            break;

        if (flag == false)
        {
            cap >> mainframe;
            Mat maindst(mainframe, Rect(X_ORIGIN, Y_ORIGIN, X_SIZE, Y_SIZE));
            resize(maindst, maindst, Size(FRAME_X_SIZE, FRAME_Y_SIZE));
            cvtColor(maindst, mainhsv, COLOR_BGR2HSV);
            inRange(mainhsv, Scalar(c_range[0], c_range[1], c_range[2]), Scalar(c_range[3], c_range[4], c_range[5]), maindst);
            erode(maindst, maindst, Mat(), Point(-1, -1), 3);
            dilate(maindst, maindst, Mat(), Point(-1, -1), 5);

            //ラべリング
            Mat LabelImg;
            Mat stats;
            Mat centroids;
            int nLab = connectedComponentsWithStats(maindst, LabelImg, stats, centroids);

            // 描画色決定
            vector<Vec3b> colors(nLab);
            colors[0] = Vec3b(0, 0, 0);
            for (int i = 1; i < nLab; ++i)
            {
                colors[i] = Vec3b(i * 50, 200 ,200); //色をランダムで決定する（255で範囲指定しないといけないからビット演算）
            }

            //描画
            Mat Dst(maindst.size(), CV_8UC3);
            for (int i = 0; i < Dst.rows; ++i)
            {
                int *lb = LabelImg.ptr<int>(i);
                Vec3b *pix = Dst.ptr<Vec3b>(i);
                for (int j = 0; j < Dst.cols; ++j)
                {
                    pix[j] = colors[lb[j]];
                }
            }

            //重心計算
            int centerX[nLab];
            int centerY[nLab];
            for (int i = 1; i < nLab; ++i)
            {
                double *param = centroids.ptr<double>(i);
                centerX[i] = static_cast<int>(param[0]);
                centerY[i] = static_cast<int>(param[1]);
                circle(Dst, Point(centerX[i], centerY[i]), 3, Scalar(0, 0, 255), -1);
            }

            int x_object;
            int y_object;

            int y_max_area = 0;
            int max_area = 0;
            int current_area = 0;

            //座標
            for (int i = 1; i < nLab; ++i)
            {
                int *param = stats.ptr<int>(i);
                if (param[cv::ConnectedComponentsTypes::CC_STAT_AREA] > GET_RANGE)
                {
                    x_object = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
                    y_object = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
                    current_area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];
                    int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
                    int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
                    rectangle(Dst, Rect(x_object, y_object, width, height), Scalar(0, 255, 0), 2);
                    stringstream obj;
                    obj << i;
                    putText(Dst, obj.str(), Point(x_object + 5, y_object + 20), FONT_HERSHEY_COMPLEX, 0.7, Scalar(0, 255, 255), 2);
                    if (max_area < current_area)
                    {
                        max_area = current_area;
                        x_max_area = x_object;
                        y_max_area = y_object;
                    }
                }
            }

            msg.x = x_max_area - 250;
            msg.y = y_max_area - 250;

            ros_camera_pub.publish(msg);

            imshow("Data", Dst);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    cout << "*----------Main finish-----------*" << endl;
    return 0;
}

bool color_range_select(void)
{
    VideoCapture cap(1);
    if (!cap.isOpened())
    {
        cout << "Not Opened" << endl;
        return -1;
    }
    else
    {
        cout << "Open Successful" << endl;
    }

    while (ros::ok())
    {
        Mat img;
        Mat frame;
        Mat hsv;
        cap >> img;
        cap >> frame;
        Mat rsi(img, Rect(X_ORIGIN, Y_ORIGIN, X_SIZE, Y_SIZE));
        resize(rsi, rsi, Size(FRAME_X_SIZE, FRAME_Y_SIZE));
        imshow("img", rsi);

        int key = waitKey(50);
        bool swflag = false;

        switch (key)
        {
        case 113:
            c_range[0] = c_range[0] + 5;
            break;
        case 97:
            c_range[0] = c_range[0] - 3;
            break;
        case 119:
            c_range[1] = c_range[1] + 5;
            break;
        case 115:
            c_range[1] = c_range[1] - 3;
            break;
        case 101:
            c_range[2] = c_range[2] + 5;
            break;
        case 100:
            c_range[2] = c_range[2] - 3;
            break;
        case 117:
            c_range[3] = c_range[3] - 5;
            break;
        case 106:
            c_range[3] = c_range[3] + 3;
            break;
        case 105:
            c_range[4] = c_range[4] - 5;
            break;
        case 107:
            c_range[4] = c_range[4] + 3;
            break;
        case 111:
            c_range[5] = c_range[5] - 5;
            break;
        case 108:
            c_range[5] = c_range[5] + 3;
            break;
        case 122:
            swflag = true;
            break;
        }

        if (swflag == true)
        {
            break;
        }
        printf("(%d,%d,%d)\n", c_range[0], c_range[1], c_range[2]);
        printf("(%d,%d,%d)\n", c_range[3], c_range[4], c_range[5]);
        Mat dst(frame, Rect(X_ORIGIN, Y_ORIGIN, X_SIZE, Y_SIZE));
        resize(dst, dst, Size(FRAME_X_SIZE, FRAME_Y_SIZE));
        cvtColor(dst, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(c_range[0], c_range[1], c_range[2]), Scalar(c_range[3], c_range[4], c_range[5]), frame);
        erode(frame, frame, Mat(), Point(-1, -1), 3);
        dilate(frame, frame, Mat(), Point(-1, -1), 5);
        imshow("binary img", frame);
    }
    destroyAllWindows();
    return true;
}
