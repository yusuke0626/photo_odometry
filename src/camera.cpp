#include <cstdio>
#include <iostream>
#include <random>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <photo_odometry/camera.h>

#define GET_RANGE 300
#define X_ORIGIN 0
#define Y_ORIGIN 0
#define X_SIZE 500
#define Y_SIZE 300
#define FRAME_X_SIZE 500
#define FRAME_Y_SIZE 300

using namespace cv;
using namespace std;

photo_odometry::camera msg;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_info");
    ros::NodeHandle nh;

    ros::Publisher ros_camera_pub = nh.advertise<photo_odometry::camera>("camera", 1000);
    ros::Rate loop_rate(30);

    bool range_flag = false;
    bool flag = false;

    unsigned short int a = 10;
    unsigned short int b = 100;
    unsigned short int c = 100;
    unsigned short int d = 40;
    unsigned short int e = 255;
    unsigned short int f = 255;

    int av_count = 0;
    long int sum_A = 0;
    long int sum_B = 0;
    long int sum_C = 0;
    int average[3] = {};
    Mat rec;

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
            a = a + 5;
            break;
        case 97:
            a = a - 3;
            break;
        case 119:
            b = b + 5;
            break;
        case 115:
            b = b - 3;
            break;
        case 101:
            c = c + 5;
            break;
        case 100:
            c = c - 3;
            break;
        case 117:
            d = d - 5;
            break;
        case 106:
            d = d + 3;
            break;
        case 105:
            e = e - 5;
            break;
        case 107:
            e = e + 3;
            break;
        case 111:
            f = f - 5;
            break;
        case 108:
            f = f + 3;
            break;
        case 122:
            swflag = true;
            break;
        }

        if (swflag == true)
        {
            break;
        }
        printf("(%d,%d,%d)\n", a, b, c);
        printf("(%d,%d,%d)\n", d, e, f);
        Mat dst(frame, Rect(X_ORIGIN, Y_ORIGIN, X_SIZE, Y_SIZE));
        resize(dst, dst, Size(FRAME_X_SIZE, FRAME_Y_SIZE));
        cvtColor(dst, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(a, b, c), Scalar(d, e, f), frame);
        erode(frame, frame, Mat(), Point(-1, -1), 3);
        dilate(frame, frame, Mat(), Point(-1, -1), 5);
        imshow("binary img", frame);
    }

    destroyAllWindows();
    cout << "*---------Main Start---------*" << endl;

    while (true)
    {
        cout << "aaa"<< endl;
        Mat mainframe;
        Mat mainhsv;
        int stopkey = waitKey(50);

        if (stopkey == 97)
            break;

        if (flag == false)
        {
            cout << "bbb" <<endl;
            cap >> mainframe;
            Mat maindst(mainframe, Rect(X_ORIGIN, Y_ORIGIN, X_SIZE, Y_SIZE));
            resize(maindst, maindst, Size(FRAME_X_SIZE, FRAME_Y_SIZE));
            cvtColor(maindst, mainhsv, COLOR_BGR2HSV);
            inRange(mainhsv, Scalar(a, b, c), Scalar(d, e, f), maindst);
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
                colors[i] = Vec3b((rand() & 255), (rand() & 255), (rand() & 255)); //色をランダムで決定する（255で範囲指定しないといけないからビット演算）
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

            int x_max_area = 0;
            int y_max_area = 0;
            int max_area = 0;
            int current_area = 0;

            int obcount = 0;
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
                    if(max_area < current_area){
                        max_area = current_area;
                        x_max_area = x_object;
                        y_max_area = y_object;
                    } 
                }
            }

            msg.x = x_max_area;
            msg.y = y_max_area;

            ros_camera_pub.publish(msg);
        
            imshow("Data", Dst);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    cout << "*----------Main finish-----------*" << endl;
    return 0;
}
