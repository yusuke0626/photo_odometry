#include <cstdio>
#include <iostream>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <photo_odometry/camera.h>
#include <photo_odometry/cam_operator.h>

#define GET_RANGE 2000
#define X_ORIGIN 0
#define Y_ORIGIN 200
#define X_SIZE 640 //630
#define Y_SIZE 250
#define FRAME_X_SIZE 640 //630
#define FRAME_Y_SIZE 250
//0 off,1 petbottle RED, ,2 petbottle BLUE ,feed 3
using namespace cv;
using namespace std;

short c_range[6] = {10, 90, 130, 35, 250, 210};

short mode = 0;
int x_center;
bool color_range_select(void);

bool send_response(photo_odometry::cam_operator::Request &cam_req, photo_odometry::cam_operator::Response &cam_res)
{
    mode = cam_req.sign;
    cam_res.x_diff = x_center;
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


    bool flag = false;

    int av_count = 0;
    long int sum_A = 0;
    long int sum_B = 0;
    long int sum_C = 0;
    int average[3] = {};

    bool mode_sel = 1;

    Mat rec;

    //cout << "adjust color 0, not 1" << endl;
    //cin >> mode_sel;

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
            switch (mode)
            {
                case 0:
                    c_range[0] = 0;
                    c_range[1] = 0;
                    c_range[2] = 0;
                    c_range[3] = 0;
                    c_range[4] = 0;
                    c_range[5] = 0;
                    break;
                case 1: // red
                    c_range[0] = 0;
                    c_range[1] = 114;
                    c_range[2] = 67;
                    c_range[3] = 6;
                    c_range[4] = 247;
                    c_range[5] = 158;
                    break;
                case 2: // blue
                    c_range[0] = 105;
                    c_range[1] = 130;
                    c_range[2] = 25;
                    c_range[3] = 130;
                    c_range[4] = 255;
                    c_range[5] = 255;
                    break;
                case 3: // bottle
                    c_range[0] = 20;
                    c_range[1] = 35;
                    c_range[2] = 90;
                    c_range[3] = 180;
                    c_range[4] = 130;
                    c_range[5] = 230;
                    break;
                case 4:
                    c_range[0] = 0;
                    c_range[1] = 114;
                    c_range[2] = 67;
                    c_range[3] = 6;
                    c_range[4] = 247;
                    c_range[5] = 158;
                    break;
                case 5:
                    c_range[0] = 105;
                    c_range[1] = 130;
                    c_range[2] = 25;
                    c_range[3] = 130;
                    c_range[4] = 255;
                    c_range[5] = 255;
                    break;
                default:
                    mode = 0;
                    break;
            }

            inRange(mainhsv, Scalar(c_range[0], c_range[1], c_range[2]), Scalar(c_range[3], c_range[4], c_range[5]), maindst);
            erode(maindst, maindst, Mat(), Point(-1, -1), 3);
            dilate(maindst, maindst, Mat(), Point(-1, -1), 6);

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
                colors[i] = Vec3b(250,250,50);
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

            int ob_two[4] = {0,0,0,0};
            int wit_two[4] = {0,0,0,0};
            int hei_two[4] = {0,0,0,0};
            int q = 0;

            bool binary_flag = true;
            bool noise_flag = false;
            //bool error_flag  = false;
            //座標
            for (int i = 1; i < nLab; ++i)
            {
                int *param = stats.ptr<int>(i);
                if (param[cv::ConnectedComponentsTypes::CC_STAT_AREA] > GET_RANGE)
                {
                    x_object = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
                    y_object = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
                    ob_two[q] = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
                    wit_two[q] = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
                    current_area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];
                    int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
                    int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

                    rectangle(Dst, Rect(x_object, y_object, width, height), Scalar(0, 255, 0), 2);
                    stringstream obj;
                    obj << q;
                    putText(Dst, obj.str(), Point(x_object + 5, y_object + 20), FONT_HERSHEY_COMPLEX, 0.7, Scalar(0, 255, 255), 2);
                    if (max_area < current_area)
                    {
                        max_area = current_area;
                        x_center = (x_object + width) / 2;
                        y_max_area = y_object;
                    }
                    q++;
                }
                if(q >= 3 || q <= 1){
                    binary_flag = false;
                }
                if(q >= 2){
                    noise_flag = true;
                }
            }

            int center_two[4] = {0,0,0,0};

            center_two[1] = ((ob_two[1] + wit_two[1]) / 2);
            center_two[0] = ((ob_two[0] + wit_two[0]) / 2);

            if(center_two[0] < center_two[1]){
                int insted = center_two[1];
                center_two[1] = center_two[0];
                center_two[0] = insted;
            }


            if(mode == 4 || mode == 5){
                if(binary_flag == true){
                    msg.x = -1 * ((X_SIZE/2 - center_two[1]) - center_two[0]);
                    msg.y = y_object;
                    cout << "mode 4 or 5";
                }else{
                    msg.x = 5000;
                    msg.y = 5000;
                    cout << "noise in mode 4 or 5"; 
                }
            }else if(mode == 0){
                cout << "camera mode off (mode == 0)";
                //msg.x = 0;
                //msg.y = 0;
            }else if(mode == 1 || mode == 2 || mode == 3){
                if (noise_flag == false)
                {
                    msg.x = x_center - 3; //-232
                    msg.y = y_max_area - 250;
                    cout << "mode 1 or 2 or 3";
                }else{
                    cout << "noise in mode 1 or 2 or 3";
                    msg.x = 5000;
                    msg.y = 5000;
                }
            }else{
                msg.x = 5000;
                msg.y = 5000;
                cout << "error";
            }

            cout << "0: " << center_two[0] << endl;
            cout << "1: " << center_two[1] << endl;

            ros_camera_pub.publish(msg);

            imshow("Data", Dst);
        }
        ros::spinOnce();
        loop_rate.sleep();
        imshow("before",mainhsv);
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
    bool red_m = 0;
    cin >> red_m;
    while (ros::ok())
    {
        Mat img;
        Mat frame;
        Mat hsv;
        cap >> img;
        cap >> frame;

        Mat rsi(img, Rect(X_ORIGIN, Y_ORIGIN, X_SIZE, Y_SIZE));
        resize(rsi, rsi, Size(FRAME_X_SIZE, FRAME_Y_SIZE));
        //imshow("img", rsi);

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
        imshow("aaa",hsv);
        inRange(hsv, Scalar(c_range[0], c_range[1], c_range[2]), Scalar(c_range[3], c_range[4], c_range[5]), frame);
        erode(frame, frame, Mat(), Point(-1, -1), 3);
        dilate(frame, frame, Mat(), Point(-1, -1), 5);
        imshow("binary img", frame);
    }
    destroyAllWindows();
    return true;
}
