#include <iostream>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.h>
#include <sophus/se3.h>

using namespace std;
using namespace cv;

#include <iostream>
#include "boost/lexical_cast.hpp"


// 将相机位姿mCameraPose由Mat类型转化为OpenGlMatrix类型
void GetCurrentOpenGLCameraMatrix(Mat Rwc,Mat twc,pangolin::OpenGlMatrix &M)
{
    if(!Rwc.empty()&&!twc.empty())
    {


        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}


void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    //相机模型大小：宽度占总宽度比例为0.08
    //const float w = 0.7;  // KITTI
    const float w = 0.07;   // TUM
    const float h = w*0.75;
    const float z = w*0.6;

    //百度搜索：glPushMatrix 百度百科
    glPushMatrix();

    //将4*4的矩阵Twc.m右乘一个当前矩阵
    //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
    //因为OpenGL中的矩阵为列优先存储，因此实际为Tcw，即相机在世界坐标下的位姿
#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    //设置绘制图形时线的宽度
    glLineWidth(3);   //在yaml中读的
    //设置当前颜色为绿色(相机图标显示为绿色)
    glColor3f(0.0f,0.0f,1.0f);
    //用线将下面的顶点两两相连
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void DrawKeyFrames(const bool bDrawKF,const bool bDrawGraph,vector<Mat> v_Key_Twc,vector<Mat> v_Key_Ow,Mat temp_now_Ow)
{
    //const float w = 0.7;       // KITTI
    const float w = 0.07;      // TUM
    const float h = w*0.75;
    const float z = w*0.6;

    if(bDrawKF)
    {
        for(int i=0;i<v_Key_Twc.size();i++)
        {
            cv::Mat Twc_col_main = v_Key_Twc[i].t();

            glPushMatrix();

            //Twc=Twc.t();

            glMultMatrixf(Twc_col_main.ptr<GLfloat>(0));

            //设置绘制图形时线的宽度
            glLineWidth(3);
            //设置当前颜色为蓝色(关键帧图标显示为蓝色)
            glColor3f(0.0f,1.0f,0.0f);
            //用线将下面的顶点两两相连
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();


            //画当前帧与最近参考帧的连线
            if(v_Key_Ow.size()!=0)
            {
                glLineWidth(3);
                glColor4f(1.0f,0.0f,0.0f,0.6f);

                glBegin(GL_LINES);
                glVertex3f(temp_now_Ow.at<float>(0),temp_now_Ow.at<float>(1),temp_now_Ow.at<float>(2));
                glVertex3f(v_Key_Ow[v_Key_Ow.size()-1].at<float>(0),v_Key_Ow[v_Key_Ow.size()-1].at<float>(1),v_Key_Ow[v_Key_Ow.size()-1].at<float>(2));
                glEnd();
            }
        }
    }


    if(bDrawGraph)
    {
        //设置绘制图形时线的宽度
        glLineWidth(3);
        //设置共视图连接线为绿色，透明度为0.6f
        glColor4f(1.0f,0.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=1; i<v_Key_Ow.size(); i++)
        {
            glVertex3f(v_Key_Ow[i].at<float>(0),v_Key_Ow[i].at<float>(1),v_Key_Ow[i].at<float>(2));
            glVertex3f(v_Key_Ow[i-1].at<float>(0),v_Key_Ow[i-1].at<float>(1),v_Key_Ow[i-1].at<float>(2));

        }
    }
    glEnd();

}

//Tcw_.copyTo(Tcw);
//cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
//cv::Mat tcw = Tcw.rowRange(0,3).col(3);
//cv::Mat Rwc = Rcw.t();
//Ow = -Rwc*tcw;

Mat Get_Ow(Mat& Twc)
{
    Mat Twc_;
    Twc.copyTo(Twc_);
    Mat Rwc_ = Twc_.rowRange(0,3).colRange(0,3);
    Mat Tcw_ = Twc_.inv();
    Mat tcw_ = Tcw_.rowRange(0,3).col(3);
    Mat Ow_ = -Rwc_*tcw_;
    return Ow_;
    //return Twc_.clone();
}

vector<Mat> Load_kitti_Twc(string file_kitti,int return_flag)
{
    vector<Mat> v_Rwc;
    vector<Mat> v_twc;
    ifstream fin(file_kitti);
    while (!fin.eof())  //读取真实轨迹参数
    {
        float data[12] = {0};
        for (auto &d:data)
            fin >> d;

        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 3, CV_32F);

        Rwc.at<float>(0, 0) = data[0];
        Rwc.at<float>(0, 1) = data[1];
        Rwc.at<float>(0, 2) = data[2];

        Rwc.at<float>(1, 0) = data[4];
        Rwc.at<float>(1, 1) = data[5];
        Rwc.at<float>(1, 2) = data[6];

        Rwc.at<float>(2, 0) = data[8];
        Rwc.at<float>(2, 1) = data[9];
        Rwc.at<float>(2, 2) = data[10];

        twc.at<float>(0) = data[3];
        twc.at<float>(1) = data[7];
        twc.at<float>(2) = data[11];

        v_Rwc.push_back(Rwc);
        v_twc.push_back(twc);
    }
    v_Rwc.pop_back();
    v_twc.pop_back();
    if(return_flag==0)
    {
        return v_Rwc;
    }
    if(return_flag==1)
    {
        return v_twc;
    }
}


vector<Mat> Load_TUM_Twc(string txt_file_TUM,int return_flag)
{
    vector<Mat> v_Rwc;
    vector<Mat> v_twc;
    ifstream fin(txt_file_TUM);

    while (!fin.eof())  //读取真实轨迹参数
    {
        float data[8] = {0};
        for (auto &d:data)
            fin >> d;

        Eigen::Quaterniond Quaternion(data[7],data[4],data[5],data[6]);

        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 1, CV_32F);

        Eigen::Matrix3d Rotation;
        Rotation = Quaternion.matrix();

        Rwc.at<float>(0, 0) = float(Rotation(0,0));
        Rwc.at<float>(0, 1) = float(Rotation(0,1));
        Rwc.at<float>(0, 2) = float(Rotation(0,2));

        Rwc.at<float>(1, 0) = float(Rotation(1,0));
        Rwc.at<float>(1, 1) = float(Rotation(1,1));
        Rwc.at<float>(1, 2) = float(Rotation(1,2));

        Rwc.at<float>(2, 0) = float(Rotation(2,0));
        Rwc.at<float>(2, 1) = float(Rotation(2,1));
        Rwc.at<float>(2, 2) = float(Rotation(2,2));

        twc.at<float>(0) = data[1];
        twc.at<float>(1) = data[2];
        twc.at<float>(2) = data[3];

        v_twc.push_back(twc);
        v_Rwc.push_back(Rwc);
    }
    v_twc.pop_back();
    v_Rwc.pop_back();

    if(return_flag==0)
        return v_Rwc;
    if(return_flag==1)
        return v_twc;
}


vector<string> Load_RGB_D_str(string strAssociationFilename,string img_path,int return_flag)
{
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());

    while (!fAssociation.eof()) {
        string s;
        getline(fAssociation, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;

            ss >> t;
            vTimestamps.push_back(t);

            ss >> sRGB;
            sRGB = img_path + sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);

            ss >> t;
            ss >> sD;
            sD = img_path + sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }
    if(return_flag == 0)
    {
        return vstrImageFilenamesRGB;
    }

    if(return_flag == 1)
    {
        return vstrImageFilenamesD;
    }

}

vector<Mat> Get_Twc(vector<Mat> v_Rwc,vector<Mat> v_twc)
{
    cv::Mat Twc_temp(4, 4, CV_32F);

    vector<Mat> v_Twc;
    for(int b=0;b < v_Rwc.size();b++)
    {

        Twc_temp.at<float>(0, 0)=v_Rwc[b].at<float>(0,0);
        Twc_temp.at<float>(0, 1)=v_Rwc[b].at<float>(0,1);
        Twc_temp.at<float>(0, 2)=v_Rwc[b].at<float>(0,2);
        Twc_temp.at<float>(0, 3)=v_twc[b].at<float>(0);

        Twc_temp.at<float>(1, 0)=v_Rwc[b].at<float>(1,0);
        Twc_temp.at<float>(1, 1)=v_Rwc[b].at<float>(1,1);
        Twc_temp.at<float>(1, 2)=v_Rwc[b].at<float>(1,2);
        Twc_temp.at<float>(1, 3)=v_twc[b].at<float>(1);

        Twc_temp.at<float>(2, 0)=v_Rwc[b].at<float>(2,0);
        Twc_temp.at<float>(2, 1)=v_Rwc[b].at<float>(2,1);
        Twc_temp.at<float>(2, 2)=v_Rwc[b].at<float>(2,2);
        Twc_temp.at<float>(2, 3)=v_twc[b].at<float>(2);

        Twc_temp.at<float>(3, 0)=0;
        Twc_temp.at<float>(3, 1)=0;
        Twc_temp.at<float>(3, 2)=0;
        Twc_temp.at<float>(3, 3)=1;


        Mat temp;
        Twc_temp.copyTo(temp);

        //注意,不能直接v_Twc.push_back(Twc_temp);
        v_Twc.push_back(temp);
    }
    return v_Twc;
}

void DrawMapPoints(vector<vector<float>> all_points)
{
    glPointSize(2);
    glBegin(GL_POINTS);

    for(int i=0;i<all_points.size();i++)
    {
        glColor3f(all_points[i][3]/255.0,all_points[i][4]/255.0,all_points[i][5]/255.0);

        glVertex3f(all_points[i][0],all_points[i][1],all_points[i][2]);

    }
    glEnd();
}


//typedef Eigen::Matrix<float, 6, 1> Vector6d;
//void SaveToPLYFile(string filename, vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud)
void SaveToPLYFile(string filename, vector<vector<float>> &pointcloud)
{
    string saveFilename=filename;

    ofstream fin;
    fin.open(filename.c_str());

    size_t  size_=pointcloud.size();


    fin << "ply";
    fin << "\nformat " << "binary_little_endian" << " 1.0";

    // Vertices
    fin << "\nelement vertex "<< size_;
    fin << "\nproperty float x"
            "\nproperty float y"
            "\nproperty float z";

    fin << "\nproperty uchar red"
            "\nproperty uchar green"
            "\nproperty uchar blue";


    fin << "\nend_header\n";

    fin.close ();

    std::ofstream fpout (filename.c_str (), std::ios::app | std::ios::binary);

    for(size_t i=0;i<pointcloud.size();i++)
    {

        float value;

        float x= static_cast<float>(pointcloud[i][0]);
        float y= static_cast<float>(pointcloud[i][1]);
        float z= static_cast<float>(pointcloud[i][2]);


        memcpy (&value, &x, sizeof (float));
        fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

        memcpy (&value, &y, sizeof (float));
        fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

        memcpy (&value, &z, sizeof (float));
        fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));


        unsigned char r = static_cast<uchar>(int(pointcloud[i][3]*255));
        unsigned char g = static_cast<uchar>(int(pointcloud[i][4]*255));
        unsigned char b = static_cast<uchar>(int(pointcloud[i][5]*255));


        fpout.write (reinterpret_cast<const char*> (&r), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&g), sizeof (unsigned char));
        fpout.write (reinterpret_cast<const char*> (&b), sizeof (unsigned char));


    }

    fin.close ();

}



int main( int argc, char** argv){


    string strAssociationFilename = "/home/tq/a4_dataset/TUM_dataset/rgbd_dataset_freiburg1_desk2/associate.txt";
    string img_path = "/home/tq/a4_dataset/TUM_dataset/rgbd_dataset_freiburg1_desk2/";

    vector<string> vstrImageFilenamesRGB,vstrImageFilenamesD;

    vstrImageFilenamesRGB = Load_RGB_D_str(strAssociationFilename,img_path,0);
    vstrImageFilenamesD = Load_RGB_D_str(strAssociationFilename,img_path,1);

    string txt_file_kitti = "../03.txt";
    //string txt_file_TUM = "../TUM_groundtruth_xyz.txt";
    string txt_file_TUM = "../CameraTrajectory.txt";

    double fps = 10.0;
    double mT  = 1e3/fps;
    //double mT = 10;

    vector<Mat> v_Rwc;
    vector<Mat> v_twc;

    //v_Rwc = Load_kitti_Twc(txt_file_kitti,0);
    //v_twc = Load_kitti_Twc(txt_file_kitti,1);

    v_Rwc = Load_TUM_Twc(txt_file_TUM,0);
    v_twc = Load_TUM_Twc(txt_file_TUM,1);

    vector<Mat> v_Twc;

    v_Twc = Get_Twc(v_Rwc,v_twc);

    //yaml_3
    //double fx = 535.4;
    //double fy = 539.2;
    //double cx = 320.1;
    //double cy = 247.6;


    //yaml_1
    double fx = 517.306408;
    double fy = 516.469215;
    double cx = 318.643040;
    double cy = 255.313989;


    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0,1.0,0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);

    //pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);  //只跟踪

    //pangolin::Var<bool> menuReset("menu.Reset",false,false);

    //double mViewpointX = 0;
    //double mViewpointY = -100;
    //double mViewpointZ = -0.1;

    //double mViewpointX = 0;
    //double mViewpointY = -4;
    //double mViewpointZ = -50;

    double mViewpointX = 0;
    double mViewpointY = -4;
    double mViewpointZ = -20;

    double mViewpointF = 2000;  //KITTI
    //double mViewpointF = 535.4;  //TUM

    //Camera.fx: 535.4
    //Camera.fy: 539.2
    //Camera.cx: 320.1
    //Camera.cy: 247.6

    pangolin::OpenGlRenderState s_cam(
            //pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),    //KITTI
            pangolin::ProjectionMatrix(1024,768,517.306408,516.469215,318.643040,255.313989,0.1,1000),  //TUM_desk2
            //俯视
            pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
    );


    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    //cv::namedWindow("ORB-SLAM2: Current Frame");　　//画图片的窗口

    bool bFollow = true;
    bool bLocalizationMode = false;


    vector<Mat> v_Key_Twc;

    Mat temp_Key_Ow;
    vector<Mat> v_Key_Ow;
    Mat temp_now_Ow;






    vector<vector<float>> all_points;

    for(int i=0;i<vstrImageFilenamesRGB.size();i++)
    {
        Mat color = imread(vstrImageFilenamesRGB[i]);
        Mat depth = imread(vstrImageFilenamesD[i],CV_LOAD_IMAGE_UNCHANGED);

        float mDepthMapFactor = 5000;
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;

        if((fabs(mDepthMapFactor-1.0f)>1e-5) || depth.type()!=CV_32F)
            depth.convertTo(depth,CV_32F,mDepthMapFactor);

        for ( int m=0; m<depth.rows; m+=3 )
        {
            for ( int n=0; n<depth.cols; n+=3 )
            {
                //unsigned int d = depth.ptr<unsigned short>(m)[n];
                //d/=5000;
                float d = depth.ptr<float>(m)[n];
                if (d < 0.01 || d>10)
                    continue;

                vector<float> points;

                Mat point_cam_temp(3, 1, CV_32F);
                point_cam_temp.at<float>(0) = float( ( n - cx) * d / fx );
                point_cam_temp.at<float>(1) = float( ( m - cy) * d / fy );
                point_cam_temp.at<float>(2) = d;

                Mat point_world_temp(3, 1, CV_32F);

                point_world_temp = v_Rwc[i]*point_cam_temp+v_twc[i];

                points.push_back(point_world_temp.at<float>(0));
                points.push_back(point_world_temp.at<float>(1));
                points.push_back(point_world_temp.at<float>(2));


                points.push_back(color.ptr<uchar>(m)[n*3]);
                points.push_back(color.ptr<uchar>(m)[n*3+1]);
                points.push_back(color.ptr<uchar>(m)[n*3+2]);

                all_points.push_back(points);

                points.clear();
            }
        }



        cout<<"all_points.size()="<<all_points.size()<<endl;


        // 清除缓冲区中的当前可写的颜色缓冲 和 深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 步骤1：得到最新的相机位姿
        GetCurrentOpenGLCameraMatrix(v_Rwc[i],v_twc[i],Twc);

        // 步骤2：根据相机的位姿调整视角
        // menuFollowCamera为按钮的状态，bFollow为真实的状态
        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }


        d_cam.Activate(s_cam);

        // 步骤3：绘制地图和图像
        // 设置为白色，glClearColor(red, green, blue, alpha），数值范围(0, 1)
        glClearColor(0.0f,0.0f,0.0f,0.0f);
        DrawCurrentCamera(Twc);

        /*
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();*/

        //if(i%10==0)  //KITTI
        if(i%10==0)  //TUM
        {
            v_Key_Twc.push_back(v_Twc[i]);

            temp_Key_Ow = Get_Ow(v_Twc[i]);

            Get_Ow(v_Twc[i]);

            v_Key_Ow.push_back(temp_Key_Ow);
        }

        temp_now_Ow = Get_Ow(v_Twc[i]);

        DrawKeyFrames(menuShowKeyFrames,menuShowGraph,v_Key_Twc,v_Key_Ow,temp_now_Ow);

        if(menuShowPoints)
            DrawMapPoints(all_points);

        cv::waitKey(mT);

        pangolin::FinishFrame();

    }

    SaveToPLYFile("output.ply",all_points);

    return 0;
}