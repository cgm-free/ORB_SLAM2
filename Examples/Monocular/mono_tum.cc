/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<unistd.h>
using namespace std;


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        ////cerr流是标准错误流，被指定与显示器关联。不经过缓冲区，直接输出给屏幕。只能在显示器输出。
        return 1;
    }
    //  ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml TUM_Dataset/rgbd_dataset_freiburg1_xyz
    //   argc=4 ：命令名+3个参数，一共4个参数
    //   argv[0]的值是 ./Examples/Monocular/mono_tum
    //   argv[1]的值是 Vocabulary/ORBvoc.tx
    //   argv[2]的值是 Examples/Monocular/TUM1.yaml
    //   argv[3]的值是 TUM_Dataset/rgbd_dataset_freiburg1_xyz
    
    // Retrieve paths to images 检索图像路径
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";//查看rgb.txt的内容，有助于理解代码 //strFile 指向rgb.txt的路径
    cout << "strFile:" << strFile << endl;// TUM_Dataset/rgbd_dataset_freiburg1_xyz/rgb.txt
    cout << "argv[0]:" << argv[0] << endl;// ./Examples/Monocular/mono_tum
    cout << "argv[1]:" << argv[1] << endl;// Vocabulary/ORBvoc.tx
    cout << "argv[2]:" << argv[2] << endl;// Examples/Monocular/TUM1.yaml
    cout << "argv[3]:" << argv[3] << endl;// TUM_Dataset/rgbd_dataset_freiburg1_xyz
    /**
     * （1）读取图片及时间戳信息：通过LoadImages函数完成，该函数把图片的路径、图片名称（string）、时间戳信息（double）分别读入三个vector容器：striFile、vstrImageFilenames、vTimestamps。
     **/
    LoadImages(strFile, vstrImageFilenames, vTimestamps);
    int nImages = vstrImageFilenames.size();//获取 vstrImageFilenames的元素个数;rgbd_dataset_freiburg1_xyz/rgb.txt记录了798张图像；nImages =798

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    /**
     * （2）创建SLAM系统：使用ORB_SLAM2::System类的构造函数创建SLAM系统，初始化了系统的各个线程，准备好处理输入的帧。
     *  ORB_SLAM2::System SLAM(字典文件ORBvoc.tx,配置文件TUM1.yaml,单目MONOCULAR,可以可视化);
     **/
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;// 统计追踪一帧耗时 (仅Tracker线程)
    vTimesTrack.resize(nImages);//vTimesTrack和vstrImageFilenames的元素个数一样  0 1 2 3 4...797

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;//开始处理序列 ...
    cout << "Images in the sequence: " << nImages << endl << endl;//序列中的图像:798

    // Main loop
    /**
     *（3）进行SLAM  // 主循环，依次追踪序列中的每一张图像
     * 使用cv::Mat来创建存储图像像素矩阵，使用cv::imread读取图片。
     * 使用了ORB_SLAM2::System中的TrackMonocular函数，输入就是：图片、对应时间戳。
     **/
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file //进入rgb文件夹读取某幅图片的数据
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],cv::IMREAD_UNCHANGED);
        //string(argv[3])   TUM_Dataset/rgbd_dataset_freiburg1_xyz
        //vstrImageFilenames  彩色图片名称  如：rgb/1305031102.175304.png
        //CV_LOAD_IMAGE_UNCHANGED = -1这个标识在新版本中已被废置
        //IMREAD_UNCHANGED = -1,如果设置，则按原样返回加载的图像（使用 alpha 通道，否则会被裁剪）。忽略 EXIF 方向。表示读取原图。
        //ni=0时，相当于是按原图读取第一张图  im = cv::imread(TUM_Dataset/rgbd_dataset_freiburg1_xyz/rgb/1305031102.175304.png,按原图读取)
        double tframe = vTimestamps[ni];//double类型的时间戳，把vector容器vTimestamps的某张[ni]图像的时间戳信息赋值给tframe

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "//cerr流是标准错误流，被指定与显示器关联。不经过缓冲区，直接输出给屏幕。只能在显示器输出。
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;//return 1代表程序异常退出
        }

#ifdef COMPILEDWITHC11//#ifdef 如果标识符COMPILEDWITHC11被定义过，就会运行后面内容
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        //steady_clock 是单调的时钟，相当于教练手中的秒表；只会增长，适合用于记录程序耗时；
        //system_clock 是系统的时钟；因为系统的时钟可以修改；甚至可以网络对时； 所以用系统时间计算时间差可能不准。
        //CLOCK_MONOTONIC：以绝对时间为准，获取的时间为系统重启到现在的时间，更改系统时间对它没有影响。   
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        // 追踪当前图像；输入：图像+时间
        SLAM.TrackMonocular(im,tframe);//输入为单目图像时的追踪器接口函数
        // 追踪完成,停止当前帧的图像计时, 并计算追踪耗时

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();//现在追踪当前图像所耗费的时间

        vTimesTrack[ni]=ttrack;
        //根据图像时间戳中记录的两张图像之间的时间和现在追踪当前图像所耗费的时间,继续等待指定的时间以使得下一张图像能够 按照时间戳被送入到SLAM系统中进行跟踪

        // Wait to load the next frame
        double T=0;//两张图像的时间间隔，注意转换一下单位
        if(ni<nImages-1) //如果  ni<798-1 即0-797张图
            T = vTimestamps[ni+1]-tframe;// T=下一帧图像的时间戳-当前图像的时间戳  ni=797时，T = vTimestamps[798]-vTimestamps[797]
        else if(ni>0) //最后一张图像798
            T = tframe-vTimestamps[ni-1];// T = vTimestamps[798]-vTimestamps[797]，这是负数吗？？？？？

        if(ttrack<T)//现在追踪当前图像所耗费的时间  <  两张图像的时间间隔
            usleep((T-ttrack)*1e6);//usleep()函数是把调用该函数的线程挂起一段时间，单位是微秒（百万分之一秒）。延迟 1e6微秒=1 秒
    }

    // Stop all threads
    /**
     * （4）停止SLAM：使用System类中的Shutdown函数停止所有线程。
     **/ 
    SLAM.Shutdown();

    // Tracking time statistics
    // 计算平均耗时
    sort(vTimesTrack.begin(),vTimesTrack.end());// 排序，从小到大
    float totaltime = 0;//跟踪时间的总和
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];//跟踪时间的总和
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;//跟踪时间中位数:
    cout << "mean tracking time: " << totaltime/nImages << endl;//平均追踪时间:

    // Save camera trajectory
    // 保存TUM格式的相机轨迹
    // 估计是单目时有尺度漂移, 而LGA GBA都只能优化关键帧使尺度漂移最小, 普通帧所产生的轨迹漂移这里无能为力, 我猜作者这样就只保存了关键帧的位姿,从而避免普通帧带有尺度漂移的位姿对最终误差计算的影响
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;//return 0 代表程序正常退出;return 1 代表程序异常退出
}

/**
 * @brief 导入图片
 * （1）读取图片及时间戳信息
 *  通过LoadImages函数完成，该函数把图片的路径、图片名称（string）、时间戳信息（double）分别读入三个vector容器：striFile、vstrImageFilenames、vTimestamps。
 * @param[in] strFile                   读入的文件名称 如：UM_Dataset/rgbd_dataset_freiburg1_xyz/rgb.txt
 * @param[in&out] vstrImageFilenames    彩色图片名称  如：rgb/1305031102.175304.png
 * @param[in&out] vTimestamps           记录时间戳<double>   如：1.30503e+09
 */
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;//读操作
    f.open(strFile.c_str());//open()成员函数的参数是一个char *类型的量，要通过调用 string 类的 c_str() 函数返回一个C风格的字符串
    // 功能：open 函数将 filename 转换为一个文件描述符，并且返回描述符数字（整型变量0~255）。返回的描述符总是在进程中当前没有打开的最小描述符。

    // skip first three lines
    // 读入的文件前三行是注释，跳过

    /** 
     * 例如rgb.txt的文件前三行为：
     *1 # color image
     *2 # file: 'rgbd_dataset_freiburg1_xyz.bag
     *3 # timestamp filename
     *4 1305031102.175304 rgb/1305031102.175304.png
     *5 1305031102.211214 rgb/1305031102.211214.png
     *6 1305031102.243211 rgb/1305031102.243211.png
     **/ 
    string s0;
    getline(f,s0);//getline()函数可读取整行，包括前导和嵌入的空格，并将其存储在字符串对象中。读取文件的第一行.f是数据流，s0是要写入的字符串名
    // cout << "s0:  "<< s0 << endl;//s0:  # color images
    getline(f,s0);//读取文件的第二行
    // cout << "s0:  "<< s0 << endl;//s0:  # file: 'rgbd_dataset_freiburg1_xyz.bag'
    getline(f,s0);//读取文件的第三行
    // cout << "s0:  "<< s0 << endl;//s0:  # timestamp filename


    while(!f.eof())//eof()函数可以帮助我们⽤来判断⽂件是否为空，抑或是判断其是否读到⽂件结尾。
    {
        string s;
        getline(f,s);
        // cout << "s:   "<< s << endl;//s:   1305031102.175304 rgb/1305031102.175304.png
        if(!s.empty())
        {
            stringstream ss; //<sstream> 定义了三个类：istringstream：流的输入；ostringstream：流的输出；stringstream：流的输入输出。    
            ss << s;  //将string类型的值放入输入流中
            double t;
            string sRGB;
            ss >> t;                				// 输出时间戳数据（double型），
            //cout<< "t:     " << t << endl;//t:     1.30503e+09
            vTimestamps.push_back(t);   			// 将时间戳数据放在vTimestamps中.push_back函数将一个新的元素加到vector的最后面，位置为当前最后一个元素的下一个元素
            ss >> sRGB;             				// 输出彩色图片名称的数据
            //cout << "sRGB:    "<< sRGB << endl;//sRGB:    rgb/1305031102.175304.png
            vstrImageFilenames.push_back(sRGB);     // 将图像名称数据放在vstrImageFilenames中
        }
    }
}
