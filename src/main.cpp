#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <sstream>
#include <iostream>
#include <string>

#include <zmq.hpp>
#include "serialization.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
#include <sys/prctl.h>
#include <sys/time.h>

static inline void delay(_word_size_t ms)
{
    while (ms>=1000)
    {
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;
using namespace cv;


#define CLUSTER_DISTANCE 21 
#define PI 3.1415926
#define N_SIZE 360 
#define VIBRATION_DISTANCE 10
#define BUFFER_SIZE 3
/*
 * 过滤掉检测区域外目标数据，只保留进入检测区域内的数据
*/
template<typename T>
void ThresholdData(std::vector<std::pair<T,T> > &data, std::vector<T>& angles, T& height, T& width, T& width_left)
{
    
    T width_right = width - width_left;

    T delt;
    if((T)PI < angles[4])
    {
        delt = -((T)N_SIZE - angles[4]);
    }
    else
    {
        delt = angles[4];
    }
    
    for(int pos=0;pos<data.size();++pos)
    {
        T i= data[pos].first - delt; 
       
        if(i<=angles[0])
        {
            data[pos].second =  data[pos].second > height/cos((T)(PI*i/180.0)) ? 0.0:data[pos].second;
        }
        else if(i>=angles[3])
        {
            data[pos].second = data[pos].second > height/cos((T)( (PI*((T)N_SIZE - i)/180.0) )) ? 0.0:data[pos].second;   
        }
        else if(i>angles[0] && i<angles[1])
        {
           data[pos].second = data[pos].second > width_left/sin((T) (PI*i/180.0)) ? 0.0:data[pos].second;   
        }
        else if(i>=angles[1] && i<=angles[2])
        {
            data[pos].second = (T)0.0 ;
        }
        else if(i>angles[2] && i<angles[3])
        {
            data[pos].second = data[pos].second > width_right/sin((T)(PI*((T)N_SIZE - i)/180.0)) ? 0.0:data[pos].second;   
        }
    }
}
/*
 * 角度取整
*/
template<typename T>
std::vector<T> process(std::vector<std::pair<T,T> > &data)
{
    int count = data.size();
   
    std::vector<T> vecT(N_SIZE);
    for(int i=0; i<N_SIZE; ++i)
    {
       vecT[i] = 0.0;
    }

    for(int i=0; i<data.size(); ++i)
    {
       vecT[(int)(data[i].first + (T)0.5)] = data[i].second;
    } 

    return vecT;
}

/*
 * 集中候选点
*/
template<typename T>
void detectHands(std::vector<T>& input_data, std::vector<int>& results)
{
    if(input_data.size() != N_SIZE)
    {
        return false;
    }

    results.clear();

    for(int i=0; i<input_data.size(); ++i)
    {
        if(abs(input_data[i] - (T)0.0) > 1e-3 )
        {
            results.push_back(i);
        }
    }
}


bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result))
    { 
        // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        //printf("1\n");
        if (healthinfo.status == RPLIDAR_STATUS_ERROR)
        {
            //printf("2\n");
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        }
        else
        {
            //printf("3\n");
            return true;   
        }
    } 
    else
    {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}


#include <signal.h>
bool ctrl_c_pressed = false;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

//k
std::vector<Point2f> Smooth(int c,std::vector<Point2f> buffer_center[],std::vector<std::vector<Point2f>> buffer_point[],std::vector<Point2f> &center,std::vector<std::vector<Point2f>> &cluster)
{
    
    int i,j,k,x,y;
    std::vector<Point2f> samecenter[center.size()];
    std::vector<int> distance[center.size()];
    std::vector<Point2f> tempcenter;
    if(c>=BUFFER_SIZE-1)
    {
        
        //for every center, get the corresponding points.
        for(i=0;i<center.size();i++)
        {
            //for every buffercluster
            for(j=1;j<BUFFER_SIZE;j++)
            {
                //every point
                for(k=0;k<buffer_center[(c+j)%BUFFER_SIZE].size();k++)
                {
                    x=buffer_center[(c+j)%BUFFER_SIZE][k].x-center[i].x;
                    y=buffer_center[(c+j)%BUFFER_SIZE][k].y-center[i].y;
                    //is the same point
                    if(x*x+y*y<=VIBRATION_DISTANCE*VIBRATION_DISTANCE)
                    {
                        samecenter[i].push_back(buffer_center[(c+j)%BUFFER_SIZE][k]);
                        distance[i].push_back(j);
                    }
                    
                }
            }
            Point2f temppoint(0,0);
            int sum=0;
            //average
            for(j=0;j<samecenter[i].size();j++)
            {
                sum+=distance[i][j];
                temppoint.x+=distance[i][j]*samecenter[i][j].x;
                temppoint.y+=distance[i][j]*samecenter[i][j].y;
            }
            temppoint.x=(temppoint.x+BUFFER_SIZE*center[i].x)/(sum+BUFFER_SIZE);
            temppoint.y=(temppoint.y+BUFFER_SIZE*center[i].y)/(sum+BUFFER_SIZE);
            tempcenter.push_back(temppoint);

        }
        buffer_center[c%BUFFER_SIZE]=tempcenter;
    }
    else if(c>0)
    {
        for(int i=0;i<center.size();i++)
        {
            for(j=1;j<=c;j++)
            {
                //every point
                for(k=0;k<buffer_center[(c-j)%BUFFER_SIZE].size();k++)
                {
                    x=buffer_center[(c-j)%BUFFER_SIZE][k].x-center[i].x;
                    y=buffer_center[(c-j)%BUFFER_SIZE][k].y-center[i].y;
                    //is the same point
                    if(x*x+y*y<=VIBRATION_DISTANCE*VIBRATION_DISTANCE)
                    {
                        samecenter[i].push_back(buffer_center[(c-j)%BUFFER_SIZE][k]);
                        distance[i].push_back(c+1-j);
                    }
                    
                }
            }
            Point2f temppoint;
            int sum=0;
            //average
            for(j=0;j<samecenter[i].size();j++)
            {
                sum+=distance[i][j];
                temppoint.x+=distance[i][j]*samecenter[i][j].x;
                temppoint.y+=distance[i][j]*samecenter[i][j].y;
            }
            temppoint.x=(temppoint.x+(c+1)*center[i].x)/(sum+c+1);
            temppoint.y=(temppoint.y+(c+1)*center[i].y)/(sum+c+1);
            tempcenter.push_back(temppoint);
        }
        buffer_center[c%BUFFER_SIZE]=tempcenter;
    }
    else if(c==0)
    {
        buffer_center[c%BUFFER_SIZE]=center;
        
    }
    buffer_point[c%BUFFER_SIZE]=cluster;
    return buffer_center[c%BUFFER_SIZE];
}



int main(int argc, const char * argv[]) 
{
  
    //std::ifstream file_calib("CalbrationFile_Radar.txt");
    std::ifstream file_calib("test.txt");
    if(!file_calib.is_open())
    {
        return -1;
    }
    
    std::vector<float> height; // h:雷达距离检测区域下沿高度
    std::vector<float> width; // w:检测区域宽度
    std::vector<float> width_left; // w_l:雷达距离检测区域左侧的横向宽度
    std::vector<std::vector<float> > vv_angles; //标定角度集合
    std::vector<cv::Point3f> v_offset; //坐标变换平移向量集合
    std::vector<std::string> vstr; //雷达序列号
    std::vector<cv::Mat> vMat; //坐标变换旋转矩阵集合
    
    // rotate: 绕x轴旋转矩阵
    cv::Mat rotate = cv::Mat::eye(3,3,CV_32FC1);
    rotate.at<float>(1,1) = -1.0f;
    rotate.at<float>(2,2) = -1.0f;

    int no_radar = -1;
    int line_no = 0;
    // h:雷达距离检测区域下沿高度
    // w:检测区域宽度
    // w_l:雷达距离检测区域左侧的横向宽度
    float h,w,w_l;
    while(!file_calib.eof())
    {
        std::string str;
        std::getline(file_calib, str);
        if(str.empty()) break;

        std::stringstream sttr(str);

        std::string seril_no;
        sttr >> no_radar >> seril_no;
        vstr.push_back(seril_no);

        sttr >> h >> w >> w_l;
        height.push_back(h);
        width.push_back(w);
        width_left.push_back(w_l);

        std::vector<float> v_angle;
        for(int i=0;i<5;i++)
        {
            float angle;
            sttr >> angle;
            v_angle.push_back(angle);

            std::cout<< angle << " ";
        }
        vv_angles.push_back(v_angle);

        float z_angle;
        cv::Point3f tmp;
        sttr >> tmp.x >> tmp.y >> tmp.z >> z_angle;
        std::cout<< tmp.x << " " << tmp.y << " " << tmp.z << " " << z_angle << std::endl;

        cv::Mat z_rotate = cv::Mat::eye(3,3,CV_32FC1);
        float a = cos(PI*z_angle/180.0f);
        float b = sin(PI*z_angle/180.0f);
        z_rotate.at<float>(0,0) = a;
        z_rotate.at<float>(0,1) = b;
        z_rotate.at<float>(1,0) = -b; 
        z_rotate.at<float>(1,1) = a;
        /*
        std::cout<<z_rotate<<std::endl;
        std::cout<<rotate<<std::endl; 
        std::cout<<z_rotate*rotate<<std::endl;    
        */ 
        vMat.push_back(rotate*z_rotate);

        v_offset.push_back(tmp);
        
        line_no++;
    }

    file_calib.close();


    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;

    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: "RPLIDAR_SDK_VERSION"\n");

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);


    if (!opt_com_path) 
    {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

  
  /**********************找寻雷达序列号对应的端口号****************************/
    int com_no = -1;
    int valid_count = 0;
    std::map<int,std::string> map_com;
    while(com_no < 10)
    {
        com_no++;

#ifdef _WIN32
        std::string opt_com_path = std::string("\\\\.\\com") + to_string(com_no);
#else
        std::string opt_com_path = std::string("/dev/ttyUSB") + std::to_string(com_no);
#endif

        // create the driver instance
        RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
        rplidar_response_device_info_t devinfo;

        if (!drv) 
        {
            fprintf(stderr, "insufficent memory, exit\n");
            com_no--;
            RPlidarDriver::DisposeDriver(drv);
            continue;
            //exit(-2);
        }

        // make connection...
        if (IS_FAIL(drv->connect(opt_com_path.c_str(), opt_com_baudrate)))
        {
            RPlidarDriver::DisposeDriver(drv);
            continue;
        }
    	
        // retrieving the device info
        op_result = drv->getDeviceInfo(devinfo);

        if (IS_FAIL(op_result)) 
        {
            fprintf(stderr, "Error, cannot get device info.\n");
            
            RPlidarDriver::DisposeDriver(drv);
            continue;
        }

        // print out the device serial number, firmware and hardware version number..
        printf("RPLIDAR S/N: ");
        char buf[32];
        int offset = 0;
        for (int pos = 0; pos < 16 ;++pos) 
        {
            int j = sprintf(buf+offset,"%02X",devinfo.serialnum[pos]);
            offset += j;
            printf("%02X", devinfo.serialnum[pos]);
        }

        std::string str_serial_no(buf);

        bool b_valid = false;
        for(int i=0;i<vstr.size();i++)
        {
            if(str_serial_no == vstr[i])
            {
                map_com[i] = opt_com_path;
                b_valid = true;
            }            
        }
      
        printf("\n"
                "Firmware Ver: %d.%02d\n"
                "Hardware Rev: %d\n"
                , devinfo.firmware_version>>8
                , devinfo.firmware_version & 0xFF
                , (int)devinfo.hardware_version);
   
        if(b_valid)
        {
            valid_count++;
        }

        RPlidarDriver::DisposeDriver(drv);
   }

    std::cout<<"Total num of valid radar is "<<valid_count<<std::endl;
    
    if(valid_count != line_no )
    {
        //std::cout<<"need to connect three radar..."<<std::endl;
        return -1;
    }  



/**********************开启多进程****************************/
    
    pid_t fpid; //fpid表示fork函数返回的值  
    int i=-1;
    for(i=0; i<valid_count; i++)
    {
        fpid=fork(); 
        if(fpid == 0 | fpid == -1)
        {
            break;
        }
    }

    signal(SIGINT, ctrlc);

    if (fpid < 0)   
        printf("error in fork!");   
    else if (fpid == 0) {
        
        int id_radar = i;
        int id_p = getpid();
        printf("I am the child process, my process id is %d\n",id_p);   

        zmq::context_t context (1);
        zmq::socket_t publisher (context, ZMQ_PUB);
        publisher.connect("ipc:///tmp/radar_results");
    
        std::stringstream sttr_win;
        sttr_win<<id_radar;
        std::string str_win = sttr_win.str();
       
        
        // create the driver instance

        RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
        rplidar_response_device_info_t devinfo;

        
        if (!drv) 
        {
            fprintf(stderr, "insufficent memory, exit\n");
            exit(-2);
        }
        
        // make connection...
        if (IS_FAIL(drv->connect(map_com[id_radar].c_str(), opt_com_baudrate)))
        {
            fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_com_path);
            goto on_finished;
        }

        
        // retrieving the device info
        op_result = drv->getDeviceInfo(devinfo);

        if (IS_FAIL(op_result)) 
        {
            fprintf(stderr, "Error, cannot get device info.\n");
            goto on_finished;
        }

        // check health...
        if (!checkRPLIDARHealth(drv)) 
        {
            //printf("1\n");
            goto on_finished;
        }

        std::vector<float> vp_after(N_SIZE);
        std::vector<float> vp_last_after(N_SIZE);
      
        struct timeval tv;
        struct timezone tz;
        gettimeofday(&tv,&tz);
        drv->startMotor();
        // start scan...
        drv->startScan();
        //set buffer
        int c=0;
        std::vector<Point2f> buffer_center[BUFFER_SIZE];
        std::vector<std::vector<Point2f>> buffer_point[BUFFER_SIZE];
        // fetech result and print it out...
        while (1) 
        {  
            //printf("1\n");
            rplidar_response_measurement_node_t nodes[N_SIZE*2];
            size_t  count = _countof(nodes);
             
            op_result = drv->grabScanData(nodes, count);

            gettimeofday(&tv,&tz);
            //std::cout << op_result;
            if (IS_OK(op_result)) 
            {
                //printf("1\n");
                std::vector<std::pair<float,float> > vp;
                
                drv->ascendScanData(nodes, count);
               
                for (int pos = 0; pos < (int)count ; ++pos) 
                { 
                    vp.push_back(std::pair<float,float>((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f, 
                                                        nodes[pos].distance_q2/40.0f)); 
                }
                //filter out things not in area
                ThresholdData<float>(vp, vv_angles[id_radar], height[id_radar], width[id_radar], width_left[id_radar]);
                float h1=width_left[id_radar]/tan((float)(PI*vv_angles[id_radar][1]/180.0));
                //aline, get the length of each angle 
                vp_after = process<float>(vp);

                std::vector<int> results;
                //detect obeject, not necessarily hand
                detectHands(vp_after,results);
                //2d coordinate
                std::vector<cv::Point2f> vPoints;
                for(int i=0; i<results.size(); ++i)
                {
                    float len = vp_after[results[i]];
                    float x,z;
                    if(results[i]<=vv_angles[id_radar][1])
                    {
                        x = -( len*sin(((float)results[i])*PI/180.0) );
                        z = len*cos( ((float)results[i])*PI/180.0 );
                    }
                    else
                    {
                        x = len*sin( ((float)N_SIZE - (float)results[i])*PI/180.0 );
                        z = len*cos( ((float)N_SIZE - (float)results[i])*PI/180.0 );
                    }

                    vPoints.push_back(cv::Point2f(x,z));
                }
                //
                int class_count = 0;
                //how many point each object contains
                std::vector<int> v_per_class_count;
                //flag
                std::vector<int> vb;
                for(int i=0;i<vPoints.size();++i)
                {
                    vb.push_back(-1);
                }
                //detect nothing
                if(vPoints.size() == 0)
                {
                    continue;
                }
                //only 1 point
                else if(vPoints.size() == 1)
                {
                    vb[0] = 0;
                    class_count = 1;
                }
                //classification based on CLUSTER_DISTANCE
                else
                {
                    for(int i=0; i<vPoints.size()-1 ; ++i)
                    { 
                        if(vb[i] == -1)
                            vb[i] = class_count;
                        cv::Point2f pts = vPoints[i] - vPoints[i+1];
                        if( (pts.x*pts.x + pts.y*pts.y) < (float)CLUSTER_DISTANCE*(float)CLUSTER_DISTANCE  )
                        {
                            vb[i+1] = vb[i];
                        }
                        else 
                        {
                            vb[i+1] = ++class_count;
                        }
                    }
                    class_count++;
                }
                //center of each object
                std::vector<cv::Point2f> vPoints_cluster;
                int class_total = class_count;
                int per_counts = 0;
                class_count = 0;
                cv::Point2f pt_sum(0.0,0.0);
                //find center of each object
                for(int i=0; i<vPoints.size(); ++i)
                {
                    if(vb[i] == class_count)
                    {
                        pt_sum += vPoints[i];
                        per_counts++;
                    }
                    else if(vb[i] != class_count)
                    {
                        pt_sum.x /= per_counts;
                        pt_sum.y /= per_counts;
                        vPoints_cluster.push_back(pt_sum);
                        v_per_class_count.push_back(per_counts);

                        pt_sum = vPoints[i];
                        per_counts = 1;
                        class_count++;
                    }
                    if(i == vPoints.size() - 1)
                    {
                        pt_sum.x /= per_counts;
                        pt_sum.y /= per_counts;
                        vPoints_cluster.push_back(pt_sum);
                        v_per_class_count.push_back(per_counts);
                        class_count++;
                    }
                    
                }
                //storage of different points in each object
                std::vector<std::vector<cv::Point2f> > vpp(vPoints_cluster.size());
                for(int i=0; i<vPoints.size(); ++i)
                {
                    std::cout<<i<<":"<<vPoints[i]<<","<<vb[i]<<std::endl;
                    vpp[vb[i]].push_back(vPoints[i]);
                }
                
                //smooth 
                std::vector<Point2f> tempcenter;
                tempcenter=Smooth(c,buffer_center,buffer_point,vPoints_cluster,vpp);
                vPoints_cluster.swap(tempcenter);
                //std::cout<<"origin: ("<<vPoints_cluster[0].x<<","<<vPoints_cluster[0].y<<")  \n changed:  ("<<buffer_center[c%BUFFER_SIZE][0].x<<","<<buffer_center[c%BUFFER_SIZE][0].y<<")"<<std::endl;
                
                /*
                if(vPoints_cluster.size()>1)
                    std::cout<<"origin: ("<<vPoints_cluster[1].x<<","<<vPoints_cluster[1].y<<")   changed:  ("<<buffer_center[c%BUFFER_SIZE][1].x<<","<<buffer_center[c%BUFFER_SIZE][1].y<<")"<<std::endl;
                
                */
                //paint
                Mat picture(300,300,CV_8UC3,Scalar(255,255,255)); 
                circle(picture,Point(width_left[id_radar],0),10,Scalar(0,0,0));
                Point P0=Point(0,h1);
                Point P2=Point(width[id_radar],height[id_radar]);
                rectangle(picture,P0,P2,Scalar(0,0,0));
                for(int i=0; i<vPoints_cluster.size(); i++)
                {
                    if(v_per_class_count[i] < 2)
                        continue;
                    for(int j=0;j<vpp[i].size()-1;j++)
                    {
                        line(picture,Point(vpp[i][j].x+width_left[id_radar],vpp[i][j].y),Point(vpp[i][j+1].x+width_left[id_radar],vpp[i][j+1].y),Scalar(0,0,0));
                    }
                    
                    
                }
                imshow("picture",picture);
                //std::cout<< "high1:"<<height[id_radar]<<"    high2:"<<h1<<"     width"<<width[id_radar]<<"       wl:"<<width_left[id_radar]<<endl;
                waitKey(0); 
                
                //radar number, timeval, timezone
                Radar_Results radar_results;
                radar_results.id_radar = id_radar;
                radar_results.time_stamp_sec = tv.tv_sec;
                radar_results.time_stamp_usec = tv.tv_usec;
                int valid_count = 0;
                for(int i=0; i<vPoints_cluster.size(); ++i)
                {
                    
                    float x = vPoints_cluster[i].x;
                    float z = vPoints_cluster[i].y;
                    std::cout<<"("<<x<<","<<z<<")"<<std::endl;
                    //coordinate info of object's center
                    std::pair<float,cv::Point3f> p_info;
                    //coordinate transformation
                    cv::Mat pt_l(3,1,CV_32FC1);
                    pt_l.at<float>(0,0) = x; 
                    pt_l.at<float>(1,0) = 0.0f;
                    pt_l.at<float>(2,0) = z; 
                    cv::Mat m_wrd = vMat[id_radar]*pt_l;
                    cv::Point3f pt_tmp(m_wrd.at<float>(0,0), m_wrd.at<float>(1,0), m_wrd.at<float>(2,0));
                    cv::Point3f pt_wrd = pt_tmp + v_offset[id_radar];
                    //std::cout<<"("<<pt_wrd.x<<","<<pt_wrd.y<<","<<pt_wrd.z<<")"<<std::endl;





                    p_info.first = 1.0f;
                    p_info.second = pt_wrd;
                    radar_results.results.push_back(p_info);

                    cv::RotatedRect rbox = minAreaRect(cv::Mat(vpp[i]));
                    //
                    cv::Rect brect = rbox.boundingRect();
                    
                    //std::cout<<brect.x<<","<<brect.y<<","<<brect.width<<","<<brect.height<<std::endl;
                    
                    radar_results.numPts_per_class.push_back(vpp[i].size());
                    radar_results.vrect.push_back(brect);
                   
                    valid_count++;
                }

                std::cout<<"---------------------------"<<std::endl;

                radar_results.nums_results = valid_count;            

                std::ostringstream os;  
		        boost::archive::text_oarchive oa(os);  
		        oa << radar_results;						
		        std::string content = "LADAR$" + os.str();//content保存了序列化后的数据。  

                zmq::message_t message(content.size());
		        memcpy(message.data(), content.c_str() , content.size());
                publisher.send(message);
            
                if (ctrl_c_pressed){ 
                std::cout<<"process "<<id_p<<" exit..."<<std::endl;
                break;   
                }
            }  
            c++;  
        
        }
        drv->stop();
        drv->stopMotor();
        // done!
on_finished:
        RPlidarDriver::DisposeDriver(drv);

        std::cout<<"child "<<id_radar<<" kill..."<<std::endl;
    } 
    else {

        printf("I am the parent process, my process id is %d\n",getpid());   
        while(!ctrl_c_pressed)
        {
            sleep(1);
        }

        std::cout<<"parent exit..."<<std::endl;
    }   
  
    return 0;
}




