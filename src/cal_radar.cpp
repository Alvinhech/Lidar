

#include <iostream>
#include <string>
#include <math.h>
#include <sstream>

#include <opencv2/opencv.hpp>

#define PI 3.1415926

int main(int argc, const char * argv[]) 
{

    std::ifstream file_calib("cord.txt");
    if(!file_calib.is_open())
    {
        return -1;
    }
    
    float height[3] = {0.0f};
    float width[3] = {0.0f};
    float width_left[3] = {0.0f};
    std::vector<std::vector<cv::Point3f> > racks_pts;
    std::vector<std::vector<float> > vv_angles;
    std::vector<float> v_x_angles;

    std::vector<std::string> vstr;

    int no_radar = 0;
    int line_no = 0;
    while(!file_calib.eof())
    {
        std::string str;
        std::getline(file_calib, str);
        if(str.empty()) break;

        std::stringstream sttr(str);

        std::string seril_no;
        sttr >> no_radar >>  seril_no;
        vstr.push_back(seril_no);

        std::vector<cv::Point3f> rack_pts;
        for(int i=0;i<5;i++)
        {
            cv::Point3f tmp_pt;
            sttr >> tmp_pt.x >> tmp_pt.y >> tmp_pt.z;
            std::cout<<"("<<tmp_pt.x<<","<<tmp_pt.y<<","<<tmp_pt.z<<")"<<std::endl;
            rack_pts.push_back(tmp_pt);
        }
        racks_pts.push_back(rack_pts);

        line_no++;
    }

    file_calib.close();
   

    std::ofstream file_output("CalbrationFile_Radar.txt");
    if(!file_output.is_open())
    {
        return -1;        
    }

    for(int i=0;i<line_no;i++)
    {
        float angle[5];
        //angle[4]
        angle[4] = 0.0f;
        cv::Point3f tmp;
        tmp = racks_pts[i][2] - racks_pts[i][1];
        //width
        width[i] = sqrt(tmp.x*tmp.x + tmp.y*tmp.y + tmp.z*tmp.z);

        //x_thea
        float x_thea;
        if(tmp.y>0.0f)
        {
            x_thea = 180.0f*acos(tmp.x/width[i])/PI;
        }
        else
        {
            x_thea = -180.0f*acos(tmp.x/width[i])/PI;
        }
        v_x_angles.push_back(x_thea);

        tmp = racks_pts[i][4] - racks_pts[i][0];
        //height
        height[i] = tmp.z;
        float s0 = sqrt(tmp.x*tmp.x + tmp.y*tmp.y + tmp.z*tmp.z);
        //angle[0]
        angle[0] = 180.0f*acos(height[i]/s0)/PI;

        width_left[i] = sqrt(s0*s0 - height[i]*height[i]);

        tmp = racks_pts[i][4] - racks_pts[i][1];
        float s1 = sqrt(tmp.x*tmp.x + tmp.y*tmp.y + tmp.z*tmp.z);
        //angle[1]
        angle[1] = 180.0f*asin(width_left[i]/s1)/PI;

        tmp = racks_pts[i][4] - racks_pts[i][2];
        float s2 = sqrt(tmp.x*tmp.x + tmp.y*tmp.y + tmp.z*tmp.z); 
        //angle[2]]
        angle[2] = 360.0f - 180.0f*asin((width[i] - width_left[i])/s2)/PI;

        tmp = racks_pts[i][4] - racks_pts[i][3];
        float s3 = sqrt(tmp.x*tmp.x + tmp.y*tmp.y + tmp.z*tmp.z);
        //angle[3]
        angle[3] = 360.0f - 180.0f*acos(height[i]/s3)/PI;
        
        std::vector<float> vf(&angle[0],&angle[5]);
        vv_angles.push_back(vf);

        file_output<< i <<" "<< vstr[i] << " " << height[i] << " " << width[i] << " " << width_left[i] << " ";
        for(int i=0;i<5;++i)
        {
            file_output<< angle[i] << " ";
        }
        
        file_output << racks_pts[i][4].x << " " << racks_pts[i][4].y << " " << racks_pts[i][4].z << " " << x_thea << std::endl;

    }
    
 
 
    return 0;
}
