#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <opencv2/opencv.hpp>

class Mat_fat 
{
public:
		int id_cam;
		long long time_stamp;
		cv::Mat frame;
};

class Radar_Results 
{
public:
		int id_radar;
		long time_stamp_sec;
        long time_stamp_usec;
        int nums_results;
        std::vector<std::pair<float,cv::Point3f> > results;
        std::vector<int> numPts_per_class;
        std::vector<cv::Rect> vrect;
};

class Cams_Status
{

friend class boost::serialization::access;  

    template<class Archive>  
    void serialize(Archive& ar, const unsigned int version)  
    {  
        ar & id_cam;  
        ar & b_sys_ready;
        ar & b_preFor_sys;
        ar & b_child_end;
    }

public:
		int get_camID(){ return id_cam; }
		bool Is_sys_ready(){ return b_sys_ready; }
		bool Is_preFor_sys(){ return b_preFor_sys; }
		bool Is_childProcess_end() { return b_child_end; }
public:
		int id_cam;
		bool b_sys_ready = false;
		bool b_preFor_sys = false;
		bool b_child_end = false;

};

using namespace std;
namespace boost {
		namespace serialization {
				template<class Archive>
        void serialize(Archive &ar, Mat_fat& mat_f, const unsigned int)
        {
            int cols, rows, type;
            bool continuous;

						int id_cam;
						long long time_stamp;

            if (Archive::is_saving::value) {
                cols = mat_f.frame.cols; rows = mat_f.frame.rows; type = mat_f.frame.type();

								id_cam = mat_f.id_cam;
								time_stamp = mat_f.time_stamp;

                continuous = mat_f.frame.isContinuous();
            }

            ar & cols & rows & type & continuous & id_cam & time_stamp ;

            if (Archive::is_loading::value)
						{
								mat_f.id_cam = id_cam;
								mat_f.time_stamp = time_stamp;
                mat_f.frame.create(rows, cols, type);
						}
            if (continuous) {
                const unsigned int data_size = rows * cols * mat_f.frame.elemSize();
                ar & boost::serialization::make_array(mat_f.frame.ptr(), data_size);
            }
            else {
                const unsigned int row_size = cols*mat_f.frame.elemSize();
                for (int i = 0; i < rows; i++) {
                    ar & boost::serialization::make_array(mat_f.frame.ptr(i), row_size);
                }
            }

        }

    }
}


using namespace std;
namespace boost {
		namespace serialization {
				template<class Archive>
        void serialize(Archive &ar, Radar_Results& radar_results, const unsigned int)
        {
            int nums_results,id_radar;
			long time_stamp_sec;
			long time_stamp_usec;
            std::vector<float> vf;
            std::vector<int> vi;
            std::vector<int> vr_int;
            std::vector<cv::Rect> vr;

            if (Archive::is_saving::value) {

                nums_results = radar_results.nums_results;
				id_radar = radar_results.id_radar;
                time_stamp_sec = radar_results.time_stamp_sec; 
                time_stamp_usec = radar_results.time_stamp_usec; 

                vr = radar_results.vrect;
                for(int i=0;i<nums_results;++i)
                {
                   vf.push_back(radar_results.results[i].first);
                   vf.push_back(radar_results.results[i].second.x); 
                   vf.push_back(radar_results.results[i].second.y); 
                   vf.push_back(radar_results.results[i].second.z); 
                   vi.push_back(radar_results.numPts_per_class[i]);
                   vr_int.push_back(vr[i].x);
                   vr_int.push_back(vr[i].y);
                   vr_int.push_back(vr[i].width);
                   vr_int.push_back(vr[i].height);
                }
            } 
            
            ar & id_radar & time_stamp_sec & time_stamp_usec & nums_results & vf & vi & vr_int;
            
            if (Archive::is_loading::value){

                                radar_results.id_radar = id_radar;
								radar_results.time_stamp_sec = time_stamp_sec;
								radar_results.time_stamp_usec = time_stamp_usec;
								radar_results.nums_results = nums_results;  
                                int k=0;
                                for(int i=0;i<nums_results;++i)
                                {
                                    std::pair<float ,cv::Point3f> p_info;
                                    cv::Point3f pts(vf[i*4+1],vf[i*4+2],vf[i*4+3]);
                                    
                                    p_info.first = vf[i*4];
                                    p_info.second = pts;
                                    
                                    radar_results.results.push_back(p_info);

                                    cv::Rect ret(vr_int[k],vr_int[k+1],vr_int[k+2],vr_int[k+3]);
                                    k += 4;
                                    vr.push_back(ret);
                                }

                               radar_results.numPts_per_class = vi;
                               radar_results.vrect = vr;
            }
        }
        
    }
}







