#include "GroundTruth.hpp"
#include "ImageReader.hpp"

using namespace std;

class DataReader{
    public:
        DataReader();
        DataReader(string image_path, string imu_path, string gt_path, char separator);
        void setProperties(string image_path, string imu_path, string gt_path, char separator);
        void getImage(int index, Mat image);
        void getImuMeasure(int index, vector<Point3d> &w_imu, vector<Point3d> &acc_imu);
        void getGroundTruth(int index, vector<Point3d> position_gt, vector<vector<double>[4]> quaternion_gt);

    private:
        char separator;
        ImageReader imageReader;
        GroundTruth groundTruthReader;
        GroundTruth imuReader;
        

};