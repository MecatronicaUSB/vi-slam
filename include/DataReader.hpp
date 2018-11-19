#include "GroundTruth.hpp"
#include "ImageReader.hpp"
#include "Plus.hpp"

using namespace std;

using namespace cv;

class DataReader{
    public:
        DataReader();
        DataReader(string image_path, string imu_path, string gt_path, char separator);
        void setProperties(string image_path, string imu_path, string gt_path, char separator);
        void UpdateDataReader(int index, int index2); // Funcion encarga de actualizar todos los datos actuales del sistema y sincronizarlos        
        void setGtOffset(int gt_offset);
        void UpdateImu(int index, int n_measures); // Adquiere solo la data de la imu
        // Datos de imu actuales entre la imagen actual y la posterior
        vector<Point3d> imuAngularVelocity;
        vector<Point3d> imuAcceleration;

        // Imagen actual
        Mat image1;
        // Imagen siguiente
        Mat image2;
        // Datos del groundtruth
        vector<Point3d> gtPosition, gtLinearVelocity;
        vector<Quaterniond> gtQuaternion;
        vector<Point3d> gtRPY;
        vector<Point3d> accBias;
        Point3d angBias;
        double currentTimeMs;
        double initialTime;
        double lastTime;
        // Indices de archivos sincronizados
        int imageIndex0;
        int gtIndex0;
        int imuIndex0;
        double timeStepImu;
        double timeStepCamara;
        double timeStepGt;
        int indexLastData;



        
    private:
        ImageReader imageReader;
        GroundTruth gtReader;
        GroundTruth imuReader;
        
        
       
        

};

