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
        void UpdateDataReader(int index); // Funcion encarga de actualizar todos los datos actuales del sistema y sincronizarlos        

        // Datos de imu actuales entre la imagen actual y la posterior
        vector<Point3d> imuAngularVelocity;
        vector<Point3d> imuAcceleration;
        // Imagen actual
        Mat image1;
        // Imagen siguiente
        Mat image2;
        // Variables groundtruth
        vector<Point3d> gtPosition;
        vector<Quaterniond> gtQuaternion;

        // Indices de archivos sincronizados
        int imageIndex0;
        int gtIndex0;
        int imuIndex0;
        double timeStepImu;
        double timeStepCamara;
        double timeStepGt;



        
    private:
        ImageReader imageReader;
        GroundTruth gtReader;
        GroundTruth imuReader;
        
        
       
        

};

