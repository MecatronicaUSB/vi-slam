#include "../include/DataReader.hpp"
#include <cmath>
DataReader::DataReader(){

}
DataReader::DataReader(string image_path, string imu_path, string gt_path, char separator){
    setProperties(image_path, imu_path, gt_path, separator);
}

void DataReader::setProperties(string image_path, string imu_path, string gt_path, char separator){
    imageReader.setPath(image_path);
    
    imageReader.searchImages();
    imageReader.computeTimeStep();

    imuReader.setFileProperties(imu_path, separator);
    imuReader.getDataFromFile();
    imuReader.computeTimeStep();

    gtReader.setFileProperties(gt_path, separator);
    gtReader.getDataFromFile();
    gtReader.computeTimeStep();

    imageIndex0 = 0;
    imuIndex0 = 0;
    gtIndex0 = 0;
    double timeFirstImage = imageReader.getImageTime(imageIndex0);
    double timeFirstLineImu = imuReader.getGroundTruthData(imuIndex0, 0);
    double timeFirstLineGt= gtReader.getGroundTruthData(gtIndex0, 0);

    // algoritmo para sincronizar el groundtruthm, imu y camara

    double delta1;
    double delta2;
    delta1 = timeFirstImage-timeFirstLineGt;
    delta2 = timeFirstImage-timeFirstLineImu;
    // encontrar el tiempo de la imagen mas cercana a los
    // tiempo de la imu y gt
    while ((delta1<0.0) || (delta2 < 0.0))
    {
        imageIndex0++;
        timeFirstImage = imageReader.getImageTime(imageIndex0);
        delta1 = timeFirstImage-timeFirstLineGt;
        delta2 = timeFirstImage-timeFirstLineImu;  
    }

    // Encontrar el indice del dato de gt mas cercano a la imagen
    delta1 = timeFirstLineGt-timeFirstImage;
    while(delta1<0.0)
    {
        gtIndex0++;
        timeFirstLineGt= gtReader.getGroundTruthData(gtIndex0, 0);
        delta1 = timeFirstLineGt-timeFirstImage;
    }

     // Encontrar el indice del dato de imu mas cercano a la imagen
    delta2 = timeFirstLineImu-timeFirstImage;
    while(delta2<0.0)
    {
        imuIndex0++;
        timeFirstLineImu= imuReader.getGroundTruthData(imuIndex0, 0);
        delta2 = timeFirstLineImu-timeFirstImage;
    }
    // 

    if ( delta2 == 0){
        cout<<"Imu sincronizada con Camara..."<<endl;
    }
    else{
        cout<<"Imu no sincronizada con Camara...\n\t-> Se tomara la medida más cercana en tiempo"<<endl;
    }
    if ( delta1 == 0){
        cout<<"Groundtruth sincronizada con Camara"<<endl;
    }
    else{
        cout<<"Groundtruth no sincronizada con Camara...\n\t-> Se tomara la medida más cercana en tiempo"<<endl;
    }


    cout<<"index camera = "<<imageIndex0<<endl;
    cout<<"index imu = "<<imuIndex0<<endl;
    cout<<"index get ="<<gtIndex0<<endl;



    timeStepCamara = imageReader.TimeStep;
    timeStepImu = imuReader.TimeStep;
    timeStepGt = gtReader.TimeStep;

     // Encontrar el indice de la ultima imagen permitida
    double timelastLineGt= gtReader.getGroundTruthData(gtReader.getRows()-2, 0);//Ultimo tiempo de gt
    initialTime = timeFirstImage;
    indexLastData = static_cast<int> (floor((timelastLineGt-initialTime)/timeStepCamara) - imageIndex0);
    lastTime = (indexLastData*timeStepCamara)/1000000;
    cout << "last time gt = " << timelastLineGt<<endl;
    cout <<" Duración data set con gt = " << lastTime<< " ms"<<endl;
    cout <<" Numero de imágenes con gt = "<< indexLastData<<endl;
    

}

void DataReader::UpdateDataReader(int index){
    image1 = imageReader.getImage(imageIndex0+index);
    image2 = imageReader.getImage(imageIndex0+index+1);
    // Localizacion de los datos de IMU y GroundTruth correspondientes a la imagen actual
    int scalerImuCamara = static_cast<int> (floor((imageReader.TimeStep/imuReader.TimeStep)*index));
    int scalerGtCamara = static_cast<int> (floor((imageReader.TimeStep/gtReader.TimeStep)*index));

    int indexGtReader;
    int indexImuReader;

    indexGtReader = scalerGtCamara+gtIndex0;
    indexImuReader = scalerImuCamara+imuIndex0;

    double timeImage = imageReader.getImageTime(imageIndex0+index); // poner en clase
    double timeLineImu = imuReader.getGroundTruthData(indexImuReader, 0);
    double timeLineGt= gtReader.getGroundTruthData(indexGtReader, 0);
    double delta1;
    double delta2;
    // Encontrar el indice del dato de gt mas cercano a la imagen
    delta1 = timeLineGt-timeImage;
    while(delta1<0.0)
    {
        indexGtReader++;
        timeLineGt= gtReader.getGroundTruthData(indexGtReader, 0);
        delta1 = timeLineGt-timeImage;
    }
     // Encontrar el indice del dato de imu posterior a la imagen1    
    
    delta2 = timeLineImu-timeImage;
    while(delta2<=0.0)
    {
        indexImuReader++;
        timeLineImu= imuReader.getGroundTruthData(indexImuReader, 0);
        delta2 = timeLineImu-timeImage;
    }
     //cout <<"ImuIndex"<<indexImuReader<<endl;
     //cout <<"GtIndex"<<indexGtReader<<endl;
        
    double timeImuReader = imuReader.getGroundTruthData(indexImuReader, 0);
    Point3d angularVelocity;
    Point3d acceleration;

    double timeGtReader = gtReader.getGroundTruthData(indexGtReader, 0);
    Point3d position;
    Quaterniond quaternion;
    // Limpiar vectores
     imuAngularVelocity.clear();
     imuAcceleration.clear();
     gtPosition.clear();
     gtQuaternion.clear();
    // Obtener todos los datos entre las dos imagenes
    while(timeImuReader<=static_cast<double>(imageReader.getImageTime(imageIndex0 +index+1)))
    {
        angularVelocity.x =  imuReader.getGroundTruthData(indexImuReader, 1);
        angularVelocity.y =  imuReader.getGroundTruthData(indexImuReader, 2);
        angularVelocity.z =  imuReader.getGroundTruthData(indexImuReader, 3);

        acceleration.x = imuReader.getGroundTruthData(indexImuReader, 4);
        acceleration.y = imuReader.getGroundTruthData(indexImuReader, 5);
        acceleration.z = imuReader.getGroundTruthData(indexImuReader, 6);
        
        imuAngularVelocity.push_back(angularVelocity);
        imuAcceleration.push_back(acceleration);

        indexImuReader++;
        timeImuReader = imuReader.getGroundTruthData(indexImuReader, 0);
    }


    while(timeGtReader<=static_cast<double>(imageReader.getImageTime(imageIndex0 +index+1)))
    {
        position.x = gtReader.getGroundTruthData(indexGtReader, 1);
        position.y = gtReader.getGroundTruthData(indexGtReader, 2);
        position.z = gtReader.getGroundTruthData(indexGtReader, 3);

        quaternion.w = gtReader.getGroundTruthData(indexGtReader, 4);
        quaternion.x = gtReader.getGroundTruthData(indexGtReader, 5);
        quaternion.y = gtReader.getGroundTruthData(indexGtReader, 6);
        quaternion.z = gtReader.getGroundTruthData(indexGtReader, 7);//w
        gtPosition.push_back(position);
        gtQuaternion.push_back(quaternion);

        indexGtReader++;
        timeGtReader = gtReader.getGroundTruthData(indexGtReader, 0);
    }

    //cout <<"ImuIndex2 "<<indexImuReader<<endl;
    //cout <<"GtIndex2 "<<indexGtReader<<endl;

    currentTimeMs = (imageReader.getImageTime(imageIndex0 +index+1)-initialTime)/1000000.0;


}

void DataReader::UpdateImu(int index, int n_measures)
{
    imuAngularVelocity.clear();
    imuAcceleration.clear();
    Point3d angularVelocity;
    Point3d acceleration;

    int indexImuReader;
    indexImuReader = index;
    for (int i = 0; i< n_measures;i++)
    {
        angularVelocity.x =  imuReader.getGroundTruthData(indexImuReader, 1);
        angularVelocity.y =  imuReader.getGroundTruthData(indexImuReader, 2);
        angularVelocity.z =  imuReader.getGroundTruthData(indexImuReader, 3);

        acceleration.x = imuReader.getGroundTruthData(indexImuReader, 4);
        acceleration.y = imuReader.getGroundTruthData(indexImuReader, 5);
        acceleration.z = imuReader.getGroundTruthData(indexImuReader, 6);
    
        imuAngularVelocity.push_back(angularVelocity);
        imuAcceleration.push_back(acceleration); 
        indexImuReader++;

    }
    
}