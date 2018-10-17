void System::InitializeSystem(string _images_path, string _ground_truth_dataset, string _ground_truth_path, string _depth_path) {
    // Check if depth images are available
    if (_depth_path != "")
        depth_available_ = true;

    // Add list of the dataset images names
    AddLists(_images_path, _depth_path);\
    
    if (start_index_>images_list_.size()) {
        cout << "The image " << start_index_ << " doesn't exist." << endl;
        cout << "Exiting..." << endl;
        exit(0);
    }
    // Obtain parameters of camera_model
    K_ = camera_model_->GetK();
    w_input_ = camera_model_->GetInputHeight();
    h_input_ = camera_model_->GetInputWidth();
    map1_ = camera_model_->GetMap1();
    map2_ = camera_model_->GetMap2();
    fx_ = camera_model_->GetK().at<float>(0,0);
    fy_ = camera_model_->GetK().at<float>(1,1);
    cx_ = camera_model_->GetK().at<float>(0,2);
    cy_ = camera_model_->GetK().at<float>(1,2);
    distortion_valid_ = camera_model_->IsValid();

    // Obtain ROI for distorted images
    if (distortion_valid_)
        CalculateROI();

    // Initialize tracker system
    tracker_ = new Tracker(depth_available_);
    tracker_->InitializePyramid(w_, h_, K_);
    tracker_->InitializeMasks();

    // Initialize map
    map_ = new Map();

    // Initialize output visualizer
    ground_truth_path_    = _ground_truth_path;
    ground_truth_dataset_ = _ground_truth_dataset;
    visualizer_ = new Visualizer(start_index_, images_list_.size(), K_, _ground_truth_dataset, _ground_truth_path);

    // Cheking if the number of depth images are greater or lower than the actual number of images
    if (depth_available_) {
        if (images_list_.size() > depth_list_.size())
            num_valid_images_ = depth_list_.size();
        if (images_list_.size() <= depth_list_.size())
            num_valid_images_ = images_list_.size();
    } else {
        num_valid_images_ = images_list_.size();
    }

    initialized_ = true;
    cout << "Initializing system ... done" << endl << endl;
}

void System::CalculateROI() {
    // Load first image
    Mat distorted, undistorted;
    distorted = imread(images_list_[0], CV_LOAD_IMAGE_GRAYSCALE);
    remap(distorted, undistorted, map1_, map2_, INTER_LINEAR);

    // Find middle x and y of image (supposing a symmetrical distortion)
    int x_middle = (undistorted.cols - 1) * 0.5;
    int y_middle = (undistorted.rows - 1) * 0.5;
    
    Point p1, p2;    
    p1.x = 0;
    p1.y = 0;
    p2.x = undistorted.cols - 1;
    p2.y = undistorted.rows - 1;

    // Search x1_ROI distance to crop
    while (undistorted.at<uchar>(y_middle, p1.x) == 0)
        p1.x++;

    // Search x2_ROI distance to crop
    while (undistorted.at<uchar>(y_middle, p2.x) == 0)
        p2.x--;

    // Search y1_ROI distance to crop
    while (undistorted.at<uchar>(p1.y, x_middle) == 0)
        p1.y++;

    // Search y2_ROI distance to crop
    while (undistorted.at<uchar>(p2.y, x_middle) == 0)
        p2.y--;

    // Considering an error margin
    p1.x += 5;
    p2.x -= 5;
    p1.y += 5;
    p2.y -= 5;

    ROI = Rect(p1,p2);
    
    // Update w_ and h_ with ROI dimentions
    w_ = p2.x - p1.x;
    h_ = p2.y - p1.y;
}