#include <random>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "sematicSLAM/sematicBase/yolo.h"

using namespace std;
using namespace semanticSLAM;

int main(void) {

    std::cout << "Hello world !!!" << std::endl;
    // Read the image file as
    // imread("default.jpg");
    cv::Mat image = imread("/home/jun/Github/SematicSLAM/data/test.jpg",
                       cv::IMREAD_ANYCOLOR);

    Yolo model = Yolo("/home/jun/Github/SematicSLAM/models/yolov5s.torchscript.pt", "/home/jun/Github/SematicSLAM/models/coco.names");

    // Error Handling
    if (image.empty()) {
        cout << "Image File "
             << "Not Found" << endl;
        // wait for any key press
        cin.get();
        return -1;
    }

    cv::Mat result = model.drawObject(image);

    // Show Image inside a window with
    // the name provided
    cv::imshow("Window Name", result);

    // Wait for any keystroke
    cv::waitKey(0);

    return 1;
}
