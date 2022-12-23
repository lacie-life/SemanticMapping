#include <random>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "semanticBase/include/yolo.h"

using namespace std;
using namespace semanticSLAM;

int main(int argc, char **argv) {

    std::cout << "Hello world !!!" << std::endl;

    cv::Mat image = imread(argv[1],
                       cv::IMREAD_ANYCOLOR);

    Yolo model = Yolo(argv[2], argv[3]);

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
