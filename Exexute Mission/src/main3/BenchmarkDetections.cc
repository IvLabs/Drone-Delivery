#include <boost/filesystem.hpp>

#include "opencv2/opencv.hpp"

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

void PrintHelp() {
    std::cout << "BenchmarkDetections:" << std::endl;
    std::cout << "\tUsage: [Tag Family] [Image Directory]" << std::endl;
    
}

int main( int argc, char* argv[] ) {

    if( argc < 3 ) {
        PrintHelp();
        exit(-1);
    }

    AprilTags::TagDetector::Ptr detector;
    AprilTags::TagCodes tagCodes;
    
    std::string tagFamily( argv[1] );
    if( tagFamily == "16h5" ) {
        tagCodes = AprilTags::tagCodes16h5;
    } else if( tagFamily == "25h7" ) {
        tagCodes = AprilTags::tagCodes25h7;
    } else if( tagFamily == "25h9" ) {
        tagCodes = AprilTags::tagCodes25h9;
    } else if( tagFamily == "36h9" ) {
        tagCodes = AprilTags::tagCodes36h9;
    } else if( tagFamily == "36h11" ) {
        tagCodes = AprilTags::tagCodes36h11;
    } else {
        std::cout << "Error: Tag family " << tagFamily << " is not supported!." << std::endl;
    }

    detector = std::make_shared<AprilTags::TagDetector>( tagCodes );

    // Get all images in the path
    boost::filesystem::path imagePath( argv[2] );
    boost::filesystem::directory_iterator dIter( imagePath );
    boost::filesystem::direcotry_iterator dEnd();

    std::vector<cv::Mat> images;
    for( ; dIter != dEnd; dIter++ ) {
        boost::filesystem::directory_entry entry = *dIter;
        std::cout << "Found file " << entry.path << std::endl;
        if( entry.path.extension() == ".png" ) {
            cv::Mat image( entry.path );
            images.push_back(image);
        }
    }
    
}