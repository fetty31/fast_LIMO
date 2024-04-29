#include "fast_limo/Common.hpp"
#include "fast_limo/Modules/Localizer.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "fast_limo");
    ros::NodeHandle nh;

    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();

    /*To DO:
        - declare Localizer and Mapper variables
        - define callbacks for handling data into Localizer
        - async spinning
    */ 

   std::cout << FAST_LIMO_v << std::endl;


   return 0;

}