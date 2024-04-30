#include "fast_limo/Common.hpp"
#include "fast_limo/Modules/Localizer.hpp"
#include "fast_limo/Modules/Mapper.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "fast_limo");
    ros::NodeHandle nh;

    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    fast_limo::Mapper& map = fast_limo::Mapper::getInstance();

    /*To DO:
        - update Localizer & Mapper parameters (maybe create config struct)
        - define callbacks for handling data into Localizer
        - async spinning
    */ 

   std::cout << FAST_LIMO_v << std::endl;


   return 0;

}