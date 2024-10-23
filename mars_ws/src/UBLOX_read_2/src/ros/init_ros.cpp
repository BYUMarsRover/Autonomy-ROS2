#include <UBLOX/ublox_ros.h>

namespace ublox_ros
{
    void UBLOX_ROS::initBase()
    {
        std::cerr<<"Initializing Base\n";

        //Get base parameters
        

        std::string base_type;
        int surveytime;
        int surveyacc;
        double base_lat;
        double base_lon;
        double base_alt;

        this->get_parameter<std::string>("base_type", base_type);
        this->get_parameter<int>("Surveytime", surveytime); //Surveyed base survey time
        this->get_parameter<int>("Surveyacc", surveyacc); //Surveyed base accuracy
        this->get_parameter<double>("base_lat", base_lat); //Fixed base latitude (deg)
        this->get_parameter<double>("base_lon", base_lon); //Fixed base longitude (deg)
        this->get_parameter<double>("base_alt", base_alt); //Fixed base altitude (m above ellipsoid)
        std::cerr << "base_type = " << base_type << "\n";
        std::cerr << "surveytime = " << surveytime << "\n";
        std::cerr << "surveyacc = " << surveyacc << "\n";
        std::cerr << "base_lat = " << base_lat << "\n";
        std::cerr << "base_lon = " << base_lon << "\n";
        std::cerr << "base_alt = " << base_alt << "\n";


        //Initialize local arrays to contain parameters from xml file
        std::string* local_host = new std::string[std::max(1, rover_quantity_)];
        uint16_t* local_port = new uint16_t[std::max(1, rover_quantity_)];

        //Initialize rover arrays to contain parameters from xml file
        std::string* rover_host = new std::string[std::max(1, rover_quantity_)];
        uint16_t* rover_port = new uint16_t[std::max(1, rover_quantity_)];
        
        //Account for the case when no numbers are used for the first rover.
        uint8_t j = 0;
        // if(nh_private_.hasParam("local_host")) {                 //TODO: add in this logic for ROS2 implementation
        //The first local host corresponds to the first rover.

        this->get_parameter<std::string>("local_host", local_host[0]);
        this->get_parameter<uint16_t>("local_port", local_port[0]);
        
        
        //First rover
        
        this->get_parameter<std::string>("rover_host", rover_host[0]);
        this->get_parameter<uint16_t>("rover_port", rover_port[0]);

        //Let the program know that we have inputted the first rover.
        j=1;
        // }

        for(int i=1+j; i <= rover_quantity_; i++) {
            this->declare_parameter<std::string>("local_host"+std::to_string(i), "");
            this->declare_parameter<uint16_t>("local_port"+std::to_string(i), 0);
            this->declare_parameter<std::string>("rover_host"+std::to_string(i), "");
            this->declare_parameter<uint16_t>("rover_port"+std::to_string(i), 0);
            
            this->get_parameter<std::string>("local_host"+std::to_string(i), local_host[i-1]);
            this->get_parameter<uint16_t>("local_port"+std::to_string(i), local_port[i-1]);
            this->get_parameter<std::string>("rover_host"+std::to_string(i), rover_host[i-1]);
            this->get_parameter<uint16_t>("rover_port"+std::to_string(i), rover_port[i-1]);
        }

        ublox_->initBase(local_host, local_port, rover_host, rover_port,
          base_type, rover_quantity_, constellation_, surveytime,
          surveyacc, base_lat, base_lon, base_alt, dynamic_model_);
    }

    void UBLOX_ROS::initRover()
    {
        std::cerr<<"Initializing Rover\n";

        //Initialize local arrays to contain parameters from xml file
        std::string* local_host = new std::string[1];
        uint16_t* local_port = new uint16_t[1];

        //Initialize base arrays to contain parameters from xml file
        std::string* base_host = new std::string[1];
        uint16_t* base_port = new uint16_t[1];

        // if(nh_private_.hasParam("local_host")) { //TODO: add in this logic for ROS2 implementation
        std::string test;

        this->get_parameter<std::string>("local_host", test);
        this->get_parameter<std::string>("local_host", local_host[0]);
        this->get_parameter<uint16_t>("local_port", local_port[0]);
        this->get_parameter<std::string>("base_host", base_host[0]);
        this->get_parameter<uint16_t>("base_port", base_port[0]);

        // }
        // else {

        //     this->get_parameter<std::string>("local_host1", local_host[0]);
        //     this->get_parameter<uint16_t>("local_port1", local_port[0]);
        //     this->get_parameter<std::string>("base_host1", base_host[0]);
        //     this->get_parameter<int>("base_port1", base_port[0]);
        // }

        ublox_->initRover(local_host[0], local_port[0], base_host[0], base_port[0], constellation_, dynamic_model_);
    }

    void UBLOX_ROS::initBrover()
    {
        std::cerr<<"Initializing Brover\n";

        // Initialize local arrays to contain parameters from xml file
        // The first local_host and local_port correspond to the base.
        std::string* local_host = new std::string[rover_quantity_+1];
        uint16_t* local_port = new uint16_t[rover_quantity_+1];

        //Initialize rover arrays to contain parameters from xml file
        std::string* rover_host = new std::string[rover_quantity_];
        uint16_t* rover_port = new uint16_t[rover_quantity_];

        //Initialize base arrays to contain parameters from xml file
        std::string* base_host = new std::string[1];
        uint16_t* base_port = new uint16_t[1];

        // Fill base arrays with their single values
        this->declare_parameter<std::string>("base_host", "localhost");
        this->declare_parameter<int>("base_port", 16140);

        this->get_parameter<std::string>("base_host", base_host[0] );
        this->get_parameter<uint16_t>("base_port", base_port[0]);

        uint8_t j = 0;
        // if(nh_private_.hasParam("local_host")) { //TODO: add in this logic for ROS2 implementation

        this->get_parameter<std::string>("local_host", local_host[0]);
        this->get_parameter<uint16_t>("local_port", local_port[0]);
        this->get_parameter<std::string>("rover_host", rover_host[0]);
        this->get_parameter<uint16_t>("rover_port", rover_port[0]);

        j=1;
        // }

        //Input parameters from xml file into respective arrays
        for(int i=1+j; i <= rover_quantity_; i++) {
        
            this->get_parameter<std::string>("local_host"+std::to_string(i), local_host[i-1]);
            this->get_parameter<uint16_t>("local_port"+std::to_string(i), local_port[i-1]);
            this->get_parameter<std::string>("rover_host"+std::to_string(i), rover_host[i-1]);
            this->get_parameter<uint16_t>("rover_port"+std::to_string(i), rover_port[i-1] );

            j = i;
        }

        // Add in extra local host values.

        this->get_parameter<std::string>("local_host"+std::to_string(j+1), local_host[j]);
        this->get_parameter<uint16_t>("local_port"+std::to_string(j+1), local_port[j]);

        //Determine whether the brover is moving, surveyed, or fixed
        std::string base_type = "moving";

        // Initiate the Brover
        ublox_->initBrover(local_host, local_port, base_host, base_port,
           rover_host, rover_port, base_type, rover_quantity_, 
           constellation_, dynamic_model_);
    }
}