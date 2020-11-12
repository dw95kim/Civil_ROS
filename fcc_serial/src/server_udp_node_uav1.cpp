#include "DefineList.h"

// setup the initial name
using namespace ros;
using namespace std;

#define ROS_FREQ  5.0    // while loop frequency [Hz]
#define DEBUG_MODE 1

#define DF_UDP_BUFFER_SIZE      3
// #define DF_UDP_PORTNUM      14550

struct struct_t_UDP
{
    int Socket;
    struct sockaddr_in ServerAddr;
};

struct struct_t_UDP           StrUDP;
struct sockaddr_in            MyAddr;

std_msgs::Float32MultiArray gps_data;

// Define Recive Variable
float RX_buff[7];

// node main loop, for ROS
int main(int argc, char** argv)
{
    // node name initialization
    init(argc, argv, "server_udp_node_uav1");

    // assign node handler
    ros::NodeHandle nh;

    // for debugging
    printf("Initiate: server_udp_node_uav1\n");

    // Publish Topic
    ros::Publisher GPS_pub = nh.advertise<std_msgs::Float32MultiArray> ("/UAV1/GPS"  ,1);

    // Get port number
    // int UDP_PORTNUM;
    // nh.getParam("udp_portnum_uav1", UDP_PORTNUM);
    int UDP_PORTNUM = 14550;

    // setup the loop speed, [Hz], synchronizing the hector slam loop
    ros::Rate loop_rate(ROS_FREQ);

    // Socket_Rear Creation
    StrUDP.Socket = socket(PF_INET, SOCK_DGRAM, 0);
    int enable = 1;
    setsockopt(StrUDP.Socket,SOL_SOCKET,SO_REUSEADDR,&enable,sizeof(enable));
    setsockopt(StrUDP.Socket,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv));
    if(StrUDP.Socket == -1)
    {
        printf("[ERROR] 'socket()'\n");
        return -1;
    }
    else
    {
        printf("[DONE] UDP socket is created\n");
    }

    //
    // UDP-IP Setting
    memset(&StrUDP.ServerAddr, 0, sizeof(StrUDP.ServerAddr)); // Clear to 0
    StrUDP.ServerAddr.sin_family      = PF_INET;
    StrUDP.ServerAddr.sin_port        = htons(UDP_PORTNUM); // PORT#
#ifdef REAR_SERVER
    StrUDP.ServerAddr.sin_addr.s_addr = inet_addr(DF_UDP_DRONE_ADDR_REAR);
#endif
#ifdef TOP_SERVER
    StrUDP.ServerAddr.sin_addr.s_addr = inet_addr(DF_UDP_DRONE_ADDR_TOP);
#endif

    memset(&MyAddr, 0, sizeof(MyAddr));
    MyAddr.sin_family = PF_INET;
    MyAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    MyAddr.sin_port = htons(UDP_PORTNUM);

    if(bind(StrUDP.Socket,(struct sockaddr *)&MyAddr, sizeof(MyAddr))!=0)
    {
        printf("bind() error!\n");
        return -1;
    }

    int size_addr = sizeof(StrUDP.ServerAddr);
    printf("[DONE] UDP initialized! size : %d\n",size_addr);

    int count_udp = 0;

    // node loop, for ROS, check ros status, ros::ok()
    while( ok() )
    {
        printf("[Waiting]\n");

        int recv_len;
        recv_len = recv(StrUDP.Socket,(char*)&RX_buff,sizeof(RX_buff), 0);

#if DEBUG_MODE
        if (recv_len > 0)
        {
            system("clear");
            printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
            printf("\t[USRG] Send the GPS data to Server\n");
            printf("[Lat] \t %2.7f \t[Lon] \t %2.7f \t[Alt] \t %3.2f \n", RX_buff[0], RX_buff[1], RX_buff[2]);
            printf("\n");
            printf("[Heading] \t %3.2f \n", RX_buff[3]);
            printf("\n");
            printf("[Vel_x] \t %2.2f \t[Vel_y] \t %2.2f \t[Vel_z] \t %2.2f \n", RX_buff[4], RX_buff[5], RX_buff[6]);
            printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
            printf("\n");
        }
#endif

        float heading = RX_buff[3];

        if(heading < 0){
            heading = 360.0 + heading;
        }

        gps_data.data.resize(7);
        gps_data.data[0] = RX_buff[0]; // Lat
        gps_data.data[1] = RX_buff[1]; // Lon
        gps_data.data[2] = RX_buff[2]; // Alt
        gps_data.data[3] = heading;
        gps_data.data[4] = RX_buff[4]; // Vel_x
        gps_data.data[5] = RX_buff[5]; // Vel_y
        gps_data.data[6] = RX_buff[6]; // Vel_z
        GPS_pub.publish(gps_data);

        // loop rate [Hz]
        loop_rate.sleep();

        count_udp++;

        // loop sampling, ros
        spinOnce();
    }

    // for debugging
    printf("Terminate: comm_udp_node_uav1\n");
    close(StrUDP.Socket);

    return 0;
}
