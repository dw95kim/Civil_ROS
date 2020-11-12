
#include "DefineList.h"

// setup the initial name
using namespace ros;
using namespace std;

#define  ROS_FREQ  5.0    // while loop frequency [Hz]
#define DEBUG_MODE 1

#define DF_UDP_BUFFER_SIZE  3
#define DF_UDP_PORTNUM      14550

struct struct_t_UDP
{
    int Socket;
    struct sockaddr_in ServerAddr;
};

struct struct_t_UDP           StrUDP;
struct sockaddr_in            MyAddr;
// struct struct_t_MsgServer	RX_buff;
struct struct_t_MsgUV      TX_buff;

void RX_FCC_Data_Callback(const fcc_serial::MsgStrRXttyO& msg_input);

// node main loop, for ROS
int main(int argc, char** argv)
{
    // node name initialization
    init(argc, argv, "client_udp_node");

    // assign node handler
    ros::NodeHandle nh;

    // for debugging
    printf("Initiate: client_udp_node\n");

    // subscribing the flight state
    ros::Subscriber RX_FCC_Data_sub = nh.subscribe("/UAV/101/FCC", 1, RX_FCC_Data_Callback);

    // setup the loop speed, [Hz], synchronizing the hector slam loop
    ros::Rate loop_rate(ROS_FREQ);

    // Socket Creation
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
    StrUDP.ServerAddr.sin_port        = htons(DF_UDP_PORTNUM); // PORT#
    StrUDP.ServerAddr.sin_addr.s_addr = inet_addr(DF_UDP_SERVER_ADDR); // IP for Server (Normally PC IP)

    memset(&MyAddr, 0, sizeof(MyAddr));
    MyAddr.sin_family = PF_INET;
    MyAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    MyAddr.sin_port = htons(DF_UDP_PORTNUM);

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

        //recvfrom(StrUDP.Socket,(char*)&RX_buff,sizeof(RX_buff), 0, (struct sockaddr *)(&StrUDP.ServerAddr), (socklen_t *)&size_addr);
        sendto(StrUDP.Socket, (char*)&TX_buff, sizeof(TX_buff), 0, (struct sockaddr *)(&StrUDP.ServerAddr), sizeof(StrUDP.ServerAddr));

#if DEBUG_MODE
	system("clear");
// send
        printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
        printf("\t[USRG] Send the FCC message to the server\n");
        printf("[Agent_Number] \t %d \t[count_udp] \t %d \t[Motor] \t %d \t[time_fcc] \t %.3f\n", TX_buff.Agent_Number, count_udp, TX_buff.FCC_Data.Mode_MotorAct, TX_buff.FCC_Data.Cur_Time_sec);
//        printf("[FlightMode] \t %d \t[VFlyingStat] \t %d \t[Voltage] \t %.2f \t[Current] \t %.2f\n", TX_buff.FCC_Data.Mode_FlightMode, TX_buff.FCC_Data.Mode_VehicleFlyingStatus, TX_buff.FCC_Data.ADC_volt[0], TX_buff.FCC_Data.ADC_volt[1]);
        printf("\n");
        printf("[Sat_Number] \t %2.f \t[Lon] \t %3.7f \t[Lat] \t %2.7f \t[Height_GPS] \t %3.2f\n", TX_buff.FCC_Data.GNSS_SatNum, TX_buff.FCC_Data.GNSS_LLH_degdegm[0], TX_buff.FCC_Data.GNSS_LLH_degdegm[1], TX_buff.FCC_Data.GNSS_LLH_degdegm[2]);
        printf("\n");
        printf("[roll]  \t %2.2f \t[pitch] \t %2.2f \t[yaw]   \t %3.2f\n", TX_buff.FCC_Data.Cur_Att_deg[0], TX_buff.FCC_Data.Cur_Att_deg[1], TX_buff.FCC_Data.Cur_Att_deg[2]);
//        printf("[roll_rate]  \t %2.2f \t[pitch_rate] \t %2.2f \t[yaw_rate] \t %3.2f\n", TX_buff.FCC_Data.Cur_AngularRate_dps[0], TX_buff.FCC_Data.Cur_AngularRate_dps[1], TX_buff.FCC_Data.Cur_AngularRate_dps[2]);
//        printf("\n");
//        printf("[LinAcc_0]  \t %2.2f \t[LinAcc_1] \t %2.2f \t[LinAcc_1] \t %3.2f\n", TX_buff.FCC_Data.Cur_LinAccAED_mpss[0], TX_buff.FCC_Data.Cur_LinAccAED_mpss[1], TX_buff.FCC_Data.Cur_LinAccAED_mpss[2]);
        printf("[Vel_for]  \t %2.2f \t[Vel_side] \t %2.2f \t[Vel_down] \t %3.2f\n", TX_buff.FCC_Data.Cur_VelAED_mps[0], TX_buff.FCC_Data.Cur_VelAED_mps[1], TX_buff.FCC_Data.Cur_VelAED_mps[2]);
        printf("\n");
        printf("[North_m]  \t %2.2f \t[East_m] \t %2.2f \t[Down_m] \t %3.2f \t[LIDAR_m] \t %3.2f\n", TX_buff.FCC_Data.Cur_PosNED_m[0], TX_buff.FCC_Data.Cur_PosNED_m[1], TX_buff.FCC_Data.Cur_PosNED_m[2], TX_buff.FCC_Data.LidarPosDown_m);
        printf("[Vel_N_m]  \t %2.2f \t[Vel_E_m] \t %2.2f \t[Vel_D_m] \t %3.2f\n", TX_buff.FCC_Data.Cur_VelNED_mps[0], TX_buff.FCC_Data.Cur_VelNED_mps[1], TX_buff.FCC_Data.Cur_VelNED_mps[2]);
        printf("\n");
//        printf("[SensorStatus] 0x%X\n", TX_buff.FCC_
        printf("[stick_Ail] \t %4.f \t[stick_Ele] \t %4.f \t[stick_Thr] \t %4.f \t[stick_Rud] \t %4.f\n", TX_buff.FCC_Data.TXStickPosAil_usec, TX_buff.FCC_Data.TXStickPosEle_usec, TX_buff.FCC_Data.TXStickPosThr_usec, TX_buff.FCC_Data.TXStickPosRud_usec);
//        printf("[TX_CH_8] \t %4.f \t[TX_CH_9] \t %4.f \t[TX_CH_13] \t %4.f \t[TX_CH_14] \t %4.f \t[TX_CH_15] \t %4.f \t[TX_CH_16] \t %4.f\n", TX_buff.FCC_Data.TX_CH_8_9_13_14_15_16_usec[0], TX_buff.FCC_Data.TX_CH_8_9_13_14_15_16_usec[1], TX_buff.FCC_Data.TX_CH_8_9_13_14_15_16_usec[2], TX_buff.FCC_Data.TX_CH_8_9_13_14_15_16_usec[3], TX_buff.FCC_Data.TX_CH_8_9_13_14_15_16_usec[4], TX_buff.FCC_Data.TX_CH_8_9_13_14_15_16_usec[5]);
        printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
        printf("\n");
#endif

        // loop rate [Hz]
        loop_rate.sleep();

        count_udp++;

        // loop sampling, ros
        spinOnce();
    }

    // for debugging
    printf("Terminate: comm_udp_node\n");
    close(StrUDP.Socket);

    return 0;
}

void RX_FCC_Data_Callback(const fcc_serial::MsgStrRXttyO& msg_input)
{
    ROS_INFO("Get the rostopic from /UAV/FCC");
    memcpy((void *)(&TX_buff.FCC_Data), (void *)(&msg_input), sizeof(TX_buff.FCC_Data));
    TX_buff.Agent_Number = msg_input.Agent_Number;
}

