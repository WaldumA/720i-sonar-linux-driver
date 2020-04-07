#include "sonar_driver.hpp"
#include <fstream>
#include <GeminiStructuresPublic.h>
#include <GeminiCommsPublic.h>
#include <unistd.h>
#include <cstring>
#include <typeinfo>

using namespace std;

//Configuration parameters
//#define DEFAULT_GAIN 12
//#define DEFAULT_SOS 1500
//#define DEFAULT_RANGE 1
#define NUM_BEAMS 256
char* SOFTWARE_MODE = (char*) "SeaNet";
unsigned short SONAR_ID = 0;
unsigned int ALT_SONAR_IP = 0;
unsigned int pingNr = 0;
sonar_msgs::sonar_raw_data msg;
static bool time_to_update = false;

bool waitingForPing = false;

void callBack(int eType, int len, char *dataBlock){
    
    switch (eType){
        case PING_HEAD:{
            CGemPingHead data = *((CGemPingHead*) dataBlock);
            cout << "Ping " << pingNr << " recieved! \n";
            pingNr++;
            waitingForPing = false;
            break;
        } 
        case PING_DATA:{
            CGemPingLine data = *((CGemPingLine*) dataBlock);
            cout << data.m_lineID<< endl;
        }
        case PING_TAIL:{
            CGemPingTail data = *((CGemPingTail*) dataBlock);
            break;
        }
        case GEM_STATUS:{
            CGemStatusPacket data = *((CGemStatusPacket*) dataBlock);
            if (SONAR_ID == 0 || ALT_SONAR_IP == 0){
                SONAR_ID = data.m_sonarId;
                ALT_SONAR_IP = data.m_sonarAltIp;
            }
            break;
        }
        case GEM_ACKNOWLEDGE:{
            CGemAcknowledge data = *((CGemAcknowledge*) dataBlock);
            cout << "Ack recieved!" << endl;
            break;
        }
        case GEM_BEARING_DATA:{
            CGemBearingData data = *((CGemBearingData*) dataBlock);
            CGemBearingData* pBearing = (CGemBearingData *)dataBlock;
            CGemBearingData* copyBearing = new CGemBearingData;
            CGemHdr copyHeader = copyBearing->m_head;
            *copyBearing = *pBearing;
            copyBearing->m_pData = (unsigned char *)malloc(copyBearing->m_noSamples);
              
            if (copyBearing->m_pData)
            {
                memcpy(copyBearing->m_pData, pBearing->m_pData, copyBearing->m_noSamples);
            }
            else
            {
                copyBearing->m_noSamples = 0;
            }
            msg.bins = copyBearing->m_noSamples;
            for (int i =0; i < copyBearing->m_noSamples; i++){

                msg.data.push_back((unsigned char)*(copyBearing->m_pData + i));
            }
          
            delete copyBearing->m_pData;
            delete copyBearing;
            break;
        }
        case PING_TAIL_EX:{
            CGemPingTailExtended data = *((CGemPingTailExtended*) dataBlock);
            time_to_update = true;
            break;
        }
        case GEM_IP_CHANGED:{
            break;
        }
        case GEM_UNKNOWN_DATA:{
            break;
        }
        default:{
            cout << "Handling of eType " << eType << " not implemented" << "\n";
            break;
        }
    }
}
void sonarDriver::run() {
        if(GEM_StartGeminiNetworkWithResult(0) == 0){
        cout << "An error has occured while initializing the Gemini library.\nIs any other running software using the library?" << endl;
    }
    else{
        GEM_SetGeminiSoftwareMode(SOFTWARE_MODE);
        GEM_SetHandlerFunction(callBack);
        cout << "Waiting for sonar..." << endl;
        while (SONAR_ID == 0 || ALT_SONAR_IP == 0){
            //Waits for sonar to be found
        }
        cout << "Sonar found with ID " << SONAR_ID << endl;
        GEMX_SetHeadType(SONAR_ID, GEM_HEADTYPE_720I);

        GEMX_SetPingMode(SONAR_ID, 0); //0: Ping once, 1: Ping continously
        GEMX_SetInterPingPeriod(SONAR_ID, 1000000);
        GEMX_SetVelocimeterMode(SONAR_ID, 0, 0);
        GEMX_SetExtModeOutOfWaterOverride(SONAR_ID, 0);
        GEMX_UseAltSonarIPAddress(SONAR_ID, 192, 168, 2, 200, 255, 255, 255, 0);
        GEMX_AutoPingConfig(SONAR_ID, range,gain, sos);
        GEMX_SendGeminiPingConfig(SONAR_ID);
        msg.range = range;
        while (true){
            GEMX_SendGeminiPingConfig(SONAR_ID);

            if (time_to_update == true) {
                sonar_data_pub.publish(msg);
                msg.data.clear();
                time_to_update = false;
            }
            
            usleep(1000);
        }
    }
    
    GEM_StopGeminiNetwork();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "sonarDriver");
    sonarDriver run_driver;
    ros::spin();
    return 0;
}