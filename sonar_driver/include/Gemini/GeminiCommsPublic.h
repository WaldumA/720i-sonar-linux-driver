#pragma once

#include "DataTypes.h"

#ifdef GEMINI_COMMS_USE_STATIC
  #define PUB_GEMINI_COMMS
#else
  #if defined WIN32 || defined __CYGWIN__
    #ifdef BUILDING_GEMINI_COMMS
      #ifdef __GNUC__
        #define PUB_GEMINI_COMMS __attribute__((dllexport))
      #else
        #define PUB_GEMINI_COMMS extern "C" __declspec(dllexport)
      #endif
    #else
      #ifdef __GNUC__
        #define PUB_GEMINI_COMMS __attribute__((dllimport))
      #else
        #define PUB_GEMINI_COMMS extern "C" __declspec(dllimport)
      #endif
    #endif
  #else
    #if __GNUC__ >= 4
      #define PUB_GEMINI_COMMS __attribute__((visibility("default")))
    #else
      #define PUB_GEMINI_COMMS
    #endif
  #endif
#endif


/**************************************************************************************************/
/*                             NETWORK CONFIGURATION API                                          */
/**************************************************************************************************/

/*************************************//**
Purpose: Initialises the network interface ready to operate.

Parameters:
    sonarID             sonar ID

Returns:                0:A failure occurred initialising the network interface.
                          The most likely cause is that another piece of software using the
                          Gemini library is already running, and so the port could not be opened
                        1: The network initialised correctly and is ready to communicate.
****************************************/
PUB_GEMINI_COMMS int  GEM_StartGeminiNetworkWithResult(unsigned short sonarID);

/*************************************//**
Purpose: Set the library operating mode

Parameters:
    softwareMode        Evo: The Gemini library sends the data from the Gemini sonar head to the
                        calling program unprocessed as CGemPingHead, CGemPingLine and
                        CGemPingTailExtended messages using the callback function.

                        EvoC: The Gemini library sends the data from the Gemini sonar head to the
                        calling program unprocessed as CGemPingHead, CGemPingLine and
                        CGemPingTailExtended messages using the callback function.
                        The Gemini library makes use of the range compression feature of the
                        sonar head firmware to ensure that the head does not return more range lines
                        than is appropriate for the size and quality of the display being used by
                        the calling program.

                        SeaNet: The Gemini library processes the data and sends it to the
                        calling program as CGemPingHead, CGemBearingData and CGemPingTailExtended
                        messages using the callback function.

                        SeaNetC: The Gemini library processes the data and sends it to the
                        calling program as CGemPingHead, CGemBearingData and CGemPingTailExtended
                        messages using the callback function.
                        The Gemini library makes use of the range compression feature of the
                        sonar head firmware to ensure that the head does not return 1500 or more
                        range lines to the Seanet software.

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEM_SetGeminiSoftwareMode(char *softwareMode);

/*************************************//**
Purpose: The parent code install the callback function which will be used by the library to return
         data from the sonar head to the parent code.

Parameters:
    FnPtr               CALLBACK function

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEM_SetHandlerFunction(void (cdecl *FnPtr)(int eType, int len, char *dataBlock));

/*************************************//**
Purpose: Resets internal counters held by the library.

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEM_ResetInternalCounters(void);

/*************************************//**
Purpose: Closes the network interface and stops the internal tasks running.

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEM_StopGeminiNetwork(void);

/*************************************//**
Purpose: Returns the maximum version of the data link protocol that the library can support.

* Currently DLL link version is 2.

Returns:                DLL Link Version
****************************************/
PUB_GEMINI_COMMS unsigned char GEM_GetDLLLinkVersionNumber(void);

/*************************************//**
Purpose: Returns the length of the version string which identifies the library

Returns:                Length of version string
****************************************/
PUB_GEMINI_COMMS int  GEM_GetVStringLen(void);

/*************************************//**
Purpose: Gets the copy of version string in the data buffer

* e.g. 'Gemini Comms V2.0.0 Tritech International Ltd.'

Parameters:
    data                User specified buffer
    len                 Length of the data buffer

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEM_GetVString(char *data, int len);

/*************************************//**
Purpose: Returns the length of the version number string which identifies the library

Returns:                Length of version string
****************************************/
PUB_GEMINI_COMMS int  GEM_GetVNumStringLen(void);

/*************************************//**
Purpose: Gets the copy of version number string in the data buffer

* e.g. 'V2.0.0'

Parameters:
    data                User specified buffer
    len                 Length of the data buffer

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEM_GetVNumString(char *data, int len);


/**************************************************************************************************/
/*                                 INITIALISATION API's                                           */
/**************************************************************************************************/


/*************************************//**
Purpose: Sets the compression factor in EVOC mode so that less than number of range lines are returned
         in each mode.

Parameters:
    sonarID             sonar ID
    evoQualitySetting   Quality Settings : [0-7]
                        [7]   = 4096;
                        [6]   = 2048;
                        [5]   = 1024;
                        [4]   = 512;
                        [3]   = 256;
                        [2]   = 128;
                        [1]   = 64;
                        [0]   = 32;

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SetGeminiEvoQuality(unsigned short sonarID, unsigned char evoQualitySetting);

/*************************************//**
Purpose: This is required at the start of initialisation to configure the library for the
         correct sonar head type.

Parameters:
    sonarID             sonar ID
    headType            One of the head type

                        GEM_HEADTYPE_720I
                        GEM_HEADTYPE_720ID
                        GEM_HEADTYPE_NBI
                        GEM_HEADTYPE_MK2_720IS
                        GEM_HEADTYPE_MK2_720IK

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SetHeadType(unsigned short sonarID, int headType);


/*************************************//**
Purpose: Gets the alternate IP address and subnet mask of the Gemini Sonar.
         e.g. IP address : 192.168.2.201, Subnet : 255.255.255.0

Parameters:
    sonarID             Sonar ID
    a1                  a1 = 192
    a2                  a2 = 168
    a3                  a3 = 2
    a4                  a4 = 201
    s1                  s1 = 255
    s2                  s2 = 255
    s3                  s3 = 255
    s4                  s4 = 0

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_GetAltSonarIPAddress(unsigned short sonarID,
                            unsigned char *a1, unsigned char *a2, unsigned char *a3, unsigned char *a4,
                            unsigned char *s1, unsigned char *s2, unsigned char *s3, unsigned char *s4);

/*************************************//**
Purpose: Sets the new alternate IP address and subnet mask on the Gemini Sonar head.
         e.g. IP address : 192.168.2.201, Subnet : 255.255.255.0

Parameters:
    sonarID             Sonar ID
    a1                  a1 = 192
    a2                  a2 = 168
    a3                  a3 = 2
    a4                  a4 = 201
    s1                  s1 = 255
    s2                  s2 = 255
    s3                  s3 = 255
    s4                  s4 = 0

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SetAltSonarIPAddress(unsigned short sonarID,
                            unsigned char a1, unsigned char a2, unsigned char a3, unsigned char a4,
                            unsigned char s1, unsigned char s2, unsigned char s3, unsigned char s4);

/*************************************//**
Purpose: Uses the alternate IP address and subnet mask of the Gemini Sonar for communication.
         e.g. IP address : 192.168.2.201, Subnet : 255.255.255.0

Parameters:
    sonarID             Sonar ID
    a1                  a1 = 192
    a2                  a2 = 168
    a3                  a3 = 2
    a4                  a4 = 201
    s1                  s1 = 255
    s2                  s2 = 255
    s3                  s3 = 255
    s4                  s4 = 0

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_UseAltSonarIPAddress(unsigned short sonarID,
                            unsigned char a1, unsigned char a2, unsigned char a3, unsigned char a4,
                            unsigned char s1, unsigned char s2, unsigned char s3, unsigned char s4);

/*************************************//**
Purpose: Use alternate or fixed IP address for communication with sonar

Parameters:
    sonarID             Sonar ID
    useAltIPAddress     0: Use fixed IP address
                        1: Use alternate IP address

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_TxToAltIPAddress(unsigned short sonarID, int useAltIPAddress);


/**************************************************************************************************/
/*                                 END OF INITIALISATION                                          */
/**************************************************************************************************/





/**************************************************************************************************/
/*                                  PING CONFIGURATION                                            */
/**************************************************************************************************/

/*************************************//**
Purpose: Reset ping configuration to the factory defaults

Parameters:
    sonarID             sonar ID

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SetPingToDefaults(unsigned short sonarID);

/*************************************//**
Purpose: Configure ping parameters ( only for MK1 products )

Parameters:
    sonarID             Sonar ID
    range               Range in meters ( 0.1 to 150 )m
    percentGain         Percentage Gain
    sos                 Speed of sound (in m/s)

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_AutoPingConfig(
                                    unsigned short  sonarID,
                                    float           range,
                                    unsigned short  percentGain,
                                    float           sos
                                    );


/*************************************//**
Purpose: Negotiate a VDSL adaption rate ( Not supported for 1200ik)

Parameters:
    sonarID             Sonar ID
    level               0: Normal electrical noise environment
                        1: Medium electrical noise environment
                        2: High electrical noise environment

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SetVDSLSetting(unsigned short sonarID, unsigned short level);

/*************************************//**
Purpose: Returns last negotiated VDSL adaption level ( Not supported for 1200ik)

Parameters:
    sonarID             Sonar ID

Returns:                Level : ( 0 - 2 )
****************************************/
PUB_GEMINI_COMMS unsigned short GEMX_GetVDSLSetting(unsigned short sonarID );

/*************************************//**
Purpose: Returns the compression factor applied when a ping was requested in EvoC mode.

Parameters:
    sonarID             Sonar ID

Returns:                Compression Factor : ( 1, 2, 4, 8, 16 )
****************************************/
PUB_GEMINI_COMMS unsigned short GEMX_GetRequestedCompressionFactor(unsigned short sonarID);

/*************************************//**
Purpose: Configure the ping mode

Parameters:
    sonarID             Sonar ID
    pingMethod          0: Ping once on receipt of ping configuration message
                        1: Ping repeatedly at interval fixed by GEMX_SetInterPingPeriod

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SetPingMode(unsigned short sonarID, unsigned short pingMethod);

/*************************************//**
Purpose: Returns the ping mode.

Parameters:
    sonarID             Sonar ID

Returns:                Ping Mode [ 0 - 1 ]
****************************************/
PUB_GEMINI_COMMS unsigned short GEMX_GetPingMode(unsigned short sonarID);

/*************************************//**
Purpose: Sets the time delay between start of the one ping and start of the next ping

* Max inter-ping period is 999 milli-seconds.

Parameters:
    sonarID                 Sonar ID
    periodInMicroSeconds    Time delay ( In Micro seconds )

Returns:                None
****************************************/
PUB_GEMINI_COMMS void      GEMX_SetInterPingPeriod(unsigned short sonarID, unsigned int periodInMicroSeconds);

/*************************************//**
Purpose: Returns the time delay between two pings

Parameters:
    sonarID             Sonar ID

Returns:                Time delay in micro-seconds
****************************************/
PUB_GEMINI_COMMS unsigned int  GEMX_GetInterPingPeriod(unsigned short sonarID);

/*************************************//**
Purpose: Sets the optimum transmit pulse length for a given range, GEMX_SetTXLength is an alternative
         to this API

Parameters:
    sonarID             Sonar ID
    range               Range( In meters )

Returns:                Returns the length of the transmit pulse in cycles
****************************************/
PUB_GEMINI_COMMS unsigned short  GEMX_AutoTXLength(unsigned short sonarID, float range);

/*************************************//**
Purpose: Sets the length of the transmit pulse, use GEMX_AutoTXLength instead to set the optimum
         transmit pulse length for a given range

Parameters:
    sonarID             Sonar ID
    txLength            The length of the transmit pulse in cycles

Returns:                None
****************************************/
PUB_GEMINI_COMMS void      GEMX_SetTXLength(unsigned short sonarID, unsigned short txLength);



/*************************************//**
Purpose: Returns the length of the transmit pulse in cycles

Parameters:
    sonarID             Sonar ID

Returns:                the length of the transmit pulse in cycles
****************************************/
PUB_GEMINI_COMMS unsigned short  GEMX_GetTXLength(unsigned short sonarID);


/*************************************//**
Purpose: Sets the out of water override flag in the ping configuration which will be sent to the
         sonar when ping is requested.

Parameters:
    sonarID             Sonar ID
    outOfWaterOverride  0 : Do not ping when out of water
                        1 : Ping regardless of out of water indicator

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SetExtModeOutOfWaterOverride(unsigned short sonarID, unsigned short outOfWaterOverride);

/*************************************//**
Purpose: Sets the gain and velocimeter mode in the ping configuration

Parameters:
    sonarID             Sonar ID
    gainMode            0: Auto gain
                        1 : Manual gain
    outputMode          0: Use velocimeter calculated speed of sound
                        1: Use speed of sound specified in this ping configuration message

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SetVelocimeterMode(
                                    unsigned short sonarID,
                                    unsigned short gainMode,
                                    unsigned short outputMode
                                    );

/*************************************//**
Purpose: Sets the Range compression level and compression type

*For MK2 Chirp operation, it is recommended that peak compressionType (=1) be used for better results

Parameters:
    sonarID             Sonar ID
    compressionLevel    0: No range compression
                        1:  2 * range compression
                        2:  4 * range compression
                        3:  8 * range compression
                        4: 16 * range compression
    compressionType     0: Use average compression
                        1: Use peak compression

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SetRangeCompression(
                                    unsigned short sonarID,
                                    unsigned short compressionLevel,
                                    unsigned short compressionType
                                    );

/*************************************//**
Purpose: Gets the active range compression level and compression type

Parameters:
    sonarID             Sonar ID
    compressionLevel    Active compression level
    compressionType     Active compression type

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_GetRangeCompression(
                                    unsigned short sonarID,
                                    unsigned short *compressionLevel,
                                    unsigned short *compressionType
                                    );

/*************************************//**
Purpose: May reduce the bandwidth required to get the data from the Gemini sonar head to the
         hardware running the calling software

Parameters:
    sonarID             Sonar ID
    threshold           0:  No run length encoding
                        1:  Not available for use, will be increased to 3
                        2:  Not available for use, will be increased to 3
                        3:
                        ...
                        255: Maximum run length encoding

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SetRLEThreshold(unsigned short sonarID, unsigned short threshold);

/*************************************//**
Purpose: Returns the number of beams (nBeams) that the Gemini will form.

* If pBrgTbl is not NULL, then GEMX_GetGeminiBeams will write a table of the bearings in radians
  for each beam at this address

Parameters:
    sonarID             Sonar ID
    pBrgTbl             User allocated buffer [ 256 / 512 ]

Returns:                Number of beams
****************************************/
PUB_GEMINI_COMMS int  GEMX_GetGeminiBeams(unsigned short sonarID, float *pBrgTbl );

/*************************************//**
Purpose: Sets the number of beams (nBeams) that the Gemini will form.

* Only supported for the MK2 platform

Parameters:
    sonarID             Sonar ID
    nBeams              Beams [256/512] default: 256

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SetGeminiBeams(unsigned short sonarID, unsigned short nBeams);


/*************************************//**
Purpose: Send ping configuration command to the sonar

Parameters:
    sonarID             Sonar ID

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SendGeminiPingConfig(unsigned short sonarID);

/*************************************//**
Purpose: Configure main port (Only for 720im)

** Supported only on Windows platform, please see 720im-ReadMe.txt file for further details

Parameters:
    sonarID             Sonar ID
    negotiate           true: reset baudrate to 115200 then goes up,
                        false: used fixed baudrate specified by the user
    rs232               true: RS232, false: RS485
    baudRate            baudrate: 115200, 230400, 460800, 921600
    comPortName         COM1, COM2... COM(N)

Returns:                0: failed, 1: success
****************************************/
PUB_GEMINI_COMMS bool  GEMX_ConfigureMainPort(
                                unsigned short sonarID,
                                bool            negotiate,
                                bool            rs232,
                                unsigned int    baudRate,
                                const char*     portName
                                );

/*************************************//**
Purpose: Configure High Range resolution (Only for Mk2 platforms supported from 0x200e firmware version)

* Automatically switch high range resolution based on the range selected. By enabling high range
* resolution would improve the sonar imagery in lower range e.g.

**  720khz : reduce the range lines from 8mm to 4mm
** 1200khz : reduce the range lines from 4mm to 2.4

Parameters:
    sonarID             Sonar ID
    enable              true/false

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_AutoHighRangeResolution(unsigned short sonarID, bool enable);


/*************************************//**
Purpose: Configure Auto Range frequency (Only for 1200ik)

User can configure auto range (1-50)m to switch between high/low frequency

Parameters:
    sonarID             Sonar ID
    autoRangeConfig     RangeFrequencyConfig

Returns:                true, if configuration parameters passed are valid else false
****************************************/
PUB_GEMINI_COMMS bool GEMX_ConfigureAutoRangeFrequency(
                                            unsigned short          sonarID,
                                            RangeFrequencyConfig   autoRangeConfig
                                            );

/*************************************//**
Purpose: Configure chirp mode

Parameters:
    sonarID             Sonar ID
    chirpMode           CHIRP_MODE
            CHIRP_DISABLED  : Chirp will be disabled
            CHIRP_ENABLED   : Chirp will be enabled
            CHIRP_AUTO      : chirp will be enabled/disabled automatically based on the range selected

Returns:                true, if configuration parameters passed are valid else false
****************************************/
PUB_GEMINI_COMMS bool GEMX_ConfigureChirpMode(unsigned short sonarID, CHIRP_MODE chirpMode );

/*************************************//**
Purpose: Get the active chirp mode

Parameters:
    sonarID             Sonar ID

Returns:                true, chirp on else off
****************************************/
PUB_GEMINI_COMMS bool GEMX_GetActiveChirpMode(unsigned short sonarID );


/*************************************//**
Purpose: Increase update rate in low frequency mode at longer ranges

Parameters:
    sonarID             Sonar ID
    enable              true/false

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SetHigherFrameRateinLFMode(unsigned short sonarID, bool enable );

/**************************************************************************************************/
/*                                   Sonar Specific                                               */
/**************************************************************************************************/

/*************************************//**
Purpose: Build a list of sonars and return the number sof sonars

Parameters:
    pList               List of sonars

Returns:                Number of sonars in the list
****************************************/
PUB_GEMINI_COMMS unsigned short GEMX_GetSonars(unsigned short *pList);

/*************************************//**
Purpose: Delete the sonar from the list, if sonar ID is specified as 0 then
         all sonars gets deleted

Parameters:
    sonarID             sonar ID

Returns:                Number of sonars deleted from the internal list
****************************************/
PUB_GEMINI_COMMS unsigned short GEMX_DeleteSonarID(unsigned short sonarID);

/*************************************//**
Purpose: Copies a null-terminated ASCII string name for the device into pBuf.

Parameters:
    sonarID             sonar ID
    pBuf                User specified buffer
    len                 Length of the pBuf

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_GetDeviceName(unsigned short sonarID, char* pBuf, unsigned int len);

/*************************************//**
Purpose: Returns the operating frequency of the sonar in Hz.

Parameters:
    sonarID             Sonar ID

Returns:                Frequency in Hz
****************************************/
PUB_GEMINI_COMMS int  GEMX_GetGeminiFrequency(unsigned short sonarID);

/*************************************//**
Purpose: Returns the modulation frequency in conjustion with the speed of sound

Parameters:
    sonarID             Sonar ID

Returns:                The modulation frequency in Hz.
****************************************/
PUB_GEMINI_COMMS int  GEMX_GetGeminiModFrequency(unsigned short sonarID);


/*************************************//**
Purpose: Keep the communication alive between PC and Sonar

Parameters:
    sonarID             Sonar ID

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_SendGeminiStayAlive(unsigned short sonarID);

/*************************************//**
Purpose: Reboot sonar

Parameters:
    sonarID             Sonar ID

Returns:                None
****************************************/
PUB_GEMINI_COMMS void GEMX_RebootSonar(unsigned short sonarID);


/**************************************************************************************************/
/*                                        END                                                     */
/**************************************************************************************************/

