#pragma once

// This file contains basic Gemini SDK structure definitions

/*****************************************************************************//**
************** Message types received in the callback function *******************
*********************************************************************************/

#define PING_HEAD                   0
#define PING_DATA                   1
#define PING_TAIL                   2
#define GEM_STATUS                  3
#define GEM_ACKNOWLEDGE             4
#define GEM_BEARING_DATA            7
#define PING_TAIL_EX                10
#define GEM_IP_CHANGED              11
#define GEM_UNKNOWN_DATA            12

/*****************************************************************************//**
**************************** Supported Product ID's *****************************
*********************************************************************************/
#define  GEM_HEADTYPE_720I            0x0
#define  GEM_HEADTYPE_720ID           0x1
#define  GEM_HEADTYPE_NBI             0x2
#define  GEM_HEADTYPE_MK2_720IS       0x21
#define  GEM_HEADTYPE_MK2_1200IK      0x1D
#define  GEM_HEADTYPE_MK2_720IK       0x1E
#define  GEM_HEADTYPE_720IM           0x1F


/*****************************************************************************//**
********** Beam Spacing mode used in GEMX_GetBeamSpacing  ***********************
*********************************************************************************/
#define  GEM_BEAMS_DEFAULT        (-1)
#define  GEM_BEAMS_INVERSESINE      0
#define  GEM_BEAMS_EQUIDISTANT      1
#define  GEM_BEAMS_EQUIANGULAR      2

/*****************************************************************************//**
********** Serial Port ID's and maximum supported serial ports*******************
** 720is    Maximum : 2 Serial ports.
            Serial port 'A' can switch betwen RS232/RS485
            Serial port 'B' RS232 only

** 720ik    Maximum : 1 Serial port.
            Serial port 'A' RS232 only

*********************************************************************************/
// Serial ports
#define SERIAL_PORT_A               0
#define SERIAL_PORT_B               1
#define MAX_SERIAL_PORTS            2

/*****************************************************************************//**
********** FPGA ID's *******************
** MK1      1 FPGA with ID 0
** MK2      Maximum of 4 FPGA's

** 720is    Maximum of 2 FPGA's
            BEAMFORMER_FPGA, DATA_ACQ_FPGA_0

** 720ik    Maximum of 2 FPGA's
            BEAMFORMER_FPGA, DATA_ACQ_FPGA_0

*********************************************************************************/
// FPGA ID's
#define BEAMFORMER_FPGA   2  // 0x02
#define DATA_ACQ_FPGA_0   4  // 0x04
#define DATA_ACQ_FPGA_1   8  // 0x08
#define DATA_ACQ_FPGA_2  16  // 0x10
#define ALL_FPGA  BEAMFORMER_FPGA | DATA_ACQ_FPGA_0 | DATA_ACQ_FPGA_1 | DATA_ACQ_FPGA_2


/*****************************************************************************//**
*********** Number of FPGA in a single sonar *******************
** MK1      1 FPGA with number 0
** MK2      Maximum of 4 FPGA's

** 720is, 720ik [0,1]
*********************************************************************************/
// Number of FPGA in MK2 platform
#define BEAMFORMER_FPGA_ENUM    0
#define DATA_ACQ_FPGA_0_ENUM    1
#define DATA_ACQ_FPGA_1_ENUM    2
#define DATA_ACQ_FPGA_2_ENUM    3

/*****************************************************************************//**
*********** Header structure of each message *******************
*********************************************************************************/
#pragma pack (push,1)

class CGemHdr
{
public:
  unsigned char  m_type;
  unsigned char  m_version;
  unsigned short m_deviceID;
  unsigned short m_packetLatency;
  /*
  ** First 8 bytes are source sub device id (Hardware)
  ** while 2nd 8 bytes are destinataions sub device ID  (Software)
  ** because of endian, these should be declared otherway around
  */
  unsigned char m_dst_sub_device_id;
  unsigned char m_src_sub_device_id;

  CGemHdr()
  {
    m_type      = 0;
    m_version     = 1;
    m_deviceID    = 0;
    m_packetLatency = 0;
    m_src_sub_device_id = 1; /*software*/
    m_dst_sub_device_id = 0; /*to be overwritten on a message by message basis*/
  }
};

class CGemNetMsg
{
public:
  CGemHdr m_head;
};

#pragma pack (pop)
#pragma pack (push,2)

/*****************************************************************************//**
********************************* MK1 Status Message ****************************
*********************************************************************************/
class CGemStatusPacket: public CGemNetMsg
{
public:
  unsigned short m_firmwareVer;
  unsigned short m_sonarId;
  unsigned int   m_sonarFixIp;
  unsigned int   m_sonarAltIp;
  unsigned int   m_surfaceIp;
  unsigned short m_flags;
  unsigned short m_vccInt;
  unsigned short m_vccAux;
  unsigned short m_dcVolt;
  unsigned short m_dieTemp;
  unsigned short m_dipSwitch; /*Only for Mk2*/
  unsigned short m_vga1aTemp;
  unsigned short m_vga1bTemp;
  unsigned short m_vga2aTemp;
  unsigned short m_vga2bTemp;
  unsigned short m_psu1Temp;
  unsigned short m_psu2Temp;
  unsigned int   m_currentTimestampL;
  unsigned int   m_currentTimestampH;
  unsigned short m_transducerFrequency;
  unsigned int   m_subnetMask;
  unsigned short m_TX1Temp;
  unsigned short m_TX2Temp;
  unsigned short m_TX3Temp;
  unsigned int   m_BOOTSTSRegister;
  unsigned short m_shutdownStatus;
  unsigned short m_dieOverTemp;
  unsigned short m_vga1aShutdownTemp;
  unsigned short m_vga1bShutdownTemp;
  unsigned short m_vga2aShutdownTemp;
  unsigned short m_vga2bShutdownTemp;
  unsigned short m_psu1ShutdownTemp;
  unsigned short m_psu2ShutdownTemp;
  unsigned short m_TX1ShutdownTemp;
  unsigned short m_TX2ShutdownTemp;
  unsigned short m_TX3ShutdownTemp;
  unsigned short m_linkType;
  unsigned short m_VDSLDownstreamSpeed1;
  unsigned short m_VDSLDownstreamSpeed2;
  unsigned short m_macAddress1;
  unsigned short m_macAddress2;
  unsigned short m_macAddress3;
  unsigned short m_VDSLUpstreamSpeed1;
  unsigned short m_VDSLUpstreamSpeed2;

  CGemStatusPacket()
  {
    m_head.m_type = 0x40;
  }
};

/*****************************************************************************//**
********************** MK2 Beamformer Status Message ****************************
*********************************************************************************/
class CGemMk2BFStatusPacket: public CGemNetMsg
{
public:
  unsigned short m_firmwareVer;
  unsigned short m_sonarId;
  unsigned int   m_sonarFixIp;
  unsigned int   m_sonarAltIp;
  unsigned int   m_surfaceIp;
  unsigned short m_flags;
  unsigned short m_vccInt;
  unsigned short m_vccAux;
  unsigned short m_unused0;
  unsigned short m_dieTemp;
  unsigned short m_dipSwitch; /*Only for Mk2*/;
  unsigned short m_pcbTemp;
  unsigned short m_commsTemp;
  unsigned short m_txTemp;
  unsigned short m_unused2;
  unsigned short m_psuTemp;
  unsigned short m_unused3;
  unsigned int   m_currentTimestampL;
  unsigned int   m_currentTimestampH;
  unsigned short m_transducerFrequency;
  unsigned int   m_subnetMask;
  unsigned short m_unused4;
  unsigned short m_unused5;
  unsigned short m_unused6;
  unsigned int   m_BOOTSTSRegister;
  unsigned short m_shutdownStatus;
  unsigned short m_dieOverTemp;
  unsigned short m_pcbShutdownTemp;
  unsigned short m_commsShutdownTemp;
  unsigned short m_txShutdownTemp;
  unsigned short m_unused7;
  unsigned short m_psuShutdownTemp;
  unsigned short m_unused8;
  unsigned short m_unused9;
  unsigned short m_unused10;
  unsigned short m_unused11;
  unsigned short m_linkType;
  unsigned short m_VDSLDownstreamSpeed1;
  unsigned short m_VDSLDownstreamSpeed2;
  unsigned short m_macAddress1;
  unsigned short m_macAddress2;
  unsigned short m_macAddress3;
  unsigned short m_VDSLUpstreamSpeed1;
  unsigned short m_VDSLUpstreamSpeed2;

  CGemMk2BFStatusPacket()
  {
    m_head.m_type = 0x40;
  }
};

/*****************************************************************************//**
********************** MK2 Data Acqusition Status Message ***********************
*********************************************************************************/
class CGemMk2DAStatusPacket: public CGemNetMsg
{
public:
  unsigned short m_firmwareVer;
  unsigned short m_unused0;
  unsigned int   m_unused1;
  unsigned int   m_unused2;
  unsigned int   m_unused3;
  unsigned short m_unused4;
  unsigned short m_unused5;
  unsigned short m_unused6;
  unsigned short m_unused7;
  unsigned short m_unused8;
  unsigned short m_dipSwitch; /*Only for Mk2*/;
  unsigned short m_pcbTemp;
  unsigned short m_afe0TopTemp;
  unsigned short m_afe0BotTemp;
  unsigned short m_afe1TopTemp;
  unsigned short m_afe1BotTemp;
  unsigned short m_afe2TopTemp;
  unsigned int   m_unused10;
  unsigned int   m_unused11;
  unsigned short m_unused12;
  unsigned int   m_unused13;
  unsigned short m_afe2BotTemp;
  unsigned short m_afe3TopTemp;
  unsigned short m_afe3BotTemp;
  unsigned int   m_BOOTSTSRegister;
  unsigned short m_unused15;
  unsigned short m_unused16;
  unsigned short m_pcbShutdownTemp;
  unsigned short m_afe0topShutdownTemp;
  unsigned short m_afe0botShutdownTemp;
  unsigned short m_afe1topShutdownTemp;
  unsigned short m_afe1botShutdownTemp;
  unsigned short m_afe2topShutdownTemp;
  unsigned short m_afe2botShutdownTemp;
  unsigned short m_afe3topShutdownTemp;
  unsigned short m_afe3botShutdownTemp;
  unsigned short m_unused17;
  unsigned short m_unused18;
  unsigned short m_unused19;
  unsigned short m_unused20;
  unsigned short m_unused21;
  unsigned short m_unused22;
  unsigned short m_unused23;
  unsigned short m_unused24;

  CGemMk2DAStatusPacket()
  {
    m_head.m_type = 0x40;
  }
};

#pragma pack (pop)

/*****************************************************************************//**
*************** Ping Head structure received against the ping command ************
*********************************************************************************/
class CGemPingHead: public CGemNetMsg
{
public:
  unsigned short m_pingID;
  unsigned short m_extMode;
  unsigned int   m_transmitTimestampL;
  unsigned int   m_transmitTimestampH;
  unsigned short m_startRange;
  unsigned short m_endRange;
  unsigned int   m_lineTime;
  unsigned short m_numBeams;
  unsigned short m_numChans;
  unsigned char  m_sampChan;
  unsigned char  m_baseGain;
  unsigned short m_spdSndVel;
  unsigned short m_velEchoTime;
  unsigned short m_velEntries;
  unsigned short m_velGain;
  unsigned short m_sosUsed;
  unsigned char  m_RLEThresholdUsed;
  unsigned char  m_rangeCompressionUsed;
  unsigned char  m_decimation;

  CGemPingHead()
  {
    m_head.m_type = 0x41;
    m_pingID      = 0xff;
    m_extMode     = 0;
    m_transmitTimestampL = 0;
    m_transmitTimestampH = 0;
    m_startRange  = 0;
    m_endRange    = 0;
    m_lineTime    = 0;
    m_numBeams    = 0;
    m_numChans    = 0;
    m_sampChan    = 0;
    m_baseGain    = 0;
    m_spdSndVel   = 0;
    m_velEchoTime = 0;
    m_velEntries  = 0;
    m_velGain     = 0;
    m_sosUsed     = 0;
    m_RLEThresholdUsed = 0;
    m_rangeCompressionUsed = 0;
    m_decimation  = 0;
  }
};

/*****************************************************************************//**
*************** Ping Line structure received against the ping command ************
*********************************************************************************/
class CGemPingLine: public CGemNetMsg
{
public:
  unsigned char  m_gain;
  unsigned char  m_pingID;
  unsigned short m_lineID;
  unsigned short m_scale;
  unsigned short m_lineInfo;
  unsigned char  m_startOfData;

  CGemPingLine()
  {
    m_head.m_type = 0x42;
    m_gain        = 0;
    m_pingID      = 0xff;
    m_lineID      = 0;
    m_scale       = 0;
    m_lineInfo    = 0;
    m_startOfData = 0;
  }
  int GetLineWidth( ) const { return ( ( ( m_lineInfo & 0xff80) >> 7 ) * 4 ); }
};

/*****************************************************************************//**
*************** Ping Tail structure received against the ping command ************
*********************************************************************************/
class CGemPingTail: public CGemNetMsg
{
public:
  unsigned char  m_pingID;
  unsigned char  m_flags;
  unsigned short m_spare;

  CGemPingTail()
  {
    m_head.m_type = 0x43;
    m_flags       = 0;
    m_spare       = 0;
  }
  CGemPingTail& operator = ( const CGemPingTail& pingTail )
  {
    m_head              = pingTail.m_head;
    m_pingID            = pingTail.m_pingID;
    m_flags             = pingTail.m_flags;
    m_spare             = pingTail.m_spare;
    return *this;
  }
};

/*****************************************************************************//**
********** Ping Tail extended structure received against the ping command *******
*********************************************************************************/
class CGemPingTailExtended: public CGemPingTail
{
public:
  unsigned short m_firstPassRetries;
  unsigned short m_secondPassRetries;
  unsigned short m_tailRetries;
  unsigned short m_interMessageGap;
  unsigned long  m_packetCount;
  unsigned long  m_recvErrorCount;
  unsigned long  m_linesLostThisPing;
  unsigned long  m_generalCount;

  CGemPingTailExtended()
  {
    m_head.m_type       = 0x61;
    m_firstPassRetries  = 0;
    m_secondPassRetries = 0;
    m_tailRetries       = 0;
    m_interMessageGap   = 0;
    m_packetCount       = 0;
    m_recvErrorCount    = 0;
    m_linesLostThisPing = 0;
    m_generalCount      = 0;
  }

  CGemPingTailExtended& operator = ( const CGemPingTailExtended& exTail )
  {
    CGemPingTail::operator = ( exTail );

    m_head.m_type       = 0x61;
    m_firstPassRetries  = exTail.m_firstPassRetries ;
    m_secondPassRetries = exTail.m_secondPassRetries;
    m_tailRetries       = exTail.m_tailRetries      ;
    m_interMessageGap   = exTail.m_interMessageGap  ;
    m_packetCount       = exTail.m_packetCount      ;
    m_recvErrorCount    = exTail.m_recvErrorCount   ;
    m_linesLostThisPing = exTail.m_linesLostThisPing;
    m_generalCount      = exTail.m_generalCount     ;
    return *this;
  }
  CGemPingTailExtended& operator += ( const CGemPingTailExtended& exTail )
  {
    CGemPingTail::operator = ( exTail );

    m_head.m_type       = 0x61;
    m_firstPassRetries  += exTail.m_firstPassRetries ;
    m_secondPassRetries += exTail.m_secondPassRetries;
    m_tailRetries       += exTail.m_tailRetries      ;
    m_interMessageGap   += exTail.m_interMessageGap  ;
    m_packetCount       += exTail.m_packetCount      ;
    m_recvErrorCount    += exTail.m_recvErrorCount   ;
    m_linesLostThisPing += exTail.m_linesLostThisPing;
    m_generalCount      += exTail.m_generalCount     ;
    return *this;
  }
};

/*****************************************************************************//**
**************************** TimeStamp acknowledge ******************************
*********************************************************************************/
class CGemAcknowledge: public CGemNetMsg
{
public:
  unsigned int   m_receiptTimestampL;
  unsigned int   m_receiptTimestampH;
  unsigned int   m_replyTimestampL;
  unsigned int   m_replyTimestampH;

  CGemAcknowledge()
  {
    m_head.m_type       = 0x49;
    m_receiptTimestampL = 0;
    m_receiptTimestampH = 0;
    m_replyTimestampL   = 0;
    m_replyTimestampH   = 0;
  }
};


/*****************************************************************************//**
**************************** Bearing Structure**** ******************************
*********************************************************************************/
class CGemBearingData : public CGemNetMsg
{
public:
  unsigned short m_bearingLineNo;
  unsigned short m_noSamples;
  unsigned char  *m_pData;

  CGemBearingData()
  {
    m_head.m_type   = 0x60;
    m_bearingLineNo = 0;
    m_noSamples     = 0;
    m_pData         = nullptr;
  }
};

// User configuration to select speed of sound
enum CHIRP_MODE{
    CHIRP_DISABLED,
    CHIRP_ENABLED,
    CHIRP_AUTO
};


// 720/1200 ik: Frequency Mode ( Only applies 1200ik product )
enum FREQUENCY{
    FREQUENCY_AUTO, // Switch between low/high resolution based on the range threshold specified
    FREQUENCY_LOW,  // Force to run with low frequency 720Hz
    FREQUENCY_HIGH  // Force to run with high frequency 1200Hz
};

/*!
 * \struct RangeFrequencyConfig
 * \brief User configuration to select resolution mode
 */
struct RangeFrequencyConfig{
    RangeFrequencyConfig()
    : m_frequency( FREQUENCY_AUTO ) // default: resolution mode
    , m_rangeThreshold( 40.0 ) // only used in auto mode
    {
    }
    FREQUENCY   m_frequency;
    double      m_rangeThreshold;   // Configure the range threshold to switch between low/high
                                    // resolution. Range ( 1m - 50m )
};



