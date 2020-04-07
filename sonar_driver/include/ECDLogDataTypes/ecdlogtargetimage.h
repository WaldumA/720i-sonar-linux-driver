/******************************************************************************************
 *
 * \file    ecdlogtargetimage.h
 * \author  Phil Eccles
 * \date    26-09-2016
 * \brief   Classes used for archiving information related to the sonar image and
 *          associated target records.
 *
 ******************************************************************************************/

#ifndef ECDLOGTARGETIMAGE_H
#define ECDLOGTARGETIMAGE_H

#include <boost/serialization/serialization.hpp>

#include "MathsLib/CV3.h"

#include "ecdlogdatatypesglobal.h"
#include "ecdlogmisc.h"


/*!
 * \class CLogTgt
 * \brief A class to represent the sonar target information logged to ECD file.
 */

class ECDLOGDATATYPESSHARED_EXPORT CLogTgt
{
public:
    CLogTgt ();     //!< \brief Constructor

    unsigned int    m_iRngML;       //!< The range in milliLines
    unsigned int    m_iBrgML;       //!< The bearing in milliLines
    double          m_dRng;         //!< The range in metres
    unsigned short  m_usIntensity;  //!< The intensity
    unsigned char   m_ucQuality;    //!< The quality
    unsigned char   m_ucFlags;      //!< The additional information

private:
    friend class boost::serialization::access;

    /*!
     * \brief implementation of the boost serialize() method
     * \returns nothing
     */
    template<typename Archive>
    void serialize (Archive &ar /*!< [in] input or output archive */, const unsigned int /*!< version number UNUSED */);
};


// Following macros allow use of standard serialize method e.g. ar << myClassInstance, ar & myClassInstance
// \cond   doxygen ignore directive
BOOST_CLASS_IMPLEMENTATION(CLogTgt, boost::serialization::object_serializable)  // Serialize without class or version info.
BOOST_CLASS_TRACKING(CLogTgt, boost::serialization::track_never)
// \endcond



/*!
 * \class CPingLineExtraInfo
 * \brief A class to represent additional ping information logged in a separate record.
 */

class ECDLOGDATATYPESSHARED_EXPORT CPingLineExtraInfo
{
public:
    CPingLineExtraInfo ();     //!< \brief Constructor

    unsigned short m_usScale;        //!< Scale value
    unsigned short m_usLineInfo;     //!< Gemini specific information, ext mode and line width

private:
    friend class boost::serialization::access;

    /*!
     * \brief implementation of the boost serialize() method
     * \returns nothing
     */
    template<typename Archive>
    void serialize (Archive &ar /*!< [in] input or output archive */, const unsigned int /*!< version number UNUSED */);
};


// Following macros allow use of standard serialize method e.g. ar << myClassInstance, ar & myClassInstance
// \cond   doxygen ignore directive
BOOST_CLASS_IMPLEMENTATION(CPingLineExtraInfo, boost::serialization::object_serializable)  // Serialize without class or version info.
BOOST_CLASS_TRACKING(CPingLineExtraInfo, boost::serialization::track_never)
// \endcond




/*!
 * \class CTargetRecordECDType2
 * \brief A class to implement ECD record type 2.
 *
 * The sole purpose of this class is to serialize data to and from an ECD format stream.
 */
class ECDLOGDATATYPESSHARED_EXPORT CTargetRecordECDType2 {
public:
    CTargetRecordECDType2 (); //!< \brief Constructor
    ~CTargetRecordECDType2 (); //!< \brief Destructor

    /*!
     * \brief Calculate approximate size if written to disk.
     * \returns size
     */
    static unsigned int sizeOnDisk(const unsigned int numTargets /*!< [in] number of targets */);

    CLogTgt             *m_pcTargets;       //!< Targets stored
    unsigned int        m_uiNumTargets;     //!< Number of targets

private:
    friend class boost::serialization::access;

    /*!
     * \brief implementation of the boost serialize() method
     * \returns nothing
     */
    template<typename Archive>
    void serialize (Archive &ar /*!< [in] input or output archive */, const unsigned int /*!< version number UNUSED */);

    // Should be no need for assignment or equality operator, or copy constructor. Make private so we don't have to define.
    bool operator==(const CTargetRecordECDType2& other);
    CTargetRecordECDType2 ( const CTargetRecordECDType2 & );
    CTargetRecordECDType2& operator = ( const CTargetRecordECDType2 &);
};



// Following macros allow use of standard serialize method e.g. ar << myClassInstance, ar & myClassInstance
// \cond   doxygen ignore directive
BOOST_CLASS_IMPLEMENTATION(CTargetRecordECDType2, boost::serialization::object_serializable)  // Serialize without class or version info.
BOOST_CLASS_TRACKING(CTargetRecordECDType2, boost::serialization::track_never)
// \endcond






/*!
 * \class CTargetImageRecordECDType3
 * \brief A class to implement ECD record type 3.
 *
 * The sole purpose of this class is to serialize data to and from an ECD format stream.
 */

class ECDLOGDATATYPESSHARED_EXPORT CTargetImageRecordECDType3 {
public:
    CTargetImageRecordECDType3();  //!< brief Constructor
    ~CTargetImageRecordECDType3(); //!< brief Destructor

    /*!
     * \brief Calculate approximate size if written to disk.
     * \returns size
     */
    static unsigned int sizeOnDisk(const unsigned int numBearings,
                                   const unsigned int dataSize);



    short               m_sVersion;      		//!< Version of the ping structure
    unsigned char       m_ucPid;                //!< The unique id for this ping
    unsigned int        m_uiHalfArray;          //!< Only use the centre half of the array
    unsigned char       m_ucTxLength;           //!< tx multplies of modulation frequency
    unsigned char       m_ucScanRate;           //!< rx multplies of modulation frequency
    float               m_fSosAtXd;      		//!< The SOS at the transducer
    short               m_sShading;      		//!< Apply shading
    short               m_sMainGain;     		//!< The main gain
    short               m_sGainBlank;    		//!< Blank for auto gain
    short               m_sAdcInput;     		//!< required input level for ADC
    short               m_sSpreadGain;          //!< Spreading gain for the transducer
    short               m_sAbsorbGain;          //!< Absorbtion gain for the transducer
    unsigned int        m_uiBeamFormFocus;      //!< focus type (none, one-way, two-way, fxied)
    short               m_sBeamFormGain;        //!< The beamform gain
    float               m_fBeamFormAperture;    //!< Calculate the number of beams to provide the correct aperture
    short               m_sTxStart;      		//!< the start of the array to use
    short               m_sTxLength;        	//!< The length of the array to use
    float               m_sTxRadius;     		//!< The radius to use for defocusing
    float               m_sTxRange;        		//!< The range to focus onto
    int                 m_iModulationFrequency; //!< The modulation frequency
    short               m_sNumBeams;     		//!< The number of beams
    short               m_sRx1;          		//!< Id for rx array 1, 0 = none
    short               m_sRx2;          		//!< Id for rx array 2, 0 = none
    short               m_sTx1;          		//!< Id for tx array 1, 0 = none
    short               m_sPingFlags;    		//!< Ping Flags SOS:0x000f;
    unsigned char       m_ucRx1Array;           //!< Array type for rx array 1, 0 = none
    unsigned char       m_ucRx2Array;           //!< Array type for rx array 2, 0 = none
    unsigned char       m_ucTx1Array;           //!< Array type for tx array 1, 0 = none
    unsigned char       m_ucTx2Array;           //!< Array type for tx array 2, 0 = none
    unsigned short      m_usTaskId;             //!< Task id
    unsigned short      m_usPingId;             //!< Ping id
    double              m_dbTxTime;             //!< Time of transmit
    double              m_dbEndTime;            //!< Time of the last record
    double              m_dbTxAngle;            //!< Transmit angle (degrees)
    double              m_dbSosAvg;             //!< average speed of sound in m/s generated during this record
    unsigned int        m_uiStateFlags;         //!< State Flags (For Gemini = TIS_SCAN | TIS_COMP)
    unsigned char       m_ucBpp;                //!< characters per pixel
    unsigned int        m_uiNumRanges;          //!< number of ranges
    int                 m_iB0;                  //!< The start bearing
    int                 m_iB1;                  //!< The end bearing
    int                 m_iR0;                  //!< The start range
    int                 m_iR1;                  //!< The end range
    unsigned int        m_uiDualRx;             //!< dual receivers
    unsigned int        m_uiNumBearings;        //!< number of bearings
    double              *m_pBearingTable;       //!< bearing table pointer
    unsigned char       *m_pData;               //!< compressed data pointer
    unsigned int        m_uiSearchBoundsCount;  //!< number of search bounds

private:
    friend class boost::serialization::access;

    /*!
     * \brief implementation of the boost serialize() method
     * \returns nothing
     */
    template<typename Archive>
    void serialize (Archive &ar /*!< [in] input or output archive */, const unsigned int /*!< version number UNUSED */);

    // Should be no need for assignment or equality operator, or copy constructor. Make private so we don't have to define.
    bool operator==(const CTargetImageRecordECDType3& other);
    CTargetImageRecordECDType3 ( const CTargetImageRecordECDType3 & );
    CTargetImageRecordECDType3& operator = ( const CTargetImageRecordECDType3 &);

    void Uncompress(unsigned char* pData, unsigned char* pBlockIn, unsigned int size);
    unsigned int Compress(unsigned char* pData, unsigned char* pBlockOut, unsigned char threshold);
};


// Following macros allow use of standard serialize method e.g. ar << myClassInstance, ar & myClassInstance
// \cond   doxygen ignore directive
BOOST_CLASS_IMPLEMENTATION(CTargetImageRecordECDType3, boost::serialization::object_serializable)  // Serialize without class or version info.
BOOST_CLASS_TRACKING(CTargetImageRecordECDType3, boost::serialization::track_never)
// \endcond




/*!
 * \class CPingExtRecordECDType4
 * \brief A class to implement ECD record type 4.
 *
 * The sole purpose of this class is to serialize data to and from an ECD format stream.
 */
class ECDLOGDATATYPESSHARED_EXPORT CPingExtRecordECDType4 {
public:
    CPingExtRecordECDType4 (); //!< \brief Constructor
    ~CPingExtRecordECDType4 (); //!< \brief Destructor

    bool IsImagerType() const;     //!< Work out from header tupe if this is an imager.
    void SetHeaderType( const bool bIsImagerType );  //!< Set correct header type value.

    /*!
     * \brief Calculate approximate size if written to disk.
     * \returns size
     */
    static unsigned int sizeOnDisk(const unsigned int numElements);

    unsigned int    m_uiNumElements;        //!< Number of PingEx Data Elements

    // Software gain parameters in Ping Tail Extension Record
    unsigned short  m_usSoftwareGainRamp;    //!< Software gain ramp
    unsigned short	m_usRangeCompUsed;       //!< Range compression used in this ping
    unsigned short	m_usGainType;            //!< 0 = Gemini Mk1 imager (spreading), 1 = Gemini Mk1 profiler (software gain), 2 = Gemini Mk2 (BMG rescale)
    short			m_sTxSourceLevel;        //!< Transmit source level

    // Added for version 2
    short           m_sPercentGain;          //!< Percentage gain used by Gemini Mk1 imager (spreading) and Gemini Mk2 (BMG rescale)
    // End Software gain parameters in Ping Tail Extension Record

    // Roll compensation data in Ping Tail Extension Record. Profiler only.
    unsigned char m_ucStartApertureBeamIndex;
    unsigned char m_ucStopApertureBeamIndex;
    float m_fRollFromDMG;
    unsigned char m_ucRollCompensationStatus; //!< has roll compensation been applied?; is it failing?
    // End Roll compensation data in Ping Tail Extension Record

    // Tidal Flow vector in Ping Tail Extension Record.
    CV3 m_vTidalFlow_TI;
    // End Tidal Flow vector in Ping Tail Extension Record.

    // Imager and Profiler are main sonar types. This gives information on subtypes,
    // 720ik, 720im, 720is etc.
    unsigned short      m_usSonarSubType;

    // Unused variable but gets recorded in ECD file
    unsigned char       m_ucSonarOrientation; // Up/Down
    unsigned char       m_ucSpare;  // Spare variable

    CPingLineExtraInfo*  m_pLineExtraInfo;    //!< Pointer to the Ping Line Extra information

private:
    static const unsigned short ECD_PING_EX_HEADER_TYPE_VERSION_2_OFFSET =  32;
    static const unsigned short ECD_PING_EX_IMAGER_VERSION_1_HEADER      =  0;
    static const unsigned short ECD_PING_EX_PROFILER_VERSION_1_HEADER    =  1;
    static const unsigned short ECD_PING_EX_SEATEC_VERSION_1_HEADER      =  20;
    static const unsigned short ECD_PING_EX_IMAGER_VERSION_2_HEADER      =  0 + ECD_PING_EX_HEADER_TYPE_VERSION_2_OFFSET;
    static const unsigned short ECD_PING_EX_PROFILER_VERSION_2_HEADER    =  1 + ECD_PING_EX_HEADER_TYPE_VERSION_2_OFFSET;
    static const unsigned short ECD_PING_EX_SEATEC_VERSION_2_HEADER      =  20 + ECD_PING_EX_HEADER_TYPE_VERSION_2_OFFSET;

    friend class boost::serialization::access;

    /*!
     * \brief implementation of the boost save() method
     * \returns nothing
     */
    template<typename Archive>
    void save(Archive &ar /*!< [in] input or output archive */, const unsigned int /*!< version number UNUSED */) const;

    /*!
     * \brief implementation of the boost load() method
     * \returns nothing
     */
    template<typename Archive>
    void load(Archive &ar /*!< [in] input or output archive */, const unsigned int /*!< version number UNUSED */);

    BOOST_SERIALIZATION_SPLIT_MEMBER()

    // Should be no need for assignment or equality operator, or copy constructor. Make private so we don't have to define.
    bool operator==(const CPingExtRecordECDType4& other);
    CPingExtRecordECDType4 ( const CPingExtRecordECDType4 & );
    CPingExtRecordECDType4& operator = ( const CPingExtRecordECDType4 &);

    unsigned int    getHeaderBytesToWrite() const;

    // Type of header from Ping Tail Extension Record
    int             m_iHeaderType;          //!< type of header

};




// Following macros allow use of standard serialize method e.g. ar << myClassInstance, ar & myClassInstance
// \cond   doxygen ignore directive
BOOST_CLASS_IMPLEMENTATION(CPingExtRecordECDType4, boost::serialization::object_serializable)  // Serialize without class or version info.
BOOST_CLASS_TRACKING(CPingExtRecordECDType4, boost::serialization::track_never)
// \endcond



/*!
 * \class CAcousticZoomRecordECDType5
 * \brief A class to implement ECD record type 5.
 *
 * The sole purpose of this class is to serialize data to and from an ECD format stream.
 */
class ECDLOGDATATYPESSHARED_EXPORT CAcousticZoomRecordECDType5 {
public:
    CAcousticZoomRecordECDType5 (); //!< \brief Constructor
    ~CAcousticZoomRecordECDType5 (); //!< \brief Destructor

    /*!
     * \brief Calculate approximate size if written to disk.
     * \returns size
     */
    static unsigned int sizeOnDisk(const bool active,
                                   const unsigned int dataSize);

    unsigned char   m_ucBytesPerPixel;    //!< set from value in type 3 record, not serialized separately.
    unsigned int    m_uiChirp;            //!< switch betrween chirp and CW

    short           m_sAzID;              //!< Acoustic zoom ID
    bool            m_bActive;            //!< Active acoustic per sonar
    double          m_dbRange;            //!< Range in metres of centre of acoustic zoom box
    double          m_dbBearing;          //!< Bearing in radians of centre of acoustic zoom box
    unsigned short  m_usRangeCompUsed;    //!< Range compression used in this ping
    short           m_sNumBeams;          //!< The number of beams
    double          m_dbMagnitude;        //!< Magnification Value

    unsigned int    m_uiNumAZRngs;        //!< number of ranges
    int             m_iAZb0;              //!< The start bearing
    int             m_iAZb1;              //!< The end bearing
    int             m_iAZr0;              //!< The start range
    int             m_iAZr1;              //!< The end range
    unsigned int    m_uiNumAZBrgs;        //!< number of bearings
    unsigned char*  m_pAZData;            //!< acoustic zoom data

private:
    friend class boost::serialization::access;

    /*!
     * \brief implementation of the boost serialize() method
     * \returns nothing
     */
    template<typename Archive>
    void serialize (Archive &ar /*!< [in] input or output archive */, const unsigned int /*!< version number UNUSED */);

    // Should be no need for assignment or equality operator, or copy constructor. Make private so we don't have to define.
    bool operator==(const CAcousticZoomRecordECDType5& other);
    CAcousticZoomRecordECDType5 ( const CAcousticZoomRecordECDType5 & );
    CAcousticZoomRecordECDType5& operator = ( const CAcousticZoomRecordECDType5 &);
};




// Following macros allow use of standard serialize method e.g. ar << myClassInstance, ar & myClassInstance
// \cond   doxygen ignore directive
BOOST_CLASS_IMPLEMENTATION(CAcousticZoomRecordECDType5, boost::serialization::object_serializable)  // Serialize without class or version info.
BOOST_CLASS_TRACKING(CAcousticZoomRecordECDType5, boost::serialization::track_never)
// \endcond





/*!
 * \class CTgt
 * \brief A class to represent the sonar target information.
 */

class ECDLOGDATATYPESSHARED_EXPORT CTgt: public CLogTgt, CV3
{

};





/*!
 * \class CBeamInfo
 * \brief A class containing all the information related to number of beams for one ping.
 *        Stored separately from CPing class so all the dynamic fields are in one place,
 *        makes maintenance of assigment operator and copy constructor for CPing easier.
 */
class ECDLOGDATATYPESSHARED_EXPORT CBeamInfo
{
public:
    CBeamInfo (); //!< \brief Constructor
    ~CBeamInfo (); //!< \brief Destructor

    CBeamInfo ( const CBeamInfo & /*!< [in] instance to copy */ );   //!< \brief copy constructor
    CBeamInfo& operator = ( const CBeamInfo & /*!< [in] instance to copy */); //!< \brief assignment operator

    //!< \brief Allocate space for bearing table.
    void AllocateBearingTable( const unsigned int uiNewBearingSize /*!< [in] number of beams */);

    //!< \brief Load data from ECD type 3 record into the object.
    void LoadFromECDRecord( const CTargetImageRecordECDType3 &cTargetImageRecord );
    //!< \brief Store data from the object into ECD type 3 record
    void SaveToECDRecord( CTargetImageRecordECDType3 &cTargetImageRecord ) const;

    short               m_sNumBeams;     		//!< The number of beams
    double              *m_pBearingTable;       //!< bearing table pointer

private:
    // force a compile error if == operator called, only define if necessary.
    bool operator==(const CBeamInfo& other);
    void CopyValues(const CBeamInfo & );
    void InitValues();
};







/*!
 * \class CAcousticZoom
 * \brief A class containing all the acoustic zoom data for one ping.
 */
class ECDLOGDATATYPESSHARED_EXPORT CAcousticZoom
{
public:
    CAcousticZoom (); //!< \brief Constructor

    //!< \brief Load data from ECD type 5 record into the object.
    void LoadFromECDRecord( const CAcousticZoomRecordECDType5 &cAcousticZoomRecord );
    //!< \brief Store data from the object into ECD type 5 record.
    void SaveToECDRecord( CAcousticZoomRecordECDType5 &cAcousticZoomRecord ) const;

    short           m_sAzID;              //!< Acoustic zoom ID
    bool            m_bActive;            //!< Active acoustic per sonar
    double          m_dbRange;            //!< Range in metres of centre of acoustic zoom box
    double          m_dbBearing;          //!< Bearing in radians of centre of acoustic zoom box
    unsigned short  m_usRangeCompUsed;    //!< Range compression used in this ping
    short           m_sNumBeams;          //!< The number of beams
    double          m_dbMagnitude;        //!< Magnification Value
};




/*!
 * \class CPing.
 * \brief A class containing all the data required for one sonar ping.
 */
class ECDLOGDATATYPESSHARED_EXPORT CPing {
public:
    /*!
     * \enum  EBeamFormFocusType
     * \brief Different types of beamform focus.
     */
    enum EBeamFormFocusType
    {
        eFocusTypeNone            = 0,    //!< Undefined record type.
        eFocusTypeOneWay          = 1,    //!< One way focus
        eFocusTypeTwoWay          = 2,    //!< Two way focus
        eFocusTypeFixed           = 3     //!< Fixed focus
    };

    CPing (); //!< \brief Constructor

    //!< \brief Load data from ECD type 3 record into the object.
    void LoadFromECDRecord( const CTargetImageRecordECDType3 &cTargetImageRecord );
    //!< \brief Store data from the object into ECD type 3 record
    void SaveToECDRecord( CTargetImageRecordECDType3 &cTargetImageRecord ) const;

    //!< \brief Load data from ECD type 5 record into the object.
    void LoadFromECDRecord( const CAcousticZoomRecordECDType5 &cAcousticZoomRecord );
    //!< \brief Store data from the object into ECD type 5 record.
    void SaveToECDRecord( CAcousticZoomRecordECDType5 &cAcousticZoomRecord ) const;

    //!< \brief Get the equivalent range in metres ffrom a specified range line.
    double GetRangeLineInMetres(const unsigned int) const;

    unsigned char       m_ucPid;                //!< The unique id for this ping
    bool                m_bHalfArray;           //!< Only use the centre half of the array
    unsigned char       m_ucTxLength;           //!< tx multplies of modulation frequency
    unsigned char       m_ucScanRate;           //!< rx multplies of modulation frequency
    float               m_fSosAtXd;      		//!< The SOS at the transducer
    short               m_sShading;      		//!< Apply shading
    short               m_sMainGain;     		//!< The main gain
    short               m_sGainBlank;    		//!< Blank for auto gain
    short               m_sAdcInput;     		//!< required input level for ADC
    short               m_sSpreadGain;          //!< Spreading gain for the transducer
    short               m_sAbsorbGain;          //!< Absorbtion gain for the transducer
    EBeamFormFocusType  m_eBeamFormfFocus;      //!< focus type (none, one-way, two-way, fxied)
    short               m_sBeamFormGain;        //!< The beamform gain
    float               m_fBeamFormAperture;    //!< Calculate the number of beams to provide the correct aperture
    short               m_sTxStart;      		//!< the start of the array to use
    short               m_sTxLength;        	//!< The length of the array to use
    float               m_sTxRadius;     		//!< The radius to use for defocusing
    float               m_sTxRange;        		//!< The range to focus onto
    int                 m_iModulationFrequency; //!< The modulation frequency
    short               m_sRx1;          		//!< Id for rx array 1, 0 = none
    short               m_sRx2;          		//!< Id for rx array 2, 0 = none
    short               m_sTx1;          		//!< Id for tx array 1, 0 = none
    short               m_sPingFlags;    		//!< Ping Flags SOS:0x000f;
    unsigned char       m_ucRx1Array;           //!< Array type for rx array 1, 0 = none
    unsigned char       m_ucRx2Array;           //!< Array type for rx array 2, 0 = none
    unsigned char       m_ucTx1Array;           //!< Array type for tx array 1, 0 = none
    unsigned char       m_ucTx2Array;           //!< Array type for tx array 2, 0 = none
    bool                m_bChirp;               //!< switch betrween chirp and CW
    CAcousticZoom       m_clAcousticZoom;       //!< Acoustic Zoom Information
    CBeamInfo           m_clBeamInfo;           //!< Information related to number of beams

};






/*!
 * \class CTgtRec
 * \brief Stores an array of targets for one image frame.
 */
class ECDLOGDATATYPESSHARED_EXPORT CTgtRec {
public:
    CTgtRec (); //!< \brief Constructor
    ~CTgtRec (); //!< \brief Destructor

    CTgtRec ( const CTgtRec & /*!< [in] instance to copy */ );   //!< \brief copy constructor
    CTgtRec& operator = ( const CTgtRec & /*!< [in] instance to copy */); //!< \brief assignment operator

    CTgt& operator[] (const int index);  //!< \brief array subscript operator

    unsigned int Size () const;  //!< \brief return size of target array

    void SetSize (const unsigned int count);   //!< set size of target array

    //!< \brief Load data from ECD type 2 record into the object.
    void LoadFromECDRecord( const CTargetRecordECDType2 &cTargetRecord );
    //!< \brief Store data from the object into ECD type 2 record
    void SaveToECDRecord( CTargetRecordECDType2 &cTargetRecord ) const;

    //!< \brief Load data from ECD type 3 record into the object.
    void LoadFromECDRecord( const CTargetImageRecordECDType3 &cTargetImageRecord );
    //!< \brief Store data from the object into ECD type 3 record
    void SaveToECDRecord( CTargetImageRecordECDType3 &cTargetImageRecord ) const;

    //!< \brief Load data from ECD type 5 record into the object.
    void LoadFromECDRecord( const CAcousticZoomRecordECDType5 &cAcousticZoomRecord );
    //!< \brief Store data from the object into ECD type 5 record.
    void SaveToECDRecord( CAcousticZoomRecordECDType5 &cAcousticZoomRecord ) const;



    CPing               m_clPing;			//!< Ping record
    unsigned short      m_usTaskId;			//!< Task id
    unsigned short      m_usPingId;			//!< Ping id
    double              m_dbTxTime;			//!< Time of transmit
    double              m_dbEndTime;		//!< Time of the last record
    double              m_dbTxAngle;		//!< Transmit angle (degrees)
    double              m_dbSosAvg;			//!< average speed of sound in m/s generated during this record

private:
    std::vector<CTgt> *m_pVecTargets;  // Pointer to stop STL warnings when exporting DLL
    // force a compile error if == operator called, only define if necessary.
    bool operator==(const CTgtRec& other);
    void CopyValues(const CTgtRec & );
    void InitValues();
};




/*!
 * \class CTgtImg
 * \brief A class to contain all of the sonar acquisition data for one ping.
 */
class ECDLOGDATATYPESSHARED_EXPORT CTgtImg {
public:
    /*!
     * \enum  EGeminiGainType
     * \brief Different types of gain..
     */
    enum EGeminiGainType
    {
        eGeminiGainMk1Imager            = 0,    //!< Gemini Mk1 imager (spreading)
        eGeminiGainMk1Profiler          = 1,    //!< Gemini Mk1 profiler (software gain)
        eGeminiGainMk2Imager            = 2     //!< Gemini Mk2 (BMG? rescale)
    };

    /*!
     * \enum  ESonarSubTypeType
     * \brief Gives further information on the exact type of sonar
     * Imager and Profiler are main sonar types. This gives information on subtypes,
     * 720ik, 720im, 720is etc.
     */
    enum ESonarSubTypeType
    {
        eSonarSubTypeNone          = 0,    //!< No further information on the type of sonar
        eGemini720is               = 1,    //!< Gemini 720is
        eGemini720ik               = 2,    //!< Gemini 720ik
        eGemini720im               = 3,    //!< Gemini 720im
        eGemini1200ik              = 4     //!< Gemini 1200ik
    };


    static const unsigned char  TARGET_IMAGE_QI                   =    0x01;
    static const unsigned char  TARGET_IMAGE_PHASE_AMPLITUDE      =    0x02;
    static const unsigned char  TARGET_IMAGE_SCAN                 =    0x04;
    static const unsigned char  TARGET_IMAGE_COMPRESSED           =    0x08;
    static const unsigned char  TARGET_IMAGE_RANGE_BEARING        =    0x10;
    static const unsigned char  TARGET_IMAGE_TARGET               =    0x20;
    static const unsigned char  TARGET_IMAGE_XYZ                  =    0x40;
    static const unsigned char  TARGET_IMAGE_SHARPEN              =    0x80;

    CTgtImg(); //!< brief Constructor

    // Make destructor virtual because we are almost bound to derive from this class.
    virtual ~CTgtImg();    //!< brief Destructor

    //!< \brief copy constructor
    CTgtImg ( const CTgtImg & /*!< [in] instance to copy */);

    //!< \brief assignment operator
    CTgtImg& operator = ( const CTgtImg & /*!< [in] instance to copy */);

    //!< \brief Allocate space for ping data
    void AllocateData(const unsigned int uiRanges, /*!< [in] Range lines value */
                      const unsigned int uiBearings /*!< [in] numbe of bearing lines  */
                      );
    //!< \brief Allocate space for extra line Info structure.
    void AllocateExtraLineInfo(const unsigned int uiRanges, /*!< [in] Range lines value */
                               const unsigned int uiLines /*!< [in] numbe of range lines  */
                              );
    //!< \brief Allocate space for bearing table.
    void AllocateBearingTable( const unsigned int uiNewBearingSize /*!< [in] number of beams */
                                            );

    //!< \brief Allocate space for acoustic zoom data.
    void AllocateAZData(const unsigned int uiRanges, /*!< [in] Range lines value */
                                    const unsigned int uiBearings /*!< [in] numbe of bearing lines  */
                                    );

    //!< \brief Load data from ECD type 2 record into the object.
    void LoadFromECDRecord( const CTargetRecordECDType2 &cTargetRecord );
    //!< \brief Store data from the object into ECD type 2 record.
    void SaveToECDRecord( CTargetRecordECDType2 &cTargetRecord ) const;

    //!< \brief Load data from ECD type 3 record into the object.
    void LoadFromECDRecord( const CTargetImageRecordECDType3 &cTargetImageRecord );
    //!< \brief Store data from the object into ECD type 3 record.
    void SaveToECDRecord( CTargetImageRecordECDType3 &cTargetImageRecord ) const;

    //!< \brief Load data from ECD type 4 record into the object.
    void LoadFromECDRecord( const CPingExtRecordECDType4 &cPingExtRecord );
    //!< \brief Store data from the object into ECD type 4 record.
    void SaveToECDRecord( CPingExtRecordECDType4 &cPingExtRecord ) const;

    //!< \brief Load data from ECD type 5 record into the object.
    void LoadFromECDRecord( const CAcousticZoomRecordECDType5 &cAcousticZoomRecord );
    //!< \brief Store data from the object into ECD type 5 record.
    void SaveToECDRecord( CAcousticZoomRecordECDType5 &cAcousticZoomRecord ) const;

    /*!
     * \brief Calculate approximate size if written to disk.
     * \returns size
     */
    unsigned int sizeOnDisk();

    /*!
     * \brief  Given an actual range and bearing, return the range and bearing number
     * (essentially an index into the number of range lines or bearing lines)
     *
     * \returns Returns: true if able to work out range and bearing number, false otherwise.
     */
    bool getRangeBearingNumbers(double rangeMetres, double bearingDegrees,
                                int &rangeNumber, int &bearingNumber) const;

    /*!
     * \brief  Given a range and bearing number (essentially an index into the number of range lines
     * or bearing lines), return actual range and bearing.
     *
     * \returns Returns: true if able to work out range and bearing number, false otherwise.
     */
    bool getRangeBearingFromNumbers(const int rangeNumber, const int bearingNumber,
                                    double &rangeMetres, double &bearingDegrees) const;

    double getTime() const {return m_clTgtRec.m_dbTxTime;}

    CTgtRec 		m_clTgtRec;			 //!< Target Record
    unsigned int 	m_uiStateFlags;      //!< State Flags (For Gemini = TIS_SCAN | TIS_COMP)
    unsigned int	m_uiNumRanges;       //!< number of ranges
    int 			m_iB0;               //!< The start bearing
    int 			m_iB1;               //!< The end bearing
    int 			m_iR0;               //!< The start range
    int 			m_iR1;               //!< The end range
    bool            m_bDualRx;			 //!< dual receivers
    unsigned int    m_uiNumBearings;	 //!< number of bearings
    unsigned char  *m_pData;			 //!< compressed data pointer

    // Software gain parameters in Ping Tail Extension Record
    unsigned short  m_usSoftwareGainRamp;    //!< Software gain ramp
    unsigned short	m_usRangeCompUsed;       //!< Range compression used in this ping
    EGeminiGainType	m_eGainType;             //!< 0 = Gemini Mk1 imager (spreading), 1 = Gemini Mk1 profiler (software gain), 2 = Gemini Mk2 (BMG rescale)
    short			m_sTxSourceLevel;        //!< Transmit source level

    // Added for version 2
    short           m_sPercentGain;          //!< Percentage gain used by Gemini Mk1 imager (spreading) and Gemini Mk2 (BMG rescale)
    // End Software gain parameters in Ping Tail Extension Record

    // Roll compensation data in Ping Tail Extension Record. Profiler only.
    unsigned char m_ucStartApertureBeamIndex;
    unsigned char m_ucStopApertureBeamIndex;
    float m_fRollFromDMG;
    unsigned char m_ucRollCompensationStatus; //!< has roll compensation been applied?; is it failing?
    // End Roll compensation data in Ping Tail Extension Record

    // Tidal Flow vector in Ping Tail Extension Record.
    CV3 m_vTidalFlow_TI;
    // End Tidal Flow vector in Ping Tail Extension Record.

    // Imager and Profiler are main sonar types. This gives information on subtypes,
    // 720ik, 720im, 720is etc.
    ESonarSubTypeType      m_eSonarSubType;

    // Unused variable but gets recorded in ECD file
    unsigned char       m_ucSonarOrientation; // Up/Down
    unsigned char       m_ucSpare;  // Spare variable


    CPingLineExtraInfo*  m_pLineExtraInfo;    //!< Pointer to the Ping Line Extra information

    unsigned int    m_uiNumAZRngs;        //!< number of acoustic zoom ranges
    int             m_iAZb0;              //!< The AZ start bearing
    int             m_iAZb1;              //!< The AZ end bearing
    int             m_iAZr0;              //!< The AZ start range
    int             m_iAZr1;              //!< The AZ end range
    unsigned int    m_uiNumAZBrgs;        //!< number of acoustic zoom bearings
    unsigned char*  m_pAZData;            //!< acoustic zoom data

//private:
    /*!
     * \enum  ESonarType
     * \brief Different types of sonar.
     */
    enum ESonarType
    {
        eSonarTypeImager            = 0,    //!< Imager type sonar
        eSonarTypeProfiler          = 1     //!< Profiler type sonar
    };

    // force a compile error if == operator called, only define if necessary.
    bool operator==(const CTgtImg& other);
    void CopyValues(const CTgtImg & );
    void InitValues();

    // Type of sonar from Ping Tail Extension Record
    ESonarType             m_eSonarType;        //!< type of sonar

    unsigned char 	m_ucBpp;                // characters per pixel
    unsigned int    m_uiLineXAllocSize;     // Number of PingEx Data Elements
};


#endif // ECDLOGTARGETIMAGE_H

