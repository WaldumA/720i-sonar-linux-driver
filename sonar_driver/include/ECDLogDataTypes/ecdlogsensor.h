/******************************************************************************************
 *
 * \file    ecdlogsensor.h
 * \author  Phil Eccles
 * \date    26-09-2016
 * \brief   Classes used to represent sensor data on a stream, GPS, gyro etc.
 *
 ******************************************************************************************/


#ifndef ECDLOGSENSOR_H
#define ECDLOGSENSOR_H

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>

#include "MathsLib/CV3.h"

#include "ecdlogdatatypesglobal.h"
#include "template_memory.h"
#include "ecdlogmisc.h"
#include "TimeLib/Time80.h"

#include <sstream>

//using namespace boost::archive;

/*!
 * \class CSensorInfo
 * \brief A base class for specific sensor stream data. Can also be instantiated.
 */
class ECDLOGDATATYPESSHARED_EXPORT CSensorInfo
{
public:
    /*!
     * \enum  ESensorType
     * \brief Different types of sensor data.
     */
    enum ESensorType
    {
        eSensorTypeRaw          = 0,    //!< Basic sensor data
        eSensorTypePos          = 1,    //!< Positional sensor data
        eSensorTypeGyro         = 2,    //!< Gyro sensor data
        eSensorTypeMru          = 3,    //!< Motion Reference sensor data
        eSensorTypeZ            = 4,    //!< Depth (pressure) sensor data
        eSensorTypeArt          = 5,    //!< Articulation sensor message
        eSensorTypeSos          = 6,    //!< Speed of Sound sensor message
        eSensorTypeTime         = 7,    //!< Timestamp sensor message
        eSensorTypeAlt          = 8,    //!< Altimeter sensor message
        eSensorTypeClient       = 9,    //!< Client specific sensor message
        eSensorTypeSpeedCourse  = 10,   //!< Speed and Course
        eSensorTypeSettings     = 11,   //!< Settings change
        eSensorTypePressure     = 18,   //!< Psea pressure
        eSensorTypeClientBinaryData = 19 //!< Binary data specific to the client
    };

    CSensorInfo ();    //!< \brief Constructor

    virtual ~CSensorInfo(){}

    // Cannot have one templated method as it is virtual.
    //! Serialize from input archive
    virtual bool Serialize (boost::archive::binary_iarchive &ai /*! [in] input archive */)
    {return SerializeTemplate (ai);}

    //! Serialize to output archive
    virtual bool Serialize (boost::archive::binary_oarchive &ao /*! [in] output archive */)
    {return SerializeTemplate (ao);}

    /*!
     * Returns a sensor class of the appropriate type based upon input
     */
    static CSensorInfo* Create(const ESensorType type /*!< [in] sensor type */);

    // Data
    ESensorType      m_eType;       //!< Type (equivalent to ESensorType) PAE
    unsigned char    m_ucBank;      //!< Bank number
    CMFC_CString     m_clStr;       //!< String containing information
    short            m_sMsgStatus;  //!< Status
    double           m_dbTime;      //!< Timestamp

private:
    // Cannot make serialize a virtual method as it is templated.
    template<typename Archive>
    bool SerializeTemplate (Archive &ar);
};


/*!
 * \class CSensorPos
 * \brief A class for positional sensor data.
 */
class CSensorPos: public CSensorInfo
{
public:
    CSensorPos ();    //!< \brief Constructor

    //! Serialize from input archive
    virtual bool Serialize (boost::archive::binary_iarchive &ai /*! [in] input archive */)
    {return SerializeTemplate (ai);}

    //! Serialize to output archive
    virtual bool Serialize (boost::archive::binary_oarchive &ao /*! [in] output archive */)
    {return SerializeTemplate (ao);}

    // Data
    CV3   m_clGeo;      //!< The geodetic position
    CV3   m_clGrid;     //!< The grid position
    float m_fConv;      //!< The convergence
    float m_fHDop;      //!< The horizontal dilution of precision
    float m_fPDop;      //!< The positional dilution of precision
    short m_sNumSats;   //!< Number of satellites

private:
    template<typename Archive>
    bool SerializeTemplate (Archive &ar);

};



/*!
 * \class CSensorGyro
 * \brief A class for gyro sensor data.
 */

class CSensorGyro: public CSensorInfo
{
public:
    CSensorGyro ();    //!< \brief Constructor

    //! Serialize from input archive
    virtual bool Serialize (boost::archive::binary_iarchive &ai /*! [in] input archive */)
    {return SerializeTemplate (ai);}

    //! Serialize to output archive
    virtual bool Serialize (boost::archive::binary_oarchive &ao /*! [in] output archive */)
    {return SerializeTemplate (ao);}

    // Data
    float m_fHeading;       //!< Heading in degrees
    float m_fRateOfTurn;    //!< Rate of Turn in deg/s

private:
    template<typename Archive>
    bool SerializeTemplate (Archive &ar);
};




/*!
 * \class CSensorMRU
 * \brief A class for motion referenced sensor data.
 */
class CSensorMRU: public CSensorInfo
{
public:
    CSensorMRU ();    //!< \brief Constructor

    //! Serialize from input archive
    virtual bool Serialize (boost::archive::binary_iarchive &ai /*! [in] input archive */)
    {return SerializeTemplate (ai);}

    //! Serialize to output archive
    virtual bool Serialize (boost::archive::binary_oarchive &ao /*! [in] output archive */)
    {return SerializeTemplate (ao);}

    // Data
    float m_fPitch;   //!< The pitch value +Anti looking-'ve X
    float m_fRoll;    //!< The roll value +Clock looking +'ve Y
    float m_fHeave;   //!< The heave value +Up
    float m_fSway;    //!< The sway value +Stdb
    float m_fSurge;   //!< The surge value +Fwd

private:
    template<typename Archive>
    bool SerializeTemplate (Archive &ar);
};




/*!
 * \class CSensorZ
 * \brief A class for water depth (pressure) sensor data.
 */
class CSensorZ: public CSensorInfo
{
public:
    CSensorZ ();    //!< \brief Constructor

    //! Serialize from input archive
    virtual bool Serialize (boost::archive::binary_iarchive &ai /*! [in] input archive */)
    {return SerializeTemplate (ai);}

    //! Serialize to output archive
    virtual bool Serialize (boost::archive::binary_oarchive &ao /*! [in] output archive */)
    {return SerializeTemplate (ao);}

    // Data
    float m_fZ;      //!< The z value as a height
    float m_fScale;  //!< The scale factor
    // Not sure what units means, may be irrelevant. Phil Eccles
    short m_sUnits;  //!< The units of z

private:
    template<typename Archive>
    bool SerializeTemplate (Archive &ar);

};




/*!
 * \class CSensorArt
 * \brief A class for articulation sensor data.
 */
class CSensorArt: public CSensorInfo
{
public:
    enum EArticulationType
    {
        eArticulationTypeX = 0,
        eArticulationTypeY = 1,
        eArticulationTypeZ = 2
    };

    CSensorArt ();    //!< \brief Constructor

    //! Serialize from input archive
    virtual bool Serialize (boost::archive::binary_iarchive &ai /*! [in] input archive */)
    {return SerializeTemplate (ai);}

    //! Serialize to output archive
    virtual bool Serialize (boost::archive::binary_oarchive &ao /*! [in] output archive */)
    {return SerializeTemplate (ao);}

    // Data
    int   m_iArticulationType;      //!< specifies orientation angle is with respect to
    float m_fArticlulation;         //!< Actual value to use
    float m_fArticulationRef0;      //!< reference angle 0
    float m_fArticulationRef1;      //!< reference angle 1

private:
    template<typename Archive>
    bool SerializeTemplate (Archive &ar);
};


/*!
 * \class CSensorSOS
 * \brief A class for speed of sound sensor data.
 */
class CSensorSOS: public CSensorInfo
{
public:
    CSensorSOS ();    //!< \brief Constructor

    //! Serialize from input archive
    virtual bool Serialize (boost::archive::binary_iarchive &ai /*! [in] input archive */)
    {return SerializeTemplate (ai);}

    //! Serialize to output archive
    virtual bool Serialize (boost::archive::binary_oarchive &ao /*! [in] output archive */)
    {return SerializeTemplate (ao);}

    // Data
    float m_fSos;            //!< Speed of sound in m/s.

private:
    template<typename Archive>
    bool SerializeTemplate (Archive &ar);
};



/*!
 * \class CSensorTime
 * \brief A class for timestamp sensor data.
 */
class CSensorTime: public CSensorInfo
{
public:
    CSensorTime ();    //!< \brief Constructor

    //! Serialize from input archive
    virtual bool Serialize (boost::archive::binary_iarchive &ai /*! [in] input archive */)
    {return SerializeTemplate (ai);}

    //! Serialize to output archive
    virtual bool Serialize (boost::archive::binary_oarchive &ao /*! [in] output archive */)
    {return SerializeTemplate (ao);}

    // Data
    CTime80 m_recordedTime;                //!< the recorded time (not the timestamp) PAE should be CTm80 or something
    unsigned char  m_ucTzNum;       //!< The time zone number
    unsigned char  m_ucTzMins;      //!< Time zone minutes

private:
    template<typename Archive>
    bool SerializeTemplate (Archive &ar);

};



/*!
 * \class CSensorAlt
 * \brief A class for altimeter sensor data.
 */
class CSensorAlt: public CSensorInfo
{
public:
    CSensorAlt ();    //!< \brief Constructor

    //! Serialize from input archive
    virtual bool Serialize (boost::archive::binary_iarchive &ai /*! [in] input archive */)
    {return SerializeTemplate (ai);}

    //! Serialize to output archive
    virtual bool Serialize (boost::archive::binary_oarchive &ao /*! [in] output archive */)
    {return SerializeTemplate (ao);}

    // Data
    float m_fAlt; //!< The altimeter value

private:
    template<typename Archive>
    bool SerializeTemplate (Archive &ar);

};



/*!
 * \class CSensorClient
 * \brief A class for client specific sensor data.
 */
class CSensorClient: public CSensorInfo
{
public:
    CSensorClient ();    //!< \brief Constructor

    //! Serialize from input archive
    virtual bool Serialize (boost::archive::binary_iarchive &ai /*! [in] input archive */)
    {return SerializeTemplate (ai);}

    //! Serialize to output archive
    virtual bool Serialize (boost::archive::binary_oarchive &ao /*! [in] output archive */)
    {return SerializeTemplate (ao);}

    // Data
    // If CSensorClient is ever explicitly exported from the dll, need to make
    // m_vecData a pointer as bad practice to export stl types in DLLs.
    std::vector<CKeyValue> m_vecData; //!< list of key value pairs

private:
    template<typename Archive>
    bool SerializeTemplate (Archive &ar);

};



/*!
 * \class CSensorSpeedCourse
 * \brief A class for speed and course sensor data.
 */
class CSensorSpeedCourse: public CSensorInfo
{
public:
    CSensorSpeedCourse ();    //!< \brief Constructor

    //! Serialize from input archive
    virtual bool Serialize (boost::archive::binary_iarchive &ai /*! [in] input archive */)
    {return SerializeTemplate (ai);}

    //! Serialize to output archive
    virtual bool Serialize (boost::archive::binary_oarchive &ao /*! [in] output archive */)
    {return SerializeTemplate (ao);}

    // Data
    float m_fCogT;  // Course over ground (True)
    float m_fCogM;  // Course over ground (Magnetic)
    float m_fSogN;  // Speed over ground (Knots)
    float m_fSogK;  // Speed over ground (km/hr)

private:
    template<typename Archive>
    bool SerializeTemplate (Archive &ar);
};


/*!
 * \class CSensorSettings
 * \brief A class used on a stream to indicate settings have changed.
 */
class CSensorSettings: public CSensorInfo
{
};



/*!
 * \class CSensorClientBinaryData
 * \brief A class for holding client-specific binary data.
 *        Note it is the job of the client to verify the contents of the raw binary data.
 */
class CSensorClientBinaryData: public CSensorInfo
{
public:
    /*!
     * \enum  ECompressionType
     * \brief Different types of compression.
     */
    enum ECompressionType
    {
        eCompressionNone        = 0,    //!< No compression
        eCompressionFastest     = 1,    //!< Fastest compression
        eCompressionBest        = 2,    //!< Highest compression, slowest
        eCompressionDefault     = 3     //!< Default compression
    };

    CSensorClientBinaryData ();    //!< \brief Constructor
    ~CSensorClientBinaryData ();    //!< \brief Destructor

    //! Serialize from input archive
    virtual bool Serialize (boost::archive::binary_iarchive &ai /*! [in] input archive */)
    {return SerializeTemplate (ai);}

    //! Serialize to output archive
    virtual bool Serialize (boost::archive::binary_oarchive &ao /*! [in] output archive */)
    {return SerializeTemplate (ao);}

    /*!
     * Template method to encode a type in the binary array.
     * boost::serialization must be defined for the type.
     */
    template<typename T>
    void encodeData(const T &thing, const CMFC_CString &idStr,
                    const ECompressionType eCompressionType)
    {
        m_clStr = idStr;
        std::stringstream ss;

        int compressionType = 0;

        switch (eCompressionType)
        {
        case eCompressionFastest:
            compressionType = boost::iostreams::zlib::best_speed;
            break;
        case eCompressionBest:
            compressionType = boost::iostreams::zlib::best_compression;
            break;
        case eCompressionDefault:
            compressionType = boost::iostreams::zlib::default_compression;
            break;
        default:
            break;
        }

        switch (eCompressionType)
        {
        case eCompressionFastest:
        case eCompressionBest:
        case eCompressionDefault:
            // These braces are very important. No "flush" in zlib_compressor so without forcing it to go out of scope,
            // we end up without the full compressed string in ss.
            {
                boost::iostreams::filtering_streambuf<boost::iostreams::output> out;
                out.push(boost::iostreams::zlib_compressor(compressionType));
                out.push(ss);

                boost::archive::binary_oarchive oa(out, boost::archive::no_header);

                oa << thing;
            }
            break;
        default:
            {
                boost::archive::binary_oarchive oa(ss, boost::archive::no_header);
                oa << thing;
            }
            break;
        }

        unsigned int newDataSize = (unsigned int)ss.tellp();

        handleMemoryAllocation(m_data, newDataSize, m_dataSize);
        m_dataSize = newDataSize;

        ss.read((char *)m_data, newDataSize);
    }

    /*!
     * Template method to decode a type from the binary array.
     * boost::serialization must be defined for the type.
     */
    template<typename T>
    void decodeData(T &thing, const bool isCompressed)
    {
        boost::iostreams::basic_array_source<char> device((char *)m_data, m_dataSize);
        boost::iostreams::stream<boost::iostreams::basic_array_source<char> > s2(device);

        if (! isCompressed)
        {
            boost::archive::binary_iarchive ia(s2, boost::archive::no_header);
            ia >> thing;
        }
        else
        {
            boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
            in.push(boost::iostreams::zlib_decompressor());
            in.push(s2);

            boost::archive::binary_iarchive ia(in, boost::archive::no_header);
            ia >> thing;
        }
    }

    // Data
    unsigned int m_dataSize; // size of binary data
    unsigned char *m_data;   // binary data

private:
    template<typename Archive>
    bool SerializeTemplate (Archive &ar);

    // Should be no need for assignment or equality operator, or copy constructor. Make private so we don't have to define.
    bool operator==(const CSensorClientBinaryData& other);
    CSensorClientBinaryData ( const CSensorClientBinaryData & );
    CSensorClientBinaryData& operator = ( const CSensorClientBinaryData &);
};



#endif // ECDLOGSENSOR_H

