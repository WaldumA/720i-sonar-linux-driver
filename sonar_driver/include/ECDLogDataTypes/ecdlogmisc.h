/******************************************************************************************
 *
 * \file    ecdlogmisc.h
 * \author  Phil Eccles
 * \date    26-09-2016
 * \brief   Miscellaneous classes used in the ECDLogDataTypes library. Classes are used to
 *          archive data in the ECD log file.
 *
 ******************************************************************************************/

#ifndef ECDLOGMISC_H
#define ECDLOGMISC_H

#include <string>

#include <boost/serialization/tracking.hpp>
#include <boost/serialization/serialization.hpp>

#include "ecdlogdatatypesglobal.h"





/*!
 * \class CMFC_CString
 * \brief A class to represent an MFC CString. Many existing ECD files use this string type so we have to keep it.
 *        Only implemented for Unicode strings, but for strings of any length.
 */
class ECDLOGDATATYPESSHARED_EXPORT CMFC_CString {
public:
    CMFC_CString ();     //!< \brief Constructor
    ~CMFC_CString ();    //!< \brief Destructor

    CMFC_CString (const char *str1 /*!< [in] byte character string */);  //!< \brief Constructor

    CMFC_CString ( const CMFC_CString & );                //!< \brief copy constructor
    CMFC_CString& operator = ( const CMFC_CString & );    //!< \brief assignment operator

    bool operator==(const CMFC_CString& other); //!< \brief Equality operator

    bool operator==(const char *inStr);         //!< \brief Equality operator

    /*!
     * \brief equivalent of the c_str() method found in many string implementations.
     * \return a pointer to an array that contains a null-terminated sequence of 16 bit unicode characters
     *         representing the current value of the string object
     */
    const char16_t * c_str();

private:
    friend class boost::serialization::access;

    /*!
     * \brief implementation of the boost serialize() method
     * \returns nothing
     */
    template<typename Archive>
    void serialize (Archive &ar /*!< [in] input or output archive */, const unsigned int /*!< version number UNUSED */);

    std::u16string *m_u16str;   // Unicode string (16 bits per character) because widestring characters can be of different
                                // sizes depending on the platform. Pointer to stop STL warnings when exporting DLL


};



// Following macros allow use of standard serialize method e.g. ar << myClassInstance, ar & myClassInstance
// \cond   doxygen ignore directive
BOOST_CLASS_IMPLEMENTATION(CMFC_CString, boost::serialization::object_serializable)  // Serialize without class or version info.
BOOST_CLASS_TRACKING(CMFC_CString, boost::serialization::track_never)

// \endcond




/*!
 * \class CRecordHeader
 * \brief A class to represent an ECD format record header.
 */

class ECDLOGDATATYPESSHARED_EXPORT CRecordHeader {
public:
    /*!
     * \enum  ESensorType
     * \brief Different types of header record.
     */
    enum ERecordType
    {
        eRecordTypeUndefined       = 0,    //!< Undefined record type.
        eRecordTypeSensor          = 1,    //!< Sensor record
        eRecordTypeTarget          = 2,    //!< Record containing target data
        eRecordTypeTargetImage     = 3,    //!< Record containing image data
        eRecordTypePingTailEx      = 4,    //!< Record containing extra ping information
        eRecordTypeAcousticZoom    = 5     //!< Record containing
    };

    CRecordHeader ();     //!< \brief Constructor
    CRecordHeader (const ERecordType eType/*!< [in] type of record */);    //!< \brief Constructor

    ERecordType GetRecordType ();   //!< Access record type
    void SetRecordType (const ERecordType eType/*!< [in] type of record */);    //!< \brief Set record type

private:
    friend class boost::serialization::access;

    static const unsigned short SENSOR_RECORD_TAG =              0xEFEF;
    static const unsigned short TARGET_RECORD_TAG =              0xDFDF;
    static const unsigned short IMAGE_RECORD_TAG =               0xCFCF;
    static const unsigned short PING_EXTRA_RECORD_TAG =          0xBFBF;
    static const unsigned short ACOUSTIC_ZOOM_RECORD_TAG =       0xAFAF;

    /*!
     * \brief implementation of the boost serialize() method
     * \returns nothing
     */
    template<typename Archive>
    void serialize (Archive &ar /*!< [in] input or output archive */, const unsigned int /*!< version number UNUSED */);

    // Keep these private so we can keep them correctly matched internally.
    ERecordType	m_eType;           //!< Type of record
    unsigned short  m_usVersion;   //!< Version, really a tag associated with the type.

};

// Following macros allow use of standard serialize method e.g. ar << myClassInstance, ar & myClassInstance
// \cond   doxygen ignore directive
BOOST_CLASS_IMPLEMENTATION(CRecordHeader, boost::serialization::object_serializable)  // Serialize without class or version info.
BOOST_CLASS_TRACKING(CRecordHeader, boost::serialization::track_never)

// \endcond



/*!
 * \class CLogHdrRec
 * \brief A class to represent an ECD file header.
 *
 * Contains no data members. The sole purpose of this class is to ensure the
 * validity of a header either written to or read from a file.
 */
class CLogFileHeader {
private:
    static const unsigned short version = 0x0F0F;

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
BOOST_CLASS_IMPLEMENTATION(CLogFileHeader, boost::serialization::object_serializable)  // Serialize without class or version info.
BOOST_CLASS_TRACKING(CLogFileHeader, boost::serialization::track_never)
// \endcond



/*!
 * \class CKeyValue
 * \brief A class for holding a key string and a double value.
 */

class CKeyValue
{
public:
    CKeyValue ();    //!< \brief Constructor

    CMFC_CString m_key;
    double m_value;
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
BOOST_CLASS_IMPLEMENTATION(CKeyValue, boost::serialization::object_serializable)  // Serialize without class or version info.
BOOST_CLASS_TRACKING(CKeyValue, boost::serialization::track_never)
// \endcond


#endif // ECDLOGMISC_H

