#pragma once

#include <string>
#include <iostream>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

// ----------------------------------------------------------------------------
// CV3 - 3D vector class
class CV3
{
public:
    typedef std::vector<CV3> CV3List;

    CV3();
    CV3(double xi, double yi, double zi);
    CV3(const CV3& vi);

    operator double() const;
    operator double*();

    CV3&    operator =(const CV3& vi);
    CV3     operator +(const CV3& vi) const;
    CV3     operator -(const CV3& vi) const;
    CV3     operator *(const CV3& vi) const;
    CV3     operator /(const CV3& vi) const;
    CV3     operator *(const double& d) const;
    CV3     operator /(const double& d) const;
    CV3&    operator +=(const CV3& vi);
    CV3&    operator -=(const CV3& vi);
    CV3&    operator /=(const CV3& vi);
    CV3&    operator *=(const CV3& vi);
    CV3&    operator /=(const double& d);
    CV3&    operator *=(const double& d);
    bool    operator ==(const CV3& vi) const;
    bool    operator !=(const CV3& vi) const;
    bool    operator <(const CV3& vi) const;
    bool    operator >(const CV3& vi) const;
    bool    operator <=(const CV3& vi) const;
    bool    operator >=(const CV3& vi) const;
    double  operator | (const CV3& vi) const;
    CV3     operator % (const CV3& vi) const;
    double& operator [] (const int& i);

    // Methods
    double* V();
    void    Zero();
    double  Length() const;
    double  Normalise();
    bool    Finite();

    double  Angle();
    double  Brg();
    double  Slope();
    void    RotateXY(const double angleRadians);

    std::string Str();

    // Data
    double x;
    double y;
    double z;
private:

    friend std::istream& operator>>(std::istream& s, CV3& e) {
        s >> e.x;
        s >> e.y;
        s >> e.z;
        return s;
      }

    friend std::ostream& operator<<(std::ostream& s, const CV3& e) {
            s << e.x << " " << e.y << " " << e.z;
           return s;
        }

    friend class boost::serialization::access;

    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & x;
        ar & y;
        ar & z;
    }
};



// Following macros allow use of standard serialize method e.g. ar << myClassInstance, ar & myClassInstance
// \cond doxygen ignore directive
BOOST_CLASS_IMPLEMENTATION(CV3, boost::serialization::object_serializable)  // Serialize without class or version info.
BOOST_CLASS_TRACKING(CV3, boost::serialization::track_never)
// \endcond


