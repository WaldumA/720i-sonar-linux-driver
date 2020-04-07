#pragma once

#include <cstdint>
#include "DataTypes.h"

#include <string>
#include <vector>

namespace tdi{

typedef std::uint64_t UInt64;
typedef std::uint32_t UInt32;
typedef std::uint16_t UInt16;
typedef std::uint8_t  UInt8;

typedef std::int64_t  Int64;
typedef std::int32_t  Int32;
typedef std::int16_t  Int16;
typedef std::int8_t   Int8;


typedef std::uint8_t  Byte;
typedef std::int8_t   Char;

typedef UInt32      HAL_STATUS;
typedef void*       HAL_HANDLE;
typedef void*       CONSOLE_HANDLE;
typedef void*       HPROCESSOR;
typedef void*       HFTDI;
typedef void*       COM_HANDLE;

typedef UInt32      ErrorCode;

typedef std::vector<std::pair<std::string, UInt32>> CommPortMapping;
typedef CommPortMapping::iterator CommPortMappingItr;

typedef std::vector<std::string>ListDevices;
typedef ListDevices::iterator DevListItr;

typedef std::vector<std::string> VectorString;

}

