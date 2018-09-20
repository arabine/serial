#include "SerialPort.h"

#include "Util.h"
#include "serial.h"

#include <iomanip>
#include <memory>
#include <locale.h>
#include <regex>
#include <iostream>

#include <Windows.h>
#include <SetupAPI.h>
#include <devpkey.h>
#include <tchar.h>
#include <initguid.h>
#include <devpropdef.h>

//typedef std::vector<UINT> CPortsArray;
//typedef std::wstring String;
//typedef std::vector<String> CNamesArray;
//typedef std::vector<std::pair<UINT, String> > CPortAndNamesArray;


//static CPortsArray ports;

DEFINE_GUID(GUID_DEVCLASS_PORTS, 0x4D36E978, 0xE325, 0x11CE, 0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18 );


struct FtdiDevice
{
    FtdiDevice()
        : location(0U)
        , comPort(-1)
    {

    }

    FtdiDevice(unsigned long loc, int com, const std::string &ser, const std::string &des)
        : location(loc)
        , comPort(com)
        , serialNumber(ser)
        , description(des)
    {

    }

    unsigned long location;
    int comPort;
    std::string serialNumber;
    std::string description;
};

static std::vector<FtdiDevice> gFtdiDeviceList; // list for FTDI chips
static std::vector<SerialInfos> gSerialList; // System list of com ports


std::string getDeviceProperty(HDEVINFO devInfo, PSP_DEVINFO_DATA devData, DWORD property)
{
    DWORD buffSize = 0;
    std::string result;
    SetupDiGetDeviceRegistryProperty(devInfo, devData, property, nullptr, nullptr, 0, & buffSize);

    if (buffSize > 0)
    {
        BYTE* buff = new BYTE[buffSize];
        SetupDiGetDeviceRegistryProperty(devInfo, devData, property, nullptr, buff, buffSize, nullptr);
        result = Util::ToString(std::wstring(reinterpret_cast<wchar_t*>(buff)));
        delete [] buff;
    }
    return result;
}


std::string getRegKeyValue(HKEY key, LPCTSTR property)
{
    DWORD size = 0;
    DWORD type;
    RegQueryValueEx(key, property, nullptr, nullptr, nullptr, & size);

    std::string result;

    if (size > 0)
    {
        BYTE* buff = new BYTE[size];
        if( RegQueryValueEx(key, property, nullptr, &type, buff, & size) == ERROR_SUCCESS )
            result = Util::ToString(std::wstring(reinterpret_cast<wchar_t*>(buff)));
        RegCloseKey(key);
        delete [] buff;
    }
    return result;
}

//bool IsNumeric(_In_z_ LPCSTR pszString, _In_ bool bIgnoreColon) noexcept
//{
//  const size_t nLen = strlen(pszString);
//  if (nLen == 0)
//    return false;

//  //What will be the return value from this function (assume the best)
//  bool bNumeric = true;

//  for (size_t i=0; i<nLen && bNumeric; i++)
//  {
//    if (bIgnoreColon && (pszString[i] == ':'))
//      bNumeric = true;
//    else
//      bNumeric = (isdigit(static_cast<int>(pszString[i])) != 0);
//  }

//  return bNumeric;
//}

//bool IsNumeric(_In_z_ LPCWSTR pszString, _In_ bool bIgnoreColon) noexcept
//{
//  const size_t nLen = wcslen(pszString);
//  if (nLen == 0)
//    return false;

//  //What will be the return value from this function (assume the best)
//  bool bNumeric = true;

//  for (size_t i=0; i<nLen && bNumeric; i++)
//  {
//    if (bIgnoreColon && (pszString[i] == L':'))
//      bNumeric = true;
//    else
//       bNumeric = (iswdigit(pszString[i]) != 0);
//  }

//  return bNumeric;
//}

//bool QuerySerialPorts(CPortsArray& ports)
//{
//  //Make sure we clear out any elements which may already be in the array
//  ports.clear();

//  //Use QueryDosDevice to look for all devices of the form COMx. Since QueryDosDevice does
//  //not consistently report the required size of buffer, lets start with a reasonable buffer size
//  //of 4096 characters and go from there
//  DWORD nChars = 4096;
//  bool bWantStop = false;
//  while (nChars && !bWantStop)
//  {
//    std::vector<TCHAR> devices;
//    devices.resize(nChars);

//    const DWORD dwChars = QueryDosDevice(nullptr, &(devices[0]), nChars);
//    if (dwChars == 0)
//    {
//      const DWORD dwError = GetLastError();
//      if (dwError == ERROR_INSUFFICIENT_BUFFER)
//      {
//        //Expand the buffer and  loop around again
//        nChars *= 2;
//      }
//      else
//        return false;
//    }
//    else
//    {
//      bWantStop = true;

//      size_t i = 0;
//      while (devices[i] != _T('\0'))
//      {
//        //Get the current device name
//        LPCTSTR pszCurrentDevice = &(devices[i]);

//        //If it looks like "COMX" then
//        //add it to the array which will be returned
//        const size_t nLen = _tcslen(pszCurrentDevice);
//        if (nLen > 3)
//        {
//          if ((_tcsnicmp(pszCurrentDevice, _T("COM"), 3) == 0) && IsNumeric(&(pszCurrentDevice[3]), false))
//          {
//            //Work out the port number
//            const unsigned int nPort = static_cast<unsigned int>(_ttoi(&pszCurrentDevice[3]));
//            ports.push_back(nPort);
//          }
//        }

//        //Go to next device name
//        i += (nLen + 1);
//      }
//    }
//  }

//  return true;
//}

#define		CP210x_SUCCESS						0x00

// Prototype found in CP210x header file
typedef int (WINAPI *CP210x_GetDeviceSerialNumber_t)(
        const HANDLE cyHandle,
        LPVOID	lpSerialNumber,
        LPBYTE	lpbLength,
        const BOOL	bConvertToASCII );

void GetCP210xSerialNumber(SerialInfos &entry)
{
    char full_path[32] = {0};

    HANDLE hCom = nullptr;

    _snprintf(full_path, sizeof(full_path) - 1, "\\\\.\\%s", entry.portName.c_str());

    hCom = CreateFileA(full_path, GENERIC_WRITE | GENERIC_READ, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);

    if ((hCom != nullptr) && (reinterpret_cast<std::uint32_t>(hCom) != reinterpret_cast<std::uint32_t>(INVALID_HANDLE_VALUE)))
    {
        HMODULE hModule = LoadLibrary(TEXT("CP210xManufacturing.dll"));

        CP210x_GetDeviceSerialNumber_t getSerial =
                reinterpret_cast<CP210x_GetDeviceSerialNumber_t>(GetProcAddress(hModule, "CP210x_GetDeviceSerialNumber"));

        char	lpSerialNumber[1024];
        BYTE	lpbLength;

        int ret = getSerial(hCom, lpSerialNumber, &lpbLength, TRUE);

        if (ret == CP210x_SUCCESS)
        {
            entry.serial.assign(lpSerialNumber);
            entry.isCypress = true;
        }

        FreeLibrary(hModule);

        CloseHandle(hCom);
    }
}

#define FTID_SUCCESS						0
typedef PVOID	FT_HANDLE;
typedef ULONG	FT_STATUS;
//
// Device status
//
enum {
    FT_OK,
    FT_INVALID_HANDLE,
    FT_DEVICE_NOT_FOUND,
    FT_DEVICE_NOT_OPENED,
    FT_IO_ERROR,
    FT_INSUFFICIENT_RESOURCES,
    FT_INVALID_PARAMETER,
    FT_INVALID_BAUD_RATE,

    FT_DEVICE_NOT_OPENED_FOR_ERASE,
    FT_DEVICE_NOT_OPENED_FOR_WRITE,
    FT_FAILED_TO_WRITE_DEVICE,
    FT_EEPROM_READ_FAILED,
    FT_EEPROM_WRITE_FAILED,
    FT_EEPROM_ERASE_FAILED,
    FT_EEPROM_NOT_PRESENT,
    FT_EEPROM_NOT_PROGRAMMED,
    FT_INVALID_ARGS,
    FT_NOT_SUPPORTED,
    FT_OTHER_ERROR,
    FT_DEVICE_LIST_NOT_READY,
};

typedef unsigned long (*FTID_GetNumDevices_t)(unsigned long * Devices);
typedef unsigned long (*FTID_GetDeviceSerialNumber_t)(unsigned long DeviceIndex, char * SerialBuffer, unsigned long SerialBufferLength);
typedef unsigned long (*FTID_GetDeviceDescription_t)(unsigned long DeviceIndex, char * DescriptionBuffer, unsigned long DescriptionBufferLength);
typedef unsigned long (*FTID_GetDeviceLocationID_t)(unsigned long DeviceIndex, unsigned long * LocationIDBuffer);


typedef FT_STATUS (*FT_Open_t)(
    int deviceNumber,
    FT_HANDLE *pHandle
    );

typedef FT_STATUS (*FT_GetComPortNumber_t)(
    FT_HANDLE ftHandle,
    LPLONG	lpdwComPortNumber
    );

typedef FT_STATUS (*FT_Close_t)(
    FT_HANDLE ftHandle
    );

// \HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\FTDIBUS\Enum
/*
sprintf(comPortRegPath, "%s%s%s%s", "SYSTEM\\\\CurrentControlSet\\\\Enum\\\\USB\\\\", USB_DEV_VID_PID, devSerialNumber, "\\\\Device Parameters");

registryAccess = RegOpenKeyEx(HKEY_LOCAL_MACHINE, comPortRegPath, 0, KEY_READ, &registryKey);

usbVCOMportNameSize = sizeof(usbVCOMportName);

registryAccess = RegQueryValueEx(registryKey, "PortName", NULL, NULL, (LPBYTE)usbVCOMportName, &usbVCOMportNameSize);

RegCloseKey(registryKey);
 */
void ListFTDIDevices()
{
    HMODULE hModule = LoadLibrary(TEXT("FTChipID.dll"));
    HMODULE hFTDModule = LoadLibrary(TEXT("ftd2xx.dll"));

    if (hModule && hFTDModule)
    {

        FTID_GetNumDevices_t getNumDevices = reinterpret_cast<FTID_GetNumDevices_t>(GetProcAddress(hModule, "FTID_GetNumDevices"));
        FTID_GetDeviceSerialNumber_t getSerial = reinterpret_cast<FTID_GetDeviceSerialNumber_t>(GetProcAddress(hModule, "FTID_GetDeviceSerialNumber"));
        FTID_GetDeviceDescription_t getDesc = reinterpret_cast<FTID_GetDeviceDescription_t>(GetProcAddress(hModule, "FTID_GetDeviceDescription"));
        FTID_GetDeviceLocationID_t getLoc = reinterpret_cast<FTID_GetDeviceLocationID_t>(GetProcAddress(hModule, "FTID_GetDeviceLocationID"));

        FT_Open_t ftdOpen = reinterpret_cast<FT_Open_t>(GetProcAddress(hFTDModule, "FT_Open"));
        FT_GetComPortNumber_t ftdGetPortNumber = reinterpret_cast<FT_GetComPortNumber_t>(GetProcAddress(hFTDModule, "FT_GetComPortNumber"));
        FT_Close_t ftdClose = reinterpret_cast<FT_Close_t>(GetProcAddress(hFTDModule, "FT_Close"));

        unsigned long NbDevices = 0U;
        char	lpSerialNumber[1024];
        char	lpDesc[1024];
        unsigned long	lpLoc;

        gFtdiDeviceList.clear();

        if (getNumDevices(&NbDevices) == FTID_SUCCESS)
        {
            for (unsigned long i = 0U; i < NbDevices; i++)
            {
                std::string serial;
                std::string desc;
                // Here we save locally some data
                // FTDI DLL seems to goes over the buffer
                // So it may generate memory corruption
                getSerial(i, lpSerialNumber, 256);
                serial.assign(lpSerialNumber);
                getDesc(i, lpDesc, 256);
                desc.assign(lpDesc);
                getLoc(i, &lpLoc);

                FT_HANDLE fthandle;
                FT_STATUS res;
                LONG COMPORT;

                res = ftdOpen(0, &fthandle);

                if(res == FT_OK){

                    res = ftdGetPortNumber(fthandle, &COMPORT);

                    if ((res == FT_OK) && (COMPORT != -1))
                    {
                        gFtdiDeviceList.push_back(FtdiDevice(lpLoc, COMPORT, serial, desc));

                        std::cout << "FTDI: " << serial << " " << desc << " " << COMPORT << std::endl;
                    }
                }
                ftdClose(fthandle);
            }
        }
        FreeLibrary(hModule);
        FreeLibrary(hFTDModule);
    }
}

void GetFTDISerialNumber(SerialInfos &entry)
{
    for (auto const &ftdi : gFtdiDeviceList)
    {
        std::string name = entry.portName;
        name.erase(0, 3); // erase the "COMxx" part
        int comPort = Util::FromString<int>(name);
        if (ftdi.comPort == comPort)
        {
            entry.isFtdi = true;
            entry.serial = ftdi.serialNumber;
        }
    }
}

#include "Registry.hpp"

void SerialPort::EnumeratePorts()
{
    HDEVINFO DeviceInfoSet = SetupDiGetClassDevs(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);

    const auto INVALID_HANDLE = reinterpret_cast<HANDLE>(-1);

    gSerialList.clear();

    ListFTDIDevices();

    if (INVALID_HANDLE != DeviceInfoSet)
    {
        SP_DEVINFO_DATA devInfoData;
        devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
        for(DWORD i = 0; SetupDiEnumDeviceInfo(DeviceInfoSet, i, &devInfoData); i++)
        {
            SerialInfos entry;

            entry.friendName  = getDeviceProperty(DeviceInfoSet, &devInfoData, SPDRP_FRIENDLYNAME);
            entry.physName = getDeviceProperty(DeviceInfoSet, &devInfoData, SPDRP_PHYSICAL_DEVICE_OBJECT_NAME);
            entry.enumName = getDeviceProperty(DeviceInfoSet, &devInfoData, SPDRP_ENUMERATOR_NAME);
            entry.hardwareIDs = Util::ToUpper(getDeviceProperty(DeviceInfoSet, &devInfoData, SPDRP_HARDWAREID));

            HKEY devKey = SetupDiOpenDevRegKey(DeviceInfoSet, &devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
            entry.portName = getRegKeyValue(devKey, TEXT("PortName"));

            std::regex pattern("VID_(\\w+)&PID_(\\w+)");
            std::smatch matcher;
            std::string subMatch;

            std::regex_search(entry.hardwareIDs, matcher, pattern);

            if (matcher.size() >= 2)
            {
                // Extracted value is located at index 1
                entry.vendorId = matcher[1].str();
                entry.productId = matcher[2].str();
            }

            GetCP210xSerialNumber(entry);
            GetFTDISerialNumber(entry);

            std::cout << entry.friendName  << "\r\n"
                      << "    " << entry.physName << "\r\n"
                      << "    " << entry.enumName << "\r\n"
                      << "    " << entry.hardwareIDs << " ==> VendorID: " << entry.vendorId << " ProductId: " << entry.productId << "\r\n"
                      << "    " << entry.portName << std::endl;
            if (entry.isFtdi)
            {
                std::cout << "    FTDI chip with serial: " << entry.serial << std::endl;
            }
            if (entry.isCypress)
            {
                std::cout << "    Cypress chip with serial: " << entry.serial << std::endl;
            }

            gSerialList.push_back(entry);
        }
    }

    if (DeviceInfoSet)
    {
        SetupDiDestroyDeviceInfoList(DeviceInfoSet);
    }
}

int32_t SerialPort::AssociatePort(const std::string &ident, std::string &portName)
{
    int32_t retCode = cPortNotFound;
    for (auto &p : gSerialList)
    {
        if ((p.portName == ident) ||
            (p.friendName == ident) ||
            (p.FullUsbId() == ident) ||
            (p.serial == ident))
        {
            if (p.associated == false)
            {
                retCode = cPortAssociated;
                p.associated = true;
                portName = p.portName;
                std::cout << "Associated serial device: " << ident << " port name: " << portName << std::endl;
            }
            else
            {
                retCode = cPortNotFree;
            }
            break;
        }
    }

    return retCode;
}


SerialPort::SerialPort()
    : mFd(-1)
{

}

SerialPort::~SerialPort()
{

}

std::int32_t SerialPort::Open(const std::string &ident, const std::string &params)
{
    std::string portName;
    // First, find com port number using the identifier
    std::int32_t retCode = SerialPort::AssociatePort(ident, portName);
    if (retCode == cPortAssociated)
    {
        mFd = serial_open(portName.c_str());
        if (mFd < 0)
        {
            mLastError = "Cannot open serial port: " + portName;
            retCode = cPortOpenError;
        }
        else
        {
            // Eg: 9600,8,N,1
            std::vector<std::string> paramList = Util::Split(params, ",");
            if (paramList.size() == 4)
            {
                try {
                    unsigned long baudrate = static_cast<unsigned long>(std::stol(paramList[0]));
                    serial_setup(mFd, baudrate);
                    mLastSuccess = "Setup serial port " + portName + " success at " + std::to_string(baudrate) + " bauds";
                } catch (const std::exception & e) {
                    mLastError = "Bad device parameters (expected integers): " +  params + ". Error: " + e.what();
                }
            }
            else
            {
                mLastError = "Serial port parameter needs four comma separated parameters, eg: 9600,8,N,1. Got: " + params;
                retCode = cPortBadParameters;
            }
        }
    }
    else
    {
        mLastError = "Cannot find COM port with identifier: " + ident + ", please verify the parameters or device not connected";
    }

    if (retCode != cPortAssociated)
    {
        Close();
    }

    return retCode;
}

void SerialPort::Close()
{
    mFd = serial_close(mFd);
}

int32_t SerialPort::Write(const uint8_t *data, std::uint32_t size)
{
    return Write(std::string(reinterpret_cast<const char*>(data), size));
}

int32_t SerialPort::Write(const std::string &data)
{
    std::int32_t retCode = cPortWriteError;
    if (mFd != -1)
    {
        std::int32_t ret = serial_write(mFd, data.c_str(), static_cast<int>(data.size()));

        if (ret == static_cast<std::int32_t>(data.size()))
        {
            retCode = cPortWriteSuccess;
        }
    }
    else
    {
        retCode = cPortNotOpen;
        mLastError = "Cannot write to serial port, not opened!";
    }

    return retCode;
}

int32_t SerialPort::Read(std::string &data, std::int32_t timeout_sec)
{
    std::int32_t retCode = cPortReadError;
    if (mFd != -1)
    {
        std::int32_t ret = serial_read(mFd, mBuffer, COM_PORT_BUF_SIZE, timeout_sec);

        if (ret > 0)
        {
            data.assign(mBuffer, static_cast<std::uint32_t>(ret));
            retCode = cPortReadSuccess;
        }
        else
        {
            retCode = cPortReadTimeout;
        }
    }
    else
    {
        retCode = cPortNotOpen;
        mLastError = "Cannot read to serial port, not opened!";
    }

    return retCode;


}

bool SerialPort::IsOpen() const
{
    return (mFd < 0) ? false : true;
}

std::string SerialPort::GetLastError()
{
    return mLastError;
}

std::string SerialPort::GetLastSuccess()
{
    return mLastSuccess;
}
