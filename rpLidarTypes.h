#ifndef rplidarTypes_h
#include <stdint.h>  // uint8_t와 같은 타입을 정의하기 위한 헤더 파일 추가

typedef uint8_t rp_descriptor_t[7];
typedef uint8_t rq_Packet_t[9];
typedef uint8_t rq_message_t[2];

/// hold a measure point for standard scan mode
typedef struct scanDataPoint
{
	uint8_t quality;
	uint8_t angle_low;
	uint8_t angle_high;
	uint8_t distance_low;
	uint8_t distance_high;
}stScanDataPoint_t;

typedef struct Point
{
	double angle; 
	uint16_t distance; 
}point_t;

typedef struct deviceHealtStatus
{
	uint8_t status;
	uint8_t errorCode_low;
	uint8_t errorCode_high;
}stDeviceStatus_t;


typedef struct expressData
{
	uint16_t angle;
	uint16_t cabin[40];
}stExpressDataPacket_t;

typedef struct expressDataStorage
{
	uint8_t angle_low;
	uint8_t angle_high;
	uint16_t distance;
}expressDataStorage_t;

typedef struct rp_stDeviceInfo
{
  uint8_t model;
  uint8_t firmware_minor;
  uint8_t firmware_major;
  uint8_t hardware;
  uint8_t serialnumber[16];
}stDeviceInfo_t;

enum enDescriptor
{
		legacyVersion,  ///< Legacy scan version
		extendedVersion, ///< Extendet scan version
		denseVersion,	 ///< Dense scan version
		startScan,		 ///< start scan
		forceScan,		 ///< force to scan in idle mode
		deviceInfo,		 ///< deviceInfo of the Lidar
		healthInfo,		 ///< Error Codes and Status
		sampleRate,		 ///< momentary sampleRate
		deviceConf		 
};

enum enRequest
{
	rq_stop,
	rq_reset,
	rq_scan,
	rq_scanExpress,
	rq_scanForce,
	rq_info,
	rq_health,
	rq_sampleRate,
	rq_deviceConf
};

enum enMode
{
	stop,
	standard,
	express
};

#endif
