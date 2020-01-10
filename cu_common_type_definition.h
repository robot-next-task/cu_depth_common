#ifndef __CU_COMMON_TYPE_DEFINITION_H__
#define __CU_COMMON_TYPE_DEFINITION_H__

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "CommonTypeDefinition.h"

// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// for RGB-Depth Worker
#if _USE_MEERE_DEPTH
	#define DEFAULT_DEPTHSENSOR_IMG_WIDTH	(320)//(240)
	#define DEFAULT_DEPTHSENSOR_IMG_HEIGHT	(240)//(320)
#else	// G8ToF(224x172), Pico Flexx(224x171)
	#define DEFAULT_DEPTHSENSOR_IMG_WIDTH	(224)//(240)
	#define DEFAULT_DEPTHSENSOR_IMG_HEIGHT	(172)//(320)
#endif
#define DEP_HALF_IMG_WIDTH				(DEFAULT_DEPTHSENSOR_IMG_WIDTH / 2)
#define DEP_HALF_IMG_HEIGHT				(DEFAULT_DEPTHSENSOR_IMG_HEIGHT / 2)

#define DEFAULT_RGBSENSOR_IMG_WIDTH		(640)//(480)
#define DEFAULT_RGBSENSOR_IMG_HEIGHT	(480)//(640)
#define DEFAULT_RGBSENSOR_CHANNEL		(3)		// R + G + B
#define DEFAULT_RGBSENSOR_READ_DURATION	(10)	// ms

#define DEFAULT_MAX_DIST_RANGE			(0.8f)//1.5f)	// m
#define DEFAULT_MIN_DIST_RANGE			(0.05f)	// m

#define DEFAULT_MAX_HEIGHT_RANGE_M		(0.20f)
#define DEFAULT_MAX_HEIGHT_RANGE_CM		(DEFAULT_MAX_HEIGHT_RANGE_M * 100)
#define DEFAULT_MAX_HEIGHT_RANGE_MM		(DEFAULT_MAX_HEIGHT_RANGE_M * 1000)
#define DEFAULT_MIN_OVERLAP_DIST    	(30)    // mm

#define DEFAULT_CENTER2SENSOR_M			(0.165f)
#define DEFAULT_CENTER2SENSOR_CM		(DEFAULT_CENTER2SENSOR_M * 100)
#define DEFAULT_CENTER2SENSOR_MM		(DEFAULT_CENTER2SENSOR_M * 1000)

#define DEFAULT_SENSOR_DEADZONE_M		(0.05f)
#define DEFAULT_SENSOR_DEADZONE_CM		(DEFAULT_SENSOR_DEADZONE_M * 100)
#define DEFAULT_SENSOR_DEADZONE_MM		(DEFAULT_SENSOR_DEADZONE_M * 1000)

#define DEFAULT_NAVI_PASS_MARGIN_M		(0.01f)
#define DEFAULT_NAVI_PASS_MARGIN_CM		(DEFAULT_NAVI_PASS_MARGIN_M * 100)
#define DEFAULT_NAVI_PASS_MARGIN_MM		(DEFAULT_NAVI_PASS_MARGIN_M * 1000)



struct RgbdFrame
{
	uint8_t					RGB[DEFAULT_RGBSENSOR_IMG_WIDTH * DEFAULT_RGBSENSOR_IMG_HEIGHT * DEFAULT_RGBSENSOR_CHANNEL];
	// Voxel::IntensityPoint	ArrXYZFrame[DEFAULT_DEPTHSENSOR_IMG_WIDTH * DEFAULT_DEPTHSENSOR_IMG_HEIGHT];
	cv::Point3f				PC[DEFAULT_DEPTHSENSOR_IMG_WIDTH * DEFAULT_DEPTHSENSOR_IMG_HEIGHT];
	float					IT[DEFAULT_DEPTHSENSOR_IMG_WIDTH * DEFAULT_DEPTHSENSOR_IMG_HEIGHT];
	cv::Mat1w				DI[DEFAULT_DEPTHSENSOR_IMG_WIDTH * DEFAULT_DEPTHSENSOR_IMG_HEIGHT];
	int32_t					Temperature;
	unsigned long long int	TimeStamp;
	float					DepthFrame[DEFAULT_DEPTHSENSOR_IMG_WIDTH * DEFAULT_DEPTHSENSOR_IMG_HEIGHT];
};

struct SRGBDSenserObstacleInfo_t
{
	int32_t nAmount;
	XYZInt32_t pstObstPos[1000];
	XYThetaInt32_t pstRobotPos;

	inline void clear(void)
	{
		nAmount = 0;
		XYZInt32_t zero_xyz;
		XYThetaInt32_t zero_xyt;
		for(uint32_t i = 0; i < 1000; i++)
			pstObstPos[i].Set(0, 0, 0);
		pstRobotPos.Set(0, 0, 0);
	}
};


typedef struct
{
	uint16_t unPreamble;
	uint8_t unSize;
	uint8_t unChecksum;
} ST_MsgHeader;

typedef struct
{
	int32_t nVst;
	int32_t nVturn;
	int32_t unDrReset;
} ST_MsgTxData;

typedef struct
{
	int16_t nAngle;         ///< Unit : [deg] * 100, Range : -180 ~ 180 [deg]
	int16_t nAngularRate;   ///< Unit : [deg/sec] * 10
	int16_t nAccX;          ///< 1um/s^2 resolution
	int16_t nAccY;          ///< 1um/s^2 resolution
	int16_t nAccZ;          ///< 1um/s^2 resolution
	int16_t nAngle_X;         ///< Unit : [deg] * 100, Range : -180 ~ 180 [deg]
	int16_t nAngle_Y;         ///< Unit : [deg] * 100, Range : -180 ~ 180 [deg]
	int16_t nAngularRate_X;   ///< Unit : [deg/sec] * 10
	int16_t nAngularRate_Y;   ///< Unit : [deg/sec] * 10
	int16_t resolved;
} GyroValueReadData_t;


typedef struct
{
	uint32_t nMotion;           ///< OFS의 동작가능 Resister  Unit : [] ,Range :
	int32_t  nDeltaDistanceX;   ///< 직전 Read부터의 누적 변위 Unit : [um] ,Range :
	int32_t  nDeltaDistanceY;   ///< 직전 Read부터의 누적 변위 Unit : [um] ,Range :
	uint32_t unSqual;           ///< Unit : [] ,Range :
	uint32_t unMaximumPixel;    ///< Unit : [] ,Range :
	int32_t  nIsOutofBound;
} OFSValueReadData_t;

typedef struct
{
	int8_t nState; // see BatteryStatusEnum_t
	int8_t nLevel; //< Unit : [Level], Range : 1~7
	uint16_t nVoltage; //< Unit : [10mv]
	int16_t nCurrent; //< Unit : [mA]
	int8_t nRemain; //< Unit : [%], Range : 0~100 %, 배터리 잔량 표시(전하량 기준),향후 구현 에정
	int8_t bContactCharger;
} BatteryValueReadData_t;

typedef struct
{
    XYThetaInt32_t stDRData;     // 12 bytes
    int16_t nCliff_Right;        // 2bytes
    int16_t nCliff_Left;         // 2bytes
	int16_t nIR_Dist_Right;		 // 2bytes
	int16_t nIR_Dist_Left; 		 // 2bytes
    GyroValueReadData_t stGyroData; // 20bytes
    OFSValueReadData_t stOfsData;   //24bytes
    BatteryValueReadData_t stBatteryData; //8bytes
    int16_t nLeftWheelSpeed; // 2bytes
    int16_t nRightWheelSpeed; // 2bytes
	// IRValueReadData_t stIRData;
}ST_MsgRxData;

typedef struct
{
	ST_MsgHeader header;
	ST_MsgTxData data;
} ST_MsgTxPacket;

typedef struct
{
	ST_MsgHeader header;
	ST_MsgRxData data;
} ST_MsgRxPacket;

#endif // __CU_COMMON_TYPE_DEFINITION_H__