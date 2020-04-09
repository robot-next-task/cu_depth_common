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
	#define DEFAULT_DEPTHSENSOR_IMG_HEIGHT	(171)//(320)
	#define SMOOTHING_FILTER_SIZE   3
	#define VALID_Z_VALUE 0.05f
	#define CONFIDENCE_VAL_LIMIT 200
	#define DEPTH_DOWNSIZING_INTERVAL 3
	#define Resized_width (DEFAULT_DEPTHSENSOR_IMG_WIDTH/DEPTH_DOWNSIZING_INTERVAL)
	#define Resized_height (DEFAULT_DEPTHSENSOR_IMG_HEIGHT/DEPTH_DOWNSIZING_INTERVAL)
#endif
#define DEP_HALF_IMG_WIDTH				(DEFAULT_DEPTHSENSOR_IMG_WIDTH / 2)
#define DEP_HALF_IMG_HEIGHT				(DEFAULT_DEPTHSENSOR_IMG_HEIGHT / 2)

#define DEFAULT_RGBSENSOR_IMG_WIDTH		(640)//(480)
#define DEFAULT_RGBSENSOR_IMG_HEIGHT	(480)//(640)
#define DEFAULT_RGBSENSOR_CHANNEL		(3)		// R + G + B
#define DEFAULT_RGBSENSOR_READ_DURATION	(10)	// ms

#define DEFAULT_MAX_DIST_RANGE			(0.50f)//8f)//1.5f)	// m
#define DEFAULT_MIN_DIST_RANGE			(0.05f)	// m

#define DEFAULT_MAX_HEIGHT_RANGE_M		(0.13f)
#define DEFAULT_MAX_HEIGHT_RANGE_CM		(DEFAULT_MAX_HEIGHT_RANGE_M * 100)
#define DEFAULT_MAX_HEIGHT_RANGE_MM		(DEFAULT_MAX_HEIGHT_RANGE_M * 1000)
#define DEFAULT_MIN_OVERLAP_DIST    	(30)    // mm

#define DEFAULT_CENTER2SENSOR_M			(0.165f)
#define DEFAULT_CENTER2SENSOR_CM		(DEFAULT_CENTER2SENSOR_M * 100)
#define DEFAULT_CENTER2SENSOR_MM		(DEFAULT_CENTER2SENSOR_M * 1000)

#define DEFAULT_SENSOR_DEADZONE_M		(0.05f)//(0.12f)
#define DEFAULT_SENSOR_DEADZONE_CM		(DEFAULT_SENSOR_DEADZONE_M * 100)
#define DEFAULT_SENSOR_DEADZONE_MM		(DEFAULT_SENSOR_DEADZONE_M * 1000)

#define DEFAULT_NAVI_PASS_MARGIN_M		(0.03f)
#define DEFAULT_NAVI_PASS_MARGIN_CM		(DEFAULT_NAVI_PASS_MARGIN_M * 100)
#define DEFAULT_NAVI_PASS_MARGIN_MM		(DEFAULT_NAVI_PASS_MARGIN_M * 1000)

#ifndef PI_F
#define PI_F            (3.14159265359f)
#endif

#ifndef _DEG2RAD
#define _DEG2RAD(r)      (r)*PI_F/180.f
#endif

#ifndef _RAD2DEG
#define _RAD2DEG(r)      (r)*180.f/PI_F
#endif

#define _TILT_DEG	(-10)		//(-15)



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
	int32_t nAgitatorSpeed;
	int32_t nSuctionSpeed;
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
	int32_t softbumper;
} SoftBumperStatusReadData_t;

typedef struct
{
	int16_t nSide_L;
	int16_t nSide_R;
	int16_t nFront_L;
	int16_t nFront_CL;
	int16_t nFront_CR;
	int16_t nFront_R;
}TofDistanceValueReadData_t;

typedef struct
{
	int16_t nCliff_R;
	int16_t nCliff_L;
	int16_t nCliff_C;
	int16_t nReserved;
}CliffValueReadData_t;

typedef struct
{
	uint16_t req_data;
	uint8_t key;
	uint8_t remocon1;
	uint8_t remocon2;
	uint8_t remocon3;
	uint8_t remocon4;
	uint8_t docking1;
	uint8_t docking2;
	uint8_t docking3;
	uint8_t docking4;
	int8_t 	nDustBinTheta;
}UIValueReadData_t;

typedef struct
{
    XYThetaInt32_t stDRData;     	// 12 bytes
	XYThetaInt32_t stWheelData;     // 12 bytes
	BatteryValueReadData_t stBatteryData; //8bytes
	TofDistanceValueReadData_t stToFData; //12bytes
	CliffValueReadData_t stCliffData; 	//8bytes
	GyroValueReadData_t stGyroData; // 20bytes
	GyroValueReadData_t stGyroData2; // 20bytes
	OFSValueReadData_t stOfsData;   //24bytes
	int16_t nLeftWheelSpeed; // 2bytes
    int16_t nRightWheelSpeed; // 2bytes
	UIValueReadData_t stUIReadData; //12 byte

	/*
    int16_t nCliff_Right;        // 2bytes
    int16_t nCliff_Left;         // 2bytes
	int16_t nCliff_Center;         // 2bytes
	int16_t nIR_Dist_Right;		 // 2bytes
	int16_t nIR_Dist_Left; 		 // 2bytes
    GyroValueReadData_t stGyroData; // 20bytes
    OFSValueReadData_t stOfsData;   //24bytes
	SoftBumperStatusReadData_t stSoftBumper;	// 4Bytes
	*/
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
