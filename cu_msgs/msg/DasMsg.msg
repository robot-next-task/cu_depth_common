
Header header
# XYThetaInt32_t for Dead-Reckorning
int32 nXmm
int32 nYmm
int32 nDegScaled100
# for Cliff
int16 nCliff_Right
int16 nCliff_Left
# for IR-TOF
int16 nIR_Dist_Right
int16 nIR_Dist_Left
# GyroValueReadData_t for Gyro
int16 nAngle        # ///< Unit : [deg] * 100, Range : -180 ~ 180 [deg]
int16 nAngularRate  # ///< Unit : [deg/sec] * 10
int16 nAccX          # ///< 1um/s^2 resolution
int16 nAccY          # ///< 1um/s^2 resolution
int16 nAccZ          # ///< 1um/s^2 resolution
int16 nAngle_X       #   ///< Unit : [deg] * 100, Range : -180 ~ 180 [deg]
int16 nAngle_Y       #   ///< Unit : [deg] * 100, Range : -180 ~ 180 [deg]
int16 nAngularRate_X #   ///< Unit : [deg/sec] * 10
int16 nAngularRate_Y #   ///< Unit : [deg/sec] * 10
int16 resolved 
# OFSValueReadData_t for ofs
uint32 nMotion         #   ///< OFS의 동작가능 Resister  Unit : [] ,Range :
int32  nDeltaDistanceX #   ///< 직전 Read부터의 누적 변위 Unit : [um] ,Range :
int32  nDeltaDistanceY #   ///< 직전 Read부터의 누적 변위 Unit : [um] ,Range :
uint32 unSqual         #   ///< Unit : [] ,Range :
uint32 unMaximumPixel  #   ///< Unit : [] ,Range :
int32  nIsOutofBound 
# BatteryValueReadData_t for Battery
int8 nState           #// see BatteryStatusEnum_t
int8 nLevel           #//< Unit : [Level], Range : 1~7
uint16 nVoltage       #//< Unit : [10mv]
int16 nCurrent        #//< Unit : [mA]
int8 nRemain          #//< Unit : [%], Range : 0~100 %, 배터리 잔량 표시(전하량 기준),향후 구현 에정
int8 bContactCharger 
# for wheel
int16 nLeftWheelSpeed 
int16 nRightWheelSpeed 