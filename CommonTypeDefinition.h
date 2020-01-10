/**
* @file CommonTypeDefine.hpp
* @date 2018-05-10
* @author Byun, Jaejung (jaejung.byun@lge.com)
* @brief
*	Common Type Definition in M9
* @remark
* @warning
*	LGE Appliance Control Research Division HA Advanced Control Research 2 Team
*	Copyright(C) 2018 LG Electronics Co., LTD., Seoul, Korea
*	All Rights are Reserved.
*/

#pragma once
#ifndef __COMMONTYPEDEFINITION_HPP___
#define __COMMONTYPEDEFINITION_HPP___

#include <math.h>
#include <float.h>
#include <stdint.h>

#ifndef	_MIN_
#define _MIN_(x, y)   (((x) < (y)) ? (x) : (y))
#endif

#ifndef	_MAX_
#define _MAX_(x, y)   (((x) > (y)) ? (x) : (y))
#endif

#ifndef	_MIN
#define _MIN(x, y)   (((x) < (y)) ? (x) : (y))
#endif

#ifndef	_MAX
#define _MAX(x, y)   (((x) > (y)) ? (x) : (y))
#endif

#ifndef __GNUC__
#ifndef	min
#define min(x, y)   (((x) < (y)) ? (x) : (y))
#endif

#ifndef	max
#define max(x, y)   (((x) > (y)) ? (x) : (y))
#endif
#endif

#ifndef PI
#define PI (3.14159263f)
#endif//PI

#ifndef RAD2DEG
#define RAD2DEG (57.29577951f)   ///< 180.0/PI
#endif//RAD2DEG

#ifndef DEG2RAD_
#define DEG2RAD_ (0.017453292f)   ///< PI/180.0
#endif//DEG2RAD_

typedef enum
{
	// Click
	eUI_REMOCON_CODE_NONE = 0x00,
	eUI_REMOCON_CODE_START,
	eUI_REMOCON_CODE_HOMING,
	eUI_REMOCON_CODE_HIT_MODE_CHANGE,
	eUI_REMOCON_CODE_HIT_REPEAT_CLEANING,
	eUI_REMOCON_CODE_STOP,

	eUI_REMOCON_CODE_HIT_TURBO,
	eUI_REMOCON_CODE_HIT_SMARTDIG,
	eUI_REMOCON_CODE_HIT_SETDATE,
	eUI_REMOCON_CODE_HIT_MUTE,
	eUI_REMOCON_CODE_HIT_MACRO,

	// REPEAT (Up / Down)
	eUI_REMOCON_CODE_UP,
	eUI_REMOCON_CODE_DOWN,
	eUI_REMOCON_CODE_LEFT,
	eUI_REMOCON_CODE_RIGHT,
	eUI_REMOCON_CODE_RESERVATION,

	// HIDDEN_CODE (repeat)
	eUI_REMOCON_CODE_HIT_HIDDEN_01,
	eUI_REMOCON_CODE_HIT_HIDDEN_02,
	eUI_REMOCON_CODE_HIT_HIDDEN_03,
	eUI_REMOCON_CODE_HIT_HIDDEN_04,
	eUI_REMOCON_CODE_HIT_HIDDEN_05,
	eUI_REMOCON_CODE_HIT_HIDDEN_06,

	// Exchange Code
	eUI_REMOCON_CODE_POWER, // eUI_REMOCON_CODE_START
	eUI_REMOCON_CODE_START_STOP, // eUI_REMOCON_CODE_STOP

	eUI_REMOCON_CODE_JIGSTART = 0xFF,
} UIRemoteControlCode_t;  // Abstracted REMOTE CON. CODE

/**
* @brief �� float ���� ������ ���Ѵ�.
* @details
*	���� �ε��Ҽ��� ���� �񱳴� �ý��ۿ� ���� ����� �ٸ��� ����
*	���� �ڵ� ��Ģ (JSF, MISRA-C, ...) ���� ����  �ε��Ҽ��� ���� �񱳴� �ڵ� ��Ģ ����
*	�̿� �� float���� ���Լ��� �߰���
* @param[in] fA ù ��° ��
* @param[in] fB �� ��° ��
* @return fA == fB ����
*/
inline bool FloatCompare(float fA, float fB)
{
	return fabsf(fA - fB) < FLT_EPSILON;
}

/**
* @brief degree ������ �Է°��� -180 ~ 180 �� ������ normalized �� ������ ��ȯ�Ѵ�.
* @param[in] fDeg �Է°�, ������ degree
* @return -180 ~ 180���� ��ȯ�� ��
*/
inline float GetNormalizedAngle180(float fDeg)
{
	fDeg = fDeg - static_cast<int32_t>(fDeg * (1.0f / 360.0f)) * 360.0f;

	if (fDeg < -180.0f)
	{
		fDeg += 360.0f;
	}
	else if (fDeg > 180.0f)
	{
		fDeg -= 360.0f;
	} // not exist else statement

	return fDeg;
}

/**
* @brief degree ������ �Է°��� 0 ~ 360 �� ������ normalized �� ������ ��ȯ�Ѵ�.
* @param[in] fDeg �Է°�, ������ degree
* @return 0 ~ 360 ���� ��ȯ�� ��
*/
inline float GetNormalizedAngle360(float fDeg)
{
	fDeg = fDeg - static_cast<int32_t>(fDeg * (1.0f / 360.0f)) * 360.0f;

	if (fDeg < 0.0f)
	{
		fDeg += 360.0f;
	}
	else if (FloatCompare(fDeg, 360.0f))
	{
		fDeg = 0.0f;
	} // not exist else statement

	return fDeg;
}

/**
* @brief degree scaled 100 ������ �Է°��� -18000 ~ 18000 ������ normalized �� ������ ��ȯ�Ѵ�.
* @param[in] nDegScaled100 �Է°�, ������ degree scaled 100
* @return -18000 ~ 18000 ���� ��ȯ�� ��
*/
inline int32_t GetNormalizedAngle18000(int32_t nDegScaled100)
{
	nDegScaled100 %= 36000;

	if (nDegScaled100 < -18000)
	{
		nDegScaled100 += 36000;
	}
	else if (nDegScaled100 > 18000)
	{
		nDegScaled100 -= 36000;
	} // not exist else statement

	return nDegScaled100;
}

/**
* @brief degree scaled 100 ������ �Է°��� 0 ~ 36000 ������ normalized �� ������ ��ȯ�Ѵ�.
* @param[in] nDegScaled100 �Է°�, ������ degree scaled 100
* @return 0 ~ 36000 ���� ��ȯ�� ��
*/
inline int32_t GetNormalizedAngle36000(int32_t nDegScaled100)
{
	nDegScaled100 %= 36000;

	if (nDegScaled100 < 0)
	{
		nDegScaled100 += 36000;
	} // not exist else statement

	return nDegScaled100;
}


struct XYInt32_t;
struct XYThetaInt32_t;

/**
* @struct XYFloat32_t
* @brief float ������ (X, Y) ����ü Ÿ�� ����
*/
struct XYFloat32_t
{
	float fXmm; ///< X, mm ����
	float fYmm; ///< Y, mm ����

	inline bool operator == (const XYFloat32_t& stRight) const
	{
		return FloatCompare(fXmm, stRight.fXmm) && FloatCompare(fYmm, stRight.fYmm);
	}

	inline bool operator != (const XYFloat32_t& stRight) const
	{
		return !(*this == stRight);
	}

	inline void Set(float fX, float fY)
	{
		fXmm = fX;
		fYmm = fY;
	}

	inline const XYFloat32_t& operator = (const XYInt32_t& oRight);
};

/**
* @struct XYInt32_t
* @brief int32_t ������ (X, Y) ����ü Ÿ�� ����
*/
struct XYInt32_t
{
	int32_t nXmm; ///< X, mm ����
	int32_t nYmm; ///< Y, mm ����

	inline bool operator == (const XYInt32_t& stRight) const
	{
		return (nXmm == stRight.nXmm) && (nYmm == stRight.nYmm);
	}

	inline bool operator != (const XYInt32_t& stRight) const
	{
		return (nXmm != stRight.nXmm) || (nYmm != stRight.nYmm);
	}

	inline void Set(int32_t nX, int32_t nY)
	{
		nXmm = nX;
		nYmm = nY;
	}

	inline const XYInt32_t& operator = (const XYFloat32_t& oRight)
	{
		Set(static_cast<int32_t>(oRight.fXmm), static_cast<int32_t>(oRight.fYmm));
		return *this;
	}
};

/**
* @struct XYThetaFloat32_t
* @brief float32_t ������ (X, Y, Theta) ����ü Ÿ�� ����
*/
struct XYThetaFloat32_t
{
	float fXmm; ///< X, mm ����
	float fYmm; ///< Y, mm ����
	float fDeg; ///< Theta, degree ����

	inline bool operator == (const XYThetaFloat32_t& stRight) const
	{
		return FloatCompare(fXmm, stRight.fXmm)
			&& FloatCompare(fYmm, stRight.fYmm)
			&& FloatCompare(fDeg, stRight.fDeg);
	}

	inline bool operator != (const XYThetaFloat32_t& stRight) const
	{
		return FloatCompare(fXmm, stRight.fXmm) == false
			|| FloatCompare(fYmm, stRight.fYmm) == false
			|| FloatCompare(fDeg, stRight.fDeg) == false;
	}

	inline void Set(float fX, float fY, float fDegree)
	{
		fXmm = fX;
		fYmm = fY;
		fDeg = fDegree;
	}

	inline void NormalizeAngle180(void)
	{
		fDeg = GetNormalizedAngle180(fDeg);
	}

	inline void NormalizeAngle360(void)
	{
		fDeg = GetNormalizedAngle360(fDeg);
	}

	inline float GetThetaRad(void) const
	{
		return DEG2RAD_ * fDeg;
	}

	inline const XYThetaFloat32_t& operator = (const XYThetaInt32_t& oRight);
};

/**
* @struct XYThetaInt32_t
* @brief int32_t ������ (X, Y, Theta) ����ü Ÿ�� ����
*/
struct XYThetaInt32_t
{
	int32_t nXmm; ///< X, mm ����
	int32_t nYmm; ///< Y, mm ����
	int32_t nDegScaled100; ///< Theta, Degree*100 ����

	inline bool operator == (const XYThetaInt32_t& stRight) const
	{
		return (nXmm == stRight.nXmm) && (nYmm == stRight.nYmm) && (nDegScaled100 == stRight.nDegScaled100);
	}

	inline bool operator != (const XYThetaInt32_t& stRight) const
	{
		return (nXmm != stRight.nXmm) || (nYmm != stRight.nYmm) || (nDegScaled100 != stRight.nDegScaled100);
	}

	inline void Set(int32_t nX, int32_t nY, int32_t nDeg)
	{
		nXmm = nX;
		nYmm = nY;
		nDegScaled100 = nDeg;
	}

	inline void NomalizeAngle18000(void)
	{
		nDegScaled100 = GetNormalizedAngle18000(nDegScaled100);
	}

	inline void NomalizeAngle36000(void)
	{
		nDegScaled100 = GetNormalizedAngle36000(nDegScaled100);
	}

	inline const XYThetaInt32_t& operator = (const XYThetaFloat32_t& oRight)
	{
		nXmm = static_cast<int32_t>(oRight.fXmm);
		nYmm = static_cast<int32_t>(oRight.fYmm);
		nDegScaled100 = static_cast<int32_t>(oRight.fDeg * 100.0f);

		return *this;
	}

	inline float GetThetaRad(void) const
	{
		return DEG2RAD_ * nDegScaled100 / 100.0f;
	}
};

/**
* @struct RotXYZFloat32_t
* @brief float ������ (XRot, YRot, ZRot) ����ü Ÿ�� ����
*/
struct RotXYZFloat32_t
{
	float fXRotDeg; ///< X, Degree ����
	float fYRotDeg; ///< Y, Degree ����
	float fZRotDeg; ///< Z, Degree ����
};

/**
*	@struct XYZFloat32_t
*	@brief	float ������ (X, Y, Z) ����ü Ÿ�� ����
*/
struct XYZFloat32_t
{
	float fXmm;
	float fYmm;
	float fZmm; ///< Z, mm ����

	inline bool operator == (const XYZFloat32_t& stRight) const
	{
		return FloatCompare(fXmm, stRight.fXmm)
			&& FloatCompare(fYmm, stRight.fYmm)
			&& FloatCompare(fZmm, stRight.fZmm);
	}

	inline bool operator != (const XYZFloat32_t& stRight) const
	{
		return FloatCompare(fXmm, stRight.fXmm) == false
			|| FloatCompare(fYmm, stRight.fYmm) == false
			|| FloatCompare(fZmm, stRight.fZmm) == false;
	}

	inline void Set(float fX, float fY, float fZ)
	{
		fXmm = fX;
		fYmm = fY;
		fZmm = fZ;
	}
};

/**
* @struct XYZInt32_t
* @brief int32_t ������ (X, Y, Z) ����ü Ÿ�� ����
*/
struct XYZInt32_t
{
	int32_t nXmm;
	int32_t nYmm;
	int32_t nZmm; ///< Z, mm ����

	inline bool operator == (const XYZInt32_t& stRight) const
	{
		return (nXmm == stRight.nXmm) && (nYmm == stRight.nYmm) && (nZmm == stRight.nZmm);
	}

	inline bool operator != (const XYZInt32_t& stRight) const
	{
		return (nXmm != stRight.nXmm) || (nYmm != stRight.nYmm) || (nZmm != stRight.nZmm);
	}

	inline void Set(int32_t nX, int32_t nY, int32_t nZ)
	{
		nXmm = nX;
		nYmm = nY;
		nZmm = nZ;
	}
};

inline const XYThetaFloat32_t& XYThetaFloat32_t::operator = (const XYThetaInt32_t& oRight)
{
	fXmm = static_cast<float>(oRight.nXmm);
	fYmm = static_cast<float>(oRight.nYmm);
	fDeg = static_cast<float>(oRight.nDegScaled100) / 100.0f;
	return *this;
}


inline const XYFloat32_t& XYFloat32_t::operator = (const XYInt32_t& oRight)
{
	fXmm = static_cast<float>(oRight.nXmm);
	fYmm = static_cast<float>(oRight.nYmm);
	return *this;
}

#if __cplusplus >= 201103L || _MSC_VER >= 1800
/**
* @warning float �� int32_t ���� �񱳰� ���� �ʱ� ������
*	XYThetaFloat32_t �� XYThetaInt32_t �� ��, �׸���
*	XYFloat32_t �� XYInt32_t �� �񱳴� ���� ���ϰ� ��
*
*	�񱳰� �ʿ��� ���, ������ ���������� ����ȯ�ؾ� �ϰ�, ���� ������ �����ؾ� ��
*/
bool operator == (const XYThetaFloat32_t& oLeft, const XYThetaInt32_t& oRight) = delete;
bool operator == (const XYThetaInt32_t& oLeft, const XYThetaFloat32_t& oRight) = delete;
bool operator == (const XYFloat32_t& oLeft, const XYInt32_t& oRight) = delete;
bool operator == (const XYInt32_t& oLeft, const XYFloat32_t& oRight) = delete;
#endif

/**
* @struct Rectangular_t
* @brief int32_t ������ Rectangular ����ü Ÿ�� ����
* @var	nLeftXmm : mm ���� <int32_t>
* @var	nRightXmm : mm ���� <int32_t>
* @var	nTopYmm : mm ���� <int32_t>
* @var	nBottomYmm : mm ���� <int32_t>
*/
struct Rectangular_t
{
	int32_t nLeftXmm;
	int32_t nBottomYmm;
	int32_t nRightXmm;
	int32_t nTopYmm;

	inline bool operator == (const Rectangular_t& oRight) const
	{
		return nLeftXmm == oRight.nLeftXmm && nRightXmm == oRight.nRightXmm &&
			nTopYmm == oRight.nTopYmm && nBottomYmm == oRight.nBottomYmm;
	}

	inline int32_t GetWidth() const
	{
		return nRightXmm - nLeftXmm + 1;
	}

	inline int32_t GetHeight() const
	{
		return nTopYmm - nBottomYmm + 1;
	}
};

/**
* @struct Area_t
* @brief Area ����ü Ÿ�� ����
* @var nWidth <int32_t>
* @var nHeight <int32_t>
* @var stLeftBottomPos <XYThetaInt32_t>
*/
struct Area_t
{
	int32_t nWidth;
	int32_t nHeight;
	XYThetaInt32_t stLeftBottomPos;
};

/**
* @struct Circle_t
* @brief Circle Ÿ�� ����
* @var nCenterXmm nRadiusMM: mm���� <int32_t>
* @var nCenterYmm : mm���� <int32_t>
* @var nRadiusMM : mm���� <int32_t>
*/
struct Circle_t
{
	int32_t	nCenterXmm;
	int32_t	nCenterYmm;
	int32_t	nRadiusMM;

	inline bool operator == (const Circle_t& oRight) const
	{
		return (nCenterXmm == oRight.nCenterXmm) && (nCenterYmm == oRight.nCenterYmm) && (nRadiusMM == oRight.nRadiusMM);
	}
};

/**
*	@enum		AxialDirection_t
*	@brief		enumeration of axial searching direction
*	@details	�� �˻� ����
*	@see
*	@since		1.10
*/
enum AxialDirection_t
{
	eAXIS_X = 1,
	eAXIS_Y = 2,
};

/**
*	@typedef Direction_t
*	@brief	8���� Ÿ�� ����
*/
enum Direction_t
{
	eDIRECTION_UP = 0,
	eDIRECTION_UPRIGHT,
	eDIRECTION_RIGHT,
	eDIRECTION_DOWNRIGHT,
	eDIRECTION_DOWN,
	eDIRECTION_DOWNLEFT,
	eDIRECTION_LEFT,
	eDIRECTION_UPLEFT,
};

/**
*	@typedef MapProperty_t
*	@brief	�� �Ӽ� ����
*/
enum MapProperty_t
{
	eMAP_PROPERTY_NONE = 0x00,
	eMAP_PROPERTY_EXPLORED = 0x01,
	eMAP_PROPERTY_COVERED = 0x02,
	eMAP_PROPERTY_CARPET = 0x04,
	eMAP_PROPERTY_FRONT_LASER = 0x08,
	eMAP_PROPERTY_SIDE_LASER = 0x10,
	eMAP_PROPERTY_CLIFF = 0x20,
	eMAP_PROPERTY_VIRTUALOBSTACLE = 0x40,
	eMAP_PROPERTY_SW_BUMP = 0x80,

	eMAP_PROPERTY_MASK_FLOOR = eMAP_PROPERTY_EXPLORED | eMAP_PROPERTY_COVERED,
	eMAP_PROPERTY_MASK_OBSTACLE_WITH_NOT_SWBUMP = eMAP_PROPERTY_CARPET | eMAP_PROPERTY_FRONT_LASER | eMAP_PROPERTY_SIDE_LASER | eMAP_PROPERTY_CLIFF | eMAP_PROPERTY_VIRTUALOBSTACLE,
	eMAP_PROPERTY_MASK_OBSTACLE = eMAP_PROPERTY_CARPET | eMAP_PROPERTY_FRONT_LASER | eMAP_PROPERTY_SIDE_LASER | eMAP_PROPERTY_CLIFF | eMAP_PROPERTY_VIRTUALOBSTACLE | eMAP_PROPERTY_SW_BUMP,
};


/**
*	@typedef MapPoint_t
*	@brief	Map Marking ��û�� ���� ������ Ÿ�� ����
*/
struct MapPoint_t
{
	XYFloat32_t stObstaclePoint;
	MapProperty_t eObsType;
};

/**
* @struct Color_t
* @brief Color ������ Ÿ�� ����
*/
struct Color_t
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
	uint8_t A;

	inline bool operator == (const Color_t oRight) const
	{
		return (R == oRight.R) && (G == oRight.G) && (B == oRight.B) && (A == oRight.A);
	}
};

#endif// __COMMONTYPEDEFINITION_HPP___
