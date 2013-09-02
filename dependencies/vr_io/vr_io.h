#ifndef _VRIO_H
#define _VRIO_H

#include <windows.h>

enum VRIO_Channel
{
	HEAD = 0,
	WEAPON = 1,
	BODY = 2,
	RIGHT_HAND = 3,
	LEFT_HAND = 4,
	RIGHT_FOOT = 5,
	LEFT_FOOT = 6
};

#if defined(IS_DLL)	// inside DLL
#	define IMPORT_EXPORT   __declspec(dllexport)
#else						// outside DLL define
#	define IMPORT_EXPORT   __declspec(dllimport)
#endif  


// Maybe not a great name, but essentially the input data for a given channel (a particular target joint, etc)
struct VRIO_Message
{
	VRIO_Channel channel;
		
	// orientation
	float pitch;
	float yaw;
	float roll;

	//position
	float x;
	float y;
	float z;

	int axisCount;

	//input axes
	float axisVals[4];

	void init( )
	{
		pitch = 0;
		yaw = 0;
		roll = 0;
		x = 0;
		y = 0;
		z = 0;

		axisCount=0;
		for (int i=0;i<4;i++) {
			axisVals[i]=0;
		}
	}
};


// For now I'm just going to have a custom type for the hydra since it's becoming so prolific
struct Hydra_Message
{
	// orientation
	float anglesRight[3];
	float anglesLeft[3];

	//positions
	float posRight[3];
	float posLeft[3];

	// joystick positions
	float rightJoyX;
	float rightJoyY;
	
	float leftJoyX;
	float leftJoyY;
	
	void init( )
	{
		for (int i=0;i<3;i++) {
			anglesRight[i]=0;
			anglesLeft[i]=0;
			posRight[i]=0;
			posLeft[i]=0;
		}

		rightJoyX=0;
		rightJoyY=0;
		leftJoyX=0;
		leftJoyY=0;
	}
};


struct HMDDeviceInfo 
{
	bool active;

	unsigned  HResolution, VResolution; 
	float     HScreenSize, VScreenSize;
	float     VScreenCenter;
	float     EyeToScreenDistance;
	float     LensSeparationDistance;
	float     InterpupillaryDistance;
	float     DistortionK[4];

	HMDDeviceInfo() {
		active = false;
	}
};

// Base interface for 
struct IVRIOClient
{
	virtual int initialize( void ) = 0;
	virtual int think( void ) = 0;
	virtual int getOrientation( VRIO_Channel, VRIO_Message& message ) = 0;
	virtual int getChannelCount( ) = 0;
	
	//hydra specific
	virtual bool hydraConnected( ) = 0;
	virtual void hydraData( Hydra_Message& message ) = 0;

	virtual HMDDeviceInfo getHMDInfo( void ) = 0;

	virtual void dispose( void ) = 0;
};

extern "C" IMPORT_EXPORT IVRIOClient* APIENTRY _vrio_getInProcessClient();


#endif