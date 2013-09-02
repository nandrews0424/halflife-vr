#include <stdio.h>
#include <math.h>
#include "..\..\..\dependencies\vr_io\vr_io.h" // WTF @ THIS.... it's in the include path, I hate VS

// WinBase.h does so strange defines that break 
#undef CreateEvent
#undef CopyFile
#undef GetObject

class MotionTracker
{
 
public:
 
	MotionTracker( void );
	~MotionTracker( void );
	void shutDown( void );
	void update();

protected:
	bool _initialized;
	IVRIOClient* _vrIO;

	

	
 
/*	QAngle  headOrientation( void );
	QAngle  weaponOrientation( void );
	QAngle  leftHandOrientation( void );
	QAngle  bodyOrientation( void );
	void	update( float originalYaw );
	void	calibrate( void );
	void	calibrateWeapon( void );

	bool	hydraConnected( void );
	void	hydraRight(HydraControllerData &data);
	void	hydraLeft(HydraControllerData &data);
	 
	HmdInfo hmdInfo( void );

	bool	initialized( void );
	bool	hasHeadTracking( void );
	bool	hasWeaponTracking( void );
	bool	hasLeftHandTracking( void );
	bool	hasAnalogInputs( void );
	
	void	getHeadOffset(Vector &headOffset, bool calibrated=true);
	void	getWeaponOffset(Vector &offset, bool calibrated=true);
	void	getLeftHandOffset(Vector &offset, bool calibrated=true);
				
protected:
	float _totalAccumulatedYaw[SENSOR_COUNT];

	float _previousYaw[SENSOR_COUNT];
	bool _initialized;
	
	IVRIOClient* _vrIO;
	

	QAngle _headAngle;
	QAngle _headCalibration;
	QAngle _weaponAngle;
	QAngle _leftHandAngle;
	QAngle _weaponCalibration;
	QAngle _leftHandCalibration;
	QAngle _bodyCalibration;
	QAngle _bodyAngle;
	Vector _weaponOffsetCalibration;
	Vector _leftHandOffsetCalibration;

	unsigned int _updateCounter; */



};

MotionTracker* g_MotionTracker();
