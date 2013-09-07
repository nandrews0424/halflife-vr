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

	bool isTrackingWeapon( void );
	bool isTrackingTorso( void );
	
	void beginCalibration();
	void update(VMatrix& torsoMatrix);
		
	void	updateViewmodelOffset(Vector& vmorigin, QAngle& vmangles);		// hooks clientvr	for now, can move directly into viewmodel_shared
	void	overrideViewOffset(VMatrix& viewMatrix);						// hooks clientvr,	updates the view matrix based on torso tracked offsets
	void	overrideWeaponMatrix(VMatrix& weaponMatrix);					// hooks clientvr	player motion, updates weapon matrix per tracked values
	void	overrideMovement(Vector& movement);								// hooks clientvr,	allows movement vector to be adjusted to account for tracked torso
	void	overrideJoystickInputs(float& lx, float& ly, float& rx, float& ry);		// in_joystick, allows hydra inputs to apply over others
protected:
	bool _initialized;
	bool _calibrate;
	
	IVRIOClient* _vrIO;
	matrix3x4_t _sixenseToWorld;
	matrix3x4_t _eyesToTorsoTracker;
	matrix3x4_t	_torsoCalibration;
	Vector	_vecBaseToTorso;

	float	_baseEngineYaw;
	float	_prevYawTorso;
	float	_accumulatedYawTorso;


	unsigned int _counter;
 
	matrix3x4_t getTrackedTorso();
	void calibrate(VMatrix& torsoMatrix);
	bool		writeDebug();
};

MotionTracker* g_MotionTracker();
