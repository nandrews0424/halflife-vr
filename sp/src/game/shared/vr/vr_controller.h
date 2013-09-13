#include <stdio.h>
#include <math.h>
#include <sixense_utils\interfaces.hpp>

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
	void	overrideJoystickInputs(float& lx, float& ly, float& rx, float& ry, bool overrideRight, bool overrideLeft);		// in_joystick, allows hydra inputs to apply over others
	void	updateSixenseButtons();											// checks sixense controller for buttons states and emulates keypress events...


protected:
	bool _initialized;
	bool _calibrate;
	
	matrix3x4_t _sixenseToWorld;
	matrix3x4_t _eyesToTorsoTracker;
	matrix3x4_t	_torsoCalibration;
	Vector	_vecBaseToTorso;

	float	_baseEngineYaw;
	float	_prevYawTorso;
	float	_accumulatedYawTorso;


	unsigned int _counter;
 
	matrix3x4_t getTrackedTorso();
	matrix3x4_t getTrackedRightHand();
	void		calibrate(VMatrix& torsoMatrix);
	bool		writeDebug();
	
	bool		_sixenseInitialized;
	struct		_sixenseAllControllerData *_sixenseControllerData;
	class		sixenseUtils::IButtonStates *_leftButtonStates, *_rightButtonStates;
	class		sixenseUtils::IControllerManager *_controllerManager;

	void		sixenseInitialize();
	void		sixenseUpdate();
	void		sixenseShutdown();


};

MotionTracker* g_MotionTracker();
