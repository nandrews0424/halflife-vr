#include "cbase.h"
#include "vr/vr_controller.h"

static ConVar vr_neck_length( "vr_neck_length", "3", 0 );
static ConVar vr_spine_length( "vr_spine_length", "30", 0 );
static ConVar vr_swap_trackers( "vr_swap_trackers", "0", 0 );
static ConVar vr_weapon_movement_scale( "vr_weapon_movement_scale", "1", FCVAR_ARCHIVE, "Scales tracked weapon positional tracking");
static ConVar vr_offset_calibration("vr_offset_calibration", "1", FCVAR_ARCHIVE, "Toggles offset (right and forward calibration), 0 is less convenient but necessary for 360 setup where there is no frame of reference for forward and side");
static ConVar vr_hydra_left_hand("vr_hydra_left_hand", "0", 0, "Experimental: Use your left hand hydra for head tracking, 1 is position only, 2 for both position and orientation");

MotionTracker* _motionTracker;

#define MM_TO_INCHES(x) (x/1000.f)*(1/METERS_PER_INCH)

extern MotionTracker* g_MotionTracker()
{
	return _motionTracker;
}

MotionTracker::MotionTracker()
{
	Msg("Initializing Motion Tracking API");
	try 
	{
		_vrIO = _vrio_getInProcessClient();
		_vrIO->initialize();

		_motionTracker = this;

		if ( _vrIO->getChannelCount() > 0 )
		{
			Msg("Motion Tracking intialized with %i devices...  \n", _vrIO->getChannelCount());
		}
		else 
		{
			Msg("Motion Tracking initialized with no devices, not active");
			return;
		}

		_initialized = true;
		Msg("Motion Tracker initialized");
	}
	catch (...)
	{
		Msg("Motion Tracker not initialized correctly!!!!!");
	}
} 

MotionTracker::~MotionTracker()
{
	shutDown();
}

void MotionTracker::shutDown()
{
	if (_initialized)
	{
		Msg("Shutting down VR Controller");
		_initialized = false;
		_vrIO->dispose();
	}
	else 
	{
		Msg("VR Controller already shut down, nothing to do here.");
	}
}


void printVec(Vector v)
{
	Msg("x:%.2f y:%.2f z:%.2f\n", v.x, v.y, v.z);
}
void printVec(float* v)
{
	Msg("x:%.2f y:%.2f z:%.2f\n", v[0], v[1], v[2]);
}


void MotionTracker::update()
{
	Msg("In motion tracker code....\n");	

	if ( !_initialized  || !_vrIO->hydraConnected() )
		return;

	Hydra_Message m;
	_vrIO->hydraData(m);

	Msg("Hydra data pulled\n");
	
	Msg("Right hand angles\n");
	printVec(m.anglesRight);
	
	Msg("Left hand angles\n");
	printVec(m.anglesLeft);
	
	Msg("Right hand offsets \n");
	printVec(m.posRight);
	
	Msg("Left hand offsets \n");
	printVec(m.posLeft);
}


/*

bool	VrController::initialized( void )
{

	return _initialized && _vrIO->getChannelCount() > 0; //for now
}

QAngle	VrController::headOrientation( void )
{
	return _headAngle;
}

QAngle	VrController::weaponOrientation( void )
{
	return _weaponAngle;
}

QAngle	VrController::leftHandOrientation( void )
{
	return _leftHandAngle;
}

QAngle	VrController::bodyOrientation( void )
{
	return _bodyAngle;
}


bool	VrController::hydraConnected( void )
{
	return _initialized && _vrIO->hydraConnected();
}

// TODO: would be better to expose these not as "hydra" controllers

void	VrController::hydraRight(HydraControllerData &data)
{
	Hydra_Message m;
	
	_vrIO->hydraData(m);
	
	data.init();
	data.angle.Init(m.anglesRight[0], m.anglesRight[1], m.anglesRight[2]);
	data.pos.Init(m.posRight[0], m.posRight[1], m.posRight[2]);
	data.xAxis = m.rightJoyX;
	data.yAxis = m.rightJoyY;
}

void	VrController::hydraLeft(HydraControllerData &data)
{
	Hydra_Message m;
	
	_vrIO->hydraData(m);
	
	data.init();
	data.angle.Init(m.anglesLeft[0], m.anglesLeft[1], m.anglesLeft[2]);
	data.pos.Init(m.posLeft[0], m.posLeft[1], m.posLeft[2]);
	data.xAxis = m.leftJoyX;
	data.yAxis = m.leftJoyY;
}
	
bool VrController::hasHeadTracking( void )
{
	return _initialized && ( (_vrIO->getChannelCount() >  1) || ( _vrIO->getChannelCount() == 1 && !hydraConnected() ) );
}

bool VrController::hasWeaponTracking( void ) 
{
	return _initialized && (_vrIO->getChannelCount() > 1 || _vrIO->hydraConnected());  
}

bool VrController::hasLeftHandTracking( void )
{
	return hydraConnected();
}

bool VrController::hasAnalogInputs( void )
{
	return hydraConnected();
}

void	VrController::update(float previousViewYaw)
{
	if (_vrIO->getChannelCount() == 0) { //todo: check vrIO state
		Msg("Trackers not initialized properly, nothing to do here...\n");
		return;
	}
	
	VRIO_Message message;
	_vrIO->think();
	

	VRIO_Channel headChannel = HEAD;
	VRIO_Channel weaponChannel = WEAPON;
	if ( vr_swap_trackers.GetBool() && _vrIO->getChannelCount() == 2 )
	{
		headChannel = WEAPON;
		weaponChannel = HEAD;
	}
	
	// HEAD ORIENTATION
	
	
	_vrIO->getOrientation(headChannel, message);
		
	_headAngle[PITCH] = message.pitch;
	_headAngle[ROLL] = message.roll;
	_headAngle[YAW] = message.yaw;

	// TODO: temp hack for experimental hydra mode...
	if ( vr_hydra_left_hand.GetInt() >= 2 )
	{
		HydraControllerData d;
		hydraLeft(d);
		_headAngle = d.angle;
	}
			
	float previousYaw = _previousYaw[HEAD];
	float currentYaw = _headAngle[YAW];
	float deltaYaw = currentYaw - previousYaw;
	
	_headAngle[YAW] = deltaYaw + previousViewYaw;
	_previousYaw[HEAD] = currentYaw; 
	_totalAccumulatedYaw[HEAD] += deltaYaw;
	_headAngle -= _headCalibration; 

	// Msg("Head angle %.1f %.1f %.1f\n", _headAngle.x, _headAngle.y, _headAngle.z);

	// BODY ORIENTATION

	// without additional tracking it is simply the head angle minus the total accumulated head yaw
	VectorCopy(_headAngle, _bodyAngle);
	_bodyCalibration[YAW] += deltaYaw;  // any head movement is negated from body orientation
	_bodyAngle -= _bodyCalibration;
	_bodyAngle[YAW] = previousViewYaw - _bodyCalibration[YAW];

	// WEAPON ORIENTATION

	if (!hasWeaponTracking()) 
	{
		VectorCopy(_headAngle, _weaponAngle);
		return;		 
	}

	_vrIO->getOrientation(weaponChannel, message);
	_weaponAngle[PITCH] = message.pitch;
	_weaponAngle[ROLL] = message.roll;
	_weaponAngle[YAW] = message.yaw;
				
	previousYaw = _previousYaw[WEAPON];
	currentYaw = _weaponAngle[YAW];
	deltaYaw = currentYaw - previousYaw;

	_previousYaw[WEAPON] = currentYaw;
	_totalAccumulatedYaw[WEAPON] += deltaYaw;
	_weaponAngle[YAW] = previousViewYaw + _totalAccumulatedYaw[WEAPON] - _totalAccumulatedYaw[HEAD];

	_weaponAngle -= _weaponCalibration;

	
	// LEFT HAND ORIENTATION
	
	if ( !hydraConnected() )
		return;

	HydraControllerData d;
	hydraLeft(d);
	
	_leftHandAngle = d.angle;
				
	previousYaw = _previousYaw[LEFT_HAND];
	currentYaw = _leftHandAngle[YAW];
	deltaYaw = currentYaw - previousYaw;

	_previousYaw[LEFT_HAND] = currentYaw;
	_totalAccumulatedYaw[LEFT_HAND] += deltaYaw;
	_leftHandAngle[YAW] = previousViewYaw + _totalAccumulatedYaw[LEFT_HAND] - _totalAccumulatedYaw[HEAD];

	_leftHandAngle -= _leftHandCalibration;
};

void VrController::calibrate()
{
	VectorCopy(_headAngle + _headCalibration, _headCalibration);
	_headCalibration[YAW] = 0;

	VectorCopy(_weaponAngle + _weaponCalibration, _weaponCalibration);
	calibrateWeapon();
}

// This recenters all elements
void VrController::calibrateWeapon() {
	
	Msg("Calibrating weapon: \n");
	
	VRIO_Message head, weapon;
	
	if ( !vr_swap_trackers.GetBool() )
	{
		_vrIO->getOrientation(HEAD, head);
		_vrIO->getOrientation(WEAPON, weapon);
	}
	else 
	{
		_vrIO->getOrientation(WEAPON, head);
		_vrIO->getOrientation(HEAD, weapon);
	}

	HydraControllerData d;
	hydraLeft(d);
	
	// zero out the calibrations except yaw, all the trackers I'm 

	if ( hydraConnected() ) {
		_leftHandCalibration[PITCH] = 0;
		_leftHandCalibration[ROLL] = 0;
		_weaponCalibration[PITCH] = 0;
		_weaponCalibration[ROLL] = 0;
	}
	
	_weaponCalibration[YAW] = weapon.yaw - head.yaw;
	_leftHandCalibration[YAW] = d.angle[YAW] - head.yaw;
	_bodyCalibration[YAW] = 0;
	
	// todo: if weapon tracking only 

	Vector weapOffset;
	getWeaponOffset(weapOffset, false);

	Msg("Zeroing weapon offsets %.1f %.1f %.1f\n", weapOffset.x, weapOffset.y, weapOffset.z);
	// todo: here we want to do a bit of offset handling for calibrating at the shoulder as that seems to be the standard with a hydra
	
	Vector shoulderCalibrationOffset;
	if ( vr_offset_calibration.GetBool() )
		shoulderCalibrationOffset.Init(-2, -2, 2);
	else
		shoulderCalibrationOffset.Init(0, 0, 5);  // no forward or side offset for 360 setup until we can explicitly track body frame to apply "forward" and "side" to
	
	_weaponOffsetCalibration = weapOffset + shoulderCalibrationOffset;

	// todo: So much cleanup needs to be done here....
	Vector offset;
	getLeftHandOffset(offset, false);
	shoulderCalibrationOffset[YAW] *= -1; // left controller should be mirrored position to right
	_leftHandOffsetCalibration = offset + shoulderCalibrationOffset;
}

void VrController::shutDown()
{
	if (_initialized)
	{
		Msg("Shutting down VR Controller");
		_initialized = false;
		_vrIO->dispose();
	}
	else 
	{
		Msg("VR Controller already shut down, nothing to do here.");
	}
	// delete _vrIO; 
}

extern VrController* VR_Controller()
{
	return _vrController;
}

void VrController::getHeadOffset(Vector &headOffset, bool calibrated)
{
	// HACK: Just temporarily dabbling with left hand positional stuff, there's a more proper place to put this but for the moment this is fine
	if ( vr_hydra_left_hand.GetInt() >= 1 )
	{
		getLeftHandOffset(headOffset, true);
		return;
	}



	float neckLength = vr_neck_length.GetFloat();
	headOffset.z -= neckLength;
	
	QAngle headAngle = headOrientation();
		
	Vector up;
	AngleVectors(headAngle, NULL, NULL, &up);
	headOffset += up*neckLength;
	
	// TODO: collision detection necessary with larger sizes 
	// Msg("getHeadOffset(%.1f) position %f %f %f\n", neckLength, headOffset.x, headOffset.y, headOffset.z);
}

void VrController::getWeaponOffset(Vector &offset, bool calibrated)
{
	offset.Init();
	if ( hydraConnected() )
	{
		HydraControllerData data;
		hydraRight(data);

		offset.y = MM_TO_INCHES(data.pos.x);
		offset.z = MM_TO_INCHES(data.pos.y);
		offset.x = -MM_TO_INCHES(data.pos.z);
		
		if (calibrated)
		{
			offset -= _weaponOffsetCalibration; // weapon calibration is stored in hydra-space

			// orient weapon offset with the body as the hydra base stays body relative (but dump any pitch & roll, only care about yaw)
			QAngle body(0, bodyOrientation().y, 0);
			Vector forward, right, up;
			AngleVectors(body, &forward, &right, &up);
			offset = forward*offset.x + right*offset.y + up*offset.z;

			offset *= vr_weapon_movement_scale.GetFloat();

		}
	}
} 


void VrController::getLeftHandOffset(Vector &offset, bool calibrated)
{
	offset.Init();
	if ( hydraConnected() )
	{
		HydraControllerData data;
		hydraLeft(data);

		offset.y = MM_TO_INCHES(data.pos.x);
		offset.z = MM_TO_INCHES(data.pos.y);
		offset.x = -MM_TO_INCHES(data.pos.z);
		
		if (calibrated)
		{
			offset -= _leftHandOffsetCalibration; // weapon calibration is stored in hydra-space

			// orient weapon offset with the body as the hydra base stays body relative (but dump any pitch & roll, only care about yaw)
			QAngle body(0, bodyOrientation().y, 0);
			Vector forward, right, up;
			AngleVectors(body, &forward, &right, &up);
			offset = forward*offset.x + right*offset.y + up*offset.z;

			offset *= vr_weapon_movement_scale.GetFloat();

		}
	}
} 




HmdInfo VrController::hmdInfo()
{
       HmdInfo h;

       HMDDeviceInfo info = _vrIO->getHMDInfo();

       for (int i=0;i<4;i++)
               h.DistortionK[i] = info.DistortionK[i];

       h.HResolution = info.HResolution;
       h.HScreenSize = info.HScreenSize;
       h.VResolution = info.VResolution;
       h.VScreenSize = info.VScreenSize;
       h.VScreenCenter = info.VScreenCenter;
       h.InterpupillaryDistance = info.InterpupillaryDistance;
       h.LensSeparationDistance = info.LensSeparationDistance;
       h.EyeToScreenDistance = info.EyeToScreenDistance;

       return h;
}


*/