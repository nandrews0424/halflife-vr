#include "cbase.h"
#include "vr/vr_controller.h"
#include <sixense_math.hpp>

using sixenseMath::Vector2;
using sixenseMath::Vector3;
using sixenseMath::Vector4;
using sixenseMath::Quat;
using sixenseMath::Line;


void mt_calibrate_f( void )
{
    Msg("Beginning Motion Tracker Calibration.... \n");
	g_MotionTracker()->beginCalibration();
}
 
ConCommand mt_calibrate( "mt_calibrate", mt_calibrate_f, "Shows a message.", 0 );

static ConVar mt_torso_movement_scale( "mt_torso_movement_scale", "1", FCVAR_ARCHIVE, "Scales positional tracking for torso tracker");
static ConVar mt_right_hand_movement_scale( "mt_right_hand_movement_scale", "1", FCVAR_ARCHIVE, "Scales positional tracking for right hand");

MotionTracker* _motionTracker;

#define MM_TO_INCHES(x) (x/1000.f)*(1/METERS_PER_INCH)

extern MotionTracker* g_MotionTracker() { 	return _motionTracker; }


// A bunch of sixense integration crap that needs to be pulled out along w/ vr.io`
matrix3x4_t ConvertMatrix( sixenseMath::Matrix4 ss_mat )
{
	sixenseMath::Matrix4 tmp_mat( ss_mat );

	tmp_mat = tmp_mat * sixenseMath::Matrix4::rotation( -3.1415926f / 2.0f, Vector3( 1, 0, 0 ) );
	tmp_mat = tmp_mat * sixenseMath::Matrix4::rotation( -3.1415926f / 2.0f, Vector3( 0, 0, 1 ) );
	tmp_mat = tmp_mat * sixenseMath::Matrix4::rotation( 3.1415926f, Vector3( 0, 1, 0 ) );
	tmp_mat = sixenseMath::Matrix4::rotation( -3.1415926f / 2.0f, Vector3( 1, 0, 0 ) ) * tmp_mat;
	tmp_mat = sixenseMath::Matrix4::rotation( -3.1415926f / 2.0f, Vector3( 0, 1, 0 ) ) * tmp_mat;
		
	matrix3x4_t retmat;

	MatrixInitialize( 
		retmat, 
		Vector( tmp_mat[3][0], tmp_mat[3][1], tmp_mat[3][2] ),
		Vector( tmp_mat[1][0], tmp_mat[1][1], tmp_mat[1][2] ),
		Vector( tmp_mat[2][0], tmp_mat[2][1], tmp_mat[2][2] ),
		Vector( tmp_mat[0][0], tmp_mat[0][1], tmp_mat[0][2] )
	);

	return retmat;
}

matrix3x4_t GetLeftMatrix(Hydra_Message m) {
	sixenseMath::Matrix4 matrix;

	matrix = sixenseMath::Matrix4::rotation( sixenseMath::Quat( m.leftRotationQuat[0], m.leftRotationQuat[1], m.leftRotationQuat[2], m.leftRotationQuat[3] ) );
	Vector3 ss_left_pos = sixenseMath::Vector3( 
								MM_TO_INCHES(m.posLeft[0]), 
								MM_TO_INCHES(m.posLeft[1]), 
								MM_TO_INCHES(m.posLeft[2])
							);
	
	matrix.set_col( 3, sixenseMath::Vector4( ss_left_pos, 1.0f ) );

	return ConvertMatrix(matrix);
}

matrix3x4_t GetRightMatrix(Hydra_Message m) {
	sixenseMath::Matrix4 matrix;

	matrix = sixenseMath::Matrix4::rotation( sixenseMath::Quat( m.rightRotationQuat[0], m.rightRotationQuat[1], m.rightRotationQuat[2], m.rightRotationQuat[3] ) );
	
	Vector3 ss_left_pos = sixenseMath::Vector3( 
								MM_TO_INCHES(m.posRight[0]), 
								MM_TO_INCHES(m.posRight[1]), 
								MM_TO_INCHES(m.posRight[2])
							);
	matrix.set_col( 3, sixenseMath::Vector4( ss_left_pos, 1.0f ) );

	return ConvertMatrix(matrix);
}


MotionTracker::MotionTracker()
{
	Msg("Initializing Motion Tracking API");
	
	_fBaseToTorsoYaw = 0;
	_prevYawTorso = 0;
	_accumulatedYawTorso = 0;
	_counter = 0;
	_calibrate = true;
			
	// todo: fake for now, need to figure out a way to capture this in calibration step...
	PositionMatrix(Vector(0, 0, -8), _eyesToTorsoTracker);

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




bool MotionTracker::isTrackingWeapon( )
{
	return _initialized && _vrIO->hydraConnected();
}

bool MotionTracker::isTrackingTorso( )
{
	return true; // todo: check for stem tracker...
}


matrix3x4_t MotionTracker::getTrackedTorso()
{
	Hydra_Message m;
	_vrIO->hydraData(m);
	return GetLeftMatrix(m);
}

void MotionTracker::updateViewmodelOffset(Vector& vmorigin, QAngle& vmangles)
{

	Hydra_Message m;
	_vrIO->hydraData(m);

	matrix3x4_t weaponMatrix	= GetRightMatrix(m);
	matrix3x4_t torsoMatrix		= getTrackedTorso();
	
	QAngle weaponAngle;
	Vector weaponPos;

	// get raw torso and weap positions, construct the distance from the diff of those two + the distance to the eyes from a properly calibrated torso tracker...
	Vector vTorso, vWeapon, vEyes;
	MatrixPosition(torsoMatrix, vTorso);
	MatrixPosition(weaponMatrix, vWeapon);
	MatrixPosition(_eyesToTorsoTracker, vEyes);
	
	Vector vEyesToWeapon = (vWeapon - _vecBaseToTorso)  + vEyes;  // was ( weapon - torso ) but since at this point the torso changes haven't been applied (get overridden in the view), that's unnecessary...
		
	PositionMatrix(vEyesToWeapon, weaponMatrix);						// position is reset rather than distance to base to distance to torso tracker
	
	MatrixMultiply(_sixenseToWorld, weaponMatrix, weaponMatrix);		// project weapon matrix by the base engine yaw
	
	MatrixAngles(weaponMatrix, weaponAngle, weaponPos);					// get the angles back off
	VectorCopy(weaponAngle, vmangles);
	vmorigin += weaponPos;
}

void MotionTracker::overrideViewOffset(VMatrix& viewMatrix)
{
	matrix3x4_t torsoMatrix = getTrackedTorso();
	Vector offset;
	MatrixPosition(torsoMatrix, offset);
	offset -= _vecBaseToTorso;
	PositionMatrix(offset, torsoMatrix);

	// TODO: NA - for whatever reason couldn't seem to get a functioning calibration matrix, too long a day I guess

	MatrixMultiply(_sixenseToWorld, torsoMatrix, torsoMatrix);
			
	Vector torsoOffset;
	MatrixPosition(torsoMatrix, torsoOffset);
		
	Vector viewTranslation = viewMatrix.GetTranslation();
	viewTranslation += torsoOffset;
	viewMatrix.SetTranslation(viewTranslation);
}


void MotionTracker::overrideWeaponMatrix(VMatrix& weaponMatrix)
{
	QAngle weaponAngle;
	MatrixToAngles(weaponMatrix, weaponAngle);
	Vector weaponOrigin = weaponMatrix.GetTranslation();	
		
	// for now let's just reuse the overrideviewmodel stuff that does exactly the same thing (from mideye view to weapon position & orientation..)
	updateViewmodelOffset(weaponOrigin, weaponAngle);

	MatrixFromAngles(weaponAngle, weaponMatrix);
	weaponMatrix.SetTranslation(weaponOrigin);
}

void MotionTracker::overrideMovement(Vector& movement)
{
	QAngle angle;
	VectorAngles(movement, angle);
	float dist = movement.Length();

	angle.x = 0;
	angle.z = 0;
	angle.y -= _accumulatedYawTorso;
	
	AngleVectors(angle, &movement);
	movement.NormalizeInPlace();
	movement*=dist;
}

// Update uses the inbound torso (view if no rift) angles and uses them update / the base matrix that should be applied to sixense inputs....
void MotionTracker::update(VMatrix& torsoMatrix)
{
	if ( !isTrackingTorso() )
		return;
	
	if ( _calibrate )
		calibrate(torsoMatrix);

	QAngle torsoAngle;
	MatrixToAngles(torsoMatrix, torsoAngle);

	QAngle trackedTorsoAngles; 
	MatrixAngles(getTrackedTorso(), trackedTorsoAngles);
	
	_accumulatedYawTorso += ( trackedTorsoAngles.y - _prevYawTorso );
	_prevYawTorso = trackedTorsoAngles.y;
	
	if ( writeDebug() )
		Msg("Updating sixense to world, tracked value %.2f \t calibration %.2f \n", torsoAngle.y, _fBaseToTorsoYaw );
	
	AngleMatrix(QAngle(0, torsoAngle.y - _fBaseToTorsoYaw, 0), _sixenseToWorld);

	_counter++;
}

void MotionTracker::beginCalibration() { _calibrate = true; }
void MotionTracker::calibrate(VMatrix& torsoMatrix)
{
	if ( !isTrackingTorso() ) {
		
		Msg("Don't know how to calibrate without a torso reading\n");
		return;	
	}

	Msg("Calibrating Motion Trackers...\n");

	// Adjust _sixenseToWorld per current torso/view angles
	QAngle trackedTorsoAngles, engineTorsoAngles; 
	MatrixToAngles(torsoMatrix, engineTorsoAngles);
	MatrixAngles(getTrackedTorso(), trackedTorsoAngles);
	
	_accumulatedYawTorso = 0;	// only used for movement vector adjustments, completely redundant ...

	QAngle angles;
	matrix3x4_t _torsoCalibration = getTrackedTorso();
	MatrixAngles(_torsoCalibration, angles, _vecBaseToTorso);
	_fBaseToTorsoYaw = angles.y;

	Msg("Torso offset calibrated at \t: %.2f %.2f %2.f \n", _vecBaseToTorso.x, _vecBaseToTorso.y, _vecBaseToTorso.z); 
	Msg("Torso angle calibrated at \t: %.2f \n", angles.y); 

	_calibrate = false;
}


void MotionTracker::overrideJoystickInputs(float& lx, float& ly, float& rx, float& ry)
{
	Hydra_Message m;
	_vrIO->hydraData(m);

	// NA TODO: for the moment, override only the right stick since we'll be using an xbox 360 controller in the left hand...
	// ly = m.leftJoyY * 32766;
	// lx = m.leftJoyX * 32766;
	ry = m.rightJoyY * 32766;
	rx = m.rightJoyX * 32766;
}


bool MotionTracker::writeDebug() {
	return (_counter % 60) == 0;
}
