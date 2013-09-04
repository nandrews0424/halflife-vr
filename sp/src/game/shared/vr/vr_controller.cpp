#include "cbase.h"
#include "vr/vr_controller.h"
#include <sixense_math.hpp>

using sixenseMath::Vector2;
using sixenseMath::Vector3;
using sixenseMath::Vector4;
using sixenseMath::Quat;
using sixenseMath::Line;

static ConVar vr_weapon_movement_scale( "vr_weapon_movement_scale", "1", FCVAR_ARCHIVE, "Scales tracked weapon positional tracking");
static ConVar vr_offset_calibration("vr_offset_calibration", "1", FCVAR_ARCHIVE, "Toggles offset (right and forward calibration), 0 is less convenient but necessary for 360 setup where there is no frame of reference for forward and side");
static ConVar vr_hydra_left_hand("vr_hydra_left_hand", "0", 0, "Experimental: Use your left hand hydra for head tracking, 1 is position only, 2 for both position and orientation");

MotionTracker* _motionTracker;

#define MM_TO_INCHES(x) (x/1000.f)*(1/METERS_PER_INCH)

extern MotionTracker* g_MotionTracker()
{
	return _motionTracker;
}


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

	_calibrationMatrix.Identity();
	_matLeftHand.Identity();
	_matRightHand.Identity();

	_counter = 0;

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
	return;

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



// TODO: get the weapon position based on offsets from center view....
 
void GetWeaponPosition(matrix3x4_t mCenterView)
{

}

bool MotionTracker::isTrackingWeapon( )
{
	return _initialized && _vrIO->hydraConnected();
}


void MotionTracker::updateViewmodelOffset(Vector& vmorigin, QAngle& vmangles)
{
	Hydra_Message m;
	_vrIO->hydraData(m);

	matrix3x4_t leftHandMatrix = GetLeftMatrix(m);
	matrix3x4_t rightHandMatrix = GetRightMatrix(m);
	
	QAngle weaponAngle;
	Vector weaponPos;

	MatrixAngles(rightHandMatrix, weaponAngle, weaponPos);
	
	_counter++;
	if ( (_counter % 30) == 0 ) {
		Msg("Weapon Angle: %.2f %.2f %.2f\n\n", weaponAngle.x, weaponAngle.y, weaponAngle.z);
		Msg("Weapon Position: %.2f %.2f %.2f\n", weaponPos.x, weaponPos.y, weaponPos.z);
	}
		
	VectorCopy(weaponAngle, vmangles);
	vmorigin += weaponPos;
}
