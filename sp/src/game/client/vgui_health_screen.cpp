 
//=============================================================================//
//
// Purpose: Screen used to render player health information 
//
//=============================================================================//
#include "cbase.h"

#include "C_VGuiScreen.h"
#include <vgui/IVGUI.h>
#include <vgui_controls/Controls.h>
#include <vgui_controls/Label.h>
#include "clientmode_hlnormal.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"


//-----------------------------------------------------------------------------
//
// In-game vgui panel which shows the RPG's ammo count
//
//-----------------------------------------------------------------------------
class CHealthScreen : public CVGuiScreenPanel
{
    DECLARE_CLASS( CHealthScreen, CVGuiScreenPanel );

public:
    CHealthScreen( vgui::Panel *parent, const char *panelName );

    virtual bool Init( KeyValues* pKeyValues, VGuiScreenInitData_t* pInitData );
    virtual void OnTick();

private:
    vgui::Label *m_pHealth;
};

DECLARE_VGUI_SCREEN_FACTORY( CHealthScreen, "health_screen" );

CHealthScreen::CHealthScreen( vgui::Panel *parent, const char *panelName )
    : BaseClass( parent, panelName, g_hVGuiCombineScheme ) { }

bool CHealthScreen::Init( KeyValues* pKeyValues, VGuiScreenInitData_t* pInitData )
{
    // Load all of the controls in
    if ( !BaseClass::Init(pKeyValues, pInitData) )
        return false;

    // Make sure we get ticked...
    vgui::ivgui()->AddTickSignal( GetVPanel() );
    
    m_pHealth =  dynamic_cast<vgui::Label*>(FindChildByName( "HealthReadout" ));

    return true;
}


void CHealthScreen::OnTick()
{
    BaseClass::OnTick();

	// Get our player
    CBasePlayer *pPlayer = C_BasePlayer::GetLocalPlayer();
    if ( !pPlayer )
        return;
	
    if ( m_pHealth )
    {
        char buf[32];
        Q_snprintf( buf, sizeof( buf ), "%d", pPlayer->GetHealth() ); //todo: actual health plz
        m_pHealth->SetText( buf );
    }
}
