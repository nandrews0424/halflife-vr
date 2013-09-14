!include "MUI2.nsh"
!define VERSION '1.1.0'

Name "Half-Life VR"

OutFile ".\output\halflife-vr-${VERSION}.exe"

Var SDK_INSTALLED
Var STEAM_EXE

;TODO: 
;!define MUI_ICON '.\images\icon.ico'
;!define MUI_HEADERIMAGE images\header.bmp ;150x57

!define MUI_HEADERIMAGE
!define MUI_HEADERIMAGE_BITMAP "${NSISDIR}\Contrib\Graphics\Header\nsis.bmp" ; optional
  
;Welcome Page Settings
!define MUI_WELCOMEFINISHPAGE_BITMAP ".\images\welcome.bmp"; 164x314 
!define MUI_WELCOMEPAGE_TITLE 'Welcome to the installation for Half-Life 2 VR v${VERSION}'
!define MUI_WELCOMEPAGE_TEXT 'Thanks for installing Half-Life VR, this should only take a few seconds.  \
If you have any issues or ideas please send feedback so we can improve the mod.'

!define MUI_DIRECTORYPAGE_TEXT 'Please verify the location of your Steam sourcemods folder.'

!define MUI_FINISHPAGE_TITLE 'Installation Completed Successfully.'
!define MUI_FINISHPAGE_TEXT 'Restart steam to see the mod in your games list on Steam.'


!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_LANGUAGE "English"

Section "" 

	SetOutPath $INSTDIR\halflife-vr
	File /r ..\mod_hl2\*
	
	WriteUninstaller $INSTDIR\halflife-vr\Uninstall.exe

SectionEnd

Function .onInit
	
	ReadRegDWORD $SDK_INSTALLED HKCU "Software\Valve\Steam\Apps\243730" "Installed"
	; Check for Source SDK 2013
	ReadRegStr $R1 HKCU "Software\Valve\Steam" "SourceModInstallPath"
	ReadRegStr $STEAM_EXE HKCU "Software\Valve\Steam" "SteamExe"
	
	StrCmp $SDK_INSTALLED "1" SDK_INSTALLED
		

		MessageBox MB_YESNO|MB_ICONQUESTION \
		    "The Source SDK 2013 is required to play this mod but wasn't \ 
		    found on your computer. Do you want cancel this installation and install it now?" \
		    IDNO SKIP_SDK_INTALL

		    execshell open "steam://install/243730"
		    
		    Abort

		SKIP_SDK_INTALL:
			

	SDK_INSTALLED:

	StrCpy $INSTDIR "$R1"
	
FunctionEnd

Section "Uninstall"
	RMDir $INSTDIR\halflife-vr
SectionEnd