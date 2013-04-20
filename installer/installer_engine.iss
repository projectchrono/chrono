; Script for INNO SETUP
;
; This script generates the ChronoEngine_v.xxx.exe SDK installer (the 'setup exe' which can
; be distributed to people which want to use the chrono::engine SDK)
; It copies the .h headers, the libs, the dls, the docs etc. on the target system. Also,
; it installs a new VisualStudio wizard.
;
; Copyright 2009 A.Tasora

#define MyAppName "ChronoEngine"
#define MyAppVersion "v1.7.0"
#define MyAppPublisher "Alessandro Tasora"
#define MyAppURL "http://www.chronoengine.info"
#define MyChronoEngineSDK   "C:\tasora\code\nightly_repo\code\ChronoEngine"
#define MyChronoEngineBUILD32 "C:\tasora\code\nightly_build"
#define MyChronoEngineBUILD64 "C:\tasora\code\nightly_build64"

[Setup]
AppName={#MyAppName}
AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}
AppCopyright=DeltaKnowledge
LicenseFile=..\license.txt
DefaultDirName={pf}\{#MyAppName}
DefaultGroupName={#MyAppName}
WizardImageFile=SetupModern20.bmp
WizardSmallImageFile=SetupModernSmall26.bmp
PrivilegesRequired=admin
;Compression=none
Compression=lzma
;SolidCompression=yes
Uninstallable=yes
OutputDir=c:\tasora\lavori\data_chrono
OutputBaseFilename=ChronoEngine_{#MyAppVersion}

[Files]
Source: {#MyChronoEngineSDK}\*; Excludes: "*.c,*.cpp,*.cu,doxygen,\bin,\lib,\scripts,\source\collision\gimpact,\source\collision\edgetempest,\source\HOWTO_COMPILE_API.txt,*.pdb,_obsolete,\installer,*.o,*.obj,*.ncb,*.bat,source\*.def,*\.svn"; DestDir: "{app}"; Flags: recursesubdirs createallsubdirs
Source: {#MyChronoEngineSDK}\source\demos\*; Excludes: "*.pdb,demo_benchmark.exe,*\.svn,*.ilk,*.idb"; DestDir: "{app}\source\demos"; Flags: recursesubdirs createallsubdirs

Source: {#MyChronoEngineBUILD32}\bin\*; Excludes: "*.pdb,demo_benchmark.exe,*\.svn,*.ilk,*.idb,\data\mpi"; DestDir: "{app}\Win32\bin"; Flags: recursesubdirs createallsubdirs
Source: {#MyChronoEngineBUILD32}\lib\*; Excludes: "*.pdb,*\.svn,*.ilk,*.idb"; DestDir: "{app}\Win32\lib"; Flags: recursesubdirs createallsubdirs

Source: {#MyChronoEngineBUILD64}\bin\*; Excludes: "*.pdb,demo_benchmark.exe,*\.svn,*.ilk,*.idb,\data\mpi,*.exe"; DestDir: "{app}\Win64\bin"; Flags: recursesubdirs createallsubdirs
Source: {#MyChronoEngineBUILD64}\lib\*; Excludes: "*.pdb,*\.svn,*.ilk,*.idb"; DestDir: "{app}\Win64\lib"; Flags: recursesubdirs createallsubdirs


Source: {#MyChronoEngineSDK}\msvc_config\ChronoEngineWizard\*; DestDir: {code:myGetPathVisual9}VCWizards\ChronoEngineWizard; Check: myFoundVisual9; Flags: recursesubdirs createallsubdirs
Source: {#MyChronoEngineSDK}\msvc_config\ChronoEngineWizard\HTML\1033\default.htm; DestDir: {code:myGetPathVisual9}VCWizards\ChronoEngineWizard\HTML\1033; AfterInstall: myAfterWizardInstall; Check: myFoundVisual9; Flags: ignoreversion
Source: {#MyChronoEngineSDK}\msvc_config\ChronoEngine\*; DestDir: {code:myGetPathVisual9pr}ChronoEngine; Check: myFoundVisual9;

Source: {#MyChronoEngineSDK}\msvc_config\ChronoEngineWizard\*; DestDir: {code:myGetPathVisual10}VCWizards\ChronoEngineWizard; Check: myFoundVisual10; Flags: recursesubdirs createallsubdirs
Source: {#MyChronoEngineSDK}\msvc_config\ChronoEngineWizard\HTML\1033\default.htm; DestDir: {code:myGetPathVisual10}VCWizards\ChronoEngineWizard\HTML\1033; AfterInstall: myAfterWizardInstall; Check: myFoundVisual10; Flags: ignoreversion
Source: {#MyChronoEngineSDK}\msvc_config\ChronoEngine\*; DestDir: {code:myGetPathVisual10pr}ChronoEngine; Check: myFoundVisual10;



[Icons]
Name: "{group}\Getting started"; Filename: "{app}\docs\WebDocumentation.URL"
Name: "{group}\Tutorials"; Filename: "{app}\docs\WebTutorials.URL"
Name: "{group}\API documentation"; Filename: "{app}\docs\html\help.chm"
Name: "{group}\Readme"; Filename: "{app}\readme.txt"
Name: "{group}\Convex decomp. utility"; Filename: "{app}\Win32\bin\Release\demo_decomposition.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Crank"; Filename: "{app}\Win32\bin\Release\demo_crank.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Four bar"; Filename: "{app}\Win32\bin\Release\demo_fourbar.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Pendulum"; Filename: "{app}\Win32\bin\Release\demo_pendulum.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Collision"; Filename: "{app}\Win32\bin\Release\demo_collision.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Bricks"; Filename: "{app}\Win32\bin\Release\demo_bricks.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Suspension"; Filename: "{app}\Win32\bin\Release\demo_suspension.exe"; WorkingDir: "{app}\Win32\bin\Release"
;Name: "{group}\demos\Racing"; Filename: "{app}\Win32\bin\Release\demo_racing.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Oscillator"; Filename: "{app}\Win32\bin\Release\demo_oscillator.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Gears"; Filename: "{app}\Win32\bin\Release\demo_gears.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Tracks"; Filename: "{app}\Win32\bin\Release\demo_tracks.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Mecanum"; Filename: "{app}\Win32\bin\Release\demo_mecanum.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Matlab"; Filename: "{app}\Win32\bin\Release\demo_matlab.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Roll.friction"; Filename: "{app}\Win32\bin\Release\demo_friction.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Forklift"; Filename: "{app}\Win32\bin\Release\demo_forklift.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Tire convex decomp."; Filename: "{app}\Win32\bin\Release\demo_tire.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Cohesion"; Filename: "{app}\Win32\bin\Release\demo_cohesion.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Conveyor"; Filename: "{app}\Win32\bin\Release\demo_conveyor.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\demos\Imp.Solidworks (Python 3 required)"; Filename: "{app}\Win32\bin\Release\demo_import_solidworks.exe"; WorkingDir: "{app}\Win32\bin\Release"
Name: "{group}\Uninstall"; Filename: "{uninstallexe}"

;[Run]
;Filename: "{app}\docs\tutorials.html"; Flags: shellexec postinstall



[Code]
var
  mPathVisual9: String;
  mPathVisual9pr: String;
  mFoundVisual9: Boolean;
  mPathVisual10: String;
  mPathVisual10pr: String;
  mFoundVisual10: Boolean;
  
  ConfigOptionPage: TInputOptionWizardPage;
  IrrlichtDirPage: TInputDirWizardPage;
  mPathIrrlicht: String;
  IrrklangDirPage: TInputDirWizardPage;
  mPathIrrklang: String;


function myFoundVisual9(): Boolean;
begin
  Result := mFoundVisual9;
end;
function myGetPathVisual9(Param: String): String;
begin
  Result := mPathVisual9;
end;
function myGetPathVisual9pr(Param: String): String;
begin
  Result := mPathVisual9pr;
end;
function myFoundVisual10(): Boolean;
begin
  Result := mFoundVisual10;
end;
function myGetPathVisual10(Param: String): String;
begin
  Result := mPathVisual10;
end;
function myGetPathVisual10pr(Param: String): String;
begin
  Result := mPathVisual10pr;
end;

function myGetPathIrrlicht(Param: String): String;
begin
  Result := mPathIrrlicht;
end;
function myGetPathIrrklang(Param: String): String;
begin
  Result := mPathIrrklang;
end;

function BackslashToDoubleBackslash(const input_string: String): String;
var
  Iin: Integer;
  output_string: String;
begin
  Iin := 1;
  output_string := '';
  while Iin <= Length(input_string) do
  begin
    output_string := output_string + input_string[Iin];
    if input_string[Iin] = '\' then
      output_string := output_string + '\';
    // Go to the next character. But do not simply increment I by 1 in case is a double-byte character.
    Iin := Iin + CharLength(input_string, Iin);
  end;
  Result := output_string;
end;



procedure myAfterWizardInstall();
var
  myContent: String;
  myContentFile: String;
  myIrrlichtDir: String;
  myIrrklangDir: String;
  myChronoDir: String;
begin
  myContentFile := ExpandConstant(CurrentFileName);
  myChronoDir   := BackslashToDoubleBackslash( ExpandConstant('{app}') );
  myIrrlichtDir := BackslashToDoubleBackslash(IrrlichtDirPage.Values[0]);
  myIrrklangDir := BackslashToDoubleBackslash(IrrklangDirPage.Values[0]);
  LoadStringFromFile(myContentFile, myContent);
  StringChange(myContent, '***CHRONODIR***', myChronoDir);
  StringChange(myContent, '***CHRONOLIB***', myChronoDir + '\\Win32\\lib');
  StringChange(myContent, '***CHRONOBIN***', myChronoDir + '\\Win32\\bin');
  StringChange(myContent, '***IRRLICHTDIR***', myIrrlichtDir);
  StringChange(myContent, '***IRRKLANGDIR***', myIrrklangDir);
  SaveStringToFile(myContentFile, myContent, false);
end;


procedure InitializeWizard;
var
  mTitleIrrOptions : String;
begin

  // CHECK MICROSOFT VISUAL C++ 9.0 INSTALLATION (try Express, than Pro)
  mFoundVisual9 := False;
  if RegQueryStringValue(HKEY_LOCAL_MACHINE,
                  'SOFTWARE\Wow6432Node\Microsoft\VCExpress\9.0\Setup\VC',
                  'ProductDir',
                  mPathVisual9) then
  begin
        mPathVisual9pr  :=  mPathVisual9 + 'Express\VCProjects\';
        mFoundVisual9 := True;
  end
  if RegQueryStringValue(HKEY_LOCAL_MACHINE,
                  'SOFTWARE\Microsoft\VCExpress\9.0\Setup\VC',
                  'ProductDir',
                  mPathVisual9) then
  begin
        mPathVisual9pr  :=  mPathVisual9 + 'Express\VCProjects\';
        mFoundVisual9 := True;
  end
  if RegQueryStringValue(HKEY_LOCAL_MACHINE,
                  'SOFTWARE\Wow6432Node\Microsoft\VisualStudio\9.0\Setup\VC',
                  'ProductDir',
                  mPathVisual9) then
  begin
        mPathVisual9pr  :=  mPathVisual9 + 'VCProjects\';
        mFoundVisual9 := True;
  end
  if RegQueryStringValue(HKEY_LOCAL_MACHINE,
                  'SOFTWARE\Microsoft\VisualStudio\9.0\Setup\VC',
                  'ProductDir',
                  mPathVisual9) then
  begin
        mPathVisual9pr  :=  mPathVisual9 + 'VCProjects\';
        mFoundVisual9 := True;
  end

  // CHECK MICROSOFT VISUAL C++ 10.0 INSTALLATION (try Express, than Pro)
  mFoundVisual10 := False;
  if RegQueryStringValue(HKEY_LOCAL_MACHINE,
                  'SOFTWARE\Wow6432Node\Microsoft\VCExpress\10.0\Setup\VC',
                  'ProductDir',
                  mPathVisual10) then
  begin
        mPathVisual10pr  :=  mPathVisual10 + 'Express\VCProjects\';
        mFoundVisual10 := True;
  end
  if RegQueryStringValue(HKEY_LOCAL_MACHINE,
                  'SOFTWARE\Microsoft\VCExpress\10.0\Setup\VC',
                  'ProductDir',
                  mPathVisual10) then
  begin
        mPathVisual10pr  :=  mPathVisual10 + 'Express\VCProjects\';
        mFoundVisual10 := True;
  end
  if RegQueryStringValue(HKEY_LOCAL_MACHINE,
                  'SOFTWARE\Wow6432Node\Microsoft\VisualStudio\10.0\Setup\VC',
                  'ProductDir',
                  mPathVisual10) then
  begin
        mPathVisual10pr  :=  mPathVisual10 + 'VCProjects\';
        mFoundVisual10 := True;
  end
  if RegQueryStringValue(HKEY_LOCAL_MACHINE,
                  'SOFTWARE\Microsoft\VisualStudio\10.0\Setup\VC',
                  'ProductDir',
                  mPathVisual10) then
  begin
        mPathVisual10pr  :=  mPathVisual10 + 'VCProjects\';
        mFoundVisual10 := True;
  end

  { Create the pages }

  //
  // Select components to be configured
  //
         
  mTitleIrrOptions :=              ' If you are using Microsoft Visual Studio, this installer will add a new '#13+
                                   'wizard type to the VC++ project templates, to ease the creation of new projects.'#13+
                                   'This installer can help you to configure the VC++ wizard. '#13+
                                   'In fact, Chrono::Engine uses the Irrlicht as a visualization library to show'#13+
                                   '3D views of the demos, and the VC++ wizard must know where to find Irrlicht'#13+
                                   'in order to create new projects. '#13+
                                   ''#13+
                                   'NOTE 1: although Irrlicht is not mandatory, we suggest you to install Irrlicht before installing Chrono::Engine.'#13+
                                   ''#13+
                                   'NOTE 2: IrrKlang is a library for sound effects: it is optional.';
                                   
  ConfigOptionPage := CreateInputOptionPage(wpSelectDir,
  'Configuration helper', 'Do you want the installer to configure your Chrono::Engine SDK?',
  mTitleIrrOptions,
  False, False);
 
  // Add items
  ConfigOptionPage.Add('I have the Irrlicht SDK installed on my PC');

  // Add items
  ConfigOptionPage.Add('I have the Irrklang SDK installed on my PC');
  
  // Set initial values (optional)
  ConfigOptionPage.Values[0] := False;
  ConfigOptionPage.Values[1] := False;

  

  //
  // Select directory dialog for Irrlicht
  //

  IrrlichtDirPage := CreateInputDirPage(ConfigOptionPage.ID,
    'Set Irrlicht directory', 'Where have you installed Irrlicht SDK?',
    'Here you must set the directory where you installed your Irrlicht SDK (for example C:\Programs\irrlicht-1.7.1) ',
    False, '');
  IrrlichtDirPage.Add('');
  IrrlichtDirPage.Values[0] := '';

  //
  // Select directory dialog for Irrlicht
  //

  IrrklangDirPage := CreateInputDirPage(IrrlichtDirPage.ID,
    'Set Irrklang directory', 'Where have you installed IrrKlang SDK?',
    'Here you can set the directory where you installed your IrrKlang SDK (for example C:\Programs\irrKlang-1.1.2) ',
    False, '');
  IrrklangDirPage.Add('');
  IrrklangDirPage.Values[0] := '';
            
end;



function ShouldSkipPage(PageID: Integer): Boolean;
begin
  Result := False;

  { Skip pages that shouldn't be shown }
  if (PageID = IrrlichtDirPage.ID)and(ConfigOptionPage.Values[0]=False) then
    Result := True
  if (PageID = IrrklangDirPage.ID)and(ConfigOptionPage.Values[1]=False) then
    Result := True
end;


function UpdateReadyMemo(Space, NewLine, MemoUserInfoInfo, MemoDirInfo, MemoTypeInfo,
  MemoComponentsInfo, MemoGroupInfo, MemoTasksInfo: String): String;
var
  S: String;
begin

  S := '';

  S := S + 'The Chrono::Engine SDK will be installed in:' + NewLine;
  S := S + Space + ExpandConstant('{app}') + NewLine + NewLine;
  
  if (mFoundVisual9 = True)  then
  begin
      S := S + 'A copy of Visual Studio 9 has been found in:' + NewLine;
      S := S + Space + mPathVisual9 + NewLine;
      S := S + '(a new code wizard will be added to your Visual C++ editor)' + NewLine + NewLine;
  end
  if (mFoundVisual10 = True)  then
  begin
      S := S + 'A copy of Visual Studio 10 has been found in:' + NewLine;
      S := S + Space + mPathVisual10 + NewLine;
      S := S + '(a new code wizard will be added to your Visual C++ editor)' + NewLine + NewLine;
  end
  if (ConfigOptionPage.Values[0] = True)  then
  begin
    S := S + 'The Irrlicht directory is:' + NewLine;
    S := S + Space + IrrlichtDirPage.Values[0] + NewLine;
  end
  if (ConfigOptionPage.Values[1] = True)  then
  begin
    S := S + 'The IrrKlang directory is:' + NewLine;
    S := S + Space + IrrKlangDirPage.Values[0] + NewLine;
  end
  
  Result := S;
end;


