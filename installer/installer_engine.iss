; Script for INNO SETUP
;
; This script generates the ChronoEngine_v.xxx.exe SDK installer (the 'setup exe' which can
; be distributed to people which want to use the chrono::engine SDK)
; It copies the .h headers, the libs, the dls, the docs etc. on the target system. Also,
; it installs a new VisualStudio wizard.
;
; Copyright 2009 A.Tasora

#define MyAppName "ChronoEngine"
#define MyAppVersion "v1.3.0"
#define MyAppPublisher "Alessandro Tasora"
#define MyAppURL "http://www.deltaknowledge.com"
#define MyChronoEngineSDK "C:\tasora\code\dynamics\code\ChronoEngine"

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
Source: {#MyChronoEngineSDK}\*; Excludes: "*.c,*.cpp,*.cu,doxygen,\bin,\scripts,\source\collision\gimpact,\source\collision\edgetempest,\source\HOWTO_COMPILE_API.txt,*.pdb,_obsolete,\installer,*.o,*.obj,*.ncb,*.bat,source\*.def,*\.svn"; DestDir: "{app}"; Flags: recursesubdirs createallsubdirs
Source: {#MyChronoEngineSDK}\bin\*; Excludes: "*.pdb,demo_benchmark.exe,*\.svn,*.ilk,*.idb,\data\mpi"; DestDir: "{app}\bin"; Flags: recursesubdirs createallsubdirs
;Source: "C:\tasora\lavori\html\chronoengine\install.html"; DestDir: "{app}\docs";
;Source: "C:\tasora\lavori\html\chronoengine\tutorials.html"; DestDir: "{app}\docs";
;Source: "C:\tasora\lavori\html\chronoengine\tutorials\*.*"; Excludes: "*.php"; DestDir: "{app}\docs\tutorials";  Flags: recursesubdirs
Source: "C:\tasora\lavori\html\chronoengine\chronoengine_clock_small.jpg"; DestDir: "{app}\docs";
Source: "chronostyle.css"; DestDir: "{app}\docs";


Source: {#MyChronoEngineSDK}\msvc_config\ChronoEngineWizard\*; DestDir: {code:myGetPathVisual9}VCWizards\ChronoEngineWizard; Check: myFoundVisual9; Flags: recursesubdirs createallsubdirs
Source: {#MyChronoEngineSDK}\msvc_config\ChronoEngineWizard\HTML\1033\default.htm; DestDir: {code:myGetPathVisual9}VCWizards\ChronoEngineWizard\HTML\1033; AfterInstall: myAfterWizardInstall; Check: myFoundVisual9; Flags: ignoreversion
Source: {#MyChronoEngineSDK}\msvc_config\ChronoEngine\*; DestDir: {code:myGetPathVisual9}Express\VCProjects\ChronoEngine; Check: myFoundVisual9;

Source: {#MyChronoEngineSDK}\msvc_config\ChronoEngineWizard\*; DestDir: {code:myGetPathVisual10}VCWizards\ChronoEngineWizard; Check: myFoundVisual10; Flags: recursesubdirs createallsubdirs
Source: {#MyChronoEngineSDK}\msvc_config\ChronoEngineWizard\HTML\1033\default.htm; DestDir: {code:myGetPathVisual10}VCWizards\ChronoEngineWizard\HTML\1033; AfterInstall: myAfterWizardInstall; Check: myFoundVisual10; Flags: ignoreversion
Source: {#MyChronoEngineSDK}\msvc_config\ChronoEngine\*; DestDir: {code:myGetPathVisual10}Express\VCProjects\ChronoEngine; Check: myFoundVisual10;



[Icons]
;Name: "{group}\Getting started"; Filename: "{app}\docs\install.html"
;Name: "{group}\Tutorials"; Filename: "{app}\docs\tutorials.html"
Name: "{group}\API documentation"; Filename: "{app}\docs\html\help.chm"
Name: "{group}\Readme"; Filename: "{app}\readme.txt"
Name: "{group}\Convex decomp. utility"; Filename: "{app}\bin\Win32_VisualStudio\demo_decomposition.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Crank"; Filename: "{app}\bin\Win32_VisualStudio\demo_crank.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Four bar"; Filename: "{app}\bin\Win32_VisualStudio\demo_fourbar.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Pendulum"; Filename: "{app}\bin\Win32_VisualStudio\demo_pendulum.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Collision"; Filename: "{app}\bin\Win32_VisualStudio\demo_collision.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Bricks"; Filename: "{app}\bin\Win32_VisualStudio\demo_bricks.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Suspension"; Filename: "{app}\bin\Win32_VisualStudio\demo_suspension.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Racing"; Filename: "{app}\bin\Win32_VisualStudio\demo_racing.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Oscillator"; Filename: "{app}\bin\Win32_VisualStudio\demo_oscillator.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Gears"; Filename: "{app}\bin\Win32_VisualStudio\demo_gears.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Tracks"; Filename: "{app}\bin\Win32_VisualStudio\demo_tracks.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Mecanum"; Filename: "{app}\bin\Win32_VisualStudio\demo_mecanum.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Matlab"; Filename: "{app}\bin\Win32_VisualStudio\demo_matlab.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Roll.friction"; Filename: "{app}\bin\Win32_VisualStudio\demo_friction.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Forklift"; Filename: "{app}\bin\Win32_VisualStudio\demo_forklift.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Tire convex decomp."; Filename: "{app}\bin\Win32_VisualStudio\demo_tire.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Cohesion"; Filename: "{app}\bin\Win32_VisualStudio\demo_cohesion.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\demos\Conveyor"; Filename: "{app}\bin\Win32_VisualStudio\demo_conveyor.exe"; WorkingDir: "{app}\bin\Win32_VisualStudio"
Name: "{group}\Uninstall"; Filename: "{uninstallexe}"

;[Run]
;Filename: "{app}\docs\tutorials.html"; Flags: shellexec postinstall



[Code]
var
  mPathVisual9: String;
  mFoundVisual9: Boolean;
  mPathVisual10: String;
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
function myFoundVisual10(): Boolean;
begin
  Result := mFoundVisual10;
end;
function myGetPathVisual10(Param: String): String;
begin
  Result := mPathVisual10;
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

// Not used anymore (it was used to init config.mak)
procedure myAfterMakesInstall();
var
  myContent: String;
  myContentFile: String;
  myIrrlichtDir: String;
  myIrrklangDir: String;
  myChronoDir: String;
begin
  myContentFile := ExpandConstant(CurrentFileName);
  myChronoDir   :=  ExpandConstant('{app}');
  myIrrlichtDir :=  IrrlichtDirPage.Values[0];
  LoadStringFromFile(myContentFile, myContent);
  StringChange(myContent, '***CHRONODIR***', myChronoDir);
  StringChange(myContent, '***IRRLICHTDIR***', myIrrlichtDir);
  SaveStringToFile(myContentFile, myContent, false);
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
  StringChange(myContent, '***IRRLICHTDIR***', myIrrlichtDir);
  StringChange(myContent, '***IRRKLANGDIR***', myIrrklangDir);
  SaveStringToFile(myContentFile, myContent, false);
end;


procedure InitializeWizard;
var
  mTitleIrrOptions : String;
begin

  // ChECK MICROSOFT VISUAL C++ 9.0 INSTALLATION
  mFoundVisual9 := False;
  if RegQueryStringValue(HKEY_LOCAL_MAChINE,
                  'SOFTWARE\Microsoft\VCExpress\9.0\Setup\VC',
                  'ProductDir',
                  mPathVisual9) then
  begin
        mFoundVisual9 := True;
  end
  // ChECK MICROSOFT VISUAL C++ 10.0 INSTALLATION
  mFoundVisual10 := False;
  if RegQueryStringValue(HKEY_LOCAL_MAChINE,
                  'SOFTWARE\Microsoft\VCExpress\10.0\Setup\VC',
                  'ProductDir',
                  mPathVisual10) then
  begin
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
      S := S + 'A copy of Visual Studio Express 9 has been found in:' + NewLine;
      S := S + Space + mPathVisual9 + NewLine;
      S := S + '(a new code wizard will be added to your Visual C++ editor)' + NewLine + NewLine;
  end
  if (mFoundVisual10 = True)  then
  begin
      S := S + 'A copy of Visual Studio Express 10 has been found in:' + NewLine;
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


