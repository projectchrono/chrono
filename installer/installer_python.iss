#include "ModifyPath.iss"

#define MyAppName "PyChronoEngine"
#define MyAppVersion "v1.7.1"
#define MyAppPublisher "Alessandro Tasora"
#define MyAppURL "http://www.chronoengine.info"
#define MyWin32PythonDir  "C:\Python32"
#define MyPythonVers "3.3"
#define MyChronoEngineSDK "C:\tasora\code\nightly_repo\code\ChronoEngine"

[Setup]
ShowLanguageDialog=yes
UserInfoPage=no
AppCopyright=A.Tasora
AppName={#MyAppName}
AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}
DefaultDirName={pf}\{#MyAppName}
DefaultGroupName={#MyAppName}
WizardImageFile=SetupModern20.bmp
WizardSmallImageFile=SetupModernSmall26.bmp
PrivilegesRequired=admin
;Compression=none
OutputDir=c:\tasora\lavori\data_chrono
OutputBaseFilename=PyChronoEngine_{#MyAppVersion}

[Files]
Source: {#MyChronoEngineSDK}\bin\data\*; Excludes: "*\.svn,\data\mpi"; DestDir: "{app}\bin\data"; Flags: recursesubdirs createallsubdirs
Source: {#MyChronoEngineSDK}\python\*; DestDir: "{app}\python"; Flags: recursesubdirs createallsubdirs

Source: {#MyWin32PythonDir}\DLLs\_ChronoEngine_PYTHON_core.pyd; DestDir: {code:myGetPathWin32PythonDLLs};  Flags: ignoreversion;  Check: myFoundWin32Python;
Source: {#MyWin32PythonDir}\DLLs\_ChronoEngine_PYTHON_postprocess.pyd; DestDir: {code:myGetPathWin32PythonDLLs};  Flags: ignoreversion;  Check: myFoundWin32Python;
Source: {#MyWin32PythonDir}\DLLs\_ChronoEngine_PYTHON_irrlicht.pyd; DestDir: {code:myGetPathWin32PythonDLLs};  Flags: ignoreversion;  Check: myFoundWin32Python;
Source: {#MyWin32PythonDir}\DLLs\ChronoEngine.dll; DestDir: {code:myGetPathWin32PythonDLLs};  Flags: ignoreversion;  Check: myFoundWin32Python;
Source: {#MyWin32PythonDir}\DLLs\ChronoEngine_POSTPROCESS.dll; DestDir: {code:myGetPathWin32PythonDLLs};  Flags: ignoreversion;  Check: myFoundWin32Python;
Source: {#MyWin32PythonDir}\DLLs\Irrlicht.dll; DestDir: {code:myGetPathWin32PythonDLLs};  Flags: ignoreversion;  Check: myFoundWin32Python;
Source: {#MyWin32PythonDir}\lib\ChronoEngine_PYTHON_core.py; DestDir: {code:myGetPathWin32PythonLib};  Flags: ignoreversion;  Check: myFoundWin32Python;
Source: {#MyWin32PythonDir}\lib\ChronoEngine_PYTHON_postprocess.py; DestDir: {code:myGetPathWin32PythonLib};  Flags: ignoreversion;  Check: myFoundWin32Python;
Source: {#MyWin32PythonDir}\lib\ChronoEngine_PYTHON_irrlicht.py; DestDir: {code:myGetPathWin32PythonLib};  Flags: ignoreversion;  Check: myFoundWin32Python;


[Icons]
Name: "{group}\Getting started"; Filename: "http://www.chronoengine.info"
Name: "{group}\Demos"; Filename: "{app}\python"
Name: "{group}\Uninstall"; Filename: "{uninstallexe}"


[Code]
var
  DataDirPage: TInputDirWizardPage;
  mFoundWin32Python: Integer;
  mPathWin32Python: String;
  mPathWin32PythonLib: String;
  mPathWin32PythonDLLs: String;
  mFoundWin64Python: Integer;
  mPathWin64Python: String;
  mPathWin64PythonLib: String;
  mPathWin64PythonDLLs: String;
  

function myFoundWin32Python: Boolean;
begin
  Result := mFoundWin32Python =1;
end;
function myGetPathWin32Python(Param: String): String;
begin
  Result := mPathWin32Python;
end;
function myGetPathWin32PythonLib(Param: String): String;
begin
  Result := mPathWin32PythonLib;
end;
function myGetPathWin32PythonDLLs(Param: String): String;
begin
  Result := mPathWin32PythonDLLs;
end;
function myFoundWin64Python: Boolean;
begin
  Result := mFoundWin64Python =1;
end;
function myGetPathWin64Python(Param: String): String;
begin
  Result := mPathWin64Python;
end;
function myGetPathWin64PythonLib(Param: String): String;
begin
  Result := mPathWin64PythonLib;
end;
function myGetPathWin64PythonDLLs(Param: String): String;
begin
  Result := mPathWin64PythonDLLs;
end;







procedure InitializeWizard;
var
  mTitlePythondir: String;
  mallDirPython: String;
  mCut: Integer;
  myvers: String;
begin


  
  // CHECK PYTHON INSTALLATION
  mFoundWin32Python := 0;
  mFoundWin64Python := 0;

  myvers := '{#MyPythonVers}';

  if (IsWin64) then
  begin
    // CASE OF 64 BIT PLATFORM

    // find 32 bit python:          
    if RegQueryStringValue(HKEY_LOCAL_MACHINE,
                    'SOFTWARE\Wow6432Node\Python\PythonCore\' +  myvers+ '\InstallPath',
                    '',
                    mallDirPython) then
    begin
          mPathWin32Python := mallDirPython; 
          mFoundWin32Python := 1;
    end
   
    // find 64 bit python:
    if RegQueryStringValue(HKEY_LOCAL_MACHINE,
                    'SOFTWARE\Python\PythonCore\' +  myvers+ '\InstallPath',
                    '',
                    mallDirPython) then
    begin
          mPathWin64Python := mallDirPython; 
          mFoundWin64Python := 1;
    end

  end 
  else
  begin
    // CASE OF 32 BIT PLATFORM

    // find 32 bit python:
    if RegQueryStringValue(HKEY_LOCAL_MACHINE,
                    'SOFTWARE\Python\PythonCore\' +  myvers+ '\InstallPath',
                    '',
                    mallDirPython) then
    begin
          mPathWin32Python := mallDirPython; 
          mFoundWin32Python := 1;
    end

  end

  mPathWin32PythonDLLs := AddBackslash(mPathWin32Python) + 'DLLs';
  mPathWin32PythonLib  := AddBackslash(mPathWin32Python) + 'Lib';
  mPathWin64PythonDLLs := AddBackslash(mPathWin64Python) + 'DLLs';
  mPathWin64PythonLib  := AddBackslash(mPathWin64Python) + 'Lib';

                                   
  { Create the pages }

  //
  // Select directory dialog for plugin
  //
  if mFoundWin32Python = 1 then begin mTitlePythondir := 'A copy of Win32 Python has been automatically detected, installed in this directory.'#13+
                                   'Do NOT modify the installation path unless you know what you are doing:'#13+
                                   'the Chrono::Engine module works only if installed in the proper Python directory!';
                    end else begin mTitlePythondir := 'The installation directory of Python has NOT been automatically found!. '#13+
                                   'Nevertheless, you can still go on with the installation, something went'#13+
                                   'wrong with your registry keys. Note: set this directory so that it is the'#13+
                                   'directory where python.exe file is installed, for Win32 bit'; end
                                   
  DataDirPage := CreateInputDirPage(wpWelcome,
    'Check Python directory', 'Where is your Python (version' +  myvers + ') directory?',
    mTitlePythondir,
    False, '');
  DataDirPage.Add('');
  DataDirPage.Values[0] := Format('%s\', [mPathWin32Python] );


end;


function NextButtonClick(CurPageID: Integer): Boolean;
var
  I: Integer;
  mfilesize: Integer;
begin

  //
  // After the user has entered data...
  //
  if CurPageID = wpWelcome then begin
      if ((mFoundWin32Python = 0) and (mFoundWin64Python = 0) ) then begin
        mPathWin32Python := '{#MyWin32PythonDir}';
        MsgBox('WARNING!'#13#13+
               'The installer was not able to detect Python (version' +  '{#MyPythonVers}' + ') '+
               'on your system.'#13#13+
               'Maybe your Python (version' +  '{#MyPythonVers}' + ') is not yet installed, or not properly installed?'#13+
               '(If so,please install/reinstall Python before installing this plug-in).', mbError, MB_OK);
      end;
      
      Result := True;

  //
  // After the user has choosen python directory..
  //
  end else if CurPageID = DataDirPage.ID then begin
  
      mPathWin32Python     := DataDirPage.Values[0];
      mPathWin32PythonDLLs := AddBackslash(mPathWin32Python) + 'DLLs';
      mPathWin32PythonLib  := AddBackslash(mPathWin32Python) + 'Lib';
        
      if FileOrDirExists(mPathWin32Python) and FileOrDirExists(mPathWin32PythonDLLs) and FileOrDirExists(mPathWin32PythonLib) then begin
        Result := True;
        mFoundWin32Python :=1;
      end else begin
        MsgBox('Error. The directory '+ mPathWin32Python + ' does not exist', mbError, MB_OK);
        Result := False;
      end
      

  end else
      Result := True;
end;

function ShouldSkipPage(PageID: Integer): Boolean;
begin
  Result := False;
  
  { Skip pages that shouldn't be shown }

  if (PageID = wpSelectDir) then
    Result := True

  if ((PageId = DataDirPage.ID) and ((mFoundWin64Python =1) or (mFoundWin32Python =1)))  then
    Result := True
end;


function UpdateReadyMemo(Space, NewLine, MemoUserInfoInfo, MemoDirInfo, MemoTypeInfo,
  MemoComponentsInfo, MemoGroupInfo, MemoTasksInfo: String): String;
var
  S: String;
begin
  { Fill the 'Ready Memo' with the normal settings and the custom settings }
  S := '';

  if (mFoundWin32Python = 1) then begin
    S := S + 'The Win32 Python directory is:' + NewLine;
    S := S + Space + mPathWin32Python + NewLine;
    S := S + 'so the Chrono::Engine 32 bit module will be installed in:' + NewLine;
    S := S + Space + mPathWin32PythonDLLs + NewLine;
    S := S + Space + mPathWin32PythonLib + NewLine;
  end

  if ((mFoundWin64Python = 1) and (mFoundWin32Python = 0)) then begin
    S := S + 'A Win64 Python has been detected in:' + NewLine;
    S := S + Space + mPathWin64Python + NewLine;
    S := S + 'but the Chrono::Engine 64 bit module is NOT INCLUDED in this installer AND CANNOT BE INSTALLED!' + NewLine;
    //S := S + 'so the Chrono::Engine 64 bit module will be installed in:' + NewLine;
    //S := S + Space + mPathWin64PythonDLLs + NewLine;
    //S := S + Space + mPathWin64PythonLib + NewLine;
  end

  if ((mFoundWin64Python = 0) and  (mFoundWin32Python = 0)) then begin
    S := S + 'Neither Win32 nor Win64 Python (version' +  '{#MyPythonVers}' + ') has been detected.' + NewLine;
    S := S + 'Chrono::Engine module CANNOT BE INSTALLED!' + NewLine;
  end

  Result := S;
end;

//function CheckSerial(Serial: String): Boolean;
//begin
//   Result:=True;
//end;
