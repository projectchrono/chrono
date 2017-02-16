//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "ModifyPath.iss"

#define MyAppName "ChronoPyEngine"
#define MyAppVersion "v2.0.8"
#define MyAppPublisher "Alessandro Tasora"
#define MyAppURL "http://www.chronoengine.info"
#define MyWin64PythonDir  "D:\build\chrono\bin\Release"
#define MyPythonVers "3.3"
#define MyChronoEngineSDK "C:\tasora\code\projectchrono\chrono"

[Setup]
ShowLanguageDialog=yes
UserInfoPage=no
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
OutputBaseFilename=ChronoPyEngine_{#MyAppVersion}
ArchitecturesInstallIn64BitMode=x64
ChangesEnvironment=yes
DisableProgramGroupPage=yes

[Files]
Source: {#MyChronoEngineSDK}\data\*; Excludes: "*\.svn,\mpi,\vehicle,\cascade"; DestDir: "{app}\data"; Flags: recursesubdirs createallsubdirs
Source: {#MyChronoEngineSDK}\src\demos\python\*; DestDir: "{app}\src\demos\python\"; Flags: recursesubdirs createallsubdirs

Source: {#MyWin64PythonDir}\_ChronoEngine_python_core.pyd; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;
Source: {#MyWin64PythonDir}\_ChronoEngine_python_postprocess.pyd; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;
Source: {#MyWin64PythonDir}\_ChronoEngine_python_irrlicht.pyd; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;
Source: {#MyWin64PythonDir}\_ChronoEngine_python_fea.pyd; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;
Source: {#MyWin64PythonDir}\ChronoEngine.dll; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;
Source: {#MyWin64PythonDir}\ChronoEngine_postprocess.dll; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;
Source: {#MyWin64PythonDir}\ChronoEngine_irrlicht.dll; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;
Source: {#MyWin64PythonDir}\ChronoEngine_fea.dll; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;
Source: {#MyWin64PythonDir}\Irrlicht.dll; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;
Source: {#MyWin64PythonDir}\ChronoEngine_python_core.py; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;
Source: {#MyWin64PythonDir}\ChronoEngine_python_postprocess.py; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;
Source: {#MyWin64PythonDir}\ChronoEngine_python_irrlicht.py; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;
Source: {#MyWin64PythonDir}\ChronoEngine_python_fea.py; DestDir: {app};  Flags: ignoreversion;  Check: myFoundWin64Python;

[Icons]
Name: "{group}\ProjectChrono"; Filename: "http://www.projectchrono.org"
Name: "{group}\Getting started"; Filename: "http://api.chrono.projectchrono.org/introduction_chrono_pyengine.html"
Name: "{group}\Demos"; Filename: "{app}\src\demos\python\"
Name: "{group}\Uninstall"; Filename: "{uninstallexe}"

[Registry]
Root: "HKCU"; Subkey: "Environment"; ValueType: string; ValueName: "PYTHONPATH"; ValueData: "{app}"; Flags: createvalueifdoesntexist;
Root: "HKCU"; Subkey: "Environment"; ValueType: string; ValueName: "PYTHONPATH"; ValueData: "{olddata};{app}"; Flags: dontcreatekey; Check: NeedsAddPath(ExpandConstant('{app}'))

[Code]
var
  DataDirPage: TInputDirWizardPage;
  RegisterPathPage: TWizardPage;
  InstallHelpCheckBox: TNewCheckBox;
  InstallHelpLabel: TLabel;
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
function CheckAppendPath: Boolean;
begin
  Result := InstallHelpCheckBox.Checked=True;
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
   

        // find 64 bit python:          
    if RegQueryStringValue(HKLM64,
                    'SOFTWARE\Python\PythonCore\' +  myvers+ '\InstallPath',
                    '',
                    mallDirPython) then
    begin
          //MsgBox('FOUND! 64bit !'#13#13 + mallDirPython, mbError, MB_OK);
          mPathWin64Python := mallDirPython; 
          mFoundWin64Python := 1;
    end


        // find 32 bit python:          
    if RegQueryStringValue(HKEY_LOCAL_MACHINE,
                    'SOFTWARE\Wow6432Node\Python\PythonCore\' +  myvers+ '\InstallPath',
                    '',
                    mallDirPython) then
    begin
          //MsgBox('Attention. Found a 32bit version of Python on your system. The Chrono::PyEngine provided with this installer will work only with the 64 bit version. !'#13#13 + mallDirPython, mbError, MB_OK);
          mPathWin32Python := mallDirPython; 
          mFoundWin32Python := 1;
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
          //MsgBox('Attention. Found a 32bit version of Python on your system. The Chrono::PyEngine provided with this installer will work only with the 64 bit version. !'#13#13 + mallDirPython, mbError, MB_OK);
          mPathWin32Python := mallDirPython; 
          mFoundWin32Python := 1;
    end

  end

  mPathWin32PythonDLLs := mPathWin32Python;// AddBackslash(mPathWin32Python) + 'DLLs';
  mPathWin32PythonLib  := mPathWin32Python;// AddBackslash(mPathWin32Python) + 'Lib';
  mPathWin64PythonDLLs := mPathWin64Python;// AddBackslash(mPathWin64Python) + 'DLLs';
  mPathWin64PythonLib  := mPathWin64Python;// AddBackslash(mPathWin64Python) + 'Lib';

                                   
  { Create the pages }

  //
  // Select directory dialog for plugin
  //
  if mFoundWin64Python = 1 then begin mTitlePythondir := 'A copy of 64bit Python has been automatically detected, installed in this directory.'#13+
                                   '- you can install the Chrono::PyEngine libraries here (check that you have administrative previleges),'#13+
                                   '- or you can choose a different directory (preferred). If so, remember that you must '#13+
                                   '  set/append this directory to the PYTHONPATH environment variable.';
                    end else begin mTitlePythondir := 'A 64bit installation of Python has NOT been automatically found!. '#13+
                                   'Nevertheless, you can still go on with the installation, something went'#13+
                                   'wrong with your registry keys. Note: after the installation, in order for'#13+
                                   'the generated Python modules to be accessible, you must set/append this directory'#13+
                                   'to the PYTHONPATH environment variable.'; end
                                   
  DataDirPage := CreateInputDirPage(wpWelcome,
    'Choose directory for installation', 'Where do you want to install the Chrono::PyEngine libraries?',
    mTitlePythondir,
    False, '');
  DataDirPage.Add('');
  DataDirPage.Values[0] := Format('%s\', [mPathWin64Python] );

  RegisterPathPage := CreateCustomPage(wpSelectDir, 
    'Set PYTHONPATH', 
    'Automatically set the PYTHONPATH environment variable'#13 + 'with the installation path?');
  InstallHelpCheckBox := TNewCheckBox.Create(RegisterPathPage);
  InstallHelpCheckBox.Parent := RegisterPathPage.Surface;
  InstallHelpCheckBox.Checked := True;
  InstallHelpCheckBox.Width := 300;
  InstallHelpCheckBox.Caption := 'Set PYTHONPATH';

  InstallHelpLabel := TLabel.Create(RegisterPathPage);
  InstallHelpLabel.Parent := RegisterPathPage.Surface;
  InstallHelpLabel.Width := 300;
  InstallHelpLabel.Height := 300;
  InstallHelpLabel.Caption := ''#13#13#13#13 + 
    'When Python is started, it looks for additional modules (.pyd files) in' + #13 +
    'its directory, in the global paths, or in the PYTHONPATH directories.' + #13 +
    'So, we need to make also Chrono modules accessible to Python via PYTHONPATH.' + #13 +
    '' + #13 +
    'This option will automatically append the install directory' + #13 + 
    'to your PYTHONPATH environment variable. ' + #13 +
    '- If PYTHONPATH does not exist, it will create it.' + #13 +
    '- If PYTHONPATH already contains the install directory (for' + #13 +
    '  instance, because you installed Chrono::PyEngine in the past)' + #13 +
    '  this will not be appended.' + #13 +
    '' + #13 +
    'NOTE! If you do not select this, you must remember to append it manually!';

end;

// OBSOLETE
function NextButtonClick(CurPageID: Integer): Boolean;
var
  I: Integer;
  mfilesize: Integer;
begin

  //
  // After the user has entered data...
  //
  if CurPageID = DataDirPage.ID then begin
  
      mPathWin64Python     := DataDirPage.Values[0];
      mPathWin64PythonDLLs := mPathWin64Python; //:= AddBackslash(mPathWin64Python) + 'DLLs';
      mPathWin64PythonLib  := mPathWin64Python; //:= AddBackslash(mPathWin64Python) + 'Lib';
        
      if FileOrDirExists(mPathWin64Python) and FileOrDirExists(mPathWin64PythonDLLs) and FileOrDirExists(mPathWin64PythonLib) then begin
        Result := True;
        mFoundWin64Python :=1;
      end else begin
        MsgBox('Error. The directory '+ mPathWin64Python + ' does not exist', mbError, MB_OK);
        Result := False;
      end
      

  end else
      Result := True;
end;

// OBSOLETE - remove DataDirPage completely
function ShouldSkipPage(PageID: Integer): Boolean;
begin
  Result := False;
  
  { Skip pages that shouldn't be shown }

  //if (PageID = wpSelectDir) then
  //  Result := True

  if (PageId = DataDirPage.ID) then
      Result := True

  //if ((PageId = DataDirPage.ID) and ((mFoundWin64Python =1) or (mFoundWin32Python =1)))  then
  //  Result := True
end;


function UpdateReadyMemo(Space, NewLine, MemoUserInfoInfo, MemoDirInfo, MemoTypeInfo,
  MemoComponentsInfo, MemoGroupInfo, MemoTasksInfo: String): String;
var
  S: String;
begin
  { Fill the 'Ready Memo' with the normal settings and the custom settings }
  S := '';

  if (mFoundWin32Python = 1) then begin
    S := S + 'A Win32 Python has been detected in:' + NewLine;
    S := S + Space + mPathWin32Python + NewLine;
    S := S + 'but the Chrono::Engine 32 bit module is NOT INCLUDED in this installer AND CANNOT BE INSTALLED!' + NewLine;
  end

  if ((mFoundWin64Python = 1) and (mFoundWin32Python = 0)) then begin
    S := S + 'A Win64 Python has been detected in:' + NewLine;
    S := S + Space + mPathWin64Python + NewLine;
    S := S + NewLine;
    S := S + NewLine;
    S := S + 'The Chrono::Engine 64 bit module will be installed in:' + NewLine;
    S := S + space + ExpandConstant('{app}') + NewLine;

    if (CheckAppendPath = True) then begin
      S := S + NewLine;
      S := S + 'Your PYTHONPATH environment variable will be automatically' + NewLine;
      S := S + 'updated by appending the ' + NewLine;
      S := S + space + ExpandConstant('{app}') + NewLine;
      S := S + 'value, if not already existing.';
    end else begin
      S := S + NewLine;
      S := S + 'REMEMBER! you must manually add the directory' + NewLine;
      S := S + space + ExpandConstant('{app}') + NewLine;
      S := S + 'to your PYTHONPATH environment variable!' + NewLine;
      S := S + 'Otherwise, Python cannot find the installed Chrono modules.'
    end
  end

  if ((mFoundWin64Python = 0) and  (mFoundWin32Python = 0)) then begin
    S := S + 'Neither Win32 nor Win64 Python (version' +  '{#MyPythonVers}' + ') has been detected.' + NewLine;
    S := S + 'Chrono::Engine module CANNOT BE INSTALLED!' + NewLine;
  end

  //HKEY_CURRENT_USER\Environment 

  Result := S;
end;


function NeedsAddPath(Param: string): boolean;
var
  OrigPath: string;
  ParamExpanded: string;
begin
  //MsgBox('NeedsAddPath. check if already set:'#13#13 + Param, mbInformation, MB_OK);
  ParamExpanded := ExpandConstant(Param);
  if not RegQueryStringValue(HKCU64,
    'Environment',
    'PYTHONPATH', OrigPath)
  then begin
    //MsgBox('Attention. Environment\PYTHONPATH not found!'#13#13, mbInformation, MB_OK);
    Result := True;
    exit;
  end;

  // look for the path with leading and trailing semicolon and with or without \ ending
  // Pos() returns 0 if not found
  Result := Pos(';' + UpperCase(ParamExpanded) + ';', ';' + UpperCase(OrigPath) + ';') = 0;  
  if Result = True then
     Result := Pos(';' + UpperCase(ParamExpanded) + '\;', ';' + UpperCase(OrigPath) + ';') = 0;

  if (CheckAppendPath = False) then begin
     Result := False;
  end;

  //MsgBox('NeedsAddPath returns:'#13#13 + IntToStr(Integer(Result)), mbInformation, MB_OK);
end;





