How to Add a New Agent to SynChrono
=============================================

SynChrono allows you to distribute the simulation of multiple vehicles, and it is possible to extend its capabilities to simulate different _Agents_. In this example we show the addition of a copter agent.

## Setting up the FlatBuffer Compiler

The message format of all SynChrono messages is controlled by FlatBuffers schemas. When changing the message format, the schemas must be re-compiled. This re-compilation causes FlatBuffers to re-generate the C++ wrapper code that allows us to unpack FlatBuffer messages.

So, while just the FlatBuffers header file is required when running SynChrono code, a built FlatBuffers compiler is needed when editing the message schemas (and adding a new message scheme is required for new agents).

#### Option 1: Build from source

As FlatBuffers is included as a sub-module of Chrono, one easy option to get the flatc compiler is to build it from source. 
1. Ensure you have a C++ compiler and CMake installed. 
2. Go in **_chrono_root_** \\src\\chrono_thirdparty\\flatbuffers 
3. Run the CMake and make commands outlined in the [FlatBuffers guide](https://google.github.io/flatbuffers/flatbuffers_guide_building.html). For example on Windows:
~~~~~~~~~~~~~~~{.bat}
cmake -G "Visual Studio 15" -DCMAKE_BUILD_TYPE=Release  .
cmake --build . --config RELEASE 
~~~~~~~~~~~~~~~
#### Option 2:  Download pre-built version
On Windows, you can directly download the [pre-compiled binaries](https://github.com/google/flatbuffers/releases/). Many Linux distributions (e.g. Arch Linux, Ubuntu) have pre-built versions available through their package managers as well.


## Generate the flatbuffer header
The _flatc_ flatbuffer compiler parses the .fbs files to create a header that takes care of serializing and de-serializing objects, providing higher level access to buffer communication.
### Editing the fbs file
Open the file **chrono_root**\\src\\src\\chrono_synchrono\\agent\\Agent.fbs
1. Add a new agent: here we specify in the _State_ table the information that has to be passed at each update and in the _Description_ table the information needed for initialization.
~~~~~~~~~~~~~~~{.fbs}
// Derived "class" of Agent
// Creates a copter agent message
namespace SynFlatBuffers.Agent.Copter;
table State {
  time:double;
  chassis:Pose;
  propellers:[Pose];
}
table Description {
  chassis_vis_file:string;
  propeller_vis_file:string;
  num_props:int;
}
root_type State;
~~~~~~~~~~~~~~~
2. Add messages for the new agent (the last 2 lines of the following snippet): 
~~~~~~~~~~~~~~~{.fbs}
union Type { 
  WheeledVehicle.State, 
  WheeledVehicle.Description, 
  TrackedVehicle.State, 
  TrackedVehicle.Description, 
  Environment.State,
  Environment.Description,
  Copter.State,             <--
  Copter.DescriptioN        <--
}
~~~~~~~~~~~~~~~

3. Compile the fbs: 

    1. Go in the flatbuffer directory: **_chrono_root_**\\ src\\chrono_synchrono\\flatbuffer\\message

    2. Launch the flatc compiler: as follows:
```..\..\..\chrono_thirdparty\flatbuffers\RELEASE\flatc.exe -c  ..\fbs\SynFlatBuffers.fbs --no-includes --gen-all ```

Now, the file ```*_chrono_root_**\ src\chrono_synchrono\flatbuffer\message\SynFlatBuffers_generated.h ``` should have changed (to reflect the modifications in the .fbs file)


## Changes to the C++ code


### Define a new message in SynCopterMessage.h/cpp 

Here we create a State and Description message (SynCopterDescriptionMessage and SynCopterStateMessage in this example).

Both child classes must implement SynMessage's pure virtual functions (ConvertFromFlatBuffers and ConvertToFlatBuffers). 


###Create a new SynAgent derived class. 
The new SynCopterAgent inherits from SynAgent and must override its pure virtual member functions.
For further details look at ```chrono_synchrono/agent/SynCopterAgent.h/cpp```. Vehicles agents also provide for json file initialization which is not available for general new agents.

1. Constructor 
    1. Initialize state and description message member variables 
    2. Via overload or if statement add a default (no arguments) constructor for the new agent. It will be used to construct an agent from a description message.
2. InitializeZombie 
    This function creates fixed bodies whose position and orientation in space will be determined by the messages coming from other ranks.
3. SynchronizeZombie 
    This function updates the position and orientation of zombie bodies according to the messages coming from other ranks.
4. Update 
    Update the state message of an agent. This state will be sent to other ranks (the other ranks see it as a zombie). It is pointless to update the state of a zombie, thus check at the beginning of the function as in SynCopterAgent.cpp:
```
if (!m_copter)
        return;
```
5. GatherMessages 
6. GatherDescriptionMessages 



### Add new classes to the "Factories" dynamic casts
1. Add new message types to ```SynMessageFactory.cpp```:
    SynMessageFactory uses dynamic cast to infer the type of the message. For this reason, whenever we add a new message type, we have to add another condition to the states and descriptions elseif statements:
```cpp
else if (agent_state->message_type() == SynFlatBuffers::Agent::Type_Copter_State) {
            message = chrono_types::make_shared<SynCopterStateMessage>(source_key, destination_key);
        }
```

2. Add a new agent to ```AgentFactory.cpp```
AgentFactory uses dynamics casts to create a zombie agent from a description coming from another rank. Since we added new message and agent classes, we have to modify this function accordingly:
```cpp
else if (auto copter_description = std::dynamic_pointer_cast<SynCopterDescriptionMessage>(description)) {
        auto copter_agent = chrono_types::make_shared<SynCopterAgent>();
        copter_agent->SetKey(source_key);
        copter_agent->SetZombieVisualizationFiles(copter_description->chassis_vis_file,  //
                                                  copter_description->propeller_vis_file);  //

        copter_agent->SetNumProps(copter_description->GetNumProps());
		agent = copter_agent;
    }
```
Please make sure to completely define the agent description (in this example we assign the mesh files and the number of propellers) .
