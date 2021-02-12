How to Add a New Agent to SynChrono
=============================================

SynChrono allows to distribute the simulation of multiple vehicles, but it is possible to extend its capabilities in order to extend it to simulate a different type _Agent_. In this example we show the addition of a copter agent.

## Setting up the FLatBuffer Compiler
#### Option 1: Build flatc
Make sure to have a C++ compiler and CMake installed. Then:
1. Go in **_chrono_root_** \src\chrono_thirdparty\flatbuffers 
2. Run the following commands in the shell making sure to use the right compiler:
~~~~~~~~~~~~~~~{.bat}
cmake -G "Visual Studio 15" -DCMAKE_BUILD_TYPE=Release  .
cmake --build . --config RELEASE 
~~~~~~~~~~~~~~~
#### Option 2:  download executable
On Windows, you can directly download the [pre-compiled binaries](https://github.com/google/flatbuffers/releases/)


## Generate the flatbuffer header
The _flatc_ flatbuffer compiler parses the .fbs files to create a header that takes care of serializing and de-serializing objects, provinding an higher level access to buffer communication.
### Editing the fbs file
Open the file **_chrono_root_** \src\src\chrono_synchrono\agent\Agent.fbs
1. Add a new agent: here we specifcy in _State_ the information that has to be passed at each update and in _Description_ the information needed for initialization.
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

    1. Go in the flatbuffer directory: **_chrono_root_**\ src\chrono_synchrono\flatbuffer\message

    2. Launch the flatc compiler: as follows:
```..\..\..\chrono_thirdparty\flatbuffers\RELEASE\flatc.exe -c  ..\fbs\SynFlatBuffers.fbs --no-includes --gen-all ```

Now, the file ```*_chrono_root_**\ src\chrono_synchrono\flatbuffer\message\SynFlatBuffers_generated.h ``` should be edited (and reflect the modifications in the .fb file)


## Changes to the C++ code


### Define a new message in SynCopterMessage.h/cpp 

Here we create a State and Description message (SynCopterDescriptionMessage and SynCopterStateMessage in this example).

Both children classes must define SynMessage's pure virtual functions (ConvertFromFlatBuffers and ConvertToFlatBuffers). 


###Create a new SynAgent derived class. 
The new SynCopterAgent inherits from SynAgent and has to ovverride its pure virtual member functions.
For further details look at ```chrono_synchrono/agent/SynCopterAgent.h/cpp```. Vehicles agents peovide also json file initializer that are not available for general new agents.

1. Constructor 
    1. Initialize state and description message member variables 
    2. Via overload or if statement add a default (no arguments) constructor for the new agent. It will be used to construct an agent from a description message.
2. InitializeZombie 
    This functions create fixed bodies whose position and orientation in space will be determined by the messages coming from other ranks.
3. SynchronizeZombie 
    This functions updates the position and orientation of zombie bodies according to the messages coming from other ranks.
4. Update 
    Update the state message of an agent. This state will be sent to other ranks (the other ranks see it a zombie). It is pointless to update the state of a zombie, thus check at the beginning of the function as in SynCopterAgent.cpp:
```
if (!m_copter)
        return;
```
5. GatherMessages 
6. GatherDescriptionMessages 



### Add new classes to the "Factories" dynamic casts
1. Add new message types to ```SynMessageFactory.cpp```:
    SynMessageFactory uses dynamic cast to infer the type of the message. For this reason, whenever we add a new message type, we have to add another condition to the states and descriptions elseif statements:
```
else if (agent_state->message_type() == SynFlatBuffers::Agent::Type_Copter_State) {
            message = chrono_types::make_shared<SynCopterStateMessage>(source_id, destination_id);
        }
```

1.Add a new agent to ```AgentFactory .cpp```
AgentFactory uses dynamics cast to create a zombie agent from a description coming from another rank. Since we added a new message and agent classes, we have to modify this function accordingly:
```
else if (auto copter_description = std::dynamic_pointer_cast<SynCopterDescriptionMessage>(description)) {
        auto copter_agent = chrono_types::make_shared<SynCopterAgent>();
        copter_agent->SetID(source_id);
        copter_agent->SetZombieVisualizationFiles(copter_description->chassis_vis_file,  //
                                                  copter_description->propeller_vis_file);  //

        copter_agent->SetNumProps(copter_description->GetNumProps());
		agent = copter_agent;
    }
```
Please make sure to completely define the agent description (in this example we assign the mesh files and the number of propellers) .

```
```
