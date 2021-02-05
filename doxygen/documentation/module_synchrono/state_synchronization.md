State Synchronization with DDS and MPI {#state_synchronization}
=============================================

\tableofcontents

## Message Passing Interface (MPI) {#state_sync_MPI}

MPI is a standardized message-passing standard for parallel computing. Actively developed over more than 25 years, MPI is a robust standard with multiple implementations and widespread use, in particular in supercomputing clusters. In addition, the MPI runtime handles many aspects of program setup, like launching tasks on each computing resource and selecting the communication type, whether shared memory, InfiniBand, TCP or others. This widespread accessibility and ease of use in a cluster environment makes it the default choice for use in SynChrono. 

In MPI-terminology, each computing resource is called a _rank_, and in SynChrono each rank will typically handle a `ChSystem`. This `ChSystem` may have one or more vehicle agents, deformable terrain, or other actors, and the MPI rank is responsible for sharing the state data from its `ChSystem` with all other ranks and receiving state data from the other ranks in return.

### SynChrono Implementation {#state_sync_MPI_synchrono}

MPI-based synchronization in SynChrono happens each heartbeat with two MPI calls. First, all ranks communicate how long their message is with `MPI_Allgather`. While in many scenarios ranks will always send the same length of messages each heartbeat (this is the case for vehicles on rigid terrain), in others (such as with deformable terrain) there is no way to know how long messages will be so this information must be communicated to each rank. After the `MPI_Allgather`, each rank executes a variable length gather, `MPI_Allgatherv` to retrieve message data from all other ranks.
## Data Distribution Service (DDS) {#state_sync_DDS}

DDS is a more recent message-passing standard built around a Real-Time Publish-Subscribe (RTPS) pattern. Whereas communication in MPI happens rank-to-rank, communication in DDS happens via _topics_ that are published and subscribed to.

There are many DDS implementations, the most popular being published by [Real-Time Innovations (RTI)](https://www.rti.com/products). Keeping with the open source philosophy of Project Chrono, [FastDDS](https://github.com/eProsima/Fast-DDS) was selected as the vendor for the first release of this interface. 

### DDS Concepts {#state_sync_DDS_concepts}

As shown in the below figure, all DDS communication is handled by DDS `Participants` which hold `Publishers` and `Subscribers`. An abstract _databus_ is the conceptual wire for which data flows. `DataWriters` are responsible for sending this data and `DataReaders` will listen for data published on their `Topic`. `Publishers` and `Subscribers` manage one or many `DataWriters` and `DataReaders`, respectively. `Topics` is an abstraction for the meta data attached to each message that helps specify where data was sent and where it should go to. All DDS `Entities` have a configuration level system, called a Quality of Service or `QoS`. Different `QoS`'s are used to configure everything from a `Participant`'s initial peer list for discovery or whether a `DataReader` will store received messages in a history buffer.

Please see [here](https://community.rti.com/static/documentation/connext-dds/5.2.0/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_GettingStarted/Content/GettingStarted/An_Introduction_to_.htm) for more details about DDS.

<img src="http://www.projectchrono.org/assets/manual/synchrono_complicated_dds_system.png" alt="DDS System" width=70%>

### SynChrono Implementation {#state_sync_DDS_synchrono}

The SynChrono usage of DDS is meant to be as minimal as possible. A [SynChronoManager](@ref chrono::synchrono::SynChronoManager) has a handle to a single [SynCommunicator](@ref chrono::synchrono::SynCommunicator). Similar to MPI, a [SynDDSCommunicator](@ref chrono::synchrono::SynDDSCommunicator) must be created with the aforementioned classes to facilitate message passing. SynChrono level classes wrap the DDS concepts and provide accessors or setters to manipulate their configuration or create new entities. These include [SynDDSPublisher](@ref chrono::synchrono::SynDDSPublisher), [SynDDSSubscriber](@ref chrono::synchrono::SynDDSSubscriber), [SynDDSTopic](@ref chrono::synchrono::SynDDSTopic) and [more](group__synchrono__communication__dds.html).

## DDS-MPI Communication

_To be added_