State Synchronization with DDS and MPI {#state_synchronization}
=============================================

- [DDS Communication](#Data-Distribution-Service-(DDS))
- [MPI Communication](#Message-Passing-Interface-(MPI))
- [DDS-MPI Communication](#DDS-MPI-Communication)

### Data Distribution Service (DDS)

The Data Distributed Service (DDS) is a middleware standard published by the Object Management Group (OMG) for data-centric, real-time, machine-to-machine communication. Defined in that standard is an interoperable, high-performance message passing interface called Real-Time Publish-Subscribe (RTPS). This is the main concept used by SynChrono to provide state synchronization between nodes.

The DDS standard defines a strict API that implementors must abide by to be considered an official "DDS Vendor" by OMG. At the time of writing, there are nine official and six unofficial DDS Vendors. The most popular is by [Real-Time Innovations (RTI)](https://www.rti.com/products). Keeping with the open source philosophy of ProjectChrono, [FastDDS](https://github.com/eProsima/Fast-DDS) was selected as the vendor for the first release of this interface. This is not necessarily a limitation, as the standard requires implementations to be interoperable. This means if FastDDS is used to send data, RTI's Connext implementation could receive and parse that same data.

#### DDS Concepts

DDS and RTPS are a powerful concepts that can be used to create complex systems. As seen in the below figure, all DDS communication is handled by DDS `Participants` which hold `Publishers` and `Subscribers`. An abstract _databus_ is the conceptual wire for which data flows. `DataWriters` are responsible for sending this data and `DataReaders` will listen for data published on their `Topic`. `Publishers` and `Subscribers` manage one or many `DataWriters` and `DataReaders`, respectively. `Topics` is an abstraction for the meta data attached to each message that helps specify where data was sent and where it should go to. All DDS `Entities` have a configuration level system, called a Quality of Service or `QoS`. Different `QoS`'s are used to configure everything from a `Participant`'s initial peer list for discovery or whether a `DataReader` will store received messages in a history buffer.

Please see [here](https://community.rti.com/static/documentation/connext-dds/5.2.0/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_GettingStarted/Content/GettingStarted/An_Introduction_to_.htm) for more details about DDS.

<img src="http://www.projectchrono.org/assets/manual/synchrono_complicated_dds_system.png" alt="DDS System" width=70%>

#### SynChrono Implementation

The SynChrono usage of DDS is meant to be as minimal as possible. A [SynChronoManager](@ref chrono::synchrono::SynChronoManager) has a handle to a single [SynCommunicator](@ref chrono::synchrono::SynCommunicator). Similar to MPI, a [SynDDSCommunicator](@ref chrono::synchrono::SynDDSCommunicator) must be created with the aforementioned classes to facilitate message passing. SynChrono level classes wrap the DDS concepts and provide accessors or setters to manipulate their configuration or create new entities. These include [SynDDSPublisher](@ref chrono::synchrono::SynDDSPublisher), [SynDDSSubscriber](@ref chrono::synchrono::SynDDSSubscriber), [SynDDSTopic](@ref chrono::synchrono::SynDDSTopic) and [more](group__synchrono__communication__dds.html).

### Message Passing Interface (MPI)

_Overview of MPI and key classes go here, Jay to work on_

### DDS-MPI Communication