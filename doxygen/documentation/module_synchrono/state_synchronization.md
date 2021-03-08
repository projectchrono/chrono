State Synchronization with MPI {#state_synchronization}
=============================================

\tableofcontents

## Message Passing Interface (MPI) {#state_sync_MPI}

MPI is a standardized message-passing standard for parallel computing. Actively developed over more than 25 years, MPI is a robust standard with multiple implementations and widespread use, in particular in supercomputing clusters. In addition, the MPI runtime handles many aspects of program setup, like launching tasks on each computing resource and selecting the communication type, whether shared memory, InfiniBand, TCP or others. This widespread accessibility and ease of use in a cluster environment makes it the default choice for use in SynChrono. 

In MPI-terminology, each computing resource is called a _rank_, and in SynChrono each rank will typically handle a `ChSystem`. This `ChSystem` may have one or more vehicle agents, deformable terrain, or other actors, and the MPI rank is responsible for sharing the state data from its `ChSystem` with all other ranks and receiving state data from the other ranks in return.

### SynChrono Implementation {#state_sync_MPI_synchrono}

MPI-based synchronization in SynChrono happens each heartbeat with two MPI calls. First, all ranks communicate how long their message is with `MPI_Allgather`. While in many scenarios ranks will always send the same length of messages each heartbeat (this is the case for vehicles on rigid terrain), in others (such as with deformable terrain) there is no way to know how long messages will be so this information must be communicated to each rank. After the `MPI_Allgather`, each rank executes a variable length gather, `MPI_Allgatherv` to retrieve message data from all other ranks.
