# Lightweight Communication Interface (LCI)
Implementation of a cool communication layer.

## Authors

- \@danghvu (danghvu@gmail.com)
- \@omor1
- \@JiakunYan (jiakunyan1998@gmail.com)
- \@snirmarc

## Overview

The Lightweight Communication Interface (LCI) is designed to be an efficient communication library
for multithreaded, irregular communications. It is a research tool to explore design choices
for such libraries.  It has the following major features:
- **Multithreaded performance as the first priority**:  No big blocking locks like those in MPI!
  We carefully design the internal data structures to minimize interference between threads.
  We use atomic operations and fine-grained try locks extensively instead of coarse-grained blocking locks.
  The posting sends/receives, polling completions, and making progress on the progress engine
  (`LCI_progress`) use different locks or no locks at all! Threads would not interfere with each other
  unless necessary.

- **Versatile communication interface**: LCI provides users with various options including:
  - Communication primitives: two-sided send/recv, one-sided put/get.
  - Completion mechanisms: synchronizers (similar to MPI requests/futures), completion queues, function handlers.
  - Protocols: small/medium/long messages mapping to eager/rendezvous protocol.
  - Communication buffers: for both source/target buffers, we can use
    user-provided/runtime-allocated buffers.
  - Registration: for long messages, users can explicitly register the buffer or leave it to runtime
    (just use `LCI_SEGMENT_ALL`).
    
  The options are orthogonal and almost all combinations are valid!
  For example, the example code [putla_queue](examples/putla_queue.c) uses 
  one-sided put + user-provided source buffer + runtime-allocated target buffer + 
  rendezvous protocol + completion queue on source/targer side + explicit registration.

- **Explicit control of communication behaviors and resources**: versatile communication interface has already
  given users a lot of control. Besides, users can control various low-level features through
  API/environmental variables/cmake variables such as
  - Replication of communication devices.
  - The semantics of send/receive tag matching.
  - All communication primitives are non-blocking and users can decide when to retry in case of
    temporarily unavailable resources.
  - LCI also gives users an explicit function (`LCI_progress`) to make progress on the communication engine.
  - Different implementation and size of completion queues/matching tables.
  
  Users can tailor the LCI configuration to reduce software overheads, or just use default settings if
  LCI is not a performance bottleneck.

Currently, LCI is implemented as a mix of C and C++ libraries. Lightweight Communication Tools (LCT)
is a C++ library providing basic tools that can be used across libraries. Lightweight Communication
Interface (LCI) is a C library implementing communication-related features.

Currently, the functionalities in the LCT library include:
- timing.
- string searching and manipulation.
- query thread ID and number.
- logging.
- performance counters.
- different implementation of queues.
- PMI (Process Management Interface) wrappers.

The actual API and (some) documentation are located in [lct.h](lct/api/lct.h) and [lci.h](lci/api/lci.h).

## Installation
```
cmake .
make
make install
```

### Important CMake variables
- `CMAKE_INSTALL_PREFIX=/path/to/install`: Where to install LCI
  - This is the same across all the cmake projects.
- `LCI_DEBUG=ON/OFF`: Enable/disable the debug mode (more assertions and logs).
  The default value is `OFF`.
- `LCI_SERVER=ibv/ofi`: Hint to which network backend to use. If both `ibv` and `ofi` are found, LCI will use the one
  indicated by this variable. The default value is `ibv`. Typically, you don't need to
  modify this variable as if `libibverbs` presents, it is likely to be the recommended one to use.
  - `ibv`: libibverbs, typically for infiniband.
  - `ofi`: libfabrics, for all other networks (slingshot-11, ethernet, shared memory).
- `LCI_FORCE_SERVER=ON/OFF`: Default value is `OFF`. If it is set to `ON`, 
  `LCI_SERVER` will not be treated as a hint but a requirement.
- `LCI_WITH_LCT_ONLY=ON/OFF`: Whether to only build LCT (The Lightweight COmmunication Tools). 
  Default is `OFF` (build both LCT and LCI).

## Run LCI applications

We use the same mechanisms as MPI to launch LCI processes, so you can use the same way
you run MPI applications to run LCI applications. Typically, it would be `mpirun` or
`srun`. For example,
```
mpirun -n 2 ./hello_world
```
or
```
srun -n 2 ./hello_world
```

## Write an LCI program

See `examples` and `tests` for some example code.

See `lci/api/lci.h` for public APIs.

`doxygen` for a full [documentation](https://uiuc-hpc.github.io/LC/).

## LICENSE
See LICENSE file.
