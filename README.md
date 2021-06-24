# libdes
A framework for parallel discrete event simulation

## What is it?
libdes is a multi-threaded C++ framework for parallel discrete event simulation (DES). Unlike many other DES solutions, libdes's highest priority is programmer productivity. Considerable effort has been placed on the API to support parallel simulations while being easy to use.

libdes supports parallel execution of events through a carefully mapping of simulation components to execution threads. Events always have "run to completion" guarantees to make it easy to use. On a standard dual socket server libdes achieves over 200 million events per second and on a SuperDomeX system is has acheived over 2 billion events per second.

## Development
This is library is currently under development and will be changing dramatically so don't rely on a stable API. Version v1.0.0 will mark the first stable release.
