# Building

1. Clone [openvr](https://github.com/ValveSoftware/openvr)
2. build openvr with `cd openvr; mkdir build; cd build; cmake ..; cmake --build .`
3. Clone this repo next to openvr (same root folder)
4. build this repo with cmake `cd robotVRTracker; mkdir build; cd build; cmake ..; cmake --build .`
5. copy `openvr_api.dll` next to `milouTracker.exe`
6. run `milouTracker.exe` 
