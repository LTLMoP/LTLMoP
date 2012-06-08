Steps:
1. Open ViconMarkerBroadcaster.csproj in Visual Studio.
2. Save solution.
3. Build solution as Release.
4. Put these 4 files wherever you want, but they must be in the same directory:
   (first three files are in /bin/Release/, while the other doesn't get transferred from the main folder)
    ViconMarkerBroadcaster.exe
    ViconMarkerBroadcaster.exe.config
    ViconDataStreamSDK_DotNET.dll
    ViconDataStreamSDK_CPP.dll
5. Make sure Vicon is on.
6. Run executable (recommend not running through Visual Studio to speed up the program).

Alternative:
4. Move ViconDataStreamSDK_CPP.dll to /bin/Release/
5. Make a shortcut to ViconMarkerBroadcaster.exe and save it to a more accessible location.
6. Make sure Vicon is on.
7. Run executable through shortcut.