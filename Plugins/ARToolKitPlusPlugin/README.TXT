The XCODE and VS projects here build the TrackerWrapper plugin for Unity desktop or iPhone.
The plugin depends on ARToolKitPlus_2.1.1 which should be placed in the directory above this one.
After building the bundle/dll/library, delete the existing one from the Unity project and drag the new one into it.
Your will need to "Reimport All" to replace the cached version of the bundle/dll with the new one.
When building the iPhone library, replace the libTrackerWrapper.a that is in the /Assets/Editor project directory.
