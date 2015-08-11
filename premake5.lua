solution "YAX.Math"
    configurations { "Debug32", "Debug64", "Release32", "Release64" }
    
project "YAX.Math"
    kind "StaticLib"
    language "C++"

    targetdir "out/%{cfg.buildcfg}/%{cfg.platform}"
    includedirs "include/"
    files "include/*.h"
    files "src/*.cpp"
    
    flags "MultiProcessorCompile"
    warnings "Extra"

    filter "configurations:*32"
        architecture "x86"
    
    filter "configurations:*64"
        architecture "x86_64"
        
    filter "configurations:Debug*"
        optimize "Off"
         
    filter "configurations:Release*"
        floatingpoint "Strict"
        optimize "Full"
        
    filter "system:windows"
        postbuildcommands {"xcopy include out\\include /I /E"}
        
    filter "system:not windows"
        postbuildcommands {"cp -r ./include ./out/"}