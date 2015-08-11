# YAX.Math

The standalone version of YAX's (and by extension XNA's) math utility classes. Every class and method uses the same names and interfaces as their XNA equivalents, except where prohibited by the language difference.

### Includes:
* MathHelper (Misc. math functions)
* Matrix (Supports up to 4x4 row-major matrices)
* Quaternion
* Vector{2,3,4}

### Building:
YAX.Math uses premake to generate project files. If you don't already have it, download premake5 from [here](http://premake.github.io/download.html), put it somewhere accessible from the command line, and run like so: <br>
```bash
cd /path/to/YAX.Math

#for Visual Studio 2013
premake5 vs2013

#for Visual Studio 2015
premake5 vs2015

#for GNU make
premake5 gmake

#xcode is not yet supported by premake, but it is on the way!
```

For other platforms that premake supports, the list is [here.](https://github.com/premake/premake-core/wiki/Using-Premake#using-premake-to-generate-project-files)

### Usage:
Place the header files in your include path (they must be in the same folder) and the .lib files in your library path, and then in whatever file you wish to use it in:
```C++ 
#include "YAX.Math.h" 
```

All functions and classes are contained within the YAX namespace. You can either use the entire namespace, or pick and choose the members:
```C++
using namespace YAX;
//or
using YAX::Matrix;
using YAX::Vector4;
//etc

//MathHelper is also a namespace, so you can do things like
using namespace YAX::MathHelper;
//to bring in all the functions, or
using YAX::MathHelper::Clamp;
using YAX::MathHelper::Hermite;
//To bring in individual members
```

### Documentation: 
Is nearly nonexistant. Until I or another kind soul has the time to write it, use [XNA's MSDN documentation](https://msdn.microsoft.com/en-us/library/microsoft.xna.framework.aspx).

### Found a bug?
If you find anything that doesn't quite match XNA's behavior (from a flipped sign to a completely wrong result), please do open up an issue or submit a pull request!
