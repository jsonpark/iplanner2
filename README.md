# iplanner2
new version of iplanner

## Libraries

* [glad](https://glad.dav1d.de/) - OpenGL
```
lib\glad\glad.c(h), lib\KHR\khrplatform.h
```

* [glfw](https://www.glfw.org/) - OpenGL window management
  * In Property Pages
    * C/C++ -> General, Additional Include Directories, put your *glfwroot*
  * Build glfw3.lib and locate at *lib*

* [stb_image](https://github.com/nothings/stb/blob/master/stb_image.h) - png, jpg loader
```
lib\stb\stb_image.h
```

* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) - Math library
  * In Property Pages
    * C/C++ -> General, Additional Include Directories, put your *eigenroot*

* Mablab 2019a
  * In Property Pages, replace *matlibroot* with the actual directory
    * Debugging, Environment, put your *PATH=matlibroot\bin\win64;%PATH%*
    * C/C++ -> General, Additional Include Directories, put your *matlibroot\extern\include*
    * Linker -> General, Additional Library Directories, put your *matlibroot\extern\lib\win64\microsoft*

* [Assimp](http://www.assimp.org/) - Mesh file loader
  * In Property Pages
    * C/C++ -> General, Additional Include Directories, put your *assimproot*
  * Build assimp-vc142-mt.lib and locate at *lib*
  * Build assimp-vc142-mt.dll and locate at *bin*
  * In case *assimp/config.h* can't be found, find it where assimp was built and copy-paste to *assimproot/include*

  
## Vision Dataset

* [UTKinect](http://cvrc.ece.utexas.edu/KinectDatasets/HOJ3D.html) dataset
```
some\where\utkinect
```

* [Watch-n-Patch](http://watchnpatch.cs.cornell.edu/) dataset
```
some\where\watch-n-patch
```

* [Occlusion MoCap]() dataset
```
some\where\occlusion
```

## Robot Data

* [Fetch robot description](https://github.com/fetchrobotics/fetch_ros/tree/indigo-devel)
  * Clone it in any place outside the project directory
  * Put the *fetch_ros* path to where *RobotModelLoader* class initializes

## Sample Image

## Training dataset

1. Converting `.mat` files to raw data

   Set the directory and conversion options in `main.cc` of `mat_convert`.

   Build and run `mat_convert`.

2. Running the motion planner

   Set directories in `engine.cc` and other codes of `iplanner`.

   (Hard-coded directories should be replaced with strings from configuration files.)