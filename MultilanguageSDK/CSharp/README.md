
# CSharp Wrapper for ScepterSDK

## Overview
This CSharp package facilitates depth IR and Color data acquisition and processing for ScepterSDK.

### Requirements

- .NET Framework : 4.6.x
- Visual Studio 2017

## Installation

- **Install the CSharp package**

  - [Install ScepterSDK](https://github.com/gmiorg/ScepterSDK)
	  ```console
    git clone https://github.com/gmiorg/ScepterSDK
    ```
	
- **Build depend libraries envrionment for CSharp**

    <b>x64</b> and <b>x86</b> are supportted by project, copy the corresponding files to the 'Bin/x64' or 'Bin/x86' is necessary. Take <b>x64</b> as an example:
  - Method1: 
    
    Copy all files in <b>ScepterSDK/BaseSDK/Windows/Bin/x64</b> to <b>ScepterSDK/MultilanguageSDK/CSharp/Bin/x64</b> manually
  - Method2: 
  
    Run the <b>ScepterSDK/MultilanguageSDK/CSharp/install.py</b>
    ```console
    python install.py x64
    ```
	
## Details
- ScepterSDK_CSharp.dll is the CSharp dynamic library of ScepterSDK
