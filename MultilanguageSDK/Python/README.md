## Python Wrapper for ScepterSDK API

Python wrapper is an opensource project of Scepter Scepter API.

The goal of this project is to help developers use Scepter ToF camera via python method easily.

PythonSDK version: V1.0.8

### Supported Devices

- NYX Camera
  - NYX650 (NYX650_R_20240325_B02 and later )
- Vzense Camera
  - DS86 (DS86_R_20230321 and later )
  - DS77C (DS77_S_20220530_B16 and later )
  - DS77 (DS77_S_20220530_B16 and later) 

### Requirements

- python version : 3.7.x
- python modules : ctypes, numpy, opencv-python(display only)

### Directory

- **NYX650**: the API and Sample code for NYX650 & NYX660
- **DS77**: the API and Sample code for DS77Lite/DS77Pro
- **DS77C**: the API and Sample code for DS77CLite/DS77CPro
- **DS86**: the API and Sample code for DS86 & DS87

### Quick Start

1. install modules:
```	 
pip install numpy
pip install opencv-python 
```
2. switch to Samples under the product directory, run the sample that you need. 
For example, go to the NYX650/Samples/FrameViewer, then run 'python FrameViewer_NYX650.py'

3. When using multiple network cards, set different IP network segments