#                               Vzense GenTL Producer Package Usage



## 1.Download and Install Halcon

Download the Halcon (**Windows version**) installation package from the Halcon Official Website and install it according to the official documentation.



**Important Note**:

The  **Image Acquisition Interfaces** package  must be installed. It is recommended to install all installation packages.



## 2.DownLoad the Scepter SDK Package

Download the official Scepter SDK package from the offical website.

The **sceptergentl.cti** file could be found in the **SDK/3rd-PartyPlugin/GenICam/Vzense GenTL Producer Package** directory.

And some gentl producer required files would be found in the **SDK/BaseSDK/Windows/Bin/x64** directory.

Those required files include:

**Drivers folder**

**Config folder**

**Scepter_api.dll**

**DSImgPreProcess.dll**



## 3.Configure the Environment Variable

Navigate to the environment variables settings and create a new environment variable named **GENICAM_GENTL64_PATH**.

Edit the **GENICAM_GENTL64_PATH** as you needed.

Then, copy the **sceptergentl.cti** file to the location specified by **GENICAM_GENTL64_PATH**.

After restarting your computer, the configuration will take effect.



## 4.Copy necessary files

Copy those required files of the **Secpeter SDK** to the **GENICAM_GENTL64_PATH** directory.

Now all necessary files should be placed in the **GENICAM_GENTL64_PATH** directory.

Those necessary files include:

**Drivers folder** 

**Config folder**

**Scepter_api.dll**

**DSImgPreProcess.dll**

**sceptergentl.cti**



## 5.Run Samples

1.Double-click on any hdev named file in the "**SDK/3rd-PartyPlugin/GenICam/Halcon_Samples**" folder

2.Click on the “Run” button to execute the hdev sample program.

3.View the running results.



**Important Note:**

Since some device models do not have a color sensor, test cases that require capturing color images or setting color sensor parameters cannot be executed on such devices. And also some device models do not support WDR, test cases related to WDR cannot be executed either.