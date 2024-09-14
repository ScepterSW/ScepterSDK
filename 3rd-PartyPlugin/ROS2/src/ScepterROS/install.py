import sys
import subprocess
import os
import re
import shutil

# check modules
import numpy
import cv2
import ctypes
 
def pullSDK(system_config):  
    libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep + "../../../BaseSDK/"))
    print(libpath)
    curPath = os.getcwd()
    print(curPath)
     
    # Windows
    if system_config == 'Ubuntu16.04':
        src = libpath + "/%s/Include" %('Ubuntu16.04')
        dst = curPath + "/dependencies/Include"
        if os.path.exists(dst):
            shutil.rmtree(dst)
        shutil.copytree(src, dst)
        
        src = libpath + "/%s/Lib" %('Ubuntu16.04')
        dst = curPath + "/dependencies/Lib"
        if os.path.exists(dst):
            shutil.rmtree(dst)

        shutil.copytree(src, dst)
        print("pull SDK success")

    elif system_config == 'Ubuntu24.04' or system_config == 'Ubuntu22.04' or system_config == 'Ubuntu20.04' or system_config == 'Ubuntu18.04':
        src = libpath + "/%s/Include" %('Ubuntu')
        dst = curPath + "/dependencies/Include"
        if os.path.exists(dst):
            shutil.rmtree(dst)
        shutil.copytree(src, dst) 

        src = libpath + "/%s/Lib" %('Ubuntu')
        dst = curPath + "/dependencies/Lib"
        if os.path.exists(dst):
            shutil.rmtree(dst)
        shutil.copytree(src, dst)     
        print("pull SDK success")
		
    elif system_config == 'AArch64':
        src = libpath + "/%s/Include" %('AArch64')
        dst = curPath + "/dependencies/Include"
        if os.path.exists(dst):
            shutil.rmtree(dst)
        shutil.copytree(src, dst) 

        src = libpath + "/%s/Lib" %('AArch64')
        dst = curPath + "/dependencies/Lib"
        if os.path.exists(dst):
            shutil.rmtree(dst)
        shutil.copytree(src, dst)     
        print("pull SDK success")
    else:
             
        print("the system is not supported ")
    
if __name__ == "__main__":    
    pullSDK(sys.argv[1])
