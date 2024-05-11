import sys
import os
import shutil

# check modules

def pullSDK(system_config):  
    libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep  + "\\..\\BaseSDK"))
    print(libpath)
    curPath = os.getcwd()
    print(curPath)
     
    # Windows
    if system_config == 'x86' or system_config == 'X86':
        shutil.move('.\\Bin\\x86\\ScepterSDK_CSharp.dll', r'.\\Samples\\ScepterSDK_CSharp.dll')
        src = libpath + "\\Windows\\Bin\\x86"
        dst = curPath + "\\Bin\\x86"
        if os.path.exists(dst):
            shutil.rmtree(dst)
            
        shutil.copytree(src, dst)
        shutil.move('.\\Samples\\ScepterSDK_CSharp.dll', r'.\\Bin\\x86\\ScepterSDK_CSharp.dll')
        print("pull x86 SDK success")

    
    elif system_config == 'x64' or system_config == 'X64':
        shutil.move('.\\Bin\\x64\\ScepterSDK_CSharp.dll', r'.\\Samples\\ScepterSDK_CSharp.dll')
        src = libpath + "\\Windows\\Bin\\x64"
        dst = curPath + "\\Bin\\x64"
        if os.path.exists(dst):
            shutil.rmtree(dst)
            
        shutil.copytree(src, dst)
        shutil.move('.\\Samples\\ScepterSDK_CSharp.dll', r'.\\Bin\\x64\\ScepterSDK_CSharp.dll')
        print("pull x64 SDK success")
    else:
             
        print("the system is not supported ")
    
if __name__ == "__main__":    
    pullSDK(sys.argv[1])
