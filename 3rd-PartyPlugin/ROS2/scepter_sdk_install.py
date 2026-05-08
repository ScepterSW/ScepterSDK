import os
import shutil
import subprocess

distro_name = ""
arch_info = ""
def get_linux_distro():
    global distro_name
    global arch_info
    try:
        with open('/etc/os-release', 'r') as f:
            info = {}
            for line in f:
                if '=' in line:
                    key, value = line.strip().split('=', 1)
                    info[key] = value.strip('"')
            
            name = info.get('NAME', 'Unknown')
            version = info.get('VERSION_ID', 'Unknown')
            distro_name = "{}{}".format(name, version)

        arch = subprocess.check_output(['uname', '-m']).decode().strip()
        if arch in ('aarch64', 'armv7l', 'armv8l'):
            arch_info = "ARM"
        elif arch == 'x86_64':
            arch_info = "x86_64"
        else:
            arch_info = "Unknown"
        
    except FileNotFoundError:
        return "Unknown"

def copyFile(src, dst):
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.copytree(src, dst, symlinks=True)

def pullSDK():
    global distro_name
    global arch_info
    libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep + "../BaseSDK/"))
    #print(libpath)
    curPath = os.getcwd()
    #print(curPath)
    folders = []
    with os.scandir(curPath + "/src") as entries:
        for entry in entries:
            if entry.is_dir():
                folders.append(entry.name)
    #print(folders)
    sdkPrefix = ""
    if arch_info == "ARM":
        if distro_name == 'Ubuntu24.04' or distro_name == 'Ubuntu22.04' or distro_name == 'Ubuntu20.04' or distro_name == 'Ubuntu18.04' or distro_name == 'Ubuntu16.04':
            sdkPrefix = "AArch64"
        else:
            print("the system is not supported")
            return
    elif arch_info == "x86_64":
        if distro_name == 'Ubuntu16.04':
            sdkPrefix = "Ubuntu16.04"
        elif distro_name == 'Ubuntu24.04' or distro_name == 'Ubuntu22.04' or distro_name == 'Ubuntu20.04' or distro_name == 'Ubuntu18.04':
            sdkPrefix = "Ubuntu"
        else:
            print("the system is not supported")
            return

    if "scepter_manager" in folders:
        # copy for scepter_manager
        srcInc = libpath + "/{}/Include".format(sdkPrefix)
        dstInc = curPath + "/src/scepter_manager/dependencies/Include"
        copyFile(srcInc, dstInc)
        srcLib = libpath + "/{}/Lib".format(sdkPrefix)
        dstLib = curPath + "/src/scepter_manager/dependencies/Lib"
        copyFile(srcLib, dstLib)
        # link for other dir
        for item in folders:
            if item != "scepter_manager":
                src_link = "../scepter_manager/dependencies"
                dst_link = "./src/{}/dependencies".format(item)
                if os.path.exists(dst_link):
                    os.unlink(dst_link)
                os.symlink(src_link, dst_link, target_is_directory= True) # dst_link -- > src_link
            print("Dependencies of package <{}> has installed successfully on Platform {}-{}".format(str(item), str(arch_info), str(distro_name)))
    else:
        print("not found scepter_manager package")

if __name__ == "__main__":
    get_linux_distro()
    pullSDK()
