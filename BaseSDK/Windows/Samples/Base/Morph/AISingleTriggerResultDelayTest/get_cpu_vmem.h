#include <iostream>
#include <thread>
#include <chrono>
#include <string.h>
#include <string>
#include <sstream>
#include <vector>
using namespace std;
#ifdef _WIN32
    #include <windows.h>
    #include <psapi.h>
    // #include <tlhelp32.h>
    #include <direct.h>
    #include <process.h>
#else
    #include <sys/stat.h>
    #include <sys/sysinfo.h>
    #include <sys/time.h>
    #include <unistd.h>
#endif

// get specific process cpu occupation ratio by pid
#ifdef _WIN32
//
// get current process pid

inline int GetCurrentPid()
{
    return _getpid();
}
static uint64_t convert_time_format(const FILETIME* ftime)
{
    LARGE_INTEGER li;

    li.LowPart  = ftime->dwLowDateTime;
    li.HighPart = ftime->dwHighDateTime;
    return li.QuadPart;
}
#else

// get current process pid

inline int GetCurrentPid()
{
    return getpid();
}
// FIXME: can also get cpu and mem status from popen cmd
// the info line num in /proc/{pid}/status file
    #define VMRSS_LINE   22
    #define PROCESS_ITEM 14

static const char* get_items(const char* buffer, unsigned int item)
{
    // read from buffer by offset
    const char* p = buffer;

    int len       = strlen(buffer);
    int count     = 0;

    for (int i = 0; i < len; i++)
    {
        if (' ' == *p)
        {
            count++;
            if (count == item - 1)
            {
                p++;
                break;
            }
        }
        p++;
    }

    return p;
}

static inline unsigned long get_cpu_total_occupy()
{
    // get total cpu use time

    // different mode cpu occupy time
    unsigned long user_time;
    unsigned long nice_time;
    unsigned long system_time;
    unsigned long idle_time;

    FILE* fd;
    char  buff[1024] = { 0 };

    fd               = fopen("/proc/stat", "r");
    if (nullptr == fd)
        return 0;

    fgets(buff, sizeof(buff), fd);
    char name[64] = { 0 };
    sscanf(buff, "%s %ld %ld %ld %ld", name, &user_time, &nice_time, &system_time, &idle_time);
    fclose(fd);

    return (user_time + nice_time + system_time + idle_time);
}

static inline unsigned long get_cpu_proc_occupy(int pid)
{
    // get specific pid cpu use time
    unsigned int  tmp_pid;
    unsigned long utime;    // user time
    unsigned long stime;    // kernel time
    unsigned long cutime;   // all user time
    unsigned long cstime;   // all dead time

    char  file_name[64] = { 0 };
    FILE* fd;
    char  line_buff[1024] = { 0 };
    sprintf(file_name, "/proc/%d/stat", pid);

    fd = fopen(file_name, "r");
    if (nullptr == fd)
        return 0;

    fgets(line_buff, sizeof(line_buff), fd);

    sscanf(line_buff, "%u", &tmp_pid);
    const char* q = get_items(line_buff, PROCESS_ITEM);
    sscanf(q, "%ld %ld %ld %ld", &utime, &stime, &cutime, &cstime);
    fclose(fd);

    return (utime + stime + cutime + cstime);
}
#endif

inline float GetCpuUsageRatio(int pid)
{
#ifdef _WIN32
    static int64_t last_time        = 0;
    static int64_t last_system_time = 0;

    FILETIME now;
    FILETIME creation_time;
    FILETIME exit_time;
    FILETIME kernel_time;
    FILETIME user_time;
    int64_t  system_time;
    int64_t  time;
    int64_t  system_time_delta;
    int64_t  time_delta;

    // get cpu num
    SYSTEM_INFO info;
    GetSystemInfo(&info);
    int cpu_num     = info.dwNumberOfProcessors;

    float cpu_ratio = 0.0;

    // get process hanlde by pid
    HANDLE process  = OpenProcess(PROCESS_ALL_ACCESS, FALSE, pid);
    // use GetCurrentProcess() can get current process and no need to close handle

    // get now time
    GetSystemTimeAsFileTime(&now);

    if (!GetProcessTimes(process, &creation_time, &exit_time, &kernel_time, &user_time))
    {
        // We don't assert here because in some cases (such as in the Task Manager)
        // we may call this function on a process that has just exited but we have
        // not yet received the notification.
        printf("GetCpuUsageRatio GetProcessTimes failed\n");
        return 0.0;
    }

    // should handle the multiple cpu num
    system_time = (convert_time_format(&kernel_time) + convert_time_format(&user_time)) / cpu_num;
    time        = convert_time_format(&now);

    if ((last_system_time == 0) || (last_time == 0))
    {
        // First call, just set the last values.
        last_system_time = system_time;
        last_time        = time;
        return 0.0;
    }

    system_time_delta = system_time - last_system_time;
    time_delta        = time - last_time;

    CloseHandle(process);

    if (time_delta == 0)
    {
        printf("GetCpuUsageRatio time_delta is 0, error\n");
        return 0.0;
    }

    // We add time_delta / 2 so the result is rounded.
    cpu_ratio         = (int)((system_time_delta * 100 + time_delta / 2) / time_delta);   // the % unit
    last_system_time  = system_time;
    last_time         = time;

    cpu_ratio        /= 100.0;   // convert to float number

    return cpu_ratio;
#else
    char buf[256] = { 0 };
    sprintf(buf, "top -bn1|grep %d", pid);
    std::string command = buf;
    FILE*       pipe    = popen(command.c_str(), "r");
    if (!pipe)
    {
        std::cout << "Failed to run the command." << std::endl;
        return 0;
    }

    char  buffer[256];
    int   npid;
    float pcpu = 0.0;
    char  name[256];
    stringstream ssm;
    vector<string> topInfo;
    string item;
    //1539 neal      20   0 3095976 321084 123932 S  0.0  5.0   0:38.12 gnome-shell
    while (fgets(buffer, sizeof(buffer), pipe))
    {
        ssm << buffer;
        // cout << ssm.str() <<endl;
        int index = 0;
        while(ssm >> item)
        {
            topInfo.push_back(item);
            index ++;
        }
        ssm.str("");
        ssm.clear();
        ssm << topInfo[0]; // pid
        ssm >> npid;
        ssm.str("");
        ssm.clear();
        ssm << topInfo[8]; // cpu
        ssm >> pcpu;
        // cout << "pcpu " << pcpu << endl;
        ssm.str("");
        ssm.clear();
        if (npid == pid)
        {
            break;
        }
    }
    pclose(pipe);
    return pcpu;
#endif
}

// get specific process physical memeory occupation size by pid (MB)
inline float GetMemoryUsage(int pid)
{
#ifdef _WIN32
    uint64_t                mem = 0, vmem = 0;
    PROCESS_MEMORY_COUNTERS pmc;

    // get process hanlde by pid
    HANDLE process = OpenProcess(PROCESS_ALL_ACCESS, FALSE, pid);
    if (GetProcessMemoryInfo(process, &pmc, sizeof(pmc)))
    {
        mem  = pmc.WorkingSetSize;
        vmem = pmc.PagefileUsage;
    }
    CloseHandle(process);

    // use GetCurrentProcess() can get current process and no need to close handle

    // convert mem from B to MB
    return vmem / 1024.0 / 1024.0;

#else
    char  file_name[64] = { 0 };
    FILE* fd;
    char  line_buff[512] = { 0 };
    sprintf(file_name, "/proc/%d/status", pid);

    fd = fopen(file_name, "r");
    if (nullptr == fd)
        return 0;

    char name[64];
    int  vmrss = 0;
    for (int i = 0; i < VMRSS_LINE - 1; i++)
        fgets(line_buff, sizeof(line_buff), fd);

    fgets(line_buff, sizeof(line_buff), fd);
    sscanf(line_buff, "%s %d", name, &vmrss);
    fclose(fd);

    // cnvert VmRSS from KB to MB
    return vmrss / 1024.0;
#endif
}
