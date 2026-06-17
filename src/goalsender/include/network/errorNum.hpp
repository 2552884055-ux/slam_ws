#ifndef __ERROR_NUM__HPP
#define __ERROR_NUM__HPP

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define INFO "\033[1;32;40mINFO >> \033[0m"

#define DEBUG "\033[1;33;40mDEBUG<FILE: " __FILE__ ", LINE: " TOSTRING(__LINE__) "> : \033[0m"

#define WARNING "\033[1;33;40mWARNING<FILE: " __FILE__ ", LINE: " TOSTRING(__LINE__) "> : \033[0m"

#define ERROR "\033[1;31;40mERROR<FILE: " __FILE__ ", LINE: " TOSTRING(__LINE__) "> : \033[0m"

#define print(TAG, fmt, ...) \
            printf(TAG fmt , ##__VA_ARGS__)

#define printError(fmt, ...) \
            print(ERROR, fmt, ##__VA_ARGS__)

enum ERROR_NUM
{
    SUCCESS = 0,
    FAILED,
    SOCK_CREATE_FAIL,
    SCOKET_BIND_FAIL,
    THREAD_CALLBACK_FAIL,
    MALLOC_FAIL,
};

#endif
