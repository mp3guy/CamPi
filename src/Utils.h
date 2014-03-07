#include <stdio.h>
#include <string.h>
#include <sys/time.h>

int find_argument(int argc, const char** argv, const char* argument_name)
{
    int i;
    for(i = 1; i < argc; ++i)
    {
        // Search for the string
        if(strcmp(argv[i], argument_name) == 0)
        {
            return i;
        }
    }
    return -1;
}

int parse_argument(int argc, const char** argv, const char* str, int * val)
{
    int index = find_argument(argc, argv, str) + 1;

    if(index > 0 && index < argc)
    {
        *val = atoi(argv[index]);
    }

    return index - 1;
}
