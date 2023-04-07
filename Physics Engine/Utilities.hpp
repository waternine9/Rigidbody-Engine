#pragma once
#include <string>

bool startsWith(std::string x, std::string starts)
{
    if (x.size() < starts.size()) return false;
    for (int i = 0;i < starts.size();i++)
    {
        if (x[i] != starts[i]) return false;
    }
    return true;
}