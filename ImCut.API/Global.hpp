#pragma once

#include <windows.h>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <algorithm>
#include <functional>
#include <array>
#include <fstream>
#include <set>
#include <Icm.h>
#include <filesystem>

#import <VGCoreAuto.tlb> \
    rename("GetClassName", "CorelGetClassName") \
    rename("FindWindow", "CorelFindWindow") \
    rename("CopyFile", "CorelCopyFile") \
    rename("GetCommandLine", "CorelGetCommandLine")


using namespace VGCore;