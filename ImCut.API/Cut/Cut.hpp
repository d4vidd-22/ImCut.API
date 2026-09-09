#pragma once

#include "..\Global.hpp"


#define IMCUT_CUT_API_VERSION 2

namespace ImCut::Cut
{
    enum class ClosureMode
    {
        Auto = 0,
        WholeSelection,
        EachSelectedGroup,
        EachSelectedObject
    };

    struct Settings
    {
        int pageWidth = 1300;
        int regmarks = 6;
        double registrationMarginMillimeters = 5.0;
        ClosureMode closureMode = ClosureMode::Auto;
        bool namePages = true;
        bool showSummary = true;
        bool forceBottomRightAnchor = true;
    };

    struct Result
    {
        int closureCount = 0;
        int processedCount = 0;
        int regmarksCreated = 0;
        int warnings = 0;
        std::string error;
        double elapsedMs = 0.0, geometryMs = 0.0, planningMs = 0.0, documentMs = 0.0;
    };

    [[nodiscard]] Result Process(
        IVGApplicationPtr& spApp,
        const Settings& settings);

    void StartCutMarks(
        IVGApplicationPtr& spApp,
        int width,
        int regmarks);
}
