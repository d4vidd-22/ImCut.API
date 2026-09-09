#pragma once
#ifndef NOMINMAX
#define NOMINMAX 1
#endif
#include <Windows.h>
#include "ScopeExit.hpp"
#include <climits>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace ImCut
{
    class AtomicIni final
    {
    public:
        explicit AtomicIni(const std::filesystem::path& destination, bool preserve = true)
            : destination_(destination)
        {
            auto dir = std::filesystem::absolute(destination).parent_path();
            wchar_t temporary[MAX_PATH]{};
            if (!GetTempFileNameW(dir.c_str(), L"icu", 0, temporary)) return;
            temporary_ = temporary;
            ScopeExit cleanup([this] { DeleteFileW(temporary_.c_str()); });
            std::wstring contents(1, L'\xFEFF');
            std::error_code ec;
            const bool exists = std::filesystem::exists(destination, ec);
            if (ec) return;
            if (preserve && exists)
            {
                std::ifstream input(destination, std::ios::binary);
                if (!input) return;
                const std::string bytes((std::istreambuf_iterator<char>(input)), {});
                if (input.bad()) return;
                if (bytes.size() >= 2 && static_cast<unsigned char>(bytes[0]) == 0xFF &&
                    static_cast<unsigned char>(bytes[1]) == 0xFE)
                {
                    if (bytes.size() % 2) return;
                    contents.resize(bytes.size() / 2);
                    memcpy(contents.data(), bytes.data(), bytes.size());
                }
                else if (!bytes.empty())
                {
                    const bool utf8 = bytes.size() >= 3 && bytes.substr(0, 3) == "\xEF\xBB\xBF";
                    const char* data = bytes.data() + (utf8 ? 3 : 0);
                    const auto size = bytes.size() - (utf8 ? 3 : 0);
                    if (size > INT_MAX) return;
                    if (size)
                    {
                        const int n = MultiByteToWideChar(utf8 ? CP_UTF8 : CP_ACP,
                            MB_ERR_INVALID_CHARS, data, static_cast<int>(size), nullptr, 0);
                        if (!n) return;
                        contents.resize(1 + n);
                        if (!MultiByteToWideChar(utf8 ? CP_UTF8 : CP_ACP, MB_ERR_INVALID_CHARS,
                            data, static_cast<int>(size), contents.data() + 1, n)) return;
                    }
                }
            }
            std::ofstream output(temporary_, std::ios::binary | std::ios::trunc);
            output.write(reinterpret_cast<const char*>(contents.data()), contents.size() * sizeof(wchar_t));
            output.close();
            ready_ = !output.fail();
            if (ready_) cleanup.Release();
        }
        ~AtomicIni() noexcept
        {
            if (!temporary_.empty()) DeleteFileW(temporary_.c_str());
        }
        AtomicIni(const AtomicIni&) = delete;
        AtomicIni& operator=(const AtomicIni&) = delete;
        bool Ready() const noexcept { return ready_; }
        const std::filesystem::path& Path() const noexcept { return temporary_; }
        bool Commit() noexcept
        {
            if (!ready_) return false;
            WritePrivateProfileStringW(nullptr, nullptr, nullptr, temporary_.c_str());
            HANDLE file = CreateFileW(temporary_.c_str(), GENERIC_WRITE, FILE_SHARE_READ,
                nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
            if (file == INVALID_HANDLE_VALUE) return false;
            const BOOL flushed = FlushFileBuffers(file);
            CloseHandle(file);
            if (!flushed || !MoveFileExW(temporary_.c_str(), destination_.c_str(),
                MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) return false;
            WritePrivateProfileStringW(nullptr, nullptr, nullptr, destination_.c_str());
            temporary_.clear();
            ready_ = false;
            return true;
        }
    private:
        std::filesystem::path destination_, temporary_;
        bool ready_ = false;
    };
}
