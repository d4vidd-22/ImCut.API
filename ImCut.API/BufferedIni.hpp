#pragma once
#include <Windows.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace ImCut
{
    
    
    class BufferedIni final
    {
        struct Less
        {
            bool operator()(const std::wstring& a, const std::wstring& b) const
            {
                return CompareStringOrdinal(a.c_str(), -1, b.c_str(), -1, TRUE) == CSTR_LESS_THAN;
            }
        };
        using Values = std::map<std::wstring, std::wstring, Less>;
        std::map<std::wstring, Values, Less> edits_;
        std::filesystem::path path_;

        static std::wstring Trim(const std::wstring& text)
        {
            const auto first = text.find_first_not_of(L" \t\r");
            return first == std::wstring::npos ? L"" :
                text.substr(first, text.find_last_not_of(L" \t\r") - first + 1);
        }
    public:
        explicit BufferedIni(const std::filesystem::path& path) : path_(path) {}
        bool Set(const wchar_t* section, const wchar_t* key, const std::wstring& value)
        {
            if (value.find_first_of(L"\r\n") != std::wstring::npos ||
                value.find(L'\0') != std::wstring::npos) return false;
            edits_[section][key] = value;
            return true;
        }
        bool Flush()
        {
            std::ifstream input(path_, std::ios::binary | std::ios::ate);
            if (!input) return false;
            const auto size = input.tellg();
            if (size < 2 || size % sizeof(wchar_t)) return false;
            std::wstring contents(static_cast<std::size_t>(size) / sizeof(wchar_t), L'\0');
            input.seekg(0);
            input.read(reinterpret_cast<char*>(contents.data()), size);
            if (!input || contents.front() != L'\xFEFF') return false;
            input.close();

            std::wistringstream lines(contents.substr(1));
            std::wstring output(1, L'\xFEFF'), line, section;
            Values written;
            const auto append = [&](const std::wstring& text) { output += text + L"\r\n"; };
            const auto flushSection = [&]
            {
                const auto found = edits_.find(section);
                if (found == edits_.end()) return;
                for (const auto& [key, value] : found->second)
                    if (!written.contains(key)) append(key + L"=" + value);
                edits_.erase(found);
                written.clear();
            };
            while (std::getline(lines, line))
            {
                if (!line.empty() && line.back() == L'\r') line.pop_back();
                const auto trimmed = Trim(line);
                if (trimmed.size() >= 2 && trimmed.front() == L'[' && trimmed.back() == L']')
                {
                    flushSection();
                    section = Trim(trimmed.substr(1, trimmed.size() - 2));
                }
                else if (!trimmed.empty() && trimmed.front() != L';')
                {
                    const auto equals = trimmed.find(L'=');
                    const auto found = edits_.find(section);
                    if (equals != std::wstring::npos && found != edits_.end())
                    {
                        const auto key = Trim(trimmed.substr(0, equals));
                        const auto edit = found->second.find(key);
                        if (edit != found->second.end())
                        {
                            
                            if (!written.contains(key)) append(key + L"=" + edit->second);
                            written[key] = L"";
                            continue;
                        }
                    }
                }
                append(line);
            }
            flushSection();
            for (const auto& [name, values] : edits_)
            {
                append(L"[" + name + L"]");
                for (const auto& [key, value] : values) append(key + L"=" + value);
            }
            std::ofstream file(path_, std::ios::binary | std::ios::trunc);
            file.write(reinterpret_cast<const char*>(output.data()), output.size() * sizeof(wchar_t));
            file.close();
            return !file.fail();
        }
    };
}
