#define NOMINMAX 1
#include "ArtifactValidation.hpp"
#include "../ScopeExit.hpp"
#include <Windows.h>
#include <array>
#include <charconv>
#include <cstring>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string_view>

#pragma comment(lib, "Version.lib")

namespace ImCut::Updater
{
    namespace
    {
        class Image final
        {
        public:
            explicit Image(const std::vector<std::uint8_t>& bytes) : bytes_(bytes) {}
            template<class T> T Read(std::size_t offset) const
            {
                if (offset > bytes_.size() || sizeof(T) > bytes_.size() - offset)
                    throw std::runtime_error("DLL truncada ou com offsets invalidos.");
                T value{};
                std::memcpy(&value, bytes_.data() + offset, sizeof(T));
                return value;
            }
            std::size_t Offset(DWORD rva, std::size_t size = 1, bool executable = false) const
            {
                for (const auto& section : sections)
                {
                    if (rva < section.VirtualAddress) continue;
                    const std::size_t delta = rva - section.VirtualAddress;
                    if (delta >= section.SizeOfRawData || size > section.SizeOfRawData - delta) continue;
                    const std::size_t offset = static_cast<std::size_t>(section.PointerToRawData) + delta;
                    if (offset > bytes_.size() || size > bytes_.size() - offset) break;
                    if (executable && !(section.Characteristics & IMAGE_SCN_MEM_EXECUTE)) break;
                    return offset;
                }
                throw std::runtime_error("DLL com RVA invalido ou export nao executavel.");
            }
            std::string Name(DWORD rva) const
            {
                std::string name;
                for (DWORD i = 0; i < 256; ++i)
                {
                    if (rva > MAXDWORD - i) break;
                    const char ch = Read<char>(Offset(rva + i));
                    if (!ch) return name;
                    name += ch;
                }
                throw std::runtime_error("Nome de export invalido.");
            }
            std::vector<IMAGE_SECTION_HEADER> sections;
        private:
            const std::vector<std::uint8_t>& bytes_;
        };

        std::array<unsigned, 4> ParseVersion(std::string_view text)
        {
            std::array<unsigned, 4> version{};
            if (!text.empty() && (text.front() == 'v' || text.front() == 'V')) text.remove_prefix(1);
            for (std::size_t i = 0; i < 4; ++i)
            {
                if (text.empty()) throw std::runtime_error("Versao remota invalida.");
                const auto end = text.find('.');
                const auto token = text.substr(0, end);
                const auto parsed = std::from_chars(token.data(), token.data() + token.size(), version[i]);
                if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() || version[i] > 65535)
                    throw std::runtime_error("Versao remota invalida.");
                if (end == std::string_view::npos)
                {
                    if (i < 2) throw std::runtime_error("A versao deve conter major.minor.patch.");
                    return version;
                }
                text.remove_prefix(end + 1);
            }
            throw std::runtime_error("Versao remota invalida.");
        }
        std::array<unsigned, 4> FileVersion(const std::filesystem::path& path)
        {
            DWORD ignored = 0;
            const DWORD size = GetFileVersionInfoSizeW(path.c_str(), &ignored);
            if (!size || size > 1024 * 1024) throw std::runtime_error("DLL sem recurso de versao valido.");
            std::vector<std::uint8_t> buffer(size);
            if (!GetFileVersionInfoW(path.c_str(), 0, size, buffer.data())) throw std::runtime_error("Falha ao ler versao da DLL.");
            VS_FIXEDFILEINFO* info = nullptr;
            UINT count = 0;
            if (!VerQueryValueW(buffer.data(), L"\\", reinterpret_cast<void**>(&info), &count) ||
                count < sizeof(*info) || info->dwSignature != VS_FFI_SIGNATURE || info->dwFileType != VFT_DLL)
                throw std::runtime_error("Recurso de versao da DLL invalido.");
            return {HIWORD(info->dwFileVersionMS), LOWORD(info->dwFileVersionMS),
                HIWORD(info->dwFileVersionLS), LOWORD(info->dwFileVersionLS)};
        }
    }

    bool ValidateImage(const std::vector<std::uint8_t>& bytes, std::string& error)
    {
        error.clear();
        try
        {
            Image image(bytes);
            const auto dos = image.Read<IMAGE_DOS_HEADER>(0);
            if (dos.e_magic != IMAGE_DOS_SIGNATURE || dos.e_lfanew < sizeof(dos)) throw std::runtime_error("Arquivo nao e uma DLL PE.");
            const auto nt = image.Read<IMAGE_NT_HEADERS64>(dos.e_lfanew);
            if (nt.Signature != IMAGE_NT_SIGNATURE || nt.FileHeader.Machine != IMAGE_FILE_MACHINE_AMD64 ||
                !(nt.FileHeader.Characteristics & IMAGE_FILE_DLL) ||
                nt.OptionalHeader.Magic != IMAGE_NT_OPTIONAL_HDR64_MAGIC ||
                nt.FileHeader.SizeOfOptionalHeader < sizeof(IMAGE_OPTIONAL_HEADER64) ||
                !nt.FileHeader.NumberOfSections || nt.FileHeader.NumberOfSections > 96 ||
                nt.OptionalHeader.NumberOfRvaAndSizes <= IMAGE_DIRECTORY_ENTRY_EXPORT)
                throw std::runtime_error("A atualizacao deve ser uma DLL x64 valida.");
            const std::size_t sectionOffset = static_cast<std::size_t>(dos.e_lfanew) + sizeof(DWORD) +
                sizeof(IMAGE_FILE_HEADER) + nt.FileHeader.SizeOfOptionalHeader;
            for (WORD i = 0; i < nt.FileHeader.NumberOfSections; ++i)
                image.sections.push_back(image.Read<IMAGE_SECTION_HEADER>(sectionOffset + i * sizeof(IMAGE_SECTION_HEADER)));
            const auto directory = nt.OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_EXPORT];
            if (!directory.VirtualAddress || directory.Size < sizeof(IMAGE_EXPORT_DIRECTORY)) throw std::runtime_error("DLL sem exports ImCut.");
            const auto exports = image.Read<IMAGE_EXPORT_DIRECTORY>(image.Offset(directory.VirtualAddress, sizeof(IMAGE_EXPORT_DIRECTORY)));
            if (!exports.NumberOfNames || exports.NumberOfNames > 65536 ||
                !exports.NumberOfFunctions || exports.NumberOfFunctions > 65536)
                throw std::runtime_error("Tabela de exports invalida.");
            const auto names = image.Offset(exports.AddressOfNames, exports.NumberOfNames * sizeof(DWORD));
            const auto ordinals = image.Offset(exports.AddressOfNameOrdinals, exports.NumberOfNames * sizeof(WORD));
            const auto functions = image.Offset(exports.AddressOfFunctions, exports.NumberOfFunctions * sizeof(DWORD));
            std::set<std::string> required{
                "ImCut_Initialize", "ImCut_ConnectCorel2026", "ImCut_ConnectCorel2026Hidden",
                "ImCut_Show", "ImCut_Hide", "ImCut_Toggle", "ImCut_Shutdown", "ImCut_IsInitialized", "ImCut_IsVisible",
                "ImCut_RunCut", "ImCut_RunBleed", "ImCut_RefreshColorProfiles", "ImCut_ConvertSelection",
                "ImCut_RunCutSaved", "ImCut_RunBleedSaved", "ImCut_ConvertSelectionSaved",
                "ImCut_SetNestingCallback", "ImCut_GetLastError", "ImCut_GetVersion"};
            for (DWORD i = 0; i < exports.NumberOfNames; ++i)
            {
                const auto name = image.Name(image.Read<DWORD>(names + i * sizeof(DWORD)));
                if (!required.count(name)) continue;
                const auto ordinal = image.Read<WORD>(ordinals + i * sizeof(WORD));
                if (ordinal >= exports.NumberOfFunctions) throw std::runtime_error("Ordinal de export invalido.");
                const DWORD rva = image.Read<DWORD>(functions + ordinal * sizeof(DWORD));
                if (rva >= directory.VirtualAddress && static_cast<std::uint64_t>(rva) <
                    static_cast<std::uint64_t>(directory.VirtualAddress) + directory.Size)
                    throw std::runtime_error("Exports ImCut encaminhados nao sao aceitos.");
                (void)image.Offset(rva, 1, true);
                required.erase(name);
            }
            if (!required.empty()) throw std::runtime_error("DLL nao implementa todos os 19 exports ImCut.");
            return true;
        }
        catch (const std::exception& exception) { error = exception.what(); return false; }
    }

    std::filesystem::path RunningModulePath()
    {
        HMODULE module = nullptr;
        if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            reinterpret_cast<LPCWSTR>(&RunningModulePath), &module)) return {};
        std::wstring path(32768, L'\0');
        const DWORD n = GetModuleFileNameW(module, path.data(), static_cast<DWORD>(path.size()));
        if (!n || n >= path.size()) return {};
        path.resize(n);
        return path;
    }

    bool ValidateArtifact(const std::filesystem::path& file, const std::filesystem::path& installedModule,
        const std::string& expectedVersion, std::string& error)
    {
        error.clear();
        try
        {
            if (_wcsicmp(file.extension().c_str(), L".dll") != 0 || installedModule.empty())
                throw std::runtime_error("Somente atualizacoes DLL sao aceitas.");
            HANDLE handle = CreateFileW(file.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
            if (handle == INVALID_HANDLE_VALUE) throw std::runtime_error("Nao foi possivel bloquear a DLL para validacao.");
            ScopeExit close([&] { CloseHandle(handle); });
            LARGE_INTEGER length{};
            if (!GetFileSizeEx(handle, &length) || length.QuadPart <= 0 || length.QuadPart > 512ll * 1024 * 1024)
                throw std::runtime_error("Tamanho da DLL invalido.");
            std::vector<std::uint8_t> bytes(static_cast<std::size_t>(length.QuadPart));
            DWORD read = 0;
            if (!ReadFile(handle, bytes.data(), static_cast<DWORD>(bytes.size()), &read, nullptr) || read != bytes.size())
                throw std::runtime_error("Falha ao ler a DLL completa.");
            if (!ValidateImage(bytes, error)) return false;
            const auto version = FileVersion(file);
            if (version != ParseVersion(expectedVersion) || version <= FileVersion(installedModule))
                throw std::runtime_error("A versao da DLL nao corresponde a versao anunciada ou nao e mais recente.");

            return true;
        }
        catch (const std::exception& exception) { error = exception.what(); return false; }
        catch (...) { error = "Nao foi possivel validar a atualizacao."; return false; }
    }
}
