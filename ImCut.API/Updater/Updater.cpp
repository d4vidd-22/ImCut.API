#define NOMINMAX 1

#include "Updater.hpp"
#include "ArtifactValidation.hpp"

#include <winhttp.h>
#include <shellapi.h>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <exception>
#include <functional>
#include <fstream>
#include <cwchar>
#include <cwctype>
#include <optional>
#include <sstream>
#include <string_view>
#include <thread>
#include <vector>

#pragma comment(lib, "winhttp.lib")
#pragma comment(lib, "shell32.lib")

namespace ImCut::Updater
{
    namespace
    {
        constexpr std::size_t kMaxTextBytes = 64 * 1024;
        constexpr std::uint64_t kMaxDownloadBytes = 512ULL * 1024ULL * 1024ULL;
        constexpr int kRetryCount = 3;

        struct InternetHandle
        {
            HINTERNET value = nullptr;

            InternetHandle() = default;
            explicit InternetHandle(HINTERNET handle) : value(handle) {}

            ~InternetHandle()
            {
                if (value)
                    WinHttpCloseHandle(value);
            }

            InternetHandle(const InternetHandle&) = delete;
            InternetHandle& operator=(const InternetHandle&) = delete;

            InternetHandle(InternetHandle&& other) noexcept
                : value(other.value)
            {
                other.value = nullptr;
            }

            InternetHandle& operator=(InternetHandle&& other) noexcept
            {
                if (this != &other)
                {
                    if (value)
                        WinHttpCloseHandle(value);

                    value = other.value;
                    other.value = nullptr;
                }

                return *this;
            }

            explicit operator bool() const noexcept
            {
                return value != nullptr;
            }
        };

        struct HttpResponse
        {
            DWORD statusCode = 0;
            std::vector<std::uint8_t> body;
            std::wstring contentDisposition;
            std::wstring contentType;
        };

        [[nodiscard]] std::string Trim(std::string value)
        {
            std::size_t first = 0;
            if (value.size() >= 3 &&
                static_cast<unsigned char>(value[0]) == 0xEF &&
                static_cast<unsigned char>(value[1]) == 0xBB &&
                static_cast<unsigned char>(value[2]) == 0xBF)
            {
                first = 3;
            }

            auto isSpace = [](unsigned char ch)
            {
                return std::isspace(ch) != 0;
            };

            while (first < value.size() &&
                isSpace(static_cast<unsigned char>(value[first])))
                ++first;

            std::size_t last = value.size();
            while (last > first &&
                isSpace(static_cast<unsigned char>(value[last - 1])))
                --last;

            if (last - first >= 2 &&
                ((value[first] == '"' && value[last - 1] == '"') ||
                 (value[first] == '\'' && value[last - 1] == '\'')))
            {
                ++first;
                --last;
            }

            return value.substr(first, last - first);
        }

        [[nodiscard]] std::wstring Utf8ToWide(const std::string& value)
        {
            if (value.empty())
                return {};

            const int length = MultiByteToWideChar(
                CP_UTF8,
                MB_ERR_INVALID_CHARS,
                value.data(),
                static_cast<int>(value.size()),
                nullptr,
                0);

            if (length <= 0)
                return {};

            std::wstring output(static_cast<std::size_t>(length), L'\0');

            if (MultiByteToWideChar(
                CP_UTF8,
                MB_ERR_INVALID_CHARS,
                value.data(),
                static_cast<int>(value.size()),
                output.data(),
                length) <= 0)
            {
                return {};
            }

            return output;
        }

        [[nodiscard]] std::string WideToUtf8(const std::wstring& value)
        {
            if (value.empty())
                return {};

            const int length = WideCharToMultiByte(
                CP_UTF8,
                0,
                value.data(),
                static_cast<int>(value.size()),
                nullptr,
                0,
                nullptr,
                nullptr);

            if (length <= 0)
                return {};

            std::string output(static_cast<std::size_t>(length), '\0');

            if (WideCharToMultiByte(
                CP_UTF8,
                0,
                value.data(),
                static_cast<int>(value.size()),
                output.data(),
                length,
                nullptr,
                nullptr) <= 0)
            {
                return {};
            }

            return output;
        }

        [[nodiscard]] std::vector<int> ParseVersion(std::string_view text)
        {
            std::vector<int> parts;
            if (!text.empty() && (text.front() == 'v' || text.front() == 'V'))
                text.remove_prefix(1);

            while (!text.empty() && parts.size() < 4)
            {
                const std::size_t dot = text.find('.');
                const std::string_view token = text.substr(0, dot);
                unsigned value = 0;
                const auto parsed = std::from_chars(
                    token.data(), token.data() + token.size(), value);
                if (token.empty() || parsed.ec != std::errc{} ||
                    parsed.ptr != token.data() + token.size() || value > 65535)
                    return {};
                parts.push_back(static_cast<int>(value));
                if (dot == std::string_view::npos)
                {
                    text = {};
                    break;
                }
                text.remove_prefix(dot + 1);
                if (text.empty())
                    return {};
            }

            if (parts.size() < 3 || !text.empty())
                return {};
            return parts;
        }

        [[nodiscard]] int CompareVersions(
            std::string_view left,
            std::string_view right)
        {
            const auto a = ParseVersion(left);
            const auto b = ParseVersion(right);
            const std::size_t count = std::max(a.size(), b.size());

            for (std::size_t i = 0; i < count; ++i)
            {
                const int av = i < a.size() ? a[i] : 0;
                const int bv = i < b.size() ? b[i] : 0;

                if (av < bv)
                    return -1;

                if (av > bv)
                    return 1;
            }

            return 0;
        }

        [[nodiscard]] bool IsRetryableStatus(DWORD status) noexcept
        {
            return status == 408 ||
                status == 429 ||
                status == 500 ||
                status == 502 ||
                status == 503 ||
                status == 504;
        }

        [[nodiscard]] std::wstring QueryHeaderString(
            HINTERNET request,
            DWORD query)
        {
            DWORD bytes = 0;

            WinHttpQueryHeaders(
                request,
                query,
                WINHTTP_HEADER_NAME_BY_INDEX,
                WINHTTP_NO_OUTPUT_BUFFER,
                &bytes,
                WINHTTP_NO_HEADER_INDEX);

            if (GetLastError() != ERROR_INSUFFICIENT_BUFFER || bytes == 0)
                return {};

            std::wstring value(bytes / sizeof(wchar_t), L'\0');

            if (!WinHttpQueryHeaders(
                request,
                query,
                WINHTTP_HEADER_NAME_BY_INDEX,
                value.data(),
                &bytes,
                WINHTTP_NO_HEADER_INDEX))
            {
                return {};
            }

            while (!value.empty() && value.back() == L'\0')
                value.pop_back();

            return value;
        }

        [[nodiscard]] bool PerformGet(
            const std::wstring& url,
            HttpResponse& response,
            std::atomic_bool& cancelRequested,
            std::uint64_t maxBytes,
            const std::function<void(std::uint64_t, std::uint64_t)>& progress,
            std::string& error)
        {
            URL_COMPONENTS components{};
            components.dwStructSize = sizeof(components);
            components.dwSchemeLength = static_cast<DWORD>(-1);
            components.dwHostNameLength = static_cast<DWORD>(-1);
            components.dwUrlPathLength = static_cast<DWORD>(-1);
            components.dwExtraInfoLength = static_cast<DWORD>(-1);

            if (!WinHttpCrackUrl(
                url.c_str(),
                static_cast<DWORD>(url.size()),
                0,
                &components))
            {
                error = "URL de atualização inválida.";
                return false;
            }

            if (components.nScheme != INTERNET_SCHEME_HTTPS)
            {
                error = "O atualizador aceita apenas links HTTPS.";
                return false;
            }

            if (!components.lpszHostName || components.dwHostNameLength == 0)
            {
                error = "A URL de atualização não contém um servidor válido.";
                return false;
            }

            const std::wstring host(
                components.lpszHostName,
                components.dwHostNameLength);

            std::wstring path;
            if (components.lpszUrlPath && components.dwUrlPathLength > 0)
                path.assign(components.lpszUrlPath, components.dwUrlPathLength);

            if (components.lpszExtraInfo && components.dwExtraInfoLength > 0)
            {
                path.append(
                    components.lpszExtraInfo,
                    components.dwExtraInfoLength);
            }

            if (path.empty())
                path = L"/";

            InternetHandle session(
                WinHttpOpen(
                    L"ImCut-Updater/1.0",
                    WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY,
                    WINHTTP_NO_PROXY_NAME,
                    WINHTTP_NO_PROXY_BYPASS,
                    0));

            if (!session)
            {
                error = "Não foi possível iniciar o WinHTTP.";
                return false;
            }

            WinHttpSetTimeouts(
                session.value,
                5000,
                5000,
                10000,
                20000);

            InternetHandle connection(
                WinHttpConnect(
                    session.value,
                    host.c_str(),
                    components.nPort,
                    0));

            if (!connection)
            {
                error = "Não foi possível conectar ao servidor de atualização.";
                return false;
            }

            InternetHandle request(
                WinHttpOpenRequest(
                    connection.value,
                    L"GET",
                    path.c_str(),
                    nullptr,
                    WINHTTP_NO_REFERER,
                    WINHTTP_DEFAULT_ACCEPT_TYPES,
                    WINHTTP_FLAG_SECURE));

            if (!request)
            {
                error = "Não foi possível criar a requisição de atualização.";
                return false;
            }

            const wchar_t headers[] =
                L"Cache-Control: no-cache\r\n"
                L"Pragma: no-cache\r\n"
                L"Accept: */*\r\n";

            if (!WinHttpSendRequest(
                request.value,
                headers,
                static_cast<DWORD>(-1),
                WINHTTP_NO_REQUEST_DATA,
                0,
                0,
                0) ||
                !WinHttpReceiveResponse(
                    request.value,
                    nullptr))
            {
                error = "Falha ao receber resposta do servidor de atualização.";
                return false;
            }

            DWORD status = 0;
            DWORD statusSize = sizeof(status);

            if (!WinHttpQueryHeaders(
                request.value,
                WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
                WINHTTP_HEADER_NAME_BY_INDEX,
                &status,
                &statusSize,
                WINHTTP_NO_HEADER_INDEX))
            {
                error = "Não foi possível obter o status HTTP da atualização.";
                return false;
            }

            response.statusCode = status;
            response.contentDisposition = QueryHeaderString(
                request.value,
                WINHTTP_QUERY_CONTENT_DISPOSITION);
            response.contentType = QueryHeaderString(
                request.value,
                WINHTTP_QUERY_CONTENT_TYPE);

            if (status < 200 || status >= 300)
            {
                std::ostringstream stream;
                stream << "Servidor de atualização respondeu HTTP " << status << ".";
                error = stream.str();
                return false;
            }

            std::uint64_t total = 0;
            DWORD contentLength = 0;
            DWORD contentLengthSize = sizeof(contentLength);

            if (WinHttpQueryHeaders(
                request.value,
                WINHTTP_QUERY_CONTENT_LENGTH | WINHTTP_QUERY_FLAG_NUMBER,
                WINHTTP_HEADER_NAME_BY_INDEX,
                &contentLength,
                &contentLengthSize,
                WINHTTP_NO_HEADER_INDEX))
            {
                total = contentLength;

                if (total > maxBytes)
                {
                    error = "O arquivo de atualização excede o limite permitido.";
                    return false;
                }
            }

            response.body.clear();

            if (total > 0)
            {
                response.body.reserve(
                    static_cast<std::size_t>(
                        std::min<std::uint64_t>(total, maxBytes)));
            }

            std::vector<std::uint8_t> buffer(64 * 1024);
            std::uint64_t received = 0;

            for (;;)
            {
                if (cancelRequested.load(std::memory_order_relaxed))
                {
                    error = "Operação cancelada.";
                    return false;
                }

                DWORD read = 0;

                if (!WinHttpReadData(
                    request.value,
                    buffer.data(),
                    static_cast<DWORD>(buffer.size()),
                    &read))
                {
                    error = "Falha durante o download da atualização.";
                    return false;
                }

                if (read == 0)
                    break;

                if (received + read > maxBytes)
                {
                    error = "O arquivo de atualização excede o limite permitido.";
                    return false;
                }

                response.body.insert(
                    response.body.end(),
                    buffer.begin(),
                    buffer.begin() + read);

                received += read;

                if (progress)
                    progress(received, total);
            }

            if (progress)
                progress(received, total > 0 ? total : received);

            return true;
        }

        [[nodiscard]] bool GetWithRetry(
            const std::wstring& url,
            HttpResponse& response,
            std::atomic_bool& cancelRequested,
            std::uint64_t maxBytes,
            const std::function<void(std::uint64_t, std::uint64_t)>& progress,
            std::string& error)
        {
            for (int attempt = 0; attempt < kRetryCount; ++attempt)
            {
                if (cancelRequested.load(std::memory_order_relaxed))
                {
                    error = "Operação cancelada.";
                    return false;
                }

                HttpResponse current;
                std::string currentError;

                if (PerformGet(
                    url,
                    current,
                    cancelRequested,
                    maxBytes,
                    progress,
                    currentError))
                {
                    response = std::move(current);
                    return true;
                }

                error = currentError;

                const bool shouldRetry =
                    IsRetryableStatus(current.statusCode) ||
                    current.statusCode == 0;

                if (!shouldRetry || attempt + 1 >= kRetryCount)
                    return false;

                const auto retryAt = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(500 * (1 << attempt));
                while (std::chrono::steady_clock::now() < retryAt)
                {
                    if (cancelRequested.load(std::memory_order_relaxed))
                    {
                        error = "Operação cancelada.";
                        return false;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(25));
                }
            }

            return false;
        }

        [[nodiscard]] bool GetText(
            const std::wstring& url,
            std::string& text,
            std::atomic_bool& cancelRequested,
            std::string& error)
        {
            HttpResponse response;

            if (!GetWithRetry(
                url,
                response,
                cancelRequested,
                kMaxTextBytes,
                {},
                error))
            {
                return false;
            }

            if (response.body.empty())
            {
                text.clear();
            }
            else
            {
                text.assign(
                    reinterpret_cast<const char*>(response.body.data()),
                    response.body.size());
            }

            text = Trim(std::move(text));
            return true;
        }

        [[nodiscard]] int Base64Value(unsigned char ch) noexcept
        {
            if (ch >= 'A' && ch <= 'Z')
                return ch - 'A';
            if (ch >= 'a' && ch <= 'z')
                return ch - 'a' + 26;
            if (ch >= '0' && ch <= '9')
                return ch - '0' + 52;
            if (ch == '+' || ch == '-')
                return 62;
            if (ch == '/' || ch == '_')
                return 63;
            return -1;
        }

        [[nodiscard]] std::optional<std::string> DecodeBase64Text(
            std::string_view value)
        {
            std::vector<std::uint8_t> bytes;
            bytes.reserve((value.size() * 3) / 4);

            std::uint32_t accumulator = 0;
            int bits = -8;

            for (const unsigned char ch : value)
            {
                if (std::isspace(ch))
                    continue;

                if (ch == '=')
                    break;

                const int decoded = Base64Value(ch);

                if (decoded < 0)
                    return std::nullopt;

                accumulator = (accumulator << 6) | decoded;
                bits += 6;

                if (bits >= 0)
                {
                    bytes.push_back(
                        static_cast<std::uint8_t>(
                            (accumulator >> bits) & 0xFF));
                    bits -= 8;
                }
            }

            if (bytes.empty())
                return std::nullopt;

            for (const std::uint8_t byte : bytes)
            {
                if (byte == 0 ||
                    (byte < 0x09) ||
                    (byte > 0x0D && byte < 0x20))
                {
                    return std::nullopt;
                }
            }

            return std::string(
                reinterpret_cast<const char*>(bytes.data()),
                bytes.size());
        }

        [[nodiscard]] bool IsHttpsUrl(std::string_view value)
        {
            if (value.size() < 8)
                return false;

            const std::string prefix(value.substr(0, 8));
            std::string lowered = prefix;

            std::transform(
                lowered.begin(),
                lowered.end(),
                lowered.begin(),
                [](unsigned char ch)
                {
                    return static_cast<char>(std::tolower(ch));
                });

            return lowered == "https://";
        }

        [[nodiscard]] std::optional<std::wstring> ResolveDownloadUrl(
            std::string manifest)
        {
            manifest = Trim(std::move(manifest));

            if (IsHttpsUrl(manifest))
                return Utf8ToWide(manifest);

            if (const auto decoded = DecodeBase64Text(manifest))
            {
                const std::string candidate = Trim(*decoded);

                if (IsHttpsUrl(candidate))
                    return Utf8ToWide(candidate);
            }

            return std::nullopt;
        }

        [[nodiscard]] std::wstring SanitizeFileName(std::wstring value)
        {
            static constexpr wchar_t forbidden[] = L"<>:\"/\\|?*";

            for (wchar_t& ch : value)
            {
                if (ch < 32 || std::wcschr(forbidden, ch))
                    ch = L'_';
            }

            while (!value.empty() &&
                (value.back() == L' ' || value.back() == L'.'))
            {
                value.pop_back();
            }

            if (value.empty())
                value = L"ImCut_Update.bin";

            return value;
        }

        [[nodiscard]] std::wstring FileNameFromDisposition(
            const std::wstring& disposition)
        {
            if (disposition.empty())
                return {};

            std::wstring lower = disposition;
            std::transform(
                lower.begin(),
                lower.end(),
                lower.begin(),
                [](wchar_t ch)
                {
                    return static_cast<wchar_t>(std::towlower(ch));
                });

            const std::wstring token = L"filename=";
            const std::size_t pos = lower.find(token);

            if (pos == std::wstring::npos)
                return {};

            std::size_t begin = pos + token.size();
            while (begin < disposition.size() && std::iswspace(disposition[begin]))
                ++begin;

            if (begin >= disposition.size())
                return {};

            if (disposition[begin] == L'"')
            {
                const std::size_t end = disposition.find(L'"', begin + 1);

                if (end != std::wstring::npos)
                    return SanitizeFileName(disposition.substr(begin + 1, end - begin - 1));
            }

            const std::size_t end = disposition.find(L';', begin);
            return SanitizeFileName(
                disposition.substr(
                    begin,
                    end == std::wstring::npos
                        ? std::wstring::npos
                        : end - begin));
        }

        [[nodiscard]] std::wstring FileNameFromUrl(
            const std::wstring& url)
        {
            URL_COMPONENTS components{};
            components.dwStructSize = sizeof(components);
            components.dwUrlPathLength = static_cast<DWORD>(-1);

            if (!WinHttpCrackUrl(
                url.c_str(),
                static_cast<DWORD>(url.size()),
                0,
                &components))
            {
                return L"ImCut_Update.bin";
            }

            std::wstring path;
            if (components.lpszUrlPath && components.dwUrlPathLength > 0)
                path.assign(components.lpszUrlPath, components.dwUrlPathLength);

            const std::size_t slash = path.find_last_of(L'/');
            std::wstring name =
                slash == std::wstring::npos
                    ? path
                    : path.substr(slash + 1);

            if (name.empty())
                name = L"ImCut_Update.bin";

            return SanitizeFileName(std::move(name));
        }

        [[nodiscard]] std::filesystem::path BuildUpdatePath(
            const std::wstring& fileName)
        {
            DWORD size = GetEnvironmentVariableW(
                L"LOCALAPPDATA",
                nullptr,
                0);

            std::filesystem::path base;

            if (size > 1)
            {
                std::wstring value(size, L'\0');
                const DWORD written = GetEnvironmentVariableW(
                    L"LOCALAPPDATA",
                    value.data(),
                    size);

                if (written > 0)
                {
                    value.resize(written);
                    base = std::filesystem::path(value) /
                        L"ImCut" /
                        L"GMSPath" /
                        L"Updates";
                }
            }

            if (base.empty())
            {
                std::error_code ec;
                const auto temp = std::filesystem::temp_directory_path(ec);

                base = !ec
                    ? temp / L"ImCut" / L"GMSPath" / L"Updates"
                    : std::filesystem::path(L".") / L"ImCut" / L"GMSPath" / L"Updates";
            }

            std::filesystem::create_directories(base);

            std::filesystem::path destination = base / fileName;

            if (!std::filesystem::exists(destination))
                return destination;

            const auto stem = destination.stem().wstring();
            const auto extension = destination.extension().wstring();

            for (int index = 1; index < 1000; ++index)
            {
                auto candidate = base /
                    (stem + L"_" + std::to_wstring(index) + extension);

                if (!std::filesystem::exists(candidate))
                    return candidate;
            }

            return base / L"ImCut_Update.bin";
        }

        [[nodiscard]] bool WriteFileAtomic(
            const std::filesystem::path& destination,
            const std::vector<std::uint8_t>& data,
            std::string& error)
        {
            const std::filesystem::path temp =
                destination.wstring() + L".part";

            std::ofstream stream(
                temp,
                std::ios::binary | std::ios::trunc);

            if (!stream)
            {
                error = "Não foi possível criar o arquivo temporário da atualização.";
                return false;
            }

            stream.write(
                reinterpret_cast<const char*>(data.data()),
                static_cast<std::streamsize>(data.size()));

            stream.close();

            if (!stream)
            {
                std::error_code ignored;
                std::filesystem::remove(temp, ignored);
                error = "Falha ao gravar o arquivo da atualização.";
                return false;
            }

            std::error_code ec;
            std::filesystem::rename(temp, destination, ec);

            if (ec)
            {
                std::filesystem::remove(temp, ec);
                error = "Não foi possível finalizar o arquivo da atualização.";
                return false;
            }

            return true;
        }

        [[nodiscard]] std::wstring LowerExtension(
            const std::filesystem::path& path)
        {
            std::wstring extension = path.extension().wstring();

            std::transform(
                extension.begin(),
                extension.end(),
                extension.begin(),
                [](wchar_t ch)
                {
                    return static_cast<wchar_t>(std::towlower(ch));
                });

            return extension;
        }
    }

    Service::Service()
    {
        snapshot_.currentVersion = CurrentVersion();
    }

    Service::~Service()
    {
        Stop();
    }

    void Service::JoinWorker()
    {
        if (worker_.joinable())
            worker_.join();
    }

    void Service::Cancel() noexcept
    {
        cancelRequested_.store(true, std::memory_order_relaxed);
    }

    void Service::Stop()
    {
        Cancel();
        JoinWorker();
        busy_.store(false, std::memory_order_release);
    }

    bool Service::IsBusy() const noexcept
    {
        return busy_.load(std::memory_order_relaxed);
    }

    Snapshot Service::GetSnapshot() const
    {
        std::scoped_lock lock(mutex_);
        return snapshot_;
    }

    void Service::SetSnapshot(
        Status status,
        std::string remoteVersion,
        std::string message,
        float progress,
        std::wstring downloadedPath)
    {
        std::scoped_lock lock(mutex_);
        snapshot_.status = status;
        snapshot_.currentVersion = CurrentVersion();
        snapshot_.remoteVersion = std::move(remoteVersion);
        snapshot_.message = std::move(message);
        snapshot_.progress = std::clamp(progress, 0.0f, 1.0f);

        if (status != Status::Error)
        {
            snapshot_.updateAvailable =
                status == Status::UpdateAvailable ||
                status == Status::ResolvingDownload ||
                status == Status::Downloading ||
                status == Status::Downloaded;
        }

        snapshot_.downloadedPath = std::move(downloadedPath);
    }

    void Service::CheckAsync()
    {
        if (busy_.exchange(true, std::memory_order_acq_rel))
            return;

        JoinWorker();
        cancelRequested_.store(false, std::memory_order_relaxed);

        std::string previousRemote;
        {
            std::scoped_lock lock(mutex_);
            previousRemote = snapshot_.remoteVersion;
        }

        SetSnapshot(
            Status::Checking,
            std::move(previousRemote),
            "Verificando atualização...",
            0.0f);

        worker_ = std::jthread(
            [this]()
            {
                try
                {
                    RunCheck();
                }
                catch (const std::exception& error)
                {
                    SetSnapshot(
                        Status::Error,
                        {},
                        std::string("Falha interna no atualizador: ") + error.what());
                }
                catch (...)
                {
                    SetSnapshot(
                        Status::Error,
                        {},
                        "Falha interna desconhecida no atualizador.");
                }

                busy_.store(false, std::memory_order_release);
            });
    }

    void Service::RunCheck()
    {
        std::string remote;
        std::string error;

        if (!GetText(
            VersionUrl(),
            remote,
            cancelRequested_,
            error))
        {
            if (!cancelRequested_.load(std::memory_order_relaxed))
            {
                SetSnapshot(
                    Status::Error,
                    {},
                    error.empty()
                        ? "Não foi possível verificar atualizações."
                        : error);
            }

            return;
        }

        remote = Trim(std::move(remote));

        if (remote.size() > 64 || ParseVersion(remote).empty())
        {
            SetSnapshot(
                Status::Error,
                remote,
                "O servidor retornou uma versão inválida.");
            return;
        }

        const int comparison = CompareVersions(
            CurrentVersion(),
            remote);

        if (comparison < 0)
        {
            SetSnapshot(
                Status::UpdateAvailable,
                remote,
                "Nova versão disponível.");
        }
        else
        {
            SetSnapshot(
                Status::UpToDate,
                remote,
                "Você está usando a versão mais recente.",
                1.0f);
        }
    }

    void Service::DownloadAsync()
    {
        if (busy_.exchange(true, std::memory_order_acq_rel))
            return;

        Snapshot current = GetSnapshot();

        if (current.status != Status::UpdateAvailable &&
            !(current.status == Status::Error && current.updateAvailable))
        {
            busy_.store(false, std::memory_order_release);
            return;
        }

        JoinWorker();
        cancelRequested_.store(false, std::memory_order_relaxed);

        SetSnapshot(
            Status::ResolvingDownload,
            current.remoteVersion,
            "Obtendo link de download...",
            0.0f);

        worker_ = std::jthread(
            [this]()
            {
                try
                {
                    RunDownload();
                }
                catch (const std::exception& error)
                {
                    const auto current = GetSnapshot();

                    SetSnapshot(
                        Status::Error,
                        current.remoteVersion,
                        std::string("Falha interna no download: ") + error.what());
                }
                catch (...)
                {
                    const auto current = GetSnapshot();

                    SetSnapshot(
                        Status::Error,
                        current.remoteVersion,
                        "Falha interna desconhecida durante o download.");
                }

                busy_.store(false, std::memory_order_release);
            });
    }

    void Service::RunDownload()
    {
        Snapshot current = GetSnapshot();

        std::string manifest;
        std::string error;

        if (!GetText(
            DownloadManifestUrl(),
            manifest,
            cancelRequested_,
            error))
        {
            if (!cancelRequested_.load(std::memory_order_relaxed))
            {
                SetSnapshot(
                    Status::Error,
                    current.remoteVersion,
                    error.empty()
                        ? "Não foi possível obter o link de download."
                        : error);
            }

            return;
        }

        const auto url = ResolveDownloadUrl(manifest);

        if (!url || url->empty())
        {
            SetSnapshot(
                Status::Error,
                current.remoteVersion,
                "O conteúdo de 'dl' não contém um link HTTPS válido. Pode ser URL direta ou Base64 de uma URL HTTPS.");
            return;
        }

        SetSnapshot(
            Status::Downloading,
            current.remoteVersion,
            "Baixando atualização...",
            0.0f);

        HttpResponse response;

        const bool downloaded = GetWithRetry(
            *url,
            response,
            cancelRequested_,
            kMaxDownloadBytes,
            [this, remote = current.remoteVersion](
                std::uint64_t received,
                std::uint64_t total)
            {
                float progress = 0.0f;

                if (total > 0)
                {
                    progress = static_cast<float>(
                        static_cast<double>(received) /
                        static_cast<double>(total));
                }

                SetSnapshot(
                    Status::Downloading,
                    remote,
                    "Baixando atualização...",
                    progress);
            },
            error);

        if (!downloaded)
        {
            if (!cancelRequested_.load(std::memory_order_relaxed))
            {
                SetSnapshot(
                    Status::Error,
                    current.remoteVersion,
                    error.empty()
                        ? "Falha ao baixar a atualização."
                        : error);
            }

            return;
        }

        if (response.body.empty())
        {
            SetSnapshot(
                Status::Error,
                current.remoteVersion,
                "O servidor retornou um arquivo de atualização vazio.");
            return;
        }

        std::wstring fileName =
            FileNameFromDisposition(
                response.contentDisposition);

        if (fileName.empty())
            fileName = FileNameFromUrl(*url);

        if (_wcsicmp(std::filesystem::path(fileName).extension().c_str(), L".dll") != 0)
        {
            SetSnapshot(Status::Error, current.remoteVersion, "O download deve ser uma DLL ImCut x64 valida.");
            return;
        }
        const auto destination = BuildUpdatePath(fileName);

        if (!WriteFileAtomic(destination, response.body, error))
        {
            SetSnapshot(
                Status::Error,
                current.remoteVersion,
                error);
            return;
        }

        if (!ValidateArtifact(destination, RunningModulePath(), current.remoteVersion, error))
        {
            std::error_code ignored;
            std::filesystem::remove(destination, ignored);
            SetSnapshot(Status::Error, current.remoteVersion, error);
            return;
        }

        SetSnapshot(
            Status::Downloaded,
            current.remoteVersion,
            "Atualização baixada. Clique em instalar e reiniciar para aplicar sem fechar o CorelDRAW.",
            1.0f,
            destination.wstring());
    }

    bool Service::OpenDownloaded(HWND owner) const
    {
        const Snapshot current = GetSnapshot();

        if (current.status != Status::Downloaded ||
            current.downloadedPath.empty())
        {
            return false;
        }

        const std::filesystem::path path(current.downloadedPath);

        if (!std::filesystem::exists(path))
            return false;

        const std::wstring extension = LowerExtension(path);

        if (extension == L".dll")
        {
            const std::wstring parameters =
                L"/select,\"" + path.wstring() + L"\"";

            const auto result = reinterpret_cast<INT_PTR>(
                ShellExecuteW(
                    owner,
                    L"open",
                    L"explorer.exe",
                    parameters.c_str(),
                    path.parent_path().c_str(),
                    SW_SHOWNORMAL));

            return result > 32;
        }

        return false;
    }
}
