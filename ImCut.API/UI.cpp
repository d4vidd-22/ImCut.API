#include "InputLimits.hpp"
#define NOMINMAX 1

#include "UI.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace ImCut::UI
{
    namespace
    {
        float gScale = 1.0f;
        ImFont* gRegularFont = nullptr;
        ImFont* gSemiboldFont = nullptr;

        Page gVisiblePage = Page::Cut;
        float gPageEnter = 1.0f;
        float gTabPosition = 0.0f;
        bool gPrimaryActionArmed = false;
        bool gPrimaryActionFocusPending = false;
        Page gPrimaryActionPage = Page::Cut;

        std::unordered_map<ImGuiID, float> gHover;
        std::unordered_map<ImGuiID, float> gPress;
        std::unordered_map<ImGuiID, float> gToggle;
        std::string gPromptedUpdateVersion;

        struct MappingDisplayCache
        {
            std::uint64_t fingerprint = 0;
            std::size_t index = static_cast<std::size_t>(-1);
            Language language = Language::Portuguese;
            std::string sourceSummary;
            std::string targetSummary;
            std::string headerLabel;
            std::string searchKey;
        };

        std::vector<MappingDisplayCache> gMappingDisplayCache;

        constexpr int TabCount = 4;

        constexpr Page TabPages[TabCount]
        {
            Page::Cut,
            Page::Bleed,
            Page::Color,
            Page::Settings
        };

        [[nodiscard]] float S(float value) noexcept
        {
            return value * gScale;
        }

        [[nodiscard]] ImVec4 C(int r, int g, int b, int a = 255) noexcept
        {
            return ImVec4(
                static_cast<float>(r) / 255.0f,
                static_cast<float>(g) / 255.0f,
                static_cast<float>(b) / 255.0f,
                static_cast<float>(a) / 255.0f);
        }

        [[nodiscard]] ImVec4 LerpColor(
            const ImVec4& a,
            const ImVec4& b,
            float t) noexcept
        {
            t = std::clamp(t, 0.0f, 1.0f);

            return ImVec4(
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t,
                a.z + (b.z - a.z) * t,
                a.w + (b.w - a.w) * t);
        }

        [[nodiscard]] ImU32 U32(const ImVec4& color)
        {
            return ImGui::ColorConvertFloat4ToU32(color);
        }

        [[nodiscard]] float Animate(
            float current,
            float target,
            float speed)
        {
            const float dt =
                std::clamp(
                    ImGui::GetIO().DeltaTime,
                    0.0f,
                    0.05f);

            const float factor =
                std::clamp(
                    speed * dt,
                    0.0f,
                    1.0f);

            return current +
                (target - current) *
                factor;
        }

        [[nodiscard]] bool Pt(const State& state) noexcept
        {
            return state.language == Language::Portuguese;
        }

        [[nodiscard]] const char* T(
            const State& state,
            const char* portuguese,
            const char* english) noexcept
        {
            return Pt(state)
                ? portuguese
                : english;
        }

        class FontScope final
        {
        public:
            explicit FontScope(ImFont* font)
                : active_(font != nullptr)
            {
                if (active_)
                    ImGui::PushFont(font);
            }

            ~FontScope()
            {
                if (active_)
                    ImGui::PopFont();
            }

            FontScope(const FontScope&) = delete;
            FontScope& operator=(const FontScope&) = delete;

        private:
            bool active_ = false;
        };

        void TextMuted(const char* text)
        {
            ImGui::PushStyleColor(
                ImGuiCol_Text,
                C(151, 151, 151));

            ImGui::TextWrapped("%s", text);
            ImGui::PopStyleColor();
        }

        void TextMutedWrapped(const char* text)
        {
            ImGui::PushStyleColor(
                ImGuiCol_Text,
                C(151, 151, 151));

            ImGui::TextWrapped(
                "%s",
                text);

            ImGui::PopStyleColor();
        }

        void PageTitle(
            const char* title,
            const char* subtitle)
        {
            {
                FontScope font(gSemiboldFont);

                ImGui::PushStyleColor(
                    ImGuiCol_Text,
                    C(236, 236, 236));

                ImGui::SetWindowFontScale(1.12f);
                ImGui::TextUnformatted(title);
                ImGui::SetWindowFontScale(1.0f);
                ImGui::PopStyleColor();
            }

            ImGui::PushStyleColor(
                ImGuiCol_Text,
                C(149, 149, 149));

            ImGui::SetWindowFontScale(0.91f);
            ImGui::TextWrapped("%s", subtitle);
            ImGui::SetWindowFontScale(1.0f);
            ImGui::PopStyleColor();

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(4.0f)));
        }

        void SectionTitle(const char* title)
        {
            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(5.0f)));

            {
                FontScope font(gSemiboldFont);

                ImGui::PushStyleColor(
                    ImGuiCol_Text,
                    C(188, 188, 188));

                ImGui::SetWindowFontScale(0.91f);
                ImGui::TextUnformatted(title);
                ImGui::SetWindowFontScale(1.0f);
                ImGui::PopStyleColor();
            }

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(2.0f)));

            ImGui::PushStyleColor(
                ImGuiCol_Separator,
                C(67, 67, 67));

            ImGui::Separator();
            ImGui::PopStyleColor();

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(4.0f)));
        }

        void FieldLabel(
            const char* label,
            const char* suffix = nullptr)
        {
            ImGui::AlignTextToFramePadding();

            {
                FontScope font(gSemiboldFont);

                ImGui::PushStyleColor(
                    ImGuiCol_Text,
                    C(207, 207, 207));

                ImGui::TextWrapped("%s", label);
                ImGui::PopStyleColor();
            }

            if (suffix &&
                *suffix)
            {
                ImGui::SameLine(
                    0.0f,
                    S(5.0f));

                ImGui::PushStyleColor(
                    ImGuiCol_Text,
                    C(126, 126, 126));

                ImGui::TextUnformatted(suffix);
                ImGui::PopStyleColor();
            }
        }

        bool ComboStringVector(
            const char* id,
            int& selected,
            const std::vector<std::string>& items,
            const char* emptyText)
        {
            const char* preview =
                emptyText;

            if (selected >= 0 &&
                selected <
                static_cast<int>(
                    items.size()))
            {
                preview =
                    items[
                        static_cast<std::size_t>(
                            selected)].c_str();
            }

            bool changed = false;

            if (ImGui::BeginCombo(
                id,
                preview,
                ImGuiComboFlags_HeightRegular))
            {
                for (int i = 0;
                    i <
                    static_cast<int>(
                        items.size());
                        ++i)
                {
                    const bool selectedNow =
                        selected == i;

                    if (ImGui::Selectable(
                        items[
                            static_cast<std::size_t>(
                                i)].c_str(),
                                selectedNow))
                    {
                        selected = i;
                        changed = true;
                    }

                    if (selectedNow)
                        ImGui::SetItemDefaultFocus();
                }

                ImGui::EndCombo();
            }

            return changed;
        }

        [[nodiscard]] bool PrimaryActionArmed(
            Page page) noexcept
        {
            return
                gPrimaryActionArmed &&
                gPrimaryActionPage == page;
        }

        void CancelPrimaryAction() noexcept
        {
            gPrimaryActionArmed = false;
            gPrimaryActionFocusPending = false;
        }

        bool ActionButton(
            const char* id,
            const char* label,
            bool enabled,
            float width = 154.0f,
            bool defaultAction = false)
        {
            ImGui::PushID(id);

            if (!enabled)
                ImGui::BeginDisabled();

            const ImVec2 p =
                ImGui::GetCursorScreenPos();

            const float buttonWidth = std::min(S(width), ImGui::GetContentRegionAvail().x);
            const float textWidth = std::max(S(20.0f), buttonWidth - S(20.0f));
            ImVec2 labelSize;
            {
                FontScope labelFont(gSemiboldFont);
                labelSize = ImGui::CalcTextSize(label, nullptr, false, textWidth);
            }
            const ImVec2 size(buttonWidth, std::max(S(34.0f), labelSize.y + S(14.0f)));

            const ImGuiID itemId =
                ImGui::GetID(
                    "##ActionButton");

            if (defaultAction &&
                gPrimaryActionFocusPending)
            {
                ImGui::SetKeyboardFocusHere();
                gPrimaryActionFocusPending = false;
            }

            const bool clicked =
                ImGui::InvisibleButton(
                    "##ActionButton",
                    size);

            if (defaultAction)
                ImGui::SetItemDefaultFocus();

            const bool hovered =
                enabled &&
                ImGui::IsItemHovered();

            const bool held =
                enabled &&
                ImGui::IsItemActive();

            const bool enterPressed =
                defaultAction &&
                enabled &&
                ImGui::IsWindowFocused(
                    ImGuiFocusedFlags_RootAndChildWindows) &&
                !ImGui::GetIO().WantTextInput &&
                ImGui::IsKeyPressed(
                    ImGuiKey_Enter,
                    false);

            if (defaultAction &&
                ImGui::IsMouseClicked(
                    ImGuiMouseButton_Left) &&
                !hovered)
            {
                CancelPrimaryAction();
                defaultAction = false;
            }

            if (!enabled)
                ImGui::EndDisabled();

            float& hover =
                gHover[itemId];

            hover =
                Animate(
                    hover,
                    (hovered || defaultAction) ? 1.0f : 0.0f,
                    15.0f);

            float& press =
                gPress[itemId];

            press =
                Animate(
                    press,
                    held ? 1.0f : 0.0f,
                    21.0f);

            const float y =
                p.y +
                S(1.0f) *
                press;

            const ImVec4 normal =
                enabled
                ? C(38, 87, 128)
                : C(64, 64, 64);

            const ImVec4 over =
                enabled
                ? C(45, 101, 148)
                : normal;

            ImDrawList* draw =
                ImGui::GetWindowDrawList();

            draw->AddRectFilled(
                ImVec2(
                    p.x,
                    y),
                ImVec2(
                    p.x + size.x,
                    y + size.y),
                U32(
                    LerpColor(
                        normal,
                        over,
                        hover)),
                S(3.0f));

            draw->AddRect(
                ImVec2(
                    p.x,
                    y),
                ImVec2(
                    p.x + size.x,
                    y + size.y),
                U32(
                    enabled
                    ? (defaultAction
                        ? C(112, 174, 220)
                        : C(57, 112, 154))
                    : C(77, 77, 77)),
                S(3.0f),
                0,
                defaultAction
                    ? S(2.0f)
                    : 1.0f);

            FontScope font(
                gSemiboldFont);

            const ImVec2 textSize = labelSize;

            draw->AddText(
                ImGui::GetFont(), ImGui::GetFontSize(),
                ImVec2(
                    p.x +
                    (size.x - textSize.x) *
                    0.5f,
                    y +
                    (size.y - textSize.y) *
                    0.5f),
                U32(
                    enabled
                    ? C(245, 245, 245)
                    : C(143, 143, 143)),
                label, nullptr, textWidth);

            const bool activated =
                enabled &&
                (clicked || enterPressed);

            if (activated && defaultAction)
                CancelPrimaryAction();

            ImGui::PopID();

            return activated;
        }

        bool TextButton(
            const char* id,
            const char* label)
        {
            ImGui::PushID(id);

            const ImVec2 p =
                ImGui::GetCursorScreenPos();

            const ImVec2 textSize =
                ImGui::CalcTextSize(
                    label);

            const ImVec2 size(
                textSize.x + S(16.0f),
                S(28.0f));

            const ImGuiID itemId =
                ImGui::GetID(
                    "##TextButton");

            const bool clicked =
                ImGui::InvisibleButton(
                    "##TextButton",
                    size);

            float& hover =
                gHover[itemId];

            hover =
                Animate(
                    hover,
                    ImGui::IsItemHovered()
                    ? 1.0f
                    : 0.0f,
                    15.0f);

            ImDrawList* draw =
                ImGui::GetWindowDrawList();

            if (hover > 0.01f)
            {
                draw->AddRectFilled(
                    p,
                    ImVec2(
                        p.x + size.x,
                        p.y + size.y),
                    U32(
                        C(
                            72,
                            72,
                            72,
                            static_cast<int>(
                                90.0f *
                                hover))),
                    S(3.0f));
            }

            draw->AddText(
                ImVec2(
                    p.x + S(8.0f),
                    p.y +
                    (size.y - textSize.y) *
                    0.5f),
                U32(
                    LerpColor(
                        C(177, 177, 177),
                        C(232, 232, 232),
                        hover)),
                label);

            ImGui::PopID();

            return clicked;
        }

        bool Toggle(
            const char* id,
            bool& value)
        {
            ImGui::PushID(id);

            const ImVec2 p =
                ImGui::GetCursorScreenPos();

            const ImVec2 size(
                S(34.0f),
                S(18.0f));

            const ImGuiID itemId =
                ImGui::GetID(
                    "##Toggle");

            const bool clicked =
                ImGui::InvisibleButton(
                    "##Toggle",
                    size);

            if (clicked)
                value = !value;

            float& animation =
                gToggle[itemId];

            animation =
                Animate(
                    animation,
                    value ? 1.0f : 0.0f,
                    18.0f);

            float& hover =
                gHover[itemId];

            hover =
                Animate(
                    hover,
                    ImGui::IsItemHovered()
                    ? 1.0f
                    : 0.0f,
                    14.0f);

            const ImVec4 off =
                LerpColor(
                    C(80, 80, 80),
                    C(91, 91, 91),
                    hover);

            const ImVec4 on =
                LerpColor(
                    C(37, 83, 119),
                    C(43, 96, 137),
                    hover);

            ImDrawList* draw =
                ImGui::GetWindowDrawList();

            draw->AddRectFilled(
                p,
                ImVec2(
                    p.x + size.x,
                    p.y + size.y),
                U32(
                    LerpColor(
                        off,
                        on,
                        animation)),
                size.y *
                0.5f);

            const float radius =
                S(6.0f);

            const float left =
                p.x +
                S(3.0f) +
                radius;

            const float right =
                p.x +
                size.x -
                S(3.0f) -
                radius;

            draw->AddCircleFilled(
                ImVec2(
                    left +
                    (right - left) *
                    animation,
                    p.y +
                    size.y *
                    0.5f),
                radius,
                U32(
                    C(
                        244,
                        244,
                        244)));

            ImGui::PopID();

            return clicked;
        }

        void OptionRow(
            const char* id,
            const char* label,
            bool& value)
        {


            const ImVec2 start =
                ImGui::GetCursorScreenPos();

            const float available =
                ImGui::GetContentRegionAvail().x;
            const float labelWidth = std::max(S(30.0f), available - S(48.0f));
            const ImVec2 labelSize = ImGui::CalcTextSize(label, nullptr, false, labelWidth);
            const float rowHeight = std::max(S(29.0f), labelSize.y + S(10.0f));

            ImGui::InvisibleButton(
                id,
                ImVec2(
                    available,
                    rowHeight));

            if (ImGui::IsItemClicked())
                value = !value;

            float& hover =
                gHover[
                    ImGui::GetItemID()];

            hover =
                Animate(
                    hover,
                    ImGui::IsItemHovered()
                    ? 1.0f
                    : 0.0f,
                    14.0f);

            ImDrawList* draw =
                ImGui::GetWindowDrawList();

            if (hover > 0.01f)
            {
                draw->AddRectFilled(
                    ImVec2(
                        start.x - S(4.0f),
                        start.y),
                    ImVec2(
                        start.x + available,
                        start.y + rowHeight),
                    U32(
                        C(
                            67,
                            67,
                            67,
                            static_cast<int>(
                                60.0f *
                                hover))),
                    S(3.0f));
            }

            ImFont* font =
                gRegularFont
                ? gRegularFont
                : ImGui::GetFont();

            const float fontSize =
                ImGui::GetFontSize();

            draw->AddText(
                font,
                fontSize,
                ImVec2(
                    start.x,
                    start.y +
                    (rowHeight - labelSize.y) *
                    0.5f),
                U32(
                    LerpColor(
                        C(201, 201, 201),
                        C(230, 230, 230),
                        hover)),
                label, nullptr, labelWidth);

            ImGui::SetCursorScreenPos(
                ImVec2(
                    start.x +
                    available -
                    S(34.0f),
                    start.y +
                    (rowHeight - S(18.0f)) *
                    0.5f));

            Toggle(
                id,
                value);

            ImGui::SetCursorScreenPos(
                ImVec2(
                    start.x,
                    start.y +
                    rowHeight));
        }

        [[nodiscard]] std::string TrimMappingAlias(
            const std::string& value)
        {
            std::size_t first = 0;
            while (first < value.size() &&
                std::isspace(
                    static_cast<unsigned char>(value[first])))
            {
                ++first;
            }

            std::size_t last = value.size();
            while (last > first &&
                std::isspace(
                    static_cast<unsigned char>(value[last - 1])))
            {
                --last;
            }

            return value.substr(first, last - first);
        }

        [[nodiscard]] std::vector<std::string> SplitMappingAliases(
            const std::string& value)
        {
            std::vector<std::string> result;
            std::string current;
            current.reserve(value.size());

            const auto flush = [&result, &current]()
            {
                std::string alias = TrimMappingAlias(current);
                if (!alias.empty())
                    result.push_back(std::move(alias));
                current.clear();
            };

            for (const char ch : value)
            {
                if (ch == ',' || ch == ';' || ch == '|' ||
                    ch == '\n' || ch == '\r')
                {
                    flush();
                    continue;
                }

                current.push_back(ch);
            }

            flush();
            return result;
        }

        [[nodiscard]] std::string SpotAliasesSummary(
            const State& state,
            const ColorMapEntry& mapping)
        {
            const std::string raw(mapping.sourceSpotName.data());
            const auto aliases = SplitMappingAliases(raw);

            if (aliases.empty())
            {
                return std::string(
                    T(
                        state,
                        "<Spot sem nome>",
                        "<Unnamed Spot>"));
            }

            std::string result;
            const std::size_t shown =
                (std::min)(aliases.size(), std::size_t{ 3 });

            for (std::size_t i = 0; i < shown; ++i)
            {
                if (!result.empty())
                    result += " + ";

                result += aliases[i];
            }

            if (aliases.size() > shown)
            {
                result += " +";
                result += std::to_string(aliases.size() - shown);
            }

            return result;
        }

        [[nodiscard]] std::string MappingSourceSummary(
            const State& state,
            const ColorMapEntry& mapping)
        {
            switch (mapping.sourceKind)
            {
            case ColorMapSourceKind::RGB:
                return
                    "RGB " +
                    std::to_string(mapping.sourceChannels[0]) +
                    ", " +
                    std::to_string(mapping.sourceChannels[1]) +
                    ", " +
                    std::to_string(mapping.sourceChannels[2]);

            case ColorMapSourceKind::CMYK:
                return
                    "CMYK " +
                    std::to_string(mapping.sourceChannels[0]) +
                    ", " +
                    std::to_string(mapping.sourceChannels[1]) +
                    ", " +
                    std::to_string(mapping.sourceChannels[2]) +
                    ", " +
                    std::to_string(mapping.sourceChannels[3]);

            case ColorMapSourceKind::Spot:
            {
                const std::string name =
                    SpotAliasesSummary(
                        state,
                        mapping);

                return
                    "Spot " +
                    name +
                    "  @" +
                    std::to_string(mapping.sourceSpotTint) +
                    "%";
            }

            default:
                return "?";
            }
        }

        [[nodiscard]] std::string MappingTargetSummary(
            const State& state,
            const ColorMapEntry& mapping)
        {
            switch (mapping.targetKind)
            {
            case ColorMapTargetKind::RGB:
                return
                    "RGB " +
                    std::to_string(mapping.targetRgb[0]) +
                    ", " +
                    std::to_string(mapping.targetRgb[1]) +
                    ", " +
                    std::to_string(mapping.targetRgb[2]);

            case ColorMapTargetKind::Spot:
            {
                const std::string name =
                    mapping.targetSpotName[0] != '\0'
                    ? std::string(mapping.targetSpotName.data())
                    : std::string(
                        T(
                            state,
                            "<Spot sem nome>",
                            "<Unnamed Spot>"));

                return
                    "Spot " +
                    name +
                    "  @" +
                    std::to_string(mapping.targetSpotTint) +
                    "%";
            }

            default:
                return "?";
            }
        }

        [[nodiscard]] std::string MappingSearchKey(
            const std::string& value)
        {
            std::string key;
            key.reserve(value.size());

            for (const unsigned char ch : value)
            {
                if (std::isalnum(ch))
                {
                    key.push_back(
                        static_cast<char>(
                            std::toupper(ch)));
                }
            }

            return key;
        }

        [[nodiscard]] static std::uint64_t MappingDisplayFingerprint(
            const ColorMapEntry& mapping) noexcept
        {
            std::uint64_t hash =
                14695981039346656037ull;

            const auto mixBytes =
                [&hash](
                    const void* data,
                    std::size_t size)
                {
                    const auto* bytes =
                        static_cast<const unsigned char*>(data);

                    for (std::size_t i = 0; i < size; ++i)
                    {
                        hash ^= bytes[i];
                        hash *= 1099511628211ull;
                    }
                };

            const auto mixString =
                [&mixBytes](const char* value)
                {
                    if (!value)
                        return;

                    const std::string_view view(value);
                    mixBytes(
                        view.data(),
                        view.size());
                };

            mixBytes(&mapping.enabled, sizeof(mapping.enabled));
            mixBytes(&mapping.sourceKind, sizeof(mapping.sourceKind));
            mixBytes(mapping.sourceChannels.data(), sizeof(mapping.sourceChannels));
            mixString(mapping.sourceSpotPalette.data());
            mixString(mapping.sourceSpotName.data());
            mixBytes(&mapping.sourceSpotTint, sizeof(mapping.sourceSpotTint));
            mixBytes(&mapping.targetKind, sizeof(mapping.targetKind));
            mixBytes(mapping.targetRgb.data(), sizeof(mapping.targetRgb));
            mixString(mapping.targetSpotPalette.data());
            mixString(mapping.targetSpotName.data());
            mixBytes(&mapping.targetSpotTint, sizeof(mapping.targetSpotTint));

            return hash;
        }

        [[nodiscard]] const MappingDisplayCache& GetMappingDisplayCache(
            const State& state,
            const ColorMapEntry& mapping,
            std::size_t index)
        {
            if (gMappingDisplayCache.size() <= index)
            {
                gMappingDisplayCache.resize(index + 1);
            }

            MappingDisplayCache& cache =
                gMappingDisplayCache[index];

            const std::uint64_t fingerprint =
                MappingDisplayFingerprint(mapping);

            if (cache.fingerprint == fingerprint &&
                cache.index == index &&
                cache.language == state.language)
            {
                return cache;
            }

            cache.fingerprint = fingerprint;
            cache.index = index;
            cache.language = state.language;
            cache.sourceSummary =
                MappingSourceSummary(
                    state,
                    mapping);
            cache.targetSummary =
                MappingTargetSummary(
                    state,
                    mapping);

            cache.headerLabel =
                std::to_string(index + 1) +
                ".  " +
                cache.sourceSummary +
                "  ->  " +
                cache.targetSummary;

            if (!mapping.enabled)
            {
                cache.headerLabel =
                    std::string(
                        T(
                            state,
                            "[DESATIVADA]  ",
                            "[DISABLED]  ")) +
                    cache.headerLabel;
            }

            cache.headerLabel +=
                "###MappingHeader";

            cache.searchKey =
                MappingSearchKey(
                    cache.sourceSummary +
                    " " +
                    cache.targetSummary +
                    " " +
                    std::string(mapping.sourceSpotName.data()) +
                    " " +
                    std::string(mapping.sourceSpotPalette.data()) +
                    " " +
                    std::string(mapping.targetSpotPalette.data()));

            return cache;
        }


        [[nodiscard]] const char* StatusText(
            const State& state)
        {
            if (state.runtime.busy)
            {
                if (state.runtime.status.rfind(
                    "Processando ",
                    0) == 0)
                {
                    return state.runtime.status.c_str();
                }

                return T(
                    state,
                    "Processando...",
                    "Processing...");
            }

            if (state.runtime.status.rfind(
                "Bleed:",
                0) == 0)
            {
                return T(
                    state,
                    "Sangria concluída",
                    "Bleed complete");
            }

            if (state.runtime.status.rfind(
                "Converted ",
                0) == 0)
            {
                return T(
                    state,
                    "Conversão concluída",
                    "Conversion complete");
            }

            return T(
                state,
                "Pronto",
                "Ready");
        }

        void DrawStatus(
            const State& state)
        {
            const float height =
                S(34.0f);

            ImGui::PushStyleColor(
                ImGuiCol_ChildBg,
                C(42, 42, 42));

            ImGui::BeginChild(
                "##StatusHeader",
                ImVec2(
                    0.0f,
                    height),
                ImGuiChildFlags_None,
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoScrollWithMouse);

            const ImVec2 origin =
                ImGui::GetWindowPos();

            const char* status =
                StatusText(state);

            const ImVec2 statusSize =
                ImGui::CalcTextSize(
                    status);

            const float right =
                ImGui::GetWindowWidth() -
                S(16.0f);

            const float dotX =
                right -
                statusSize.x -
                S(11.0f);

            ImDrawList* draw =
                ImGui::GetWindowDrawList();

            draw->AddCircleFilled(
                ImVec2(
                    origin.x + dotX,
                    origin.y +
                    height *
                    0.5f),
                S(3.0f),
                U32(
                    state.runtime.busy
                    ? C(211, 157, 79)
                    : C(84, 174, 111)));

            draw->AddText(
                ImVec2(
                    origin.x +
                    dotX +
                    S(9.0f),
                    origin.y +
                    (height - statusSize.y) *
                    0.5f),
                U32(
                    C(
                        163,
                        163,
                        163)),
                status);

            draw->AddLine(
                ImVec2(
                    origin.x,
                    origin.y +
                    height -
                    1.0f),
                ImVec2(
                    origin.x +
                    ImGui::GetWindowWidth(),
                    origin.y +
                    height -
                    1.0f),
                U32(
                    C(
                        66,
                        66,
                        66)));

            ImGui::EndChild();
            ImGui::PopStyleColor();
        }

        [[nodiscard]] int SelectedTabIndex(
            Page page) noexcept
        {
            for (int i = 0;
                i < TabCount;
                ++i)
            {
                if (TabPages[i] == page)
                    return i;
            }

            return 0;
        }

        void DrawTabs(
            State& state)
        {
            const float height =
                S(42.0f);

            ImGui::PushStyleColor(
                ImGuiCol_ChildBg,
                C(38, 38, 38));

            ImGui::BeginChild(
                "##Tabs",
                ImVec2(
                    0.0f,
                    height),
                ImGuiChildFlags_None,
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoScrollWithMouse);

            const char* labelsPt[TabCount]
            {
                "CORTE",
                "SANGRIA",
                "COR",
                "CONFIG"
            };

            const char* labelsEn[TabCount]
            {
                "CUT",
                "BLEED",
                "COLOR",
                "SETTINGS"
            };

            const char** labels =
                Pt(state)
                ? labelsPt
                : labelsEn;

            const float width =
                ImGui::GetWindowWidth();

            const float tabWidth =
                width /
                static_cast<float>(
                    TabCount);

            const int selected =
                SelectedTabIndex(
                    state.page);

            gTabPosition =
                Animate(
                    gTabPosition,
                    static_cast<float>(
                        selected),
                    17.0f);

            for (int i = 0;
                i < TabCount;
                ++i)
            {
                ImGui::SetCursorPos(
                    ImVec2(
                        tabWidth *
                        static_cast<float>(
                            i),
                        0.0f));

                ImGui::PushID(i);

                const bool clicked =
                    ImGui::InvisibleButton(
                        "##Tab",
                        ImVec2(
                            tabWidth,
                            height));

                float& hover =
                    gHover[
                        ImGui::GetItemID()];

                hover =
                    Animate(
                        hover,
                        ImGui::IsItemHovered()
                        ? 1.0f
                        : 0.0f,
                        15.0f);

                const ImVec2 itemMin =
                    ImGui::GetItemRectMin();

                ImDrawList* draw =
                    ImGui::GetWindowDrawList();

                if (hover > 0.01f)
                {
                    draw->AddRectFilled(
                        itemMin,
                        ImGui::GetItemRectMax(),
                        U32(
                            C(
                                65,
                                65,
                                65,
                                static_cast<int>(
                                    65.0f *
                                    hover))));
                }

                FontScope font(
                    i == selected
                    ? gSemiboldFont
                    : gRegularFont);

                const ImVec2 textSize =
                    ImGui::CalcTextSize(
                        labels[i]);

                draw->AddText(
                    ImVec2(
                        itemMin.x +
                        (tabWidth - textSize.x) *
                        0.5f,
                        itemMin.y +
                        (height - textSize.y) *
                        0.5f -
                        S(1.0f)),
                    U32(
                        i == selected
                        ? C(232, 232, 232)
                        : LerpColor(
                            C(148, 148, 148),
                            C(202, 202, 202),
                            hover)),
                    labels[i]);

                if (clicked)
                    state.page =
                    TabPages[i];

                ImGui::PopID();
            }

            const ImVec2 origin =
                ImGui::GetWindowPos();

            const float centerX =
                origin.x +
                tabWidth *
                (gTabPosition + 0.5f);

            const float lineWidth =
                S(42.0f);

            ImGui::GetWindowDrawList()->AddRectFilled(
                ImVec2(
                    centerX -
                    lineWidth *
                    0.5f,
                    origin.y +
                    height -
                    S(2.0f)),
                ImVec2(
                    centerX +
                    lineWidth *
                    0.5f,
                    origin.y +
                    height),
                U32(
                    C(
                        39,
                        91,
                        132)));

            ImGui::EndChild();
            ImGui::PopStyleColor();
        }

        void DrawCut(
            State& state,
            const Callbacks& callbacks)
        {
            PageTitle(
                T(
                    state,
                    "Fechamento",
                    "Cut preparation"),
                T(
                    state,
                    "Cria pares PRINT / CUT alinhados.",
                    "Create aligned PRINT / CUT page pairs."));

            SectionTitle(
                T(
                    state,
                    "Fechamento",
                    "Closure"));

            FieldLabel(
                T(
                    state,
                    "Detecção",
                    "Detection"));

            ImGui::SetNextItemWidth(
                S(270.0f));

            const char* modesPt[]
            {
                "Automático",
                "Seleção inteira",
                "Cada grupo selecionado",
                "Cada objeto selecionado"
            };

            const char* modesEn[]
            {
                "Automatic",
                "Whole selection",
                "Each selected group",
                "Each selected object"
            };

            int closureMode =
                static_cast<int>(
                    state.cut.closureMode);

            if (ImGui::Combo(
                "##ClosureMode",
                &closureMode,
                Pt(state)
                ? modesPt
                : modesEn,
                4))
            {
                state.cut.closureMode =
                    static_cast<ClosureMode>(
                        closureMode);
            }

            if (state.runtime.detectedClosures > 0)
            {
                ImGui::SameLine(
                    0.0f,
                    S(12.0f));

                std::string detected =
                    std::to_string(
                        state.runtime.detectedClosures);

                detected +=
                    state.runtime.detectedClosures == 1
                    ? T(
                        state,
                        " fechamento",
                        " closure")
                    : T(
                        state,
                        " fechamentos",
                        " closures");

                TextMuted(
                    detected.c_str());
            }

            SectionTitle(
                T(
                    state,
                    "Saída",
                    "Output"));

            if (ImGui::BeginTable(
                "##CutOutput",
                3,
                ImGuiTableFlags_SizingStretchSame,
                ImVec2(
                    -1.0f,
                    0.0f)))
            {
                ImGui::TableSetupColumn(
                    "A",
                    ImGuiTableColumnFlags_WidthStretch,
                    1.0f);

                ImGui::TableSetupColumn(
                    "B",
                    ImGuiTableColumnFlags_WidthStretch,
                    1.0f);

                ImGui::TableSetupColumn(
                    "C",
                    ImGuiTableColumnFlags_WidthStretch,
                    1.0f);

                ImGui::TableNextColumn();

                FieldLabel(
                    T(
                        state,
                        "Largura da página",
                        "Page width"),
                    "mm");

                ImGui::SetNextItemWidth(
                    -1.0f);

                ImGui::InputInt(
                    "##PageWidth",
                    &state.cut.pageWidthMm,
                    0,
                    0);

                ImGui::TableNextColumn();

                FieldLabel(
                    T(
                        state,
                        "Marcas de registro",
                        "Registration marks"));

                ImGui::SetNextItemWidth(
                    -1.0f);

                TextMuted(T(state, "Automáticas", "Automatic"));

                ImGui::TableNextColumn();

                FieldLabel(
                    T(
                        state,
                        "Margem das marcas",
                        "Mark margin"),
                    "mm");

                ImGui::SetNextItemWidth(
                    -1.0f);

                ImGui::InputFloat(
                    "##RegistrationMargin",
                    &state.cut.registrationMarginMm,
                    0.0f,
                    0.0f,
                    "%.1f");
                ImGui::EndTable();
            }

            state.cut.pageWidthMm =
                std::max(
                    state.cut.pageWidthMm,
                    1);

            if (!std::isfinite(state.cut.registrationMarginMm))
                state.cut.registrationMarginMm = 5.0f;

            state.cut.registrationMarginMm =
                std::clamp(
                    state.cut.registrationMarginMm,
                    0.0f,
                    1000.0f);

            TextMuted(T(state,
                "A margem afasta as marcas da borda. A âncora inferior direita continua encostada.",
                "The margin moves marks from the edge. The bottom-right anchor stays flush."));

            SectionTitle(
                T(
                    state,
                    "Opções",
                    "Options"));

            OptionRow(
                "##NamePages",
                T(
                    state,
                    "Nomear páginas geradas",
                    "Name generated pages"),
                state.cut.namePages);

            OptionRow(
                "##Summary",
                T(
                    state,
                    "Mostrar resumo ao finalizar",
                    "Show summary after processing"),
                state.cut.showSummary);

            SectionTitle(
                T(
                    state,
                    "Ordem das páginas",
                    "Page order"));

            ImGui::PushStyleColor(
                ImGuiCol_Text,
                C(188, 188, 188));

            ImGui::TextUnformatted(
                "PRINT 1   >   CUT 1");

            ImGui::PopStyleColor();

            if (state.runtime.detectedClosures > 1)
            {
                ImGui::SameLine(
                    0.0f,
                    S(15.0f));

                const std::string more =
                    "x" +
                    std::to_string(
                        state.runtime.detectedClosures);

                TextMuted(
                    more.c_str());
            }

            const float buttonWidth =
                S(166.0f);

            ImGui::SetCursorPosX(
                ImGui::GetWindowContentRegionMax().x -
                buttonWidth);

            if (ActionButton(
                "prepareCut",
                T(
                    state,
                    "FECHAR",
                    "PREPARE CUT"),
                !state.runtime.busy,
                166.0f,
                PrimaryActionArmed(
                    Page::Cut)))
            {
                if (callbacks.runCut)
                    callbacks.runCut(
                        state.cut);
            }
        }

        void DrawBleed(
            State& state,
            const Callbacks& callbacks)
        {
            PageTitle(
                T(
                    state,
                    "Sangria automática",
                    "Auto Bleeding"),
                T(
                    state,
                    "Gera sangria ao redor da arte selecionada.",
                    "Generate bleed around the selected artwork."));

            SectionTitle(
                T(
                    state,
                    "Geometria",
                    "Geometry"));

            FieldLabel(
                T(
                    state,
                    "Distância da sangria",
                    "Bleed distance"),
                "mm");

            ImGui::SetNextItemWidth(
                S(180.0f));

            ImGui::InputFloat(
                "##BleedDistance",
                &state.bleed.distanceMm,
                0.0f,
                0.0f,
                "%.2f");

            if (!InputLimits::ValidBleed(state.bleed.distanceMm))
                TextMutedWrapped(T(state,
                    "Informe uma distancia entre 0,001 e 1000 mm.",
                    "Enter a distance between 0.001 and 1000 mm."));

            SectionTitle(
                T(
                    state,
                    "Opções",
                    "Options"));

            OptionRow(
                "##HiddenObjects",
                T(
                    state,
                    "Detectar objetos ocultos",
                    "Detect hidden objects"),
                state.bleed.detectHiddenObjects);

            OptionRow(
                "##Ungroup",
                T(
                    state,
                    "Desagrupar antes de processar",
                    "Ungroup before processing"),
                state.bleed.ungroupBeforeProcessing);

            if (!state.bleed.ungroupBeforeProcessing)
            {
                TextMutedWrapped(
                    T(
                        state,
                        "Mantém o grupo e usa a CutContour ou a base principal.",
                        "Keeps the group and uses its CutContour or main base."));
            }

            OptionRow(
                "##CreateCutContour",
                T(
                    state,
                    "Criar CutContour",
                    "Create CutContour"),
                state.bleed.createCutline);

            SectionTitle(
                T(
                    state,
                    "Processamento",
                    "Processing"));

            TextMutedWrapped(
                T(
                    state,
                    "Validação automática da geometria.",
                    "Automatic geometry validation."));

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(7.0f)));

            const float buttonWidth =
                S(166.0f);

            ImGui::SetCursorPosX(
                ImGui::GetWindowContentRegionMax().x -
                buttonWidth);

            if (ActionButton(
                "createBleed",
                T(
                    state,
                    "CRIAR SANGRIA",
                    "CREATE BLEED"),
                !state.runtime.busy,
                166.0f,
                PrimaryActionArmed(
                    Page::Bleed)))
            {
                if (callbacks.runBleed)
                    callbacks.runBleed(
                        state.bleed);
            }
        }

        void DrawColor(
            State& state,
            const Callbacks& callbacks)
        {
            PageTitle(
                T(
                    state,
                    "Conversor de cor ICC",
                    "ICC Color Converter"),
                T(
                    state,
                    "Converte as cores da seleção e aplica suas regras.",
                    "Convert selection colors and apply your rules."));

            SectionTitle(
                T(
                    state,
                    "Perfis",
                    "Profiles"));

            FieldLabel(
                T(
                    state,
                    "Origem CMYK",
                    "CMYK source"));

            ImGui::SetNextItemWidth(
                -1.0f);

            ComboStringVector(
                "##CmykProfile",
                state.color.selectedCmyk,
                state.color.cmykProfiles,
                T(
                    state,
                    "Selecione o perfil CMYK",
                    "Select CMYK profile"));

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(4.0f)));

            FieldLabel(
                T(
                    state,
                    "Destino RGB",
                    "RGB destination"));

            ImGui::SetNextItemWidth(
                -1.0f);

            ComboStringVector(
                "##RgbProfile",
                state.color.selectedRgb,
                state.color.rgbProfiles,
                T(
                    state,
                    "Selecione o perfil RGB",
                    "Select RGB profile"));

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(4.0f)));

            TextMuted(
                T(
                    state,
                    "O perfil de escala de cinza do documento é preservado. O ImCut altera somente RGB, CMYK e a intenção de renderização.",
                    "The document grayscale profile is preserved. ImCut changes only RGB, CMYK, and the rendering intent."));

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(3.0f)));

            if (state.runtime.busy)
                ImGui::BeginDisabled();

            const bool refresh =
                TextButton(
                    "refreshProfiles",
                    T(
                        state,
                        "Atualizar perfis",
                        "Refresh profiles"));

            if (state.runtime.busy)
                ImGui::EndDisabled();

            if (refresh &&
                callbacks.refreshColorProfiles)
            {
                callbacks.refreshColorProfiles();
            }

            SectionTitle(
                T(
                    state,
                    "Conversão",
                    "Conversion"));

            FieldLabel(
                T(
                    state,
                    "Método",
                    "Method"));

            const char* conversionModesPt[]
            {
                "ICC padrão (rápido)",
                "Preservar aparência"
            };

            const char* conversionModesEn[]
            {
                "Standard ICC (fast)",
                "Preserve appearance"
            };

            int conversionMode =
                static_cast<int>(
                    state.color.conversionMode);

            ImGui::SetNextItemWidth(-1.0f);

            if (ImGui::Combo(
                "##ColorConversionMode",
                &conversionMode,
                Pt(state)
                ? conversionModesPt
                : conversionModesEn,
                2))
            {
                state.color.conversionMode =
                    static_cast<ColorConversionMode>(
                        conversionMode);
            }

            const bool preserveAppearance =
                state.color.conversionMode ==
                ColorConversionMode::PreserveAppearance;

            if (preserveAppearance)
            {
                TextMuted(
                    T(
                        state,
                        "Busca o RGB mais próximo da cor original. Pode levar mais tempo.",
                        "Finds the closest RGB to the original color. May take longer."));

            }
            else
            {
                TextMuted(
                    T(
                        state,
                        "Usa a conversão ICC/LUT atual do ImCut.",
                        "Uses ImCut's current ICC/LUT conversion."));
            }

            const bool appearanceEngineActive =
                preserveAppearance ||
                (state.color.convertSpotsToRgb &&
                    state.color.preserveSpotAppearance);

            if (!appearanceEngineActive)
                ImGui::BeginDisabled();

            OptionRow(
                "##ExhaustiveAppearanceSearch",
                T(
                    state,
                    "Busca completa de RGB (lenta)",
                    "Full RGB search (slow)"),
                state.color.exhaustiveAppearanceSearch);

            if (!appearanceEngineActive)
                ImGui::EndDisabled();

            TextMutedWrapped(
                T(
                    state,
                    "Testa todas as cores RGB. É mais lento. Esc cancela.",
                    "Tests every RGB color. Slower. Esc cancels."));

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(4.0f)));

            if (preserveAppearance)
                ImGui::BeginDisabled();

            if (ImGui::BeginTable(
                "##ColorSettings",
                2,
                ImGuiTableFlags_SizingStretchSame))
            {
                ImGui::TableNextColumn();

                FieldLabel(
                    T(
                        state,
                        "Intenção de cor",
                        "Rendering intent"));

                const char* intentsPt[]
                {
                    "Perceptual",
                    "Colorimétrico relativo",
                    "Saturação",
                    "Colorimétrico absoluto"
                };

                const char* intentsEn[]
                {
                    "Perceptual",
                    "Relative colorimetric",
                    "Saturation",
                    "Absolute colorimetric"
                };

                int intent =
                    preserveAppearance
                    ? static_cast<int>(
                        RenderingIntent::AbsoluteColorimetric)
                    : static_cast<int>(
                        state.color.intent);

                ImGui::SetNextItemWidth(
                    -1.0f);

                if (ImGui::Combo(
                    "##RenderingIntent",
                    &intent,
                    Pt(state)
                    ? intentsPt
                    : intentsEn,
                    4))
                {
                    state.color.intent =
                        static_cast<RenderingIntent>(
                            intent);
                }

                ImGui::TableNextColumn();

                FieldLabel(
                    T(
                        state,
                        "Grade preferida",
                        "Preferred grid"));

                ImGui::SetNextItemWidth(
                    -1.0f);

                ImGui::InputInt(
                    "##PreferredGrid",
                    &state.color.preferredGrid,
                    0,
                    0);

                ImGui::EndTable();
            }

            if (preserveAppearance)
                ImGui::EndDisabled();

            state.color.preferredGrid =
                std::clamp(
                    state.color.preferredGrid,
                    2,
                    65);

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(5.0f)));

            OptionRow("##AssignDocumentProfiles",
                T(state, "Aplicar perfis ao documento inteiro", "Apply profiles to the entire document"),
                state.color.assignDocumentProfiles);
            if (state.color.assignDocumentProfiles)
                ImGui::TextWrapped("%s", T(state,
                    "Esta opcao pode alterar a aparencia de objetos fora da selecao.",
                    "This option can change the appearance of unselected objects."));

            if (preserveAppearance)
                ImGui::BeginDisabled();

            OptionRow(
                "##AdaptiveLut",
                T(
                    state,
                    "LUT 4D adaptativa",
                    "Adaptive 4D LUT"),
                state.color.adaptiveLut);

            if (preserveAppearance)
                ImGui::EndDisabled();

            OptionRow(
                "##ColorShowSummary",
                T(
                    state,
                    "Mostrar resumo da conversão",
                    "Show conversion summary"),
                state.color.showSummary);

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(2.0f)));

            SectionTitle(
                T(
                    state,
                    "Limite de preto",
                    "Black floor"));

            OptionRow(
                "##BlackFloorEnabled",
                T(
                    state,
                    "Limitar pretos muito escuros",
                    "Limit very dark blacks"),
                state.color.blackFloorEnabled);

            if (!state.color.blackFloorEnabled)
                ImGui::BeginDisabled();

            FieldLabel(
                T(
                    state,
                    "RGB mínimo",
                    "Minimum RGB"));

            ImGui::SetNextItemWidth(
                S(120.0f));

            ImGui::InputInt(
                "##BlackFloorRgb",
                &state.color.blackFloorRgb,
                1,
                5);

            state.color.blackFloorRgb =
                std::clamp(
                    state.color.blackFloorRgb,
                    0,
                    255);

            ImGui::SameLine(
                0.0f,
                S(9.0f));

            const std::string blackFloorPreview =
                "RGB " +
                std::to_string(state.color.blackFloorRgb) +
                ", " +
                std::to_string(state.color.blackFloorRgb) +
                ", " +
                std::to_string(state.color.blackFloorRgb);

            TextMuted(
                blackFloorPreview.c_str());

            if (!state.color.blackFloorEnabled)
                ImGui::EndDisabled();

            SectionTitle(
                T(
                    state,
                    "Cores Spot",
                    "Spot colors"));

            OptionRow(
                "##SpotTintWhiteEnabled",
                T(
                    state,
                    "Spots claros para branco",
                    "Convert low-Tint Spot colors to white"),
                state.color.spotTintWhiteEnabled);

            if (!state.color.spotTintWhiteEnabled)
                ImGui::BeginDisabled();

            FieldLabel(
                T(
                    state,
                    "Limite de Tint",
                    "Tint threshold"));

            ImGui::SetNextItemWidth(
                S(120.0f));

            ImGui::InputInt(
                "##SpotTintWhiteThreshold",
                &state.color.spotTintWhiteThreshold,
                1,
                5);

            state.color.spotTintWhiteThreshold =
                std::clamp(
                    state.color.spotTintWhiteThreshold,
                    0,
                    100);

            ImGui::SameLine(
                0.0f,
                S(9.0f));

            const std::string spotTintPreview =
                "Tint <= " +
                std::to_string(
                    state.color.spotTintWhiteThreshold) +
                "%  ->  RGB 255, 255, 255";

            TextMuted(
                spotTintPreview.c_str());

            if (!state.color.spotTintWhiteEnabled)
                ImGui::EndDisabled();

            TextMuted(
                T(
                    state,
                    "Esta regra tem prioridade sobre o Color Mapping e a conversão Spot para RGB.",
                    "This rule has priority over Color Mapping and Spot-to-RGB conversion."));

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(3.0f)));

            OptionRow(
                "##ConvertSpotsToRgb",
                T(
                    state,
                    "Converter Spot para RGB",
                    "Automatically convert Spot to RGB"),
                state.color.convertSpotsToRgb);

            if (!state.color.convertSpotsToRgb)
                ImGui::BeginDisabled();

            OptionRow(
                "##PreserveSpotAppearance",
                T(
                    state,
                    "Preservar aparência do Spot",
                    "Preserve Spot appearance"),
                state.color.preserveSpotAppearance);

            if (!state.color.convertSpotsToRgb)
                ImGui::EndDisabled();

            TextMutedWrapped(
                T(
                    state,
                    "Busca o RGB mais próximo do Spot original. Desativado: usa a conversão do Corel.",
                    "Matches RGB to the original Spot color. Off: uses Corel conversion."));

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(3.0f)));

            OptionRow(
                "##SpotBlacklistEnabled",
                T(
                    state,
                    "Proteger cores Spot",
                    "Protect Spot colors"),
                state.color.spotBlacklistEnabled);

            if (!state.color.spotBlacklistEnabled)
                ImGui::BeginDisabled();

            FieldLabel(
                T(
                    state,
                    "Cores Spot protegidas",
                    "Protected Spot colors"));

            ImGui::SetNextItemWidth(-1.0f);
            ImGui::InputTextMultiline(
                "##SpotBlacklistNames",
                state.color.spotBlacklistNames.data(),
                state.color.spotBlacklistNames.size(),
                ImVec2(
                    -1.0f,
                    S(66.0f)));

            if (!state.color.spotBlacklistEnabled)
                ImGui::EndDisabled();

            TextMutedWrapped(
                T(
                    state,
                    "Separe os nomes por vírgulas. Essas cores não serão alteradas.",
                    "Separate names with commas. These colors will stay unchanged."));

            SectionTitle(
                T(
                    state,
                    "Mapeamento de cores",
                    "Color mapping"));

            OptionRow(
                "##ColorMappingEnabled",
                T(
                    state,
                    "Aplicar mapeamentos antes da conversão ICC",
                    "Apply mappings before ICC conversion"),
                state.color.colorMappingEnabled);

            if (!state.color.colorMappingEnabled)
            {
                TextMuted(
                    T(
                        state,
                        "Regras desativadas. Você pode continuar editando.",
                        "Rules are off. You can still edit them."));
            }

            static std::array<char, 128> mappingFilter{};
            static int openMappingIndex = -1;

            if (state.runtime.busy)
                ImGui::BeginDisabled();

            if (TextButton(
                "addColorMapping",
                T(
                    state,
                    "+ Nova regra",
                    "+ New rule")))
            {
                if (state.color.colorMappings.size() < 64)
                {
                    state.color.colorMappings.emplace_back();
                    openMappingIndex =
                        static_cast<int>(
                            state.color.colorMappings.size() - 1);
                }
            }

            ImGui::SameLine(
                0.0f,
                S(12.0f));

            if (TextButton(
                "importColorMappings",
                T(
                    state,
                    "Importar",
                    "Import")))
            {
                if (callbacks.importColorMappings)
                    callbacks.importColorMappings();
            }

            ImGui::SameLine(
                0.0f,
                S(12.0f));

            if (TextButton(
                "exportColorMappings",
                T(
                    state,
                    "Exportar",
                    "Export")))
            {
                if (callbacks.exportColorMappings)
                    callbacks.exportColorMappings();
            }

            ImGui::SameLine(
                0.0f,
                S(12.0f));

            const std::string mappingCount =
                std::to_string(
                    state.color.colorMappings.size()) +
                T(
                    state,
                    " regra(s)",
                    " rule(s)");

            TextMuted(mappingCount.c_str());

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(3.0f)));

            if (ImGui::BeginTable(
                "##MappingOptions",
                2,
                ImGuiTableFlags_SizingStretchProp))
            {
                ImGui::TableSetupColumn(
                    "##ToleranceColumn",
                    ImGuiTableColumnFlags_WidthStretch,
                    0.42f);

                ImGui::TableSetupColumn(
                    "##SearchColumn",
                    ImGuiTableColumnFlags_WidthStretch,
                    0.58f);

                ImGui::TableNextColumn();

                FieldLabel(
                    T(
                        state,
                        "Tolerância RGB",
                        "RGB tolerance"));

                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputInt(
                    "##ColorMappingTolerance",
                    &state.color.colorMappingTolerance,
                    1,
                    5);

                state.color.colorMappingTolerance =
                    std::clamp(
                        state.color.colorMappingTolerance,
                        0,
                        255);

                ImGui::TableNextColumn();

                FieldLabel(
                    T(
                        state,
                        "Filtrar regras",
                        "Filter rules"));

                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputTextWithHint(
                    "##MappingFilter",
                    T(
                        state,
                        "Ex.: 021C, BLACK, RGB 28",
                        "E.g. 021C, BLACK, RGB 28"),
                    mappingFilter.data(),
                    mappingFilter.size());

                ImGui::EndTable();
            }

            TextMuted(
                T(
                    state,
                    "A ordem define a prioridade: a primeira regra compatível vence.",
                    "Order defines priority: the first matching rule wins."));

            if (state.runtime.busy)
                ImGui::EndDisabled();

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(3.0f)));

            int removeMapping = -1;
            int duplicateMapping = -1;
            int moveMappingFrom = -1;
            int moveMappingTo = -1;
            int visibleMappings = 0;

            static std::string previousMappingFilter;
            static std::string normalizedMappingFilter;

            if (previousMappingFilter !=
                mappingFilter.data())
            {
                previousMappingFilter =
                    mappingFilter.data();

                normalizedMappingFilter =
                    MappingSearchKey(
                        previousMappingFilter);
            }

            if (gMappingDisplayCache.size() !=
                state.color.colorMappings.size())
            {
                gMappingDisplayCache.resize(
                    state.color.colorMappings.size());
            }

            for (std::size_t i = 0;
                i < state.color.colorMappings.size();
                ++i)
            {
                auto& mapping =
                    state.color.colorMappings[i];

                const MappingDisplayCache& displayCache =
                    GetMappingDisplayCache(
                        state,
                        mapping,
                        i);

                if (!normalizedMappingFilter.empty() &&
                    displayCache.searchKey.find(
                        normalizedMappingFilter) ==
                        std::string::npos)
                {
                    continue;
                }

                ++visibleMappings;

                ImGui::PushID(
                    static_cast<int>(i));

                ColorTargetPreview targetPreview;

                if (openMappingIndex ==
                    static_cast<int>(i))
                {
                    ImGui::SetNextItemOpen(
                        true,
                        ImGuiCond_Always);
                }

                ImGui::PushStyleColor(
                    ImGuiCol_Header,
                    C(49, 49, 49));

                ImGui::PushStyleColor(
                    ImGuiCol_HeaderHovered,
                    C(57, 57, 57));

                ImGui::PushStyleColor(
                    ImGuiCol_HeaderActive,
                    C(62, 62, 62));

                const bool open =
                    ImGui::CollapsingHeader(
                        displayCache.headerLabel.c_str(),
                        ImGuiTreeNodeFlags_None);

                const ImVec2 headerMin =
                    ImGui::GetItemRectMin();
                const ImVec2 headerMax =
                    ImGui::GetItemRectMax();

                const bool previewVisible =
                    open ||
                    ImGui::IsRectVisible(
                        headerMin,
                        headerMax);

                if (previewVisible &&
                    callbacks.previewColorMappingTarget)
                {
                    targetPreview =
                        callbacks.previewColorMappingTarget(
                            mapping,
                            state.color);
                }

                const float chipWidth = S(34.0f);
                const float chipHeight = S(18.0f);
                const float chipRightPadding = S(8.0f);

                const ImVec2 chipMin(
                    headerMax.x - chipRightPadding - chipWidth,
                    headerMin.y +
                        (headerMax.y - headerMin.y - chipHeight) * 0.5f);

                const ImVec2 chipMax(
                    chipMin.x + chipWidth,
                    chipMin.y + chipHeight);

                auto* drawList = ImGui::GetWindowDrawList();

                if (targetPreview.valid)
                {
                    const ImVec4 chipColor(
                        static_cast<float>(targetPreview.srgb[0]) / 255.0f,
                        static_cast<float>(targetPreview.srgb[1]) / 255.0f,
                        static_cast<float>(targetPreview.srgb[2]) / 255.0f,
                        1.0f);

                    drawList->AddRectFilled(
                        chipMin,
                        chipMax,
                        ImGui::GetColorU32(chipColor),
                        S(3.0f));

                    drawList->AddRect(
                        chipMin,
                        chipMax,
                        ImGui::GetColorU32(C(105, 105, 105)),
                        S(3.0f));
                }
                else
                {
                    drawList->AddRectFilled(
                        chipMin,
                        chipMax,
                        ImGui::GetColorU32(C(40, 40, 40)),
                        S(3.0f));

                    drawList->AddRect(
                        chipMin,
                        chipMax,
                        ImGui::GetColorU32(C(90, 90, 90)),
                        S(3.0f));

                    drawList->AddLine(
                        chipMin,
                        chipMax,
                        ImGui::GetColorU32(C(115, 115, 115)));

                    drawList->AddLine(
                        ImVec2(chipMin.x, chipMax.y),
                        ImVec2(chipMax.x, chipMin.y),
                        ImGui::GetColorU32(C(115, 115, 115)));
                }

                if (ImGui::IsMouseHoveringRect(
                    chipMin,
                    chipMax,
                    true))
                {
                    ImGui::BeginTooltip();

                    if (targetPreview.valid)
                    {
                        ImGui::Text(
                            "sRGB %d, %d, %d",
                            targetPreview.srgb[0],
                            targetPreview.srgb[1],
                            targetPreview.srgb[2]);

                        if (!targetPreview.sourceProfile.empty())
                        {
                            ImGui::Text(
                                "%s: %s",
                                T(
                                    state,
                                    "Perfil de origem",
                                    "Source profile"),
                                targetPreview.sourceProfile.c_str());
                        }
                    }
                    else
                    {
                        ImGui::TextUnformatted(
                            T(
                                state,
                                "Preview indisponível",
                                "Preview unavailable"));
                    }

                    if (!targetPreview.message.empty())
                    {
                        ImGui::Separator();
                        ImGui::TextWrapped(
                            "%s",
                            targetPreview.message.c_str());
                    }

                    ImGui::EndTooltip();
                }

                ImGui::PopStyleColor(3);

                if (openMappingIndex ==
                    static_cast<int>(i))
                {
                    openMappingIndex = -1;
                }

                if (open)
                {
                    ImGui::Dummy(
                        ImVec2(
                            0.0f,
                            S(2.0f)));

                    if (state.runtime.busy)
                        ImGui::BeginDisabled();

                    ImGui::Checkbox(
                        T(
                            state,
                            "Ativa##MappingEnabled",
                            "Enabled##MappingEnabled"),
                        &mapping.enabled);

                    ImGui::SameLine(
                        0.0f,
                        S(16.0f));

                    if (TextButton(
                        "moveMappingUp",
                        T(
                            state,
                            "Subir",
                            "Move up")) &&
                        i > 0)
                    {
                        moveMappingFrom =
                            static_cast<int>(i);
                        moveMappingTo =
                            static_cast<int>(i - 1);
                    }

                    ImGui::SameLine(
                        0.0f,
                        S(10.0f));

                    if (TextButton(
                        "moveMappingDown",
                        T(
                            state,
                            "Descer",
                            "Move down")) &&
                        i + 1 <
                            state.color.colorMappings.size())
                    {
                        moveMappingFrom =
                            static_cast<int>(i);
                        moveMappingTo =
                            static_cast<int>(i + 1);
                    }

                    ImGui::SameLine(
                        0.0f,
                        S(10.0f));

                    if (TextButton(
                        "duplicateColorMapping",
                        T(
                            state,
                            "Duplicar",
                            "Duplicate")))
                    {
                        duplicateMapping =
                            static_cast<int>(i);
                    }

                    ImGui::SameLine(
                        0.0f,
                        S(10.0f));

                    if (TextButton(
                        "removeColorMapping",
                        T(
                            state,
                            "Remover",
                            "Remove")))
                    {
                        removeMapping =
                            static_cast<int>(i);
                    }

                    if (state.runtime.busy)
                        ImGui::EndDisabled();

                    ImGui::Dummy(
                        ImVec2(
                            0.0f,
                            S(2.0f)));

                    const bool disableEditor =
                        !mapping.enabled ||
                        state.runtime.busy;

                    if (disableEditor)
                        ImGui::BeginDisabled();

                    if (ImGui::BeginTable(
                        "##MappingEditor",
                        2,
                        ImGuiTableFlags_SizingStretchProp |
                        ImGuiTableFlags_BordersInnerV |
                        ImGuiTableFlags_PadOuterX))
                    {
                        ImGui::TableSetupColumn(
                            "##SourceEditor",
                            ImGuiTableColumnFlags_WidthStretch,
                            0.5f);

                        ImGui::TableSetupColumn(
                            "##TargetEditor",
                            ImGuiTableColumnFlags_WidthStretch,
                            0.5f);

                        ImGui::TableNextColumn();

                        FieldLabel(
                            T(
                                state,
                                "ORIGEM",
                                "SOURCE"));

                        const char* sourceKinds[]
                        {
                            "RGB",
                            "CMYK",
                            "Spot"
                        };

                        int sourceKind =
                            static_cast<int>(
                                mapping.sourceKind);

                        ImGui::SetNextItemWidth(-1.0f);
                        if (ImGui::Combo(
                            "##MappingSourceKind",
                            &sourceKind,
                            sourceKinds,
                            3))
                        {
                            mapping.sourceKind =
                                static_cast<ColorMapSourceKind>(
                                    sourceKind);
                        }

                        if (mapping.sourceKind ==
                            ColorMapSourceKind::RGB)
                        {
                            FieldLabel("RGB");
                            ImGui::SetNextItemWidth(-1.0f);
                            ImGui::InputInt3(
                                "##MappingSourceRgb",
                                mapping.sourceChannels.data());

                            for (int channel = 0;
                                channel < 3;
                                ++channel)
                            {
                                mapping.sourceChannels[
                                    static_cast<std::size_t>(channel)] =
                                    std::clamp(
                                        mapping.sourceChannels[
                                            static_cast<std::size_t>(channel)],
                                        0,
                                        255);
                            }
                        }
                        else if (mapping.sourceKind ==
                            ColorMapSourceKind::CMYK)
                        {
                            FieldLabel("CMYK");
                            ImGui::SetNextItemWidth(-1.0f);
                            ImGui::InputInt4(
                                "##MappingSourceCmyk",
                                mapping.sourceChannels.data());

                            for (int channel = 0;
                                channel < 4;
                                ++channel)
                            {
                                mapping.sourceChannels[
                                    static_cast<std::size_t>(channel)] =
                                    std::clamp(
                                        mapping.sourceChannels[
                                            static_cast<std::size_t>(channel)],
                                        0,
                                        100);
                            }
                        }
                        else
                        {
                            FieldLabel(
                                T(
                                    state,
                                    "Cores Spot de origem",
                                    "Source Spot colors"));

                            ImGui::SetNextItemWidth(-1.0f);
                            ImGui::InputTextWithHint(
                                "##MappingSourceSpotName",
                                T(
                                    state,
                                    "AMARELOHUSQ, 100C, ORANGE 021...",
                                    "AMARELOHUSQ, 100C, ORANGE 021..."),
                                mapping.sourceSpotName.data(),
                                mapping.sourceSpotName.size());

                            TextMutedWrapped(
                                T(
                                    state,
                                    "Aceita vários nomes separados por vírgulas. Ex.: AMARELOHUSQ, 100C.",
                                    "Accepts multiple names separated by commas. Example: AMARELOHUSQ, 100C."));

                            FieldLabel(
                                T(
                                    state,
                                    "Paleta (opcional)",
                                    "Palette (optional)"));

                            ImGui::SetNextItemWidth(-1.0f);
                            ImGui::InputTextWithHint(
                                "##MappingSourceSpotPalette",
                                T(
                                    state,
                                    "Vazio = qualquer paleta",
                                    "Empty = any palette"),
                                mapping.sourceSpotPalette.data(),
                                mapping.sourceSpotPalette.size());

                            FieldLabel(
                                T(
                                    state,
                                    "Tint",
                                    "Tint"));

                            ImGui::SetNextItemWidth(S(120.0f));
                            ImGui::InputInt(
                                "##MappingSourceSpotTint",
                                &mapping.sourceSpotTint,
                                1,
                                5);

                            mapping.sourceSpotTint =
                                std::clamp(
                                    mapping.sourceSpotTint,
                                    0,
                                    100);
                        }

                        ImGui::TableNextColumn();

                        FieldLabel(
                            T(
                                state,
                                "DESTINO",
                                "TARGET"));

                        const char* targetKinds[]
                        {
                            "RGB",
                            "Spot"
                        };

                        int targetKind =
                            static_cast<int>(
                                mapping.targetKind);

                        ImGui::SetNextItemWidth(-1.0f);
                        if (ImGui::Combo(
                            "##MappingTargetKind",
                            &targetKind,
                            targetKinds,
                            2))
                        {
                            mapping.targetKind =
                                static_cast<ColorMapTargetKind>(
                                    targetKind);
                        }

                        if (mapping.targetKind ==
                            ColorMapTargetKind::RGB)
                        {
                            FieldLabel("RGB");
                            ImGui::SetNextItemWidth(-1.0f);
                            ImGui::InputInt3(
                                "##MappingTargetRgb",
                                mapping.targetRgb.data());

                            for (int channel = 0;
                                channel < 3;
                                ++channel)
                            {
                                mapping.targetRgb[
                                    static_cast<std::size_t>(channel)] =
                                    std::clamp(
                                        mapping.targetRgb[
                                            static_cast<std::size_t>(channel)],
                                        0,
                                        255);
                            }
                        }
                        else
                        {
                            FieldLabel(
                                T(
                                    state,
                                    "Nome exato da Spot no Corel",
                                    "Exact Spot name in Corel"));

                            ImGui::SetNextItemWidth(-1.0f);
                            ImGui::InputText(
                                "##MappingTargetSpotName",
                                mapping.targetSpotName.data(),
                                mapping.targetSpotName.size());

                            FieldLabel(
                                T(
                                    state,
                                    "ID da paleta",
                                    "Palette ID"));

                            ImGui::SetNextItemWidth(-1.0f);
                            ImGui::InputText(
                                "##MappingTargetSpotPalette",
                                mapping.targetSpotPalette.data(),
                                mapping.targetSpotPalette.size());

                            TextMutedWrapped(
                                T(
                                    state,
                                    "O destino Spot usa SpotAssignByName e por isso precisa do nome/ID reais da paleta.",
                                    "Spot targets use SpotAssignByName, so the real palette name/ID is required."));

                            FieldLabel(
                                T(
                                    state,
                                    "Tint",
                                    "Tint"));

                            ImGui::SetNextItemWidth(S(120.0f));
                            ImGui::InputInt(
                                "##MappingTargetSpotTint",
                                &mapping.targetSpotTint,
                                1,
                                5);

                            mapping.targetSpotTint =
                                std::clamp(
                                    mapping.targetSpotTint,
                                    0,
                                    100);
                        }

                        ImGui::Dummy(
                            ImVec2(
                                0.0f,
                                S(4.0f)));

                        FieldLabel(
                            T(
                                state,
                                "PREVIEW DO DESTINO",
                                "TARGET PREVIEW"));

                        const ImVec2 previewSize(
                            S(48.0f),
                            S(24.0f));

                        if (targetPreview.valid)
                        {
                            const ImVec4 previewColor(
                                static_cast<float>(
                                    targetPreview.srgb[0]) / 255.0f,
                                static_cast<float>(
                                    targetPreview.srgb[1]) / 255.0f,
                                static_cast<float>(
                                    targetPreview.srgb[2]) / 255.0f,
                                1.0f);

                            ImGui::ColorButton(
                                "##MappingTargetPreview",
                                previewColor,
                                ImGuiColorEditFlags_NoTooltip |
                                ImGuiColorEditFlags_NoDragDrop,
                                previewSize);

                            const bool previewHovered =
                                ImGui::IsItemHovered();

                            ImGui::SameLine(
                                0.0f,
                                S(9.0f));

                            const std::string previewText =
                                "sRGB " +
                                std::to_string(
                                    targetPreview.srgb[0]) +
                                ", " +
                                std::to_string(
                                    targetPreview.srgb[1]) +
                                ", " +
                                std::to_string(
                                    targetPreview.srgb[2]);

                            TextMuted(
                                previewText.c_str());

                            if (previewHovered &&
                                (!targetPreview.message.empty() ||
                                    !targetPreview.sourceProfile.empty()))
                            {
                                ImGui::BeginTooltip();

                                ImGui::TextUnformatted(
                                    T(
                                        state,
                                        "Preview sRGB gerado com gerenciamento de cor.",
                                        "sRGB preview generated with color management."));

                                if (!targetPreview.sourceProfile.empty())
                                {
                                    ImGui::Text(
                                        "%s: %s",
                                        T(
                                            state,
                                            "Perfil RGB de origem",
                                            "Source RGB profile"),
                                        targetPreview.sourceProfile.c_str());
                                }

                                if (!targetPreview.message.empty())
                                {
                                    ImGui::Separator();
                                    ImGui::TextWrapped(
                                        "%s",
                                        targetPreview.message.c_str());
                                }

                                ImGui::EndTooltip();
                            }
                        }
                        else
                        {
                            ImGui::BeginDisabled();
                            ImGui::Button(
                                "##MappingTargetPreviewUnavailable",
                                previewSize);
                            ImGui::EndDisabled();

                            const bool previewHovered =
                                ImGui::IsItemHovered(
                                    ImGuiHoveredFlags_AllowWhenDisabled);

                            ImGui::SameLine(
                                0.0f,
                                S(9.0f));

                            TextMuted(
                                T(
                                    state,
                                    "Preview indisponível",
                                    "Preview unavailable"));

                            if (previewHovered &&
                                !targetPreview.message.empty())
                            {
                                ImGui::BeginTooltip();
                                ImGui::TextWrapped(
                                    "%s",
                                    targetPreview.message.c_str());
                                ImGui::EndTooltip();
                            }
                        }

                        ImGui::EndTable();
                    }

                    if (disableEditor)
                        ImGui::EndDisabled();

                    ImGui::Dummy(
                        ImVec2(
                            0.0f,
                            S(5.0f)));
                }

                ImGui::PopID();
            }

            if (visibleMappings == 0 &&
                !state.color.colorMappings.empty() &&
                mappingFilter[0] != '\0')
            {
                TextMuted(
                    T(
                        state,
                        "Nenhuma regra corresponde ao filtro.",
                        "No rules match the filter."));
            }

            if (removeMapping >= 0 &&
                removeMapping <
                    static_cast<int>(
                        state.color.colorMappings.size()))
            {
                state.color.colorMappings.erase(
                    state.color.colorMappings.begin() +
                    removeMapping);
            }
            else if (duplicateMapping >= 0 &&
                duplicateMapping <
                    static_cast<int>(
                        state.color.colorMappings.size()) &&
                state.color.colorMappings.size() < 64)
            {
#pragma warning(suppress: 26820)
                const auto copy =
                    state.color.colorMappings[
                        static_cast<std::size_t>(
                            duplicateMapping)];

                state.color.colorMappings.insert(
                    state.color.colorMappings.begin() +
                    duplicateMapping + 1,
                    copy);

                openMappingIndex =
                    duplicateMapping + 1;
            }
            else if (moveMappingFrom >= 0 &&
                moveMappingTo >= 0 &&
                moveMappingFrom <
                    static_cast<int>(
                        state.color.colorMappings.size()) &&
                moveMappingTo <
                    static_cast<int>(
                        state.color.colorMappings.size()))
            {
                std::swap(
                    state.color.colorMappings[
                        static_cast<std::size_t>(
                            moveMappingFrom)],
                    state.color.colorMappings[
                        static_cast<std::size_t>(
                            moveMappingTo)]);

                openMappingIndex =
                    moveMappingTo;
            }

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(8.0f)));
        }

        void DrawSettings(
            State& state,
            const Callbacks& callbacks)
        {
            PageTitle(
                T(
                    state,
                    "Configurações",
                    "Settings"),
                T(
                    state,
                    "As alterações são salvas automaticamente.",
                    "Changes are saved automatically."));

            SectionTitle(
                T(
                    state,
                    "Interface",
                    "Interface"));

            FieldLabel(
                T(
                    state,
                    "Idioma",
                    "Language"));

            int language =
                static_cast<int>(
                    state.language);

            const char* languages[]
            {
                "Português",
                "English"
            };

            ImGui::SetNextItemWidth(
                S(230.0f));

            if (ImGui::Combo(
                "##Language",
                &language,
                languages,
                2))
            {
                state.language =
                    static_cast<Language>(
                        language);
            }

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(8.0f)));

            if (ImGui::BeginTable(
                "##SettingsInfo",
                2,
                ImGuiTableFlags_SizingFixedFit |
                ImGuiTableFlags_NoSavedSettings))
            {
                ImGui::TableSetupColumn(
                    "Key",
                    ImGuiTableColumnFlags_WidthFixed,
                    S(180.0f));

                ImGui::TableSetupColumn(
                    "Value",
                    ImGuiTableColumnFlags_WidthStretch);

                ImGui::TableNextColumn();
                ImGui::TextUnformatted(
                    T(
                        state,
                        "Salvamento",
                        "Saving"));

                ImGui::TableNextColumn();
                TextMuted(
                    T(
                        state,
                        "Automático",
                        "Automatic"));

                ImGui::TableNextColumn();
                ImGui::TextUnformatted(
                    T(
                        state,
                        "Conexão",
                        "Connection"));

                ImGui::TableNextColumn();
                TextMuted(
                    state.runtime.busy
                    ? T(
                        state,
                        "Processando",
                        "Processing")
                    : T(
                        state,
                        "Conectado",
                        "Connected"));

                ImGui::TableNextColumn();
                ImGui::TextUnformatted(
                    T(
                        state,
                        "Fechamentos detectados",
                        "Detected closures"));

                ImGui::TableNextColumn();

                const std::string closures =
                    std::to_string(
                        state.runtime.detectedClosures);

                TextMuted(
                    closures.c_str());

                ImGui::EndTable();
            }

            SectionTitle(
                T(
                    state,
                    "Importar / exportar",
                    "Import / export"));

            TextMutedWrapped(
                T(
                    state,
                    "Salva ou carrega todas as opções, perfis e regras de cores.",
                    "Save or load all options, profiles and color rules."));

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(8.0f)));

            if (ActionButton(
                "ImportImCutSettings",
                T(
                    state,
                    "IMPORTAR",
                    "IMPORT SETTINGS"),
                !state.runtime.busy,
                210.0f) &&
                callbacks.importSettings)
            {
                callbacks.importSettings();
            }

            ImGui::SameLine();

            if (ActionButton(
                "ExportImCutSettings",
                T(
                    state,
                    "EXPORTAR",
                    "EXPORT SETTINGS"),
                !state.runtime.busy,
                210.0f) &&
                callbacks.exportSettings)
            {
                callbacks.exportSettings();
            }

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(3.0f)));

            TextMuted(
                T(
                    state,
                    "Formato: .imcutcfg  |  Color Mapping incluído",
                    "Format: .imcutcfg  |  Color Mapping included"));

            SectionTitle(
                T(
                    state,
                    "Atualizações",
                    "Updates"));

            if (ImGui::BeginTable(
                "##UpdateInfo",
                2,
                ImGuiTableFlags_SizingFixedFit |
                ImGuiTableFlags_NoSavedSettings))
            {
                ImGui::TableSetupColumn(
                    "Key",
                    ImGuiTableColumnFlags_WidthFixed,
                    S(180.0f));

                ImGui::TableSetupColumn(
                    "Value",
                    ImGuiTableColumnFlags_WidthStretch);

                ImGui::TableNextColumn();
                ImGui::TextUnformatted(
                    T(
                        state,
                        "Versão instalada",
                        "Installed version"));

                ImGui::TableNextColumn();
                TextMuted(
                    state.update.currentVersion.empty()
                    ? "-"
                    : state.update.currentVersion.c_str());

                if (!state.update.remoteVersion.empty())
                {
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(
                        T(
                            state,
                            "Versão no servidor",
                            "Server version"));

                    ImGui::TableNextColumn();
                    TextMuted(
                        state.update.remoteVersion.c_str());
                }

                ImGui::TableNextColumn();
                ImGui::TextUnformatted(
                    T(
                        state,
                        "Status",
                        "Status"));

                ImGui::TableNextColumn();

                switch (state.update.status)
                {
                case UpdateStatus::Checking:
                    TextMuted(
                        T(
                            state,
                            "Verificando...",
                            "Checking..."));
                    break;
                case UpdateStatus::UpToDate:
                    TextMuted(
                        T(
                            state,
                            "Atualizado",
                            "Up to date"));
                    break;
                case UpdateStatus::UpdateAvailable:
                    ImGui::PushStyleColor(
                        ImGuiCol_Text,
                        C(109, 190, 123));
                    ImGui::TextUnformatted(
                        T(
                            state,
                            "Nova versão disponível",
                            "New version available"));
                    ImGui::PopStyleColor();
                    break;
                case UpdateStatus::ResolvingDownload:
                    TextMuted(
                        T(
                            state,
                            "Obtendo link...",
                            "Resolving download..."));
                    break;
                case UpdateStatus::Downloading:
                    TextMuted(
                        T(
                            state,
                            "Baixando...",
                            "Downloading..."));
                    break;
                case UpdateStatus::Downloaded:
                    ImGui::PushStyleColor(
                        ImGuiCol_Text,
                        C(109, 190, 123));
                    ImGui::TextUnformatted(
                        T(
                            state,
                            "Pronto para instalar",
                            "Ready to install"));
                    ImGui::PopStyleColor();
                    break;
                case UpdateStatus::Error:
                    ImGui::PushStyleColor(
                        ImGuiCol_Text,
                        C(220, 112, 112));
                    ImGui::TextUnformatted(
                        T(
                            state,
                            "Falha",
                            "Failed"));
                    ImGui::PopStyleColor();
                    break;
                case UpdateStatus::Idle:
                default:
                    TextMuted(
                        T(
                            state,
                            "Aguardando",
                            "Idle"));
                    break;
                }

                ImGui::EndTable();
            }

            if (!state.update.message.empty())
            {
                ImGui::Dummy(
                    ImVec2(
                        0.0f,
                        S(5.0f)));

                if (state.update.status == UpdateStatus::Error)
                {
                    ImGui::PushStyleColor(
                        ImGuiCol_Text,
                        C(220, 112, 112));
                    ImGui::TextWrapped(
                        "%s",
                        state.update.message.c_str());
                    ImGui::PopStyleColor();
                }
                else
                {
                    TextMutedWrapped(
                        state.update.message.c_str());
                }
            }

            if (state.update.status == UpdateStatus::Downloading)
            {
                ImGui::Dummy(
                    ImVec2(
                        0.0f,
                        S(5.0f)));

                ImGui::ProgressBar(
                    std::clamp(
                        state.update.progress,
                        0.0f,
                        1.0f),
                    ImVec2(
                        -1.0f,
                        S(8.0f)),
                    "");
            }

            ImGui::Dummy(
                ImVec2(
                    0.0f,
                    S(8.0f)));

            const bool updateBusy =
                state.update.status == UpdateStatus::Checking ||
                state.update.status == UpdateStatus::ResolvingDownload ||
                state.update.status == UpdateStatus::Downloading;

            if (state.update.status == UpdateStatus::UpdateAvailable)
            {
                if (ActionButton(
                    "DownloadUpdate",
                    T(
                        state,
                        "BAIXAR ATUALIZAÇÃO",
                        "DOWNLOAD UPDATE"),
                    !updateBusy,
                    190.0f) &&
                    callbacks.downloadUpdate)
                {
                    callbacks.downloadUpdate();
                }

                ImGui::SameLine();

                if (ActionButton(
                    "RecheckUpdate",
                    T(
                        state,
                        "VERIFICAR NOVAMENTE",
                        "CHECK AGAIN"),
                    !updateBusy,
                    190.0f) &&
                    callbacks.checkForUpdates)
                {
                    callbacks.checkForUpdates();
                }
            }
            else if (state.update.status == UpdateStatus::Downloaded)
            {
                if (ActionButton(
                    "InstallUpdate",
                    T(
                        state,
                        "INSTALAR ATUALIZAÇÃO",
                        "INSTALL UPDATE"),
                    true,
                    190.0f) &&
                    callbacks.openDownloadedUpdate)
                {
                    callbacks.openDownloadedUpdate();
                }

                ImGui::SameLine();

                if (ActionButton(
                    "RecheckDownloadedUpdate",
                    T(
                        state,
                        "VERIFICAR NOVAMENTE",
                        "CHECK AGAIN"),
                    true,
                    190.0f) &&
                    callbacks.checkForUpdates)
                {
                    callbacks.checkForUpdates();
                }
            }
            else if (state.update.status == UpdateStatus::Error &&
                     state.update.updateAvailable)
            {
                if (ActionButton(
                    "RetryUpdateDownload",
                    T(
                        state,
                        "TENTAR DOWNLOAD",
                        "RETRY DOWNLOAD"),
                    !updateBusy,
                    190.0f) &&
                    callbacks.downloadUpdate)
                {
                    callbacks.downloadUpdate();
                }

                ImGui::SameLine();

                if (ActionButton(
                    "RetryUpdateCheck",
                    T(
                        state,
                        "VERIFICAR NOVAMENTE",
                        "CHECK AGAIN"),
                    !updateBusy,
                    190.0f) &&
                    callbacks.checkForUpdates)
                {
                    callbacks.checkForUpdates();
                }
            }
            else
            {
                if (ActionButton(
                    "CheckUpdate",
                    updateBusy
                    ? T(
                        state,
                        "VERIFICANDO...",
                        "CHECKING...")
                    : T(
                        state,
                        "VERIFICAR AGORA",
                        "CHECK NOW"),
                    !updateBusy,
                    190.0f) &&
                    callbacks.checkForUpdates)
                {
                    callbacks.checkForUpdates();
                }
            }
        }

        void DrawUpdatePopup(
            State& state,
            const Callbacks& callbacks)
        {
            if (state.update.status == UpdateStatus::UpdateAvailable &&
                !state.update.remoteVersion.empty() &&
                gPromptedUpdateVersion != state.update.remoteVersion)
            {
                gPromptedUpdateVersion = state.update.remoteVersion;
                ImGui::OpenPopup("##ImCutUpdateAvailable");
            }

            ImGui::SetNextWindowPos(
                ImGui::GetMainViewport()->GetCenter(),
                ImGuiCond_Appearing,
                ImVec2(0.5f, 0.5f));

            ImGui::SetNextWindowSize(
                ImVec2(
                    S(540.0f),
                    S(322.0f)),
                ImGuiCond_Appearing);

            ImGui::PushStyleVar(
                ImGuiStyleVar_WindowPadding,
                ImVec2(
                    S(24.0f),
                    S(22.0f)));

            ImGui::PushStyleVar(
                ImGuiStyleVar_ItemSpacing,
                ImVec2(
                    S(8.0f),
                    S(10.0f)));

            const bool open = ImGui::BeginPopupModal(
                "##ImCutUpdateAvailable",
                nullptr,
                ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoSavedSettings |
                ImGuiWindowFlags_NoTitleBar |
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoScrollWithMouse);

            if (!open)
            {
                ImGui::PopStyleVar(2);
                return;
            }

            const ImVec2 titlePos =
                ImGui::GetCursorScreenPos();

            ImGui::GetWindowDrawList()->AddRectFilled(
                titlePos,
                ImVec2(
                    titlePos.x + S(4.0f),
                    titlePos.y + S(44.0f)),
                U32(C(46, 112, 160)),
                S(2.0f));

            ImGui::SetCursorPosX(
                ImGui::GetCursorPosX() + S(14.0f));

            {
                FontScope font(gSemiboldFont);
                ImGui::SetWindowFontScale(1.13f);
                ImGui::TextUnformatted(
                    T(
                        state,
                        "Atualização do ImCut disponível",
                        "ImCut update available"));
                ImGui::SetWindowFontScale(1.0f);
            }

            ImGui::SetCursorPosX(
                ImGui::GetCursorPosX() + S(14.0f));

            TextMutedWrapped(
                T(
                    state,
                    "Uma versão mais recente está pronta para download.",
                    "A newer version is ready to download."));

            ImGui::Dummy(ImVec2(0.0f, S(6.0f)));

            ImGui::PushStyleColor(
                ImGuiCol_ChildBg,
                C(36, 36, 36));

            ImGui::PushStyleVar(
                ImGuiStyleVar_ChildRounding,
                S(6.0f));

            ImGui::PushStyleVar(
                ImGuiStyleVar_WindowPadding,
                ImVec2(
                    S(18.0f),
                    S(14.0f)));

            if (ImGui::BeginChild(
                "##UpdateVersionCard",
                ImVec2(-1.0f, S(92.0f)),
                ImGuiChildFlags_AlwaysUseWindowPadding,
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoScrollWithMouse))
            {
                if (ImGui::BeginTable(
                    "##UpdateVersions",
                    2,
                    ImGuiTableFlags_SizingStretchSame |
                    ImGuiTableFlags_NoSavedSettings))
                {
                    ImGui::TableNextColumn();
                    TextMuted(
                        T(
                            state,
                            "VERSÃO INSTALADA",
                            "INSTALLED VERSION"));

                    {
                        FontScope font(gSemiboldFont);
                        ImGui::TextUnformatted(
                            state.update.currentVersion.empty()
                            ? "-"
                            : state.update.currentVersion.c_str());
                    }

                    ImGui::TableNextColumn();
                    ImGui::PushStyleColor(
                        ImGuiCol_Text,
                        C(109, 190, 123));
                    ImGui::TextUnformatted(
                        T(
                            state,
                            "NOVA VERSÃO",
                            "NEW VERSION"));
                    ImGui::PopStyleColor();

                    {
                        FontScope font(gSemiboldFont);
                        ImGui::PushStyleColor(
                            ImGuiCol_Text,
                            C(132, 205, 145));
                        ImGui::TextUnformatted(
                            state.update.remoteVersion.c_str());
                        ImGui::PopStyleColor();
                    }

                    ImGui::EndTable();
                }
            }

            ImGui::EndChild();
            ImGui::PopStyleVar(2);
            ImGui::PopStyleColor();

            ImGui::Dummy(ImVec2(0.0f, S(4.0f)));

            TextMutedWrapped(
                T(
                    state,
                    "O download acontece em segundo plano. Depois de concluído, o ImCut substitui a DLL e recarrega o macro sem fechar o CorelDRAW.",
                    "The download runs in the background. When it finishes, ImCut replaces the DLL and reloads the macro without closing CorelDRAW."));

            ImGui::Dummy(ImVec2(0.0f, S(5.0f)));

            if (ActionButton(
                "PopupDownloadUpdate",
                T(
                    state,
                    "BAIXAR ATUALIZAÇÃO",
                    "DOWNLOAD UPDATE"),
                true,
                288.0f))
            {
                if (callbacks.downloadUpdate)
                    callbacks.downloadUpdate();

                ImGui::CloseCurrentPopup();
            }

            ImGui::SameLine();

            if (ActionButton(
                "PopupLaterUpdate",
                T(
                    state,
                    "AGORA NÃO",
                    "NOT NOW"),
                true,
                196.0f))
            {
                ImGui::CloseCurrentPopup();
            }

            ImGui::EndPopup();
            ImGui::PopStyleVar(2);
        }

        void DrawColorActionBar(
            State& state,
            const Callbacks& callbacks)
        {
            const float height =
                S(58.0f);

            ImGui::PushStyleColor(
                ImGuiCol_ChildBg,
                C(42, 42, 42));

            ImGui::BeginChild(
                "##ColorActionBar",
                ImVec2(
                    0.0f,
                    height),
                ImGuiChildFlags_None,
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoScrollWithMouse);

            ImGui::SetCursorPos(
                ImVec2(
                    S(20.0f),
                    S(11.0f)));

            {
                FontScope font(gSemiboldFont);
                ImGui::TextUnformatted(
                    T(
                        state,
                        "Conversão da seleção",
                        "Selection conversion"));
            }

            ImGui::SetCursorPos(
                ImVec2(
                    S(20.0f),
                    S(31.0f)));

            TextMuted(
                T(
                    state,
                    "Usa as configurações abaixo.",
                    "Uses the settings below."));

            const bool canRun =
                !state.runtime.busy &&
                state.color.selectedCmyk >= 0 &&
                state.color.selectedRgb >= 0;

            const float buttonWidth =
                S(190.0f);

            ImGui::SetCursorPos(
                ImVec2(
                    ImGui::GetWindowWidth() -
                        buttonWidth -
                        S(20.0f),
                    S(10.0f)));

            if (ActionButton(
                "convertSelectionTop",
                state.runtime.busy
                    ? T(
                        state,
                        "CONVERTENDO...",
                        "CONVERTING...")
                    : T(
                        state,
                        "CONVERTER SELEÇÃO",
                        "CONVERT SELECTION"),
                canRun,
                190.0f))
            {
                if (callbacks.runColor)
                    callbacks.runColor(
                        state.color);
            }

            ImGui::EndChild();
            ImGui::PopStyleColor();
        }

        void DrawPage(
            State& state,
            const Callbacks& callbacks)
        {
            if (state.page == Page::Nesting)
                state.page = Page::Cut;

            if (state.page !=
                gVisiblePage)
            {
                gVisiblePage =
                    state.page;

                if (gPrimaryActionArmed &&
                    gPrimaryActionPage != state.page)
                {
                    CancelPrimaryAction();
                }

                gPageEnter = 0.0f;
                ImGui::SetScrollY(0.0f);
            }

            gPageEnter =
                Animate(
                    gPageEnter,
                    1.0f,
                    15.0f);

            const float alpha =
                0.78f +
                0.22f *
                gPageEnter;

            const float offset =
                S(4.0f) *
                (1.0f -
                    gPageEnter);

            ImGui::PushStyleVar(
                ImGuiStyleVar_Alpha,
                ImGui::GetStyle().Alpha *
                alpha);

            ImGui::SetCursorPosY(
                ImGui::GetCursorPosY() +
                offset);

            switch (state.page)
            {
            case Page::Cut:
                DrawCut(
                    state,
                    callbacks);
                break;

            case Page::Bleed:
                DrawBleed(
                    state,
                    callbacks);
                break;

            case Page::Color:
                DrawColor(
                    state,
                    callbacks);
                break;

            case Page::Settings:
                DrawSettings(
                    state,
                    callbacks);
                break;

            case Page::Nesting:
            default:
                state.page =
                    Page::Cut;

                DrawCut(
                    state,
                    callbacks);
                break;
            }

            ImGui::PopStyleVar();
        }
    }

    void RequestPrimaryActionFocus(
        Page page) noexcept
    {
        if (page != Page::Cut &&
            page != Page::Bleed)
        {
            CancelPrimaryAction();
            return;
        }

        gPrimaryActionPage = page;
        gPrimaryActionArmed = true;
        gPrimaryActionFocusPending = true;
    }

    void SetFonts(
        ImFont* regular,
        ImFont* semibold) noexcept
    {
        gRegularFont =
            regular;

        gSemiboldFont =
            semibold
            ? semibold
            : regular;
    }

    void ApplyCorelTheme(
        float dpiScale)
    {
        gScale =
            std::clamp(
                dpiScale,
                0.75f,
                2.50f);

        if (gHover.bucket_count() < 256)
        {
            gHover.reserve(256);
            gPress.reserve(96);
            gToggle.reserve(128);
            gMappingDisplayCache.reserve(64);
        }

        ImGuiStyle& style =
            ImGui::GetStyle();

        style.Alpha = 1.0f;
        style.DisabledAlpha = 0.48f;

        style.WindowPadding =
            ImVec2(
                0.0f,
                0.0f);

        style.FramePadding =
            ImVec2(
                S(9.0f),
                S(6.0f));

        style.CellPadding =
            ImVec2(
                S(7.0f),
                S(5.0f));

        style.ItemSpacing =
            ImVec2(
                S(7.0f),
                S(6.0f));

        style.ItemInnerSpacing =
            ImVec2(
                S(6.0f),
                S(4.0f));

        style.ScrollbarSize =
            S(9.0f);

        style.GrabMinSize =
            S(8.0f);

        style.WindowBorderSize = 0.0f;
        style.ChildBorderSize = 0.0f;
        style.PopupBorderSize = 1.0f;
        style.FrameBorderSize = 1.0f;

        style.WindowRounding = 0.0f;
        style.ChildRounding = 0.0f;
        style.FrameRounding = S(3.0f);
        style.PopupRounding = S(3.0f);
        style.ScrollbarRounding = S(3.0f);
        style.GrabRounding = S(3.0f);
        style.TabRounding = S(3.0f);

        ImVec4* colors =
            style.Colors;

        colors[ImGuiCol_Text] =
            C(217, 217, 217);

        colors[ImGuiCol_TextDisabled] =
            C(125, 125, 125);

        colors[ImGuiCol_WindowBg] =
            C(38, 38, 38);

        colors[ImGuiCol_ChildBg] =
            C(38, 38, 38);

        colors[ImGuiCol_PopupBg] =
            C(47, 47, 47);

        colors[ImGuiCol_Border] =
            C(74, 74, 74);

        colors[ImGuiCol_BorderShadow] =
            C(0, 0, 0, 0);

        colors[ImGuiCol_FrameBg] =
            C(58, 58, 58);

        colors[ImGuiCol_FrameBgHovered] =
            C(65, 65, 65);

        colors[ImGuiCol_FrameBgActive] =
            C(70, 70, 70);

        colors[ImGuiCol_TitleBg] =
            C(38, 38, 38);

        colors[ImGuiCol_TitleBgActive] =
            C(38, 38, 38);

        colors[ImGuiCol_TitleBgCollapsed] =
            C(38, 38, 38);

        colors[ImGuiCol_MenuBarBg] =
            C(42, 42, 42);

        colors[ImGuiCol_ScrollbarBg] =
            C(38, 38, 38);

        colors[ImGuiCol_ScrollbarGrab] =
            C(76, 76, 76);

        colors[ImGuiCol_ScrollbarGrabHovered] =
            C(91, 91, 91);

        colors[ImGuiCol_ScrollbarGrabActive] =
            C(104, 104, 104);

        colors[ImGuiCol_CheckMark] =
            C(238, 238, 238);

        colors[ImGuiCol_SliderGrab] =
            C(39, 91, 132);

        colors[ImGuiCol_SliderGrabActive] =
            C(46, 105, 151);

        colors[ImGuiCol_Button] =
            C(58, 58, 58);

        colors[ImGuiCol_ButtonHovered] =
            C(67, 67, 67);

        colors[ImGuiCol_ButtonActive] =
            C(72, 72, 72);

        colors[ImGuiCol_Header] =
            C(38, 87, 128);

        colors[ImGuiCol_HeaderHovered] =
            C(45, 101, 148);

        colors[ImGuiCol_HeaderActive] =
            C(47, 106, 154);

        colors[ImGuiCol_Separator] =
            C(67, 67, 67);

        colors[ImGuiCol_SeparatorHovered] =
            C(82, 82, 82);

        colors[ImGuiCol_SeparatorActive] =
            C(39, 91, 132);

        colors[ImGuiCol_ResizeGrip] =
            C(0, 0, 0, 0);

        colors[ImGuiCol_ResizeGripHovered] =
            C(0, 0, 0, 0);

        colors[ImGuiCol_ResizeGripActive] =
            C(0, 0, 0, 0);

        colors[ImGuiCol_Tab] =
            C(38, 38, 38);

        colors[ImGuiCol_TabHovered] =
            C(54, 54, 54);

        colors[ImGuiCol_TabActive] =
            C(47, 47, 47);

        colors[ImGuiCol_TabUnfocused] =
            C(38, 38, 38);

        colors[ImGuiCol_TabUnfocusedActive] =
            C(47, 47, 47);

        colors[ImGuiCol_TextSelectedBg] =
            C(39, 91, 132, 130);

        colors[ImGuiCol_NavHighlight] =
            C(39, 91, 132);

        colors[ImGuiCol_ModalWindowDimBg] =
            C(0, 0, 0, 110);
    }

    void Draw(
        State& state,
        const Callbacks& callbacks,
        bool* open)
    {
        const ImGuiViewport* viewport =
            ImGui::GetMainViewport();

        ImGui::SetNextWindowPos(
            viewport->WorkPos,
            ImGuiCond_Always);

        ImGui::SetNextWindowSize(
            viewport->WorkSize,
            ImGuiCond_Always);

        const ImGuiWindowFlags flags =
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoScrollWithMouse |
            ImGuiWindowFlags_NoBringToFrontOnFocus;

        if (!ImGui::Begin(
            "##ImCutRoot",
            open,
            flags))
        {
            ImGui::End();
            return;
        }

        DrawStatus(
            state);

        DrawTabs(
            state);

        if (state.page == Page::Color)
        {
            DrawColorActionBar(
                state,
                callbacks);
        }

        ImGui::PushStyleVar(
            ImGuiStyleVar_WindowPadding,
            ImVec2(
                S(20.0f),
                S(15.0f)));

        ImGui::PushStyleColor(
            ImGuiCol_ChildBg,
            C(38, 38, 38));

        ImGui::BeginChild(
            "##Content",
            ImVec2(
                0.0f,
                0.0f),
            ImGuiChildFlags_AlwaysUseWindowPadding,
            ImGuiWindowFlags_None);

        DrawPage(
            state,
            callbacks);

        ImGui::EndChild();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();

        DrawUpdatePopup(
            state,
            callbacks);

        ImGui::End();
    }
}
