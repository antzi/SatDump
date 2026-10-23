#include "common/geodetic/geodetic_coordinates.h"
#include "common/tracking/tle.h"
#include "core/style.h"
#include "i18n.h"
#include "imgui/imgui.h"
#include "imgui/imgui_stdlib.h"
#include "init.h"
#include "object_tracker.h"
#include "utils/string.h"
#include "utils/time.h"

namespace satdump
{
    void ObjectTracker::renderPolarPlot()
    {
        int d_pplot_size = ImGui::GetWindowContentRegionMax().x;
        ImDrawList *draw_list = ImGui::GetWindowDrawList();
        draw_list->AddRectFilled(ImGui::GetCursorScreenPos(), ImVec2(ImGui::GetCursorScreenPos().x + d_pplot_size, ImGui::GetCursorScreenPos().y + d_pplot_size), style::theme.widget_bg);

        // Draw the "target-like" plot with elevation rings
        float radius = 0.45;
        float radius1 = d_pplot_size * radius * (3.0 / 9.0);
        float radius2 = d_pplot_size * radius * (6.0 / 9.0);
        float radius3 = d_pplot_size * radius * (9.0 / 9.0);

        draw_list->AddCircle({ImGui::GetCursorScreenPos().x + (d_pplot_size / 2), ImGui::GetCursorScreenPos().y + (d_pplot_size / 2)}, radius1, style::theme.green, 0, 2);
        draw_list->AddCircle({ImGui::GetCursorScreenPos().x + (d_pplot_size / 2), ImGui::GetCursorScreenPos().y + (d_pplot_size / 2)}, radius2, style::theme.green, 0, 2);
        draw_list->AddCircle({ImGui::GetCursorScreenPos().x + (d_pplot_size / 2), ImGui::GetCursorScreenPos().y + (d_pplot_size / 2)}, radius3, style::theme.green, 0, 2);

        draw_list->AddLine({ImGui::GetCursorScreenPos().x + (d_pplot_size / 2), ImGui::GetCursorScreenPos().y},
                           {ImGui::GetCursorScreenPos().x + (d_pplot_size / 2), ImGui::GetCursorScreenPos().y + d_pplot_size}, style::theme.green, 2);
        draw_list->AddLine({ImGui::GetCursorScreenPos().x, ImGui::GetCursorScreenPos().y + (d_pplot_size / 2)},
                           {ImGui::GetCursorScreenPos().x + d_pplot_size, ImGui::GetCursorScreenPos().y + (d_pplot_size / 2)}, style::theme.green, 2);

        // Draw the satellite's trace
        if (upcoming_pass_points.size() > 1)
        {
            upcoming_passes_mtx.lock();
            for (int i = 0; i < (int)upcoming_pass_points.size() - 1; i++)
            {
                auto &p1 = upcoming_pass_points[i];
                auto &p2 = upcoming_pass_points[i + 1];

                float point_x1, point_x2, point_y1, point_y2;
                point_x1 = point_x2 = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
                point_y1 = point_y2 = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);

                point_x1 += az_el_to_plot_x(d_pplot_size, radius, p1.az, p1.el);
                point_y1 -= az_el_to_plot_y(d_pplot_size, radius, p1.az, p1.el);

                point_x2 += az_el_to_plot_x(d_pplot_size, radius, p2.az, p2.el);
                point_y2 -= az_el_to_plot_y(d_pplot_size, radius, p2.az, p2.el);

                draw_list->AddLine({point_x1, point_y1}, {point_x2, point_y2}, style::theme.orange, 2.0);
            }
            upcoming_passes_mtx.unlock();
        }

        // Draw the current satellite position
        if (sat_current_pos.el > 0)
        {
            float point_x = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
            float point_y = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);

            point_x += az_el_to_plot_x(d_pplot_size, radius, sat_current_pos.az, sat_current_pos.el);
            point_y -= az_el_to_plot_y(d_pplot_size, radius, sat_current_pos.az, sat_current_pos.el);

            draw_list->AddCircleFilled({point_x, point_y}, 5 * ui_scale, style::theme.red);
        }

        if (rotator_handler && rotator_handler->is_connected())
        {
#if 1
            {
                float point_x = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
                float point_y = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);

                point_x += az_el_to_plot_x(d_pplot_size, radius, rot_current_pos.az, rot_current_pos.el);
                point_y -= az_el_to_plot_y(d_pplot_size, radius, rot_current_pos.az, rot_current_pos.el);

                draw_list->AddCircle({point_x, point_y}, 9 * ui_scale, style::theme.light_cyan, 0, 2.0);
            }
#else // WIP, the idea is to draw the *actual* antenna beamwidth
            {
                float beamwidth = 10;

                for (int i = 0; i < 50; i++)
                {
                    float point_x = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
                    float point_y = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);
                    float point_x2 = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
                    float point_y2 = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);

                    float az1 = rot_current_req_pos.az + beamwidth * sin((i / 50.0) * 2 * M_PI);
                    float el1 = rot_current_req_pos.el + beamwidth * cos((i / 50.0) * 2 * M_PI);

                    int y = i + 1;

                    float az2 = rot_current_req_pos.az + beamwidth * sin((y / 50.0) * 2 * M_PI);
                    float el2 = rot_current_req_pos.el + beamwidth * cos((y / 50.0) * 2 * M_PI);

                    point_x += az_el_to_plot_x(d_pplot_size, radius, az1, el1);
                    point_y -= az_el_to_plot_y(d_pplot_size, radius, az1, el1);

                    point_x2 += az_el_to_plot_x(d_pplot_size, radius, az2, el2);
                    point_y2 -= az_el_to_plot_y(d_pplot_size, radius, az2, el2);

                    draw_list->AddLine({point_x, point_y}, {point_x2, point_y2}, style::theme.light_cyan, 2.0);
                }
            }
#endif

            if (rotator_engaged)
            {
                float point_x = ImGui::GetCursorScreenPos().x + (d_pplot_size / 2);
                float point_y = ImGui::GetCursorScreenPos().y + (d_pplot_size / 2);

                point_x += az_el_to_plot_x(d_pplot_size, radius, rot_current_req_pos.az, rot_current_req_pos.el);
                point_y -= az_el_to_plot_y(d_pplot_size, radius, rot_current_req_pos.az, rot_current_req_pos.el);

                draw_list->AddLine({point_x - 5 * ui_scale, point_y}, {point_x - 12 * ui_scale, point_y}, style::theme.light_cyan, 2.0);
                draw_list->AddLine({point_x + 5 * ui_scale, point_y}, {point_x + 12 * ui_scale, point_y}, style::theme.light_cyan, 2.0);
                draw_list->AddLine({point_x, point_y - 5 * ui_scale}, {point_x, point_y - 12 * ui_scale}, style::theme.light_cyan, 2.0);
                draw_list->AddLine({point_x, point_y + 5 * ui_scale}, {point_x, point_y + 12 * ui_scale}, style::theme.light_cyan, 2.0);
            }
        }
#if 1
        if (next_aos_time != 0 && next_los_time != 0)
        {
            double timeOffset = 0, ctime = getTime();
            if (next_aos_time > ctime)
                timeOffset = next_aos_time - ctime;
            else
                timeOffset = next_los_time - ctime;

            int hours = timeOffset / 3600;
            int minutes = fmod(timeOffset / 60, 60);
            int seconds = fmod(timeOffset, 60);

            std::string time_dis =
                (hours < 10 ? "0" : "") + std::to_string(hours) + ":" + (minutes < 10 ? "0" : "") + std::to_string(minutes) + ":" + (seconds < 10 ? "0" : "") + std::to_string(seconds);

            auto &style = ImGui::GetStyle();
            ImVec2 cur = ImGui::GetCursorPos();
            ImVec2 curs = ImGui::GetCursorScreenPos();
            std::string obj_name = "None";
            if (tracking_mode == TRACKING_HORIZONS)
                obj_name = horizonsoptions[current_horizons_id].second;
            else if (tracking_mode == TRACKING_SATELLITE)
                obj_name = satoptions[current_satellite_id];
            ImVec2 size = ImGui::CalcTextSize(obj_name.c_str());
            draw_list->AddRectFilled(curs, ImVec2(curs.x + size.x + 2 * style.FramePadding.x, curs.y + size.y), style::theme.overlay_bg);
            ImGui::TextColored(style::theme.green, "%s", obj_name.c_str());
            curs = ImGui::GetCursorScreenPos();
            char buff[9];
            snprintf(buff, sizeof(buff), "Az: %.1f", sat_current_pos.az);
            size = ImGui::CalcTextSize(buff);
            draw_list->AddRectFilled(curs, ImVec2(curs.x + size.x + 2 * style.FramePadding.x, curs.y + size.y), style::theme.overlay_bg);
            ImGui::TextColored(style::theme.green, "Az: %.1f", sat_current_pos.az);
            curs = ImGui::GetCursorScreenPos();
            snprintf(buff, sizeof(buff), "El: %.1f", sat_current_pos.el);
            size = ImGui::CalcTextSize(buff);
            draw_list->AddRectFilled(curs, ImVec2(curs.x + size.x + 2 * style.FramePadding.x, curs.y + size.y), style::theme.overlay_bg);
            ImGui::TextColored(style::theme.green, "El: %.1f", sat_current_pos.el);

            if (next_aos_time != -1 && next_los_time != -1)
            {
                if (next_aos_time > ctime)
                    size = ImGui::CalcTextSize(("AOS in" + time_dis).c_str());
                else
                    size = ImGui::CalcTextSize(("LOS in" + time_dis).c_str());
            }

            ImGui::SetCursorPosY(cur.y + d_pplot_size - 20 * ui_scale);
            curs = ImGui::GetCursorScreenPos();
            draw_list->AddRectFilled(curs, ImVec2(curs.x + size.x + 2 * style.FramePadding.x, curs.y + size.y), style::theme.overlay_bg);
            ImGui::TextColored(style::theme.green, "%s in %s", next_aos_time > ctime ? "AOS" : "LOS", time_dis.c_str());

            ImGui::SetCursorPos(cur);
        }
#endif
        ImGui::Dummy(ImVec2(d_pplot_size + 3 * ui_scale, d_pplot_size + 3 * ui_scale));
    }

    void ObjectTracker::renderSelectionMenu()
    {
        // Take a consistent snapshot for this ImGui frame. Never keep
        // general_mutex locked while drawing ImGui widgets or doing I/O.
        TrackingMode display_mode;
        int selected_satellite;
        int selected_horizons;
        std::vector<std::pair<std::string, int>> satellites; // Name, NORAD ID
        decltype(horizonsoptions) horizons;
        {
            std::lock_guard<std::mutex> lock(general_mutex);
            display_mode = tracking_mode;
            selected_satellite = current_satellite_id;
            selected_horizons = current_horizons_id;
            horizons = horizonsoptions;

            const auto &reg = db_keplers->all_;
            for (size_t i = 0; i < satoptions.size() && i < reg.size(); ++i)
                satellites.emplace_back(satoptions[i], reg[i].norad);
        }

        // Use the same value for the push and pop, even if the backend
        // finishes updating in the middle of this frame.
	const bool disable_controls = backend_needs_update.load();
        bool selection_requested = false;
        TrackingMode requested_mode = TRACKING_NONE;
        int requested_object_id = 0;

	if (disable_controls)
            style::beginDisabled();

        if (ImGui::BeginTable("##trackingradiotable", 2, ImGuiTableFlags_None))
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            if (ImGui::RadioButton("Satellites", display_mode == TRACKING_SATELLITE))
            {
                if (!satellites.empty())
                {
                    const int i = selected_satellite >= 0 &&
                                  selected_satellite < (int)satellites.size()
                                      ? selected_satellite : 0;
                    display_mode = TRACKING_SATELLITE;
                    requested_mode = TRACKING_SATELLITE;
                    requested_object_id = satellites[i].second;
                    selection_requested = true;
                }
            }

            ImGui::TableSetColumnIndex(1);
            if (ImGui::RadioButton("Horizons", display_mode == TRACKING_HORIZONS))
            {
                // Preserve lazy loading, but perform the network request
                // OUTSIDE general_mutex. Publish the list while holding it.
                if (horizons.size() == 1)
                {
                    auto fetched = pullHorizonsList();
                    std::lock_guard<std::mutex> lock(general_mutex);
                    if (horizonsoptions.size() == 1)
                        horizonsoptions = fetched;
                    horizons = horizonsoptions;
                    selected_horizons = current_horizons_id;
                }

                if (!horizons.empty())
                {
                    const int i = selected_horizons >= 0 &&
                                  selected_horizons < (int)horizons.size()
                                      ? selected_horizons : 0;
                    display_mode = TRACKING_HORIZONS;
                    requested_mode = TRACKING_HORIZONS;
                    requested_object_id = horizons[i].first;
                    selection_requested = true;
                }
            }
            ImGui::EndTable();
        }

        ImGui::SetNextItemWidth(ImGui::GetWindowContentRegionMax().x);
        if (display_mode == TRACKING_HORIZONS)
        {
            const bool valid = selected_horizons >= 0 &&
                               selected_horizons < (int)horizons.size();
            const char *preview = valid ? horizons[selected_horizons].second.c_str()
                                        : "No Horizons target";
            if (ImGui::BeginCombo("###horizonsselectcombo", preview))
            {
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
                ImGui::InputTextWithHint("##horizonssatellitetracking", u8"\uf422   Search", &horizons_searchstr);
                for (int i = 0; i < (int)horizons.size(); ++i)
                {
                    if (!horizons_searchstr.empty() &&
                        !satdump::isStringPresent(horizons[i].second, horizons_searchstr))
                        continue;

                    if (ImGui::Selectable(horizons[i].second.c_str(), i == selected_horizons))
                    {
                        requested_mode = TRACKING_HORIZONS;
                        requested_object_id = horizons[i].first;
                        selection_requested = true;
                    }
                }
                ImGui::EndCombo();
            }

            if (valid && ImGui::IsItemHovered())
                ImGui::SetTooltip("Horizons ID %d", horizons[selected_horizons].first);
        }
        else if (display_mode == TRACKING_SATELLITE)
        {
            const bool valid = selected_satellite >= 0 &&
                               selected_satellite < (int)satellites.size();
            const char *preview = valid ? satellites[selected_satellite].first.c_str()
                                        : "No satellite";
            if (ImGui::BeginCombo("###satelliteselectcombo", preview))
            {
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
                ImGui::InputTextWithHint("##searchsatellitetracking", u8"\uf422   Search", &satellite_searchstr);
                for (int i = 0; i < (int)satellites.size(); i++)
                {
                    if (!satellite_searchstr.empty() &&
                        !isStringPresent(satellites[i].first, satellite_searchstr))
                        continue;

                    if (ImGui::Selectable(satellites[i].first.c_str(), i == selected_satellite))
                    {
			requested_mode = TRACKING_SATELLITE;
			requested_object_id = db_keplers->all_[i].norad;
			selection_requested = true;
                    }
                }
                ImGui::EndCombo();
            }

            if (valid && ImGui::IsItemHovered())
                ImGui::SetTooltip("NORAD ID %d", satellites[selected_satellite].second);
        }

        if (disable_controls)
            style::endDisabled();

        // setObject() owns general_mutex and requests a backend refresh.
        // Do not call it while holding our snapshot lock or during ImGui UI.
        if (selection_requested)
            setObject(requested_mode, requested_object_id);
    }

    void ObjectTracker::renderObjectStatus()
    {
        if (ImGui::BeginTable("##trackingwidgettable", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text(_("Azimuth"));
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f", sat_current_pos.az);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text(_("Elevation"));
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f", sat_current_pos.el);

            if (next_aos_time != 0 && next_los_time != 0)
            {
                double timeOffset = 0, ctime = getTime();
                if (next_aos_time > ctime)
                    timeOffset = next_aos_time - ctime;
                else
                    timeOffset = next_los_time - ctime;

                int hours = timeOffset / 3600;
                int minutes = fmod(timeOffset / 60, 60);
                int seconds = fmod(timeOffset, 60);

                std::string time_dis =
                    (hours < 10 ? "0" : "") + std::to_string(hours) + ":" + (minutes < 10 ? "0" : "") + std::to_string(minutes) + ":" + (seconds < 10 ? "0" : "") + std::to_string(seconds);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(_("Next Event"));
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%s", time_dis.c_str());
            }

            if (tracking_mode == TRACKING_SATELLITE && satellite_object != nullptr)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(_("Azimuth Rate"));
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f", satellite_observation_pos.azimuth_rate * RAD_TO_DEG);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(_("Elevation Rate"));
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f", satellite_observation_pos.elevation_rate * RAD_TO_DEG);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text(_("Range (km)"));
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f", satellite_observation_pos.range);
            }

            ImGui::EndTable();
        }
    }
} // namespace satdump
