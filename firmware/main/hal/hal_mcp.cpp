/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "hal.h"
#include <board.h>
#include <display/lvgl_display/lvgl_image.h>
#include <esp_heap_caps.h>
#include <mooncake_log.h>
#include <mcp_server.h>
#include <stackchan/stackchan.h>
#include <apps/common/common.h>

using namespace stackchan;

static const std::string_view _tag = "HAL-MCP";

static const char* _background_image_urls[] = {
    "https://fei-storage.oss-cn-zhangjiakou.aliyuncs.com/stackchain/backgrounds/yui-aragaki/320x240/"
    "yui_01_320x240.jpg",
    "https://fei-storage.oss-cn-zhangjiakou.aliyuncs.com/stackchain/backgrounds/yui-aragaki/320x240/"
    "yui_02_320x240.jpg",
    "https://fei-storage.oss-cn-zhangjiakou.aliyuncs.com/stackchain/backgrounds/yui-aragaki/320x240/"
    "yui_03_320x240.jpg",
    "https://fei-storage.oss-cn-zhangjiakou.aliyuncs.com/stackchain/backgrounds/yui-aragaki/320x240/"
    "yui_04_320x240.jpg",
    "https://fei-storage.oss-cn-zhangjiakou.aliyuncs.com/stackchain/backgrounds/yui-aragaki/320x240/"
    "yui_05_320x240.jpg",
    "https://fei-storage.oss-cn-zhangjiakou.aliyuncs.com/stackchain/backgrounds/yui-aragaki/320x240/"
    "yui_06_320x240.jpg",
};

static size_t _next_background_image_index = 0;

void Hal::xiaozhi_mcp_init()
{
    mclog::tagInfo(_tag, "init");

    // https://github.com/78/xiaozhi-esp32/blob/main/docs/mcp-usage.md
    auto& mcp_server = McpServer::GetInstance();

    // System Prompt：
    // You can control the robot's head. Use get_yaw and get_pitch to sense current position. Use set_yaw for horizontal
    // movement and set_pitch for vertical movement. All angles are in degrees.

    mclog::tagInfo(_tag, "add robot.get_head_angles tool");
    mcp_server.AddTool("self.robot.get_head_angles",
                       "Returns current yaw/pitch in degrees. Neutral position is {yaw:0, pitch:0}.",
                       std::vector<Property>{}, [this](const PropertyList& properties) -> ReturnValue {
                           LvglLockGuard lock;  // StackChan motion update is under the lvgl lock

                           auto& motion      = GetStackChan().motion();
                           int current_yaw   = motion.yawServo().getCurrentAngle() / 10;
                           int current_pitch = motion.pitchServo().getCurrentAngle() / 10;

                           auto result = fmt::format(R"({{"yaw": {}, "pitch": {}}})", current_yaw, current_pitch);
                           mclog::tagInfo(_tag, "get_head_angles: {}", result);
                           return result;
                       });

    mclog::tagInfo(_tag, "add robot.set_head_angles tool");
    mcp_server.AddTool("self.robot.set_head_angles",
                       "Adjust head position. GUIDELINES: "
                       "1. For natural interaction, stay within +/- 45 degrees. "
                       "2. Only use values > 70 if the user explicitly asks to look far away/behind. "
                       "3. Max ranges: Yaw(-128 to 128, -128 as your left), Pitch(0 to 90, 90 as your up). "
                       "Speed(100-1000, 150 is natural).",
                       PropertyList({Property("yaw", kPropertyTypeInteger, -9999, -9999, 128),
                                     Property("pitch", kPropertyTypeInteger, -9999, -9999, 90),
                                     Property("speed", kPropertyTypeInteger, 150, 100, 1000)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int speed = properties["speed"].value<int>();
                           int yaw   = properties["yaw"].value<int>();
                           int pitch = properties["pitch"].value<int>();

                           mclog::tagInfo(_tag, "motion set_angles: yaw: {}, pitch: {}, speed: {}", yaw, pitch, speed);

                           LvglLockGuard lock;

                           auto& motion = GetStackChan().motion();
                           if (pitch != -9999) {
                               motion.pitchServo().moveWithSpeed(pitch * 10, speed);
                           }
                           if (yaw != -9999) {
                               motion.yawServo().moveWithSpeed(yaw * 10, speed);
                           }

                           return true;
                       });

    mclog::tagInfo(_tag, "add robot.set_led_color tool");
    mcp_server.AddTool(
        "self.robot.set_led_color",
        "Set the color of the robot's INTERNAL onboard LED. This is NOT for room lights. "
        "Values: 0-168 (safe range). Red=168,0,0; Green=0,168,0; Blue=0,0,168; White=100,100,100; Off=0,0,0.",
        PropertyList({Property("red", kPropertyTypeInteger, 0, 0, 168),
                      Property("green", kPropertyTypeInteger, 0, 0, 168),
                      Property("blue", kPropertyTypeInteger, 0, 0, 168)}),
        [this](const PropertyList& properties) -> ReturnValue {
            int r = properties["red"].value<int>();
            int g = properties["green"].value<int>();
            int b = properties["blue"].value<int>();

            mclog::tagInfo(_tag, "set_led_color: r={}, g={}, b={}", r, g, b);

            LvglLockGuard lock;

            GetStackChan().leftNeonLight().setColor(r, g, b);
            GetStackChan().rightNeonLight().setColor(r, g, b);

            return true;
        });

    mclog::tagInfo(_tag, "add screen.set_background_color tool");
    mcp_server.AddTool("self.screen.set_background_color",
                       "Set the avatar screen background color using RGB values. "
                       "Use this when the user asks to change the robot face background color.",
                       PropertyList({Property("red", kPropertyTypeInteger, 0, 255),
                                     Property("green", kPropertyTypeInteger, 0, 255),
                                     Property("blue", kPropertyTypeInteger, 0, 255)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int r = properties["red"].value<int>();
                           int g = properties["green"].value<int>();
                           int b = properties["blue"].value<int>();

                           mclog::tagInfo(_tag, "set_background_color: r={}, g={}, b={}", r, g, b);

                           LvglLockGuard lock;

                           auto& stackchan = GetStackChan();
                           if (!stackchan.hasAvatar()) {
                               return false;
                           }

                           uint32_t color = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) |
                                            static_cast<uint32_t>(b);
                           stackchan.avatar().setBackgroundColor(lv_color_hex(color));

                           return true;
                       });

    mclog::tagInfo(_tag, "add screen.next_background_image tool");
    mcp_server.AddTool("self.screen.next_background_image",
                       "Switch the avatar screen background to the next built-in online background image. "
                       "Use this when the user asks to change or rotate the robot face background picture.",
                       std::vector<Property>{}, [this](const PropertyList& properties) -> ReturnValue {
                           auto& stackchan = GetStackChan();
                           if (!stackchan.hasAvatar()) {
                               return false;
                           }

                           constexpr size_t max_image_size = 512 * 1024;
                           const size_t image_count = sizeof(_background_image_urls) / sizeof(_background_image_urls[0]);
                           size_t image_index       = _next_background_image_index % image_count;
                           const char* url          = _background_image_urls[image_index];

                           mclog::tagInfo(_tag, "next_background_image: index={}, url={}", image_index + 1, url);

                           auto http = Board::GetInstance().GetNetwork()->CreateHttp(3);
                           if (!http->Open("GET", url)) {
                               mclog::tagError(_tag, "open image url failed: {}", url);
                               return false;
                           }

                           int status_code = http->GetStatusCode();
                           if (status_code != 200) {
                               mclog::tagError(_tag, "unexpected image status: {}", status_code);
                               http->Close();
                               return false;
                           }

                           size_t content_length = http->GetBodyLength();
                           if (content_length == 0 || content_length > max_image_size) {
                               mclog::tagError(_tag, "invalid image size: {}", content_length);
                               http->Close();
                               return false;
                           }

                           char* data = (char*)heap_caps_malloc(content_length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                           if (data == nullptr) {
                               mclog::tagError(_tag, "alloc image failed: {} bytes", content_length);
                               http->Close();
                               return false;
                           }

                           size_t total_read = 0;
                           while (total_read < content_length) {
                               int ret = http->Read(data + total_read, content_length - total_read);
                               if (ret < 0) {
                                   mclog::tagError(_tag, "read image failed");
                                   heap_caps_free(data);
                                   http->Close();
                                   return false;
                               }
                               if (ret == 0) {
                                   break;
                               }
                               total_read += ret;
                           }
                           http->Close();

                           if (total_read != content_length) {
                               mclog::tagError(_tag, "image download incomplete: {}/{}", total_read, content_length);
                               heap_caps_free(data);
                               return false;
                           }

                           std::unique_ptr<LvglImage> image;
                           try {
                               image = std::make_unique<LvglAllocatedImage>(data, content_length);
                           } catch (const std::exception& e) {
                               mclog::tagError(_tag, "create image failed: {}", e.what());
                               heap_caps_free(data);
                               return false;
                           }

                           {
                               LvglLockGuard lock;
                               stackchan.avatar().setBackgroundImage(std::move(image));
                           }

                           _next_background_image_index = (image_index + 1) % image_count;
                           return fmt::format(R"({{"index":{},"url":"{}"}})", image_index + 1, url);
                       });

    mclog::tagInfo(_tag, "add robot.create_reminder tool");
    mcp_server.AddTool("self.robot.create_reminder",
                       "Create a reminder. Duration is in seconds. Message is what to say when time is up. Set repeat "
                       "to true to repeat the reminder.",
                       PropertyList({Property("duration_seconds", kPropertyTypeInteger, 60, 1, 86400),
                                     Property("message", kPropertyTypeString, std::string("Time's up!")),
                                     Property("repeat", kPropertyTypeBoolean, false)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int duration_seconds = properties["duration_seconds"].value<int>();
                           std::string message  = properties["message"].value<std::string>();
                           bool repeat          = properties["repeat"].value<bool>();

                           // Default message
                           if (message.empty()) {
                               message = "Time's up!";
                           }

                           mclog::tagInfo(_tag, "create_reminder: duration={}s, message={}, repeat={}",
                                          duration_seconds, message, repeat);

                           int id = tools::create_reminder(duration_seconds * 1000, message, repeat);

                           return id;
                       });

    mclog::tagInfo(_tag, "add robot.get_reminders tool");
    mcp_server.AddTool("self.robot.get_reminders", "Get list of active reminders.", std::vector<Property>{},
                       [this](const PropertyList& properties) -> ReturnValue {
                           mclog::tagInfo(_tag, "get_reminders");
                           auto reminders          = tools::get_active_reminders();
                           std::string result_json = "[";
                           for (size_t i = 0; i < reminders.size(); ++i) {
                               const auto& r = reminders[i];
                               result_json +=
                                   fmt::format(R"({{"id": {}, "duration_ms": {}, "message": "{}", "repeat": {}}})",
                                               r.id, r.durationMs, r.message, r.repeat ? "true" : "false");
                               if (i < reminders.size() - 1) {
                                   result_json += ", ";
                               }
                           }
                           result_json += "]";
                           mclog::tagInfo(_tag, "get_reminders result: {}", result_json);
                           return result_json;
                       });

    mclog::tagInfo(_tag, "add robot.stop_reminder tool");
    mcp_server.AddTool("self.robot.stop_reminder", "Stop a reminder by ID.",
                       PropertyList({Property("id", kPropertyTypeInteger, -1)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int id = properties["id"].value<int>();
                           mclog::tagInfo(_tag, "stop_reminder: id={}", id);
                           tools::stop_reminder(id);
                           return true;
                       });
}
