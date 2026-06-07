/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "hal.h"
#include <board.h>
#include <display/lvgl_display/lvgl_image.h>
#include <driver/gpio.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <jpg/jpeg_to_image.h>
#include <mooncake_log.h>
#include <mcp_server.h>
#include <stackchan/stackchan.h>
#include <stackchan/modifiers/dance.h>
#include <apps/common/common.h>
#include <algorithm>
#include <array>
#include <cstring>

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

class LedStripController {
public:
    enum class Mode {
        Off,
        Solid,
        Blink,
        Rainbow,
        Chase,
        Neon,
        Aurora,
        Breath,
        Comet,
        Meteor,
        Theater,
    };

    bool setColor(int red, int green, int blue, int brightness)
    {
        if (!ensure_started()) {
            return false;
        }

        update_state(Mode::Solid, red, green, blue, brightness, 0, 0);
        return true;
    }

    bool blink(int red, int green, int blue, int brightness, int interval_ms)
    {
        if (!ensure_started()) {
            return false;
        }

        update_state(Mode::Blink, red, green, blue, brightness, interval_ms, 0);
        return true;
    }

    bool rainbow(int brightness, int speed_ms)
    {
        if (!ensure_started()) {
            return false;
        }

        update_state(Mode::Rainbow, 0, 0, 0, brightness, speed_ms, 0);
        return true;
    }

    bool chase(int red, int green, int blue, int brightness, int speed_ms, int width)
    {
        if (!ensure_started()) {
            return false;
        }

        update_state(Mode::Chase, red, green, blue, brightness, speed_ms, width);
        return true;
    }

    bool neon(int brightness, int speed_ms)
    {
        if (!ensure_started()) {
            return false;
        }

        update_state(Mode::Neon, 0, 0, 0, brightness, speed_ms, 0);
        return true;
    }

    bool aurora(int brightness, int speed_ms)
    {
        if (!ensure_started()) {
            return false;
        }

        update_state(Mode::Aurora, 0, 0, 0, brightness, speed_ms, 0);
        return true;
    }

    bool breath(int red, int green, int blue, int brightness, int speed_ms)
    {
        if (!ensure_started()) {
            return false;
        }

        update_state(Mode::Breath, red, green, blue, brightness, speed_ms, 0);
        return true;
    }

    bool comet(int red, int green, int blue, int brightness, int speed_ms, int tail_width)
    {
        if (!ensure_started()) {
            return false;
        }

        update_state(Mode::Comet, red, green, blue, brightness, speed_ms, tail_width);
        return true;
    }

    bool meteor(int brightness, int speed_ms)
    {
        if (!ensure_started()) {
            return false;
        }

        update_state(Mode::Meteor, 0, 0, 0, brightness, speed_ms, 0);
        return true;
    }

    bool theater(int red, int green, int blue, int brightness, int speed_ms, int spacing)
    {
        if (!ensure_started()) {
            return false;
        }

        update_state(Mode::Theater, red, green, blue, brightness, speed_ms, spacing);
        return true;
    }

    bool clear()
    {
        if (!ensure_started()) {
            return false;
        }

        update_state(Mode::Off, 0, 0, 0, 0, 0, 0);
        return true;
    }

    const char* lastError() const
    {
        return _last_error;
    }

private:
    struct State {
        Mode mode = Mode::Off;
        uint8_t red = 0;
        uint8_t green = 0;
        uint8_t blue = 0;
        uint8_t brightness = 32;
        uint32_t interval_ms = 50;
        size_t chase_width = 10;
        bool dirty = true;
    };

    static constexpr gpio_num_t kDataPin = GPIO_NUM_9;
    static constexpr uint32_t kRmtResolutionHz = 10000000;
    static constexpr size_t kLedCount = 160;
    static constexpr size_t kRmtMemBlockSymbols = 64;

    std::array<uint8_t, kLedCount * 3> _pixels = {};
    rmt_channel_handle_t _tx_channel = nullptr;
    rmt_encoder_handle_t _encoder = nullptr;
    SemaphoreHandle_t _mutex = nullptr;
    TaskHandle_t _task = nullptr;
    State _state;
    uint8_t _rainbow_phase = 0;
    size_t _chase_head = 0;
    uint32_t _meteor_seed = 0x12345678;
    bool _blink_on = false;
    uint32_t _last_frame_ms = 0;
    const char* _last_error = "not_started";

    static rmt_symbol_word_t symbol(int level0, uint16_t duration0, int level1, uint16_t duration1)
    {
        rmt_symbol_word_t value = {};
        value.level0 = level0;
        value.duration0 = duration0;
        value.level1 = level1;
        value.duration1 = duration1;
        return value;
    }

    static size_t encoder_callback(const void* data, size_t data_size, size_t symbols_written, size_t symbols_free,
                                   rmt_symbol_word_t* symbols, bool* done, void*)
    {
        const size_t data_pos = symbols_written / 8;
        if (data_pos < data_size) {
            if (symbols_free < 8) {
                return 0;
            }

            const auto* data_bytes = static_cast<const uint8_t*>(data);
            size_t symbol_pos = 0;
            for (uint8_t bit_mask = 0x80; bit_mask != 0; bit_mask >>= 1) {
                symbols[symbol_pos++] = (data_bytes[data_pos] & bit_mask) ? symbol(1, 9, 0, 3) : symbol(1, 3, 0, 9);
            }
            return symbol_pos;
        }

        if (symbols_free < 1) {
            return 0;
        }

        symbols[0] = symbol(0, 1250, 0, 1250);
        *done = true;
        return 1;
    }

    static void task_entry(void* arg)
    {
        static_cast<LedStripController*>(arg)->task_loop();
    }

    bool ensure_started()
    {
        if (_tx_channel != nullptr && _encoder != nullptr && _task != nullptr) {
            return true;
        }

        if (_mutex == nullptr) {
            _mutex = xSemaphoreCreateMutex();
            if (_mutex == nullptr) {
                _last_error = "mutex_failed";
                return false;
            }
        }

        if (_tx_channel == nullptr) {
            rmt_tx_channel_config_t tx_config = {};
            tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
            tx_config.gpio_num = kDataPin;
            tx_config.mem_block_symbols = kRmtMemBlockSymbols;
            tx_config.resolution_hz = kRmtResolutionHz;
            tx_config.trans_queue_depth = 4;

            esp_err_t err = rmt_new_tx_channel(&tx_config, &_tx_channel);
            if (err != ESP_OK) {
                mclog::tagError(_tag, "led strip rmt_new_tx_channel failed: {}", esp_err_to_name(err));
                _last_error = "rmt_channel_failed";
                return false;
            }
        }

        if (_encoder == nullptr) {
            rmt_simple_encoder_config_t encoder_config = {};
            encoder_config.callback = encoder_callback;

            esp_err_t err = rmt_new_simple_encoder(&encoder_config, &_encoder);
            if (err != ESP_OK) {
                mclog::tagError(_tag, "led strip rmt_new_simple_encoder failed: {}", esp_err_to_name(err));
                _last_error = "rmt_encoder_failed";
                return false;
            }
        }

        esp_err_t err = rmt_enable(_tx_channel);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            mclog::tagError(_tag, "led strip rmt_enable failed: {}", esp_err_to_name(err));
            _last_error = "rmt_enable_failed";
            return false;
        }

        if (_task == nullptr) {
            BaseType_t ret = xTaskCreate(task_entry, "led_strip_mcp", 4096, this, 3, &_task);
            if (ret != pdPASS) {
                _last_error = "task_failed";
                return false;
            }
        }

        _last_error = "ok";
        return true;
    }

    void update_state(Mode mode, int red, int green, int blue, int brightness, int interval_ms, int chase_width)
    {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_state.mode != mode) {
            _rainbow_phase = 0;
            _chase_head = 0;
            _blink_on = false;
        }
        _state.mode = mode;
        _state.red = static_cast<uint8_t>(std::clamp(red, 0, 255));
        _state.green = static_cast<uint8_t>(std::clamp(green, 0, 255));
        _state.blue = static_cast<uint8_t>(std::clamp(blue, 0, 255));
        _state.brightness = static_cast<uint8_t>(std::clamp(brightness, 0, 255));
        _state.interval_ms = static_cast<uint32_t>(std::clamp(interval_ms, 20, 5000));
        _state.chase_width = static_cast<size_t>(std::clamp(chase_width, 1, 40));
        _state.dirty = true;
        xSemaphoreGive(_mutex);
    }

    void task_loop()
    {
        while (true) {
            render_if_needed();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    void render_if_needed()
    {
        State state;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        state = _state;
        if (!_state.dirty && (state.mode == Mode::Solid || state.mode == Mode::Off)) {
            xSemaphoreGive(_mutex);
            return;
        }
        _state.dirty = false;
        xSemaphoreGive(_mutex);

        uint32_t now = GetHAL().millis();
        if (!state.dirty && state.mode != Mode::Solid && now - _last_frame_ms < state.interval_ms) {
            return;
        }
        _last_frame_ms = now;

        switch (state.mode) {
            case Mode::Off:
                clear_pixels();
                break;
            case Mode::Solid:
                fill_solid(state.red, state.green, state.blue, state.brightness);
                break;
            case Mode::Blink:
                _blink_on = !_blink_on;
                if (_blink_on) {
                    fill_solid(state.red, state.green, state.blue, state.brightness);
                } else {
                    clear_pixels();
                }
                break;
            case Mode::Rainbow:
                fill_rainbow(_rainbow_phase, state.brightness);
                _rainbow_phase = static_cast<uint8_t>(_rainbow_phase + 5);
                break;
            case Mode::Chase:
                fill_chase(state);
                _chase_head = (_chase_head + 1) % kLedCount;
                break;
            case Mode::Neon:
                fill_neon(state);
                _rainbow_phase = static_cast<uint8_t>(_rainbow_phase + 4);
                break;
            case Mode::Aurora:
                fill_aurora(state);
                _rainbow_phase = static_cast<uint8_t>(_rainbow_phase + 2);
                break;
            case Mode::Breath:
                fill_breath(state);
                _rainbow_phase = static_cast<uint8_t>(_rainbow_phase + 4);
                break;
            case Mode::Comet:
                fill_comet(state);
                _chase_head = (_chase_head + 1) % kLedCount;
                break;
            case Mode::Meteor:
                fill_meteor(state, state.dirty);
                break;
            case Mode::Theater:
                fill_theater(state);
                _chase_head = (_chase_head + 1) % std::max<size_t>(state.chase_width, 1);
                break;
        }

        flush_pixels();
    }

    void clear_pixels()
    {
        _pixels.fill(0);
    }

    void set_pixel_rgb(size_t index, uint8_t red, uint8_t green, uint8_t blue)
    {
        if (index >= kLedCount) {
            return;
        }
        _pixels[index * 3 + 0] = green;
        _pixels[index * 3 + 1] = red;
        _pixels[index * 3 + 2] = blue;
    }

    void fill_solid(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
    {
        red = static_cast<uint8_t>(static_cast<uint16_t>(red) * brightness / 255);
        green = static_cast<uint8_t>(static_cast<uint16_t>(green) * brightness / 255);
        blue = static_cast<uint8_t>(static_cast<uint16_t>(blue) * brightness / 255);
        for (size_t i = 0; i < kLedCount; ++i) {
            set_pixel_rgb(i, red, green, blue);
        }
    }

    void fill_rainbow(uint8_t phase, uint8_t brightness)
    {
        for (size_t i = 0; i < kLedCount; ++i) {
            uint8_t red = 0;
            uint8_t green = 0;
            uint8_t blue = 0;
            color_wheel(static_cast<uint8_t>(phase + i * 256 / kLedCount), brightness, red, green, blue);
            set_pixel_rgb(i, red, green, blue);
        }
    }

    void fill_chase(const State& state)
    {
        clear_pixels();
        uint8_t red = static_cast<uint8_t>(static_cast<uint16_t>(state.red) * state.brightness / 255);
        uint8_t green = static_cast<uint8_t>(static_cast<uint16_t>(state.green) * state.brightness / 255);
        uint8_t blue = static_cast<uint8_t>(static_cast<uint16_t>(state.blue) * state.brightness / 255);
        for (size_t dot = 0; dot < state.chase_width; ++dot) {
            set_pixel_rgb((_chase_head + dot) % kLedCount, red, green, blue);
        }
    }

    void fill_neon(const State& state)
    {
        static constexpr uint8_t palette[][3] = {
            {255, 0, 180},
            {0, 220, 255},
            {255, 220, 0},
            {120, 0, 255},
        };
        for (size_t i = 0; i < kLedCount; ++i) {
            const size_t index = ((_rainbow_phase / 12) + i / 8) % 4;
            const uint8_t pulse = static_cast<uint8_t>(128 + triangle_wave(static_cast<uint8_t>(_rainbow_phase + i * 7), 127));
            const uint8_t brightness = static_cast<uint8_t>(static_cast<uint16_t>(state.brightness) * pulse / 255);
            set_pixel_rgb(i, scale(palette[index][0], brightness), scale(palette[index][1], brightness),
                          scale(palette[index][2], brightness));
        }
    }

    void fill_aurora(const State& state)
    {
        for (size_t i = 0; i < kLedCount; ++i) {
            const uint8_t wave = triangle_wave(static_cast<uint8_t>(_rainbow_phase + i * 3), 120);
            const uint8_t brightness = static_cast<uint8_t>(static_cast<uint16_t>(state.brightness) * (80 + wave) / 200);
            const uint8_t mix = static_cast<uint8_t>(_rainbow_phase / 2 + i * 2);
            uint8_t red = scale(static_cast<uint8_t>(20 + mix / 12), brightness);
            uint8_t green = scale(static_cast<uint8_t>(120 + wave / 2), brightness);
            uint8_t blue = scale(static_cast<uint8_t>(180 + triangle_wave(static_cast<uint8_t>(mix + 90), 60)), brightness);
            set_pixel_rgb(i, red, green, blue);
        }
    }

    void fill_breath(const State& state)
    {
        uint8_t breath_level = triangle_wave(_rainbow_phase, state.brightness);
        fill_solid(state.red, state.green, state.blue, breath_level);
    }

    void fill_comet(const State& state)
    {
        clear_pixels();
        const size_t tail_width = std::max<size_t>(state.chase_width, 1);
        for (size_t dot = 0; dot < tail_width; ++dot) {
            const size_t index = (_chase_head + kLedCount - dot) % kLedCount;
            const uint8_t brightness = static_cast<uint8_t>(static_cast<uint16_t>(state.brightness) * (tail_width - dot) / tail_width);
            set_pixel_rgb(index, scale(state.red, brightness), scale(state.green, brightness), scale(state.blue, brightness));
        }
    }

    void fill_meteor(const State& state, bool reset)
    {
        if (reset) {
            clear_pixels();
        }
        for (auto& pixel : _pixels) {
            pixel = static_cast<uint8_t>(static_cast<uint16_t>(pixel) * 190 / 255);
        }
        for (size_t meteor = 0; meteor < 3; ++meteor) {
            const size_t index = next_random() % kLedCount;
            uint8_t red = 0;
            uint8_t green = 0;
            uint8_t blue = 0;
            color_wheel(static_cast<uint8_t>(_rainbow_phase + meteor * 70), state.brightness, red, green, blue);
            set_pixel_rgb(index, red, green, blue);
        }
        _rainbow_phase = static_cast<uint8_t>(_rainbow_phase + 9);
    }

    void fill_theater(const State& state)
    {
        clear_pixels();
        const size_t spacing = std::max<size_t>(state.chase_width, 2);
        const uint8_t red = scale(state.red, state.brightness);
        const uint8_t green = scale(state.green, state.brightness);
        const uint8_t blue = scale(state.blue, state.brightness);
        for (size_t i = 0; i < kLedCount; ++i) {
            if ((i + _chase_head) % spacing == 0) {
                set_pixel_rgb(i, red, green, blue);
            }
        }
    }

    static uint8_t scale(uint8_t value, uint8_t brightness)
    {
        return static_cast<uint8_t>(static_cast<uint16_t>(value) * brightness / 255);
    }

    static uint8_t triangle_wave(uint8_t phase, uint8_t max_value)
    {
        const uint8_t v = phase < 128 ? static_cast<uint8_t>(phase * 2) : static_cast<uint8_t>((255 - phase) * 2);
        return static_cast<uint8_t>(static_cast<uint16_t>(v) * max_value / 255);
    }

    uint32_t next_random()
    {
        _meteor_seed ^= _meteor_seed << 13;
        _meteor_seed ^= _meteor_seed >> 17;
        _meteor_seed ^= _meteor_seed << 5;
        return _meteor_seed;
    }

    static void color_wheel(uint8_t wheel_pos, uint8_t brightness, uint8_t& red, uint8_t& green, uint8_t& blue)
    {
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;

        if (wheel_pos < 85) {
            r = static_cast<uint8_t>(255 - wheel_pos * 3);
            g = static_cast<uint8_t>(wheel_pos * 3);
        } else if (wheel_pos < 170) {
            wheel_pos = static_cast<uint8_t>(wheel_pos - 85);
            g = static_cast<uint8_t>(255 - wheel_pos * 3);
            b = static_cast<uint8_t>(wheel_pos * 3);
        } else {
            wheel_pos = static_cast<uint8_t>(wheel_pos - 170);
            b = static_cast<uint8_t>(255 - wheel_pos * 3);
            r = static_cast<uint8_t>(wheel_pos * 3);
        }

        red = static_cast<uint8_t>(static_cast<uint16_t>(r) * brightness / 255);
        green = static_cast<uint8_t>(static_cast<uint16_t>(g) * brightness / 255);
        blue = static_cast<uint8_t>(static_cast<uint16_t>(b) * brightness / 255);
    }

    void flush_pixels()
    {
        if (_tx_channel == nullptr || _encoder == nullptr) {
            return;
        }

        rmt_transmit_config_t tx_config = {};
        tx_config.loop_count = 0;
        esp_err_t err = rmt_transmit(_tx_channel, _encoder, _pixels.data(), _pixels.size(), &tx_config);
        if (err != ESP_OK) {
            mclog::tagError(_tag, "led strip rmt_transmit failed: {}", esp_err_to_name(err));
            _last_error = "transmit_failed";
            return;
        }

        err = rmt_tx_wait_all_done(_tx_channel, pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            mclog::tagError(_tag, "led strip rmt_tx_wait_all_done failed: {}", esp_err_to_name(err));
            _last_error = "transmit_timeout";
        }
    }
};

static LedStripController _led_strip_controller;

static ReturnValue _led_strip_result(bool success, const char* mode)
{
    if (success) {
        return fmt::format(R"({{"success":true,"mode":"{}","gpio":9}})", mode);
    }
    return fmt::format(R"({{"success":false,"error":"{}","gpio":9}})", _led_strip_controller.lastError());
}

static ReturnValue _switch_next_background_image(StackChan& stackchan)
{
    if (!stackchan.hasAvatar()) {
        return R"({"success":false,"error":"avatar_not_ready"})";
    }

    constexpr size_t max_image_size = 512 * 1024;
    const size_t image_count        = sizeof(_background_image_urls) / sizeof(_background_image_urls[0]);
    size_t image_index              = _next_background_image_index % image_count;
    const char* url                 = _background_image_urls[image_index];

    mclog::tagInfo(_tag, "next_background_image: index={}, url={}", image_index + 1, url);

    auto http = Board::GetInstance().GetNetwork()->CreateHttp(3);
    if (!http->Open("GET", url)) {
        mclog::tagError(_tag, "open image url failed: {}", url);
        return R"({"success":false,"error":"open_url_failed"})";
    }

    int status_code = http->GetStatusCode();
    if (status_code != 200) {
        mclog::tagError(_tag, "unexpected image status: {}", status_code);
        http->Close();
        return fmt::format(R"({{"success":false,"error":"bad_http_status","status":{}}})", status_code);
    }

    size_t content_length = http->GetBodyLength();
    if (content_length == 0 || content_length > max_image_size) {
        mclog::tagError(_tag, "invalid image size: {}", content_length);
        http->Close();
        return fmt::format(R"({{"success":false,"error":"invalid_image_size","size":{}}})", content_length);
    }

    char* data = (char*)heap_caps_malloc(content_length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (data == nullptr) {
        mclog::tagError(_tag, "alloc image failed: {} bytes", content_length);
        http->Close();
        return fmt::format(R"({{"success":false,"error":"alloc_failed","size":{}}})", content_length);
    }

    size_t total_read = 0;
    while (total_read < content_length) {
        int ret = http->Read(data + total_read, content_length - total_read);
        if (ret < 0) {
            mclog::tagError(_tag, "read image failed");
            heap_caps_free(data);
            http->Close();
            return R"({"success":false,"error":"read_failed"})";
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
        return fmt::format(R"({{"success":false,"error":"download_incomplete","read":{},"size":{}}})", total_read,
                           content_length);
    }

    std::unique_ptr<LvglImage> image;
    uint8_t* raw_data = nullptr;
    size_t raw_len    = 0;
    size_t width      = 0;
    size_t height     = 0;
    size_t stride     = 0;
    esp_err_t decode_ret = jpeg_to_image(reinterpret_cast<const uint8_t*>(data), content_length, &raw_data, &raw_len,
                                         &width, &height, &stride);
    heap_caps_free(data);

    if (decode_ret != ESP_OK || raw_data == nullptr) {
        mclog::tagError(_tag, "decode image failed: ret={}", static_cast<int>(decode_ret));
        return fmt::format(R"({{"success":false,"error":"decode_failed","code":{}}})", static_cast<int>(decode_ret));
    }

    try {
        image = std::make_unique<LvglAllocatedImage>(raw_data, raw_len, width, height, stride, LV_COLOR_FORMAT_RGB565);
    } catch (const std::exception& e) {
        mclog::tagError(_tag, "create image failed: {}", e.what());
        heap_caps_free(raw_data);
        return fmt::format(R"({{"success":false,"error":"decode_failed","message":"{}"}})", e.what());
    }

    {
        LvglLockGuard lock;
        stackchan.avatar().setBackgroundImage(std::move(image));
    }

    _next_background_image_index = (image_index + 1) % image_count;
    return fmt::format(R"({{"success":true,"index":{},"url":"{}"}})", image_index + 1, url);
}

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
                       "Speed(100-1000, 150 is natural). Use self.robot.go_home when the user asks to return the "
                       "head to neutral, center, or home position.",
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
                           motion.setAutoAngleSyncEnabled(false);
                           if (pitch != -9999) {
                               motion.pitchServo().moveWithSpeed(pitch * 10, speed);
                           }
                           if (yaw != -9999) {
                               motion.yawServo().moveWithSpeed(yaw * 10, speed);
                           }
                           motion.setAutoAngleSyncEnabled(true);

                           return true;
                       });

    mclog::tagInfo(_tag, "add robot.go_home tool");
    mcp_server.AddTool("self.robot.go_home",
                       "Move the robot head smoothly back to the neutral home position. Use this when the user says "
                       "head back to center, face forward, reset your head, go home, or return the head to normal.",
                       PropertyList({Property("speed", kPropertyTypeInteger, 500, 100, 1000)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int speed = properties["speed"].value<int>();
                           mclog::tagInfo(_tag, "go_home: speed={}", speed);

                           LvglLockGuard lock;

                           auto& motion = GetStackChan().motion();
                           motion.setAutoAngleSyncEnabled(false);
                           motion.moveWithSpeed(0, 0, speed);
                           motion.setAutoAngleSyncEnabled(true);

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

    mclog::tagInfo(_tag, "add robot.dance tool");
    mcp_server.AddTool("self.robot.dance",
                       "Make the robot dance using built-in dance motions. This is supported. Use this when the user "
                       "asks the robot to dance, move happily, shake, perform a robot dance, or look around. Styles: "
                       "happy, robot, panic, look_around.",
                       PropertyList({Property("style", kPropertyTypeString, std::string("happy"))}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           std::string style = properties["style"].value<std::string>();
                           const auto* sequence = &DanceModifier::Happy;

                           if (style == "robot") {
                               sequence = &DanceModifier::Robot;
                           } else if (style == "panic" || style == "shake") {
                               sequence = &DanceModifier::Panic;
                           } else if (style == "look_around" || style == "look around") {
                               sequence = &DanceModifier::LookAround;
                               style    = "look_around";
                           } else {
                               style = "happy";
                           }

                           mclog::tagInfo(_tag, "dance: style={}", style);

                           LvglLockGuard lock;

                           auto& stackchan = GetStackChan();
                           if (!stackchan.hasAvatar()) {
                               return R"({"success":false,"error":"avatar_not_ready"})";
                           }

                           int id = stackchan.addModifier(std::make_unique<DanceModifier>(*sequence));
                           return fmt::format(R"({{"success":true,"style":"{}","modifier_id":{}}})", style, id);
                       });

    mclog::tagInfo(_tag, "add system.show_apps tool");
    mcp_server.AddTool("self.system.show_apps",
                       "Return to the all apps launcher screen. This restarts the device into the app list because the "
                       "AI agent runtime does not return to the launcher directly. Use this when the user asks to go "
                       "back, return to all apps, show the app list, open the launcher, or exit the AI agent.",
                       std::vector<Property>{}, [this](const PropertyList& properties) -> ReturnValue {
                           mclog::tagInfo(_tag, "show_apps requested");
                           GetHAL().requestWarmReboot(0);
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

    mclog::tagInfo(_tag, "add screen.set_expression_color tool");
    mcp_server.AddTool("self.screen.set_expression_color",
                       "Set the avatar expression foreground color using RGB values. This changes the robot face eyes "
                       "and mouth color, not the background, wallpaper, room lights, or onboard LED.",
                       PropertyList({Property("red", kPropertyTypeInteger, 255, 0, 255),
                                     Property("green", kPropertyTypeInteger, 255, 0, 255),
                                     Property("blue", kPropertyTypeInteger, 255, 0, 255)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int r = properties["red"].value<int>();
                           int g = properties["green"].value<int>();
                           int b = properties["blue"].value<int>();

                           mclog::tagInfo(_tag, "set_expression_color: r={}, g={}, b={}", r, g, b);

                           LvglLockGuard lock;

                           auto& stackchan = GetStackChan();
                           if (!stackchan.hasAvatar()) {
                               return false;
                           }

                           uint32_t color = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) |
                                            static_cast<uint32_t>(b);
                           stackchan.avatar().setExpressionColor(lv_color_hex(color));

                           return true;
                       });

    mclog::tagInfo(_tag, "add led_strip.set_color tool");
    mcp_server.AddTool("self.led_strip.set_color",
                       "Set the external WS2812/S3 Chain LED strip connected to GPIO9 to a solid RGB color. This is "
                       "for the external light strip, not the robot face background or onboard LEDs.",
                       PropertyList({Property("red", kPropertyTypeInteger, 255, 0, 255),
                                     Property("green", kPropertyTypeInteger, 255, 0, 255),
                                     Property("blue", kPropertyTypeInteger, 255, 0, 255),
                                     Property("brightness", kPropertyTypeInteger, 32, 0, 255)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int r = properties["red"].value<int>();
                           int g = properties["green"].value<int>();
                           int b = properties["blue"].value<int>();
                           int brightness = properties["brightness"].value<int>();

                           mclog::tagInfo(_tag, "led_strip set_color: r={}, g={}, b={}, brightness={}", r, g, b,
                                          brightness);
                           return _led_strip_result(_led_strip_controller.setColor(r, g, b, brightness), "solid");
                       });

    mclog::tagInfo(_tag, "add led_strip.blink tool");
    mcp_server.AddTool("self.led_strip.blink",
                       "Blink the external WS2812/S3 Chain LED strip connected to GPIO9 using an RGB color.",
                       PropertyList({Property("red", kPropertyTypeInteger, 255, 0, 255),
                                     Property("green", kPropertyTypeInteger, 255, 0, 255),
                                     Property("blue", kPropertyTypeInteger, 255, 0, 255),
                                     Property("brightness", kPropertyTypeInteger, 32, 0, 255),
                                     Property("interval_ms", kPropertyTypeInteger, 400, 50, 5000)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int r = properties["red"].value<int>();
                           int g = properties["green"].value<int>();
                           int b = properties["blue"].value<int>();
                           int brightness = properties["brightness"].value<int>();
                           int interval_ms = properties["interval_ms"].value<int>();

                           mclog::tagInfo(_tag, "led_strip blink: r={}, g={}, b={}, brightness={}, interval={}ms", r,
                                          g, b, brightness, interval_ms);
                           return _led_strip_result(_led_strip_controller.blink(r, g, b, brightness, interval_ms),
                                                    "blink");
                       });

    mclog::tagInfo(_tag, "add led_strip.rainbow tool");
    mcp_server.AddTool("self.led_strip.rainbow",
                       "Show a moving rainbow animation on the external WS2812/S3 Chain LED strip connected to GPIO9.",
                       PropertyList({Property("brightness", kPropertyTypeInteger, 30, 0, 255),
                                     Property("speed_ms", kPropertyTypeInteger, 35, 20, 1000)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int brightness = properties["brightness"].value<int>();
                           int speed_ms = properties["speed_ms"].value<int>();

                           mclog::tagInfo(_tag, "led_strip rainbow: brightness={}, speed={}ms", brightness, speed_ms);
                           return _led_strip_result(_led_strip_controller.rainbow(brightness, speed_ms), "rainbow");
                       });

    mclog::tagInfo(_tag, "add led_strip.chase tool");
    mcp_server.AddTool("self.led_strip.chase",
                       "Run a marquee/chase animation on the external WS2812/S3 Chain LED strip connected to GPIO9.",
                       PropertyList({Property("red", kPropertyTypeInteger, 0, 0, 255),
                                     Property("green", kPropertyTypeInteger, 220, 0, 255),
                                     Property("blue", kPropertyTypeInteger, 255, 0, 255),
                                     Property("brightness", kPropertyTypeInteger, 36, 0, 255),
                                     Property("speed_ms", kPropertyTypeInteger, 25, 20, 1000),
                                     Property("width", kPropertyTypeInteger, 10, 1, 40)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int r = properties["red"].value<int>();
                           int g = properties["green"].value<int>();
                           int b = properties["blue"].value<int>();
                           int brightness = properties["brightness"].value<int>();
                           int speed_ms = properties["speed_ms"].value<int>();
                           int width = properties["width"].value<int>();

                           mclog::tagInfo(_tag,
                                          "led_strip chase: r={}, g={}, b={}, brightness={}, speed={}ms, width={}", r,
                                          g, b, brightness, speed_ms, width);
                           return _led_strip_result(_led_strip_controller.chase(r, g, b, brightness, speed_ms, width),
                                                    "chase");
                       });

    mclog::tagInfo(_tag, "add led_strip.neon tool");
    mcp_server.AddTool("self.led_strip.neon",
                       "Show a neon sign style animation on the external WS2812/S3 Chain LED strip connected to GPIO9. "
                       "Use this for neon, cyberpunk, nightclub, or signboard lighting effects.",
                       PropertyList({Property("brightness", kPropertyTypeInteger, 36, 0, 255),
                                     Property("speed_ms", kPropertyTypeInteger, 45, 20, 1000)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int brightness = properties["brightness"].value<int>();
                           int speed_ms = properties["speed_ms"].value<int>();

                           mclog::tagInfo(_tag, "led_strip neon: brightness={}, speed={}ms", brightness, speed_ms);
                           return _led_strip_result(_led_strip_controller.neon(brightness, speed_ms), "neon");
                       });

    mclog::tagInfo(_tag, "add led_strip.aurora tool");
    mcp_server.AddTool("self.led_strip.aurora",
                       "Show a soft aurora / northern lights animation on the external WS2812/S3 Chain LED strip "
                       "connected to GPIO9.",
                       PropertyList({Property("brightness", kPropertyTypeInteger, 32, 0, 255),
                                     Property("speed_ms", kPropertyTypeInteger, 60, 20, 2000)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int brightness = properties["brightness"].value<int>();
                           int speed_ms = properties["speed_ms"].value<int>();

                           mclog::tagInfo(_tag, "led_strip aurora: brightness={}, speed={}ms", brightness, speed_ms);
                           return _led_strip_result(_led_strip_controller.aurora(brightness, speed_ms), "aurora");
                       });

    mclog::tagInfo(_tag, "add led_strip.breath tool");
    mcp_server.AddTool("self.led_strip.breath",
                       "Show a breathing light effect on the external WS2812/S3 Chain LED strip connected to GPIO9.",
                       PropertyList({Property("red", kPropertyTypeInteger, 0, 0, 255),
                                     Property("green", kPropertyTypeInteger, 220, 0, 255),
                                     Property("blue", kPropertyTypeInteger, 255, 0, 255),
                                     Property("brightness", kPropertyTypeInteger, 50, 0, 255),
                                     Property("speed_ms", kPropertyTypeInteger, 35, 20, 1000)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int r = properties["red"].value<int>();
                           int g = properties["green"].value<int>();
                           int b = properties["blue"].value<int>();
                           int brightness = properties["brightness"].value<int>();
                           int speed_ms = properties["speed_ms"].value<int>();

                           mclog::tagInfo(_tag, "led_strip breath: r={}, g={}, b={}, brightness={}, speed={}ms", r, g,
                                          b, brightness, speed_ms);
                           return _led_strip_result(_led_strip_controller.breath(r, g, b, brightness, speed_ms),
                                                    "breath");
                       });

    mclog::tagInfo(_tag, "add led_strip.comet tool");
    mcp_server.AddTool("self.led_strip.comet",
                       "Show a moving comet with a fading tail on the external WS2812/S3 Chain LED strip connected to "
                       "GPIO9.",
                       PropertyList({Property("red", kPropertyTypeInteger, 255, 0, 255),
                                     Property("green", kPropertyTypeInteger, 255, 0, 255),
                                     Property("blue", kPropertyTypeInteger, 255, 0, 255),
                                     Property("brightness", kPropertyTypeInteger, 48, 0, 255),
                                     Property("speed_ms", kPropertyTypeInteger, 25, 20, 1000),
                                     Property("tail_width", kPropertyTypeInteger, 18, 2, 60)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int r = properties["red"].value<int>();
                           int g = properties["green"].value<int>();
                           int b = properties["blue"].value<int>();
                           int brightness = properties["brightness"].value<int>();
                           int speed_ms = properties["speed_ms"].value<int>();
                           int tail_width = properties["tail_width"].value<int>();

                           mclog::tagInfo(_tag,
                                          "led_strip comet: r={}, g={}, b={}, brightness={}, speed={}ms, tail={}", r,
                                          g, b, brightness, speed_ms, tail_width);
                           return _led_strip_result(
                               _led_strip_controller.comet(r, g, b, brightness, speed_ms, tail_width), "comet");
                       });

    mclog::tagInfo(_tag, "add led_strip.meteor tool");
    mcp_server.AddTool("self.led_strip.meteor",
                       "Show a meteor shower / random falling stars effect on the external WS2812/S3 Chain LED strip "
                       "connected to GPIO9.",
                       PropertyList({Property("brightness", kPropertyTypeInteger, 45, 0, 255),
                                     Property("speed_ms", kPropertyTypeInteger, 45, 20, 1000)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int brightness = properties["brightness"].value<int>();
                           int speed_ms = properties["speed_ms"].value<int>();

                           mclog::tagInfo(_tag, "led_strip meteor: brightness={}, speed={}ms", brightness, speed_ms);
                           return _led_strip_result(_led_strip_controller.meteor(brightness, speed_ms), "meteor");
                       });

    mclog::tagInfo(_tag, "add led_strip.theater tool");
    mcp_server.AddTool("self.led_strip.theater",
                       "Show a theater chase animation on the external WS2812/S3 Chain LED strip connected to GPIO9.",
                       PropertyList({Property("red", kPropertyTypeInteger, 255, 0, 255),
                                     Property("green", kPropertyTypeInteger, 180, 0, 255),
                                     Property("blue", kPropertyTypeInteger, 0, 0, 255),
                                     Property("brightness", kPropertyTypeInteger, 42, 0, 255),
                                     Property("speed_ms", kPropertyTypeInteger, 80, 20, 1000),
                                     Property("spacing", kPropertyTypeInteger, 3, 2, 16)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int r = properties["red"].value<int>();
                           int g = properties["green"].value<int>();
                           int b = properties["blue"].value<int>();
                           int brightness = properties["brightness"].value<int>();
                           int speed_ms = properties["speed_ms"].value<int>();
                           int spacing = properties["spacing"].value<int>();

                           mclog::tagInfo(_tag,
                                          "led_strip theater: r={}, g={}, b={}, brightness={}, speed={}ms, spacing={}",
                                          r, g, b, brightness, speed_ms, spacing);
                           return _led_strip_result(
                               _led_strip_controller.theater(r, g, b, brightness, speed_ms, spacing), "theater");
                       });

    mclog::tagInfo(_tag, "add led_strip.clear tool");
    mcp_server.AddTool("self.led_strip.clear",
                       "Turn off the external WS2812/S3 Chain LED strip connected to GPIO9.", std::vector<Property>{},
                       [this](const PropertyList& properties) -> ReturnValue {
                           mclog::tagInfo(_tag, "led_strip clear");
                           return _led_strip_result(_led_strip_controller.clear(), "off");
                       });

    mclog::tagInfo(_tag, "add screen.next_background_image tool");
    mcp_server.AddTool("self.screen.next_background_image",
                       "Switch the avatar screen background to the next preconfigured wallpaper image. "
                       "This uses the device's built-in preset list and does not require a URL, upload, search, or "
                       "arbitrary online image support. IMPORTANT: when the user asks whether you can change the "
                       "background picture, wallpaper, photo, or image, use this tool instead of saying unsupported.",
                       std::vector<Property>{}, [this](const PropertyList& properties) -> ReturnValue {
                            return _switch_next_background_image(GetStackChan());
                        });

    mclog::tagInfo(_tag, "add screen.change_wallpaper tool");
    mcp_server.AddTool("self.screen.change_wallpaper",
                       "Change the robot face wallpaper/background image to the next built-in preset. This tool is "
                       "supported. Use it whenever the user asks to change a background image, wallpaper, picture, or "
                       "photo. It requires no parameters.",
                       std::vector<Property>{}, [this](const PropertyList& properties) -> ReturnValue {
                            return _switch_next_background_image(GetStackChan());
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
