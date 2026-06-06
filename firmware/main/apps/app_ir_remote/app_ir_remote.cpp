/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "app_ir_remote.h"

#include <apps/common/common.h>
#include <assets/assets.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <hal/hal.h>
#include <mooncake_log.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

using namespace mooncake;
using namespace uitk::lvgl_cpp;

namespace {
constexpr const char* kNvsNamespace = "ir_remote";
constexpr const char* kNvsCountKey = "count";

std::string nvs_signal_key(size_t index)
{
    char key[16] = {};
    snprintf(key, sizeof(key), "sig%02u", static_cast<unsigned>(index));
    return key;
}
}  // namespace

AppIrRemote::AppIrRemote()
{
    setAppInfo().name = "IR REMOTE";
    static auto icon = assets::get_image("icon_controller.bin");
    setAppInfo().icon = (void*)&icon;
    static uint32_t theme_color = 0xFF8A3D;
    setAppInfo().userData = (void*)&theme_color;
}

void AppIrRemote::onCreate()
{
    mclog::tagInfo(getAppInfo().name, "on create");
}

void AppIrRemote::onOpen()
{
    mclog::tagInfo(getAppInfo().name, "on open");

    _signals.clear();
    _candidate_symbols.clear();
    _learn_state = LearnState::Idle;
    _pending_learn = false;
    _pending_send_index = -1;
    _rx_symbols.resize(kMaxSymbols);

    load_signals();
    init_rmt();

    LvglLockGuard lock;
    create_ui();
    view::create_home_indicator([&]() { close(); }, 0xFFB067, 0x2D1608);
    view::create_status_bar(0xFFB067, 0x2D1608);
}

void AppIrRemote::onRunning()
{
    bool learn_requested = false;
    int send_index = -1;

    {
        LvglLockGuard lock;

        learn_requested = _pending_learn;
        _pending_learn = false;

        send_index = _pending_send_index;
        _pending_send_index = -1;

        view::update_home_indicator();
        view::update_status_bar();
    }

    if (learn_requested) {
        begin_learning();
    }

    if (send_index >= 0) {
        send_signal(static_cast<size_t>(send_index));
    }

    process_rx_events();

    if (_ui_dirty || _list_dirty) {
        LvglLockGuard lock;
        if (_ui_dirty && _status_label) {
            _status_label->setText(_status);
            _ui_dirty = false;
        }
        if (_list_dirty) {
            rebuild_signal_buttons();
            _list_dirty = false;
        }
    }
}

void AppIrRemote::onClose()
{
    mclog::tagInfo(getAppInfo().name, "on close");

    release_rmt();

    LvglLockGuard lock;
    _signal_buttons.clear();
    _empty_label.reset();
    _list_panel.reset();
    _learn_button.reset();
    _status_label.reset();
    _title_label.reset();
    _panel.reset();
    view::destroy_home_indicator();
    view::destroy_status_bar();
}

bool AppIrRemote::on_rx_done(rmt_channel_handle_t, const rmt_rx_done_event_data_t* event_data, void* user_data)
{
    auto* app = static_cast<AppIrRemote*>(user_data);
    if (app == nullptr || app->_rx_queue == nullptr) {
        return false;
    }

    RxEvent event;
    event.symbol_count = event_data->num_symbols;

    BaseType_t high_task_wakeup = pdFALSE;
    xQueueSendFromISR(app->_rx_queue, &event, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void AppIrRemote::init_rmt()
{
    _rmt_ready = false;
    _rx_queue = xQueueCreate(4, sizeof(RxEvent));
    if (_rx_queue == nullptr) {
        set_status("RMT queue failed");
        return;
    }

    rmt_rx_channel_config_t rx_config = {};
    rx_config.gpio_num = kIrRxPin;
    rx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_config.resolution_hz = kRmtResolutionHz;
    rx_config.mem_block_symbols = 256;
    rx_config.flags.invert_in = false;

    esp_err_t err = rmt_new_rx_channel(&rx_config, &_rx_channel);
    if (err != ESP_OK) {
        mclog::tagError(getAppInfo().name, "rmt_new_rx_channel failed: {}", esp_err_to_name(err));
        set_status("IR RX init failed");
        release_rmt();
        return;
    }

    rmt_rx_event_callbacks_t callbacks = {};
    callbacks.on_recv_done = on_rx_done;
    err = rmt_rx_register_event_callbacks(_rx_channel, &callbacks, this);
    if (err != ESP_OK) {
        mclog::tagError(getAppInfo().name, "rmt_rx_register_event_callbacks failed: {}", esp_err_to_name(err));
        set_status("IR RX callback failed");
        release_rmt();
        return;
    }

    err = rmt_enable(_rx_channel);
    if (err != ESP_OK) {
        mclog::tagError(getAppInfo().name, "rmt_enable rx failed: {}", esp_err_to_name(err));
        set_status("IR RX enable failed");
        release_rmt();
        return;
    }

    rmt_tx_channel_config_t tx_config = {};
    tx_config.gpio_num = kIrTxPin;
    tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_config.resolution_hz = kRmtResolutionHz;
    tx_config.mem_block_symbols = 128;
    tx_config.trans_queue_depth = 2;

    err = rmt_new_tx_channel(&tx_config, &_tx_channel);
    if (err != ESP_OK) {
        mclog::tagError(getAppInfo().name, "rmt_new_tx_channel failed: {}", esp_err_to_name(err));
        set_status("IR TX init failed");
        release_rmt();
        return;
    }

    rmt_carrier_config_t carrier_config = {};
    carrier_config.frequency_hz = 38000;
    carrier_config.duty_cycle = 0.33;
    err = rmt_apply_carrier(_tx_channel, &carrier_config);
    if (err != ESP_OK) {
        mclog::tagError(getAppInfo().name, "rmt_apply_carrier failed: {}", esp_err_to_name(err));
        set_status("IR carrier failed");
        release_rmt();
        return;
    }

    rmt_copy_encoder_config_t copy_config = {};
    err = rmt_new_copy_encoder(&copy_config, &_copy_encoder);
    if (err != ESP_OK) {
        mclog::tagError(getAppInfo().name, "rmt_new_copy_encoder failed: {}", esp_err_to_name(err));
        set_status("IR encoder failed");
        release_rmt();
        return;
    }

    err = rmt_enable(_tx_channel);
    if (err != ESP_OK) {
        mclog::tagError(getAppInfo().name, "rmt_enable tx failed: {}", esp_err_to_name(err));
        set_status("IR TX enable failed");
        release_rmt();
        return;
    }

    _rmt_ready = true;
    set_status(_signals.empty() ? "Tap Learn, then press remote twice" : "Tap a saved button to transmit");
}

void AppIrRemote::release_rmt()
{
    if (_rx_channel != nullptr) {
        rmt_disable(_rx_channel);
        rmt_del_channel(_rx_channel);
        _rx_channel = nullptr;
    }

    if (_tx_channel != nullptr) {
        rmt_disable(_tx_channel);
        rmt_del_channel(_tx_channel);
        _tx_channel = nullptr;
    }

    if (_copy_encoder != nullptr) {
        rmt_del_encoder(_copy_encoder);
        _copy_encoder = nullptr;
    }

    if (_rx_queue != nullptr) {
        vQueueDelete(_rx_queue);
        _rx_queue = nullptr;
    }

    _rx_pending = false;
    _rmt_ready = false;
}

void AppIrRemote::start_receive()
{
    if (!_rmt_ready || _rx_pending || _rx_channel == nullptr || _rx_symbols.empty()) {
        return;
    }

    rmt_receive_config_t receive_config = {};
    receive_config.signal_range_min_ns = 1000;
    receive_config.signal_range_max_ns = 20000000;

    esp_err_t err = rmt_receive(_rx_channel, _rx_symbols.data(), _rx_symbols.size() * sizeof(rmt_symbol_word_t), &receive_config);
    if (err != ESP_OK) {
        mclog::tagError(getAppInfo().name, "rmt_receive failed: {}", esp_err_to_name(err));
        set_status("IR receive arm failed");
        return;
    }

    _rx_pending = true;
}

void AppIrRemote::process_rx_events()
{
    if (_rx_queue == nullptr) {
        return;
    }

    RxEvent event;
    while (xQueueReceive(_rx_queue, &event, 0) == pdTRUE) {
        _rx_pending = false;

        if (_learn_state != LearnState::WaitingFirst && _learn_state != LearnState::WaitingSecond) {
            continue;
        }

        const size_t symbol_count = std::min(event.symbol_count, _rx_symbols.size());
        if (symbol_count < 8) {
            set_status("Signal too short, try again");
            start_receive();
            continue;
        }

        std::vector<rmt_symbol_word_t> captured(_rx_symbols.begin(), _rx_symbols.begin() + symbol_count);
        mclog::tagInfo(getAppInfo().name, "captured {} symbols", captured.size());

        if (_learn_state == LearnState::WaitingFirst || _candidate_symbols.empty()) {
            _candidate_symbols = std::move(captured);
            _learn_state = LearnState::WaitingSecond;
            set_status("First captured, press same key again");
            start_receive();
            continue;
        }

        if (!signals_match(_candidate_symbols, captured)) {
            _candidate_symbols = std::move(captured);
            set_status("Different signal, press same key again");
            start_receive();
            continue;
        }

        if (save_signal(_candidate_symbols)) {
            _learn_state = LearnState::Learned;
            _candidate_symbols.clear();
            set_status("Learning complete, button saved");
            _list_dirty = true;
        } else {
            _learn_state = LearnState::Idle;
            _candidate_symbols.clear();
        }
    }
}

void AppIrRemote::begin_learning()
{
    if (!_rmt_ready) {
        set_status("IR hardware not ready");
        return;
    }
    if (_signals.size() >= kMaxSignals) {
        set_status("Saved buttons full");
        return;
    }

    _candidate_symbols.clear();
    _learn_state = LearnState::WaitingFirst;
    set_status("Learning: press a remote key");
    start_receive();
}

void AppIrRemote::send_signal(size_t index)
{
    if (!_rmt_ready || _tx_channel == nullptr || _copy_encoder == nullptr) {
        set_status("IR hardware not ready");
        return;
    }
    if (index >= _signals.size() || _signals[index].symbols.empty()) {
        set_status("Saved signal missing");
        return;
    }

    std::vector<rmt_symbol_word_t> tx_symbols = _signals[index].symbols;
    for (auto& symbol : tx_symbols) {
        // Demodulated IR receivers output active-low marks; the emitter needs active-high carrier marks.
        symbol.level0 = !symbol.level0;
        symbol.level1 = !symbol.level1;
    }

    rmt_transmit_config_t transmit_config = {};
    transmit_config.loop_count = 0;
    esp_err_t err = rmt_transmit(_tx_channel, _copy_encoder, tx_symbols.data(),
                                 tx_symbols.size() * sizeof(rmt_symbol_word_t), &transmit_config);
    if (err != ESP_OK) {
        mclog::tagError(getAppInfo().name, "rmt_transmit failed: {}", esp_err_to_name(err));
        set_status("Transmit failed");
        return;
    }

    rmt_tx_wait_all_done(_tx_channel, 1000);
    set_status(signal_label(index) + " transmitted");
}

bool AppIrRemote::save_signal(const std::vector<rmt_symbol_word_t>& symbols)
{
    if (symbols.empty() || _signals.size() >= kMaxSignals) {
        set_status("Saved buttons full");
        return false;
    }

    const size_t index = _signals.size();
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        mclog::tagError(getAppInfo().name, "nvs_open failed: {}", esp_err_to_name(err));
        set_status("Save failed");
        return false;
    }

    const auto key = nvs_signal_key(index);
    err = nvs_set_blob(handle, key.c_str(), symbols.data(), symbols.size() * sizeof(rmt_symbol_word_t));
    if (err == ESP_OK) {
        err = nvs_set_u32(handle, kNvsCountKey, static_cast<uint32_t>(index + 1));
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        mclog::tagError(getAppInfo().name, "nvs save failed: {}", esp_err_to_name(err));
        set_status("Save failed");
        return false;
    }

    _signals.push_back(IrSignal{symbols});
    return true;
}

void AppIrRemote::load_signals()
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return;
    }

    uint32_t count = 0;
    if (nvs_get_u32(handle, kNvsCountKey, &count) != ESP_OK) {
        nvs_close(handle);
        return;
    }

    count = std::min<uint32_t>(count, kMaxSignals);
    for (uint32_t i = 0; i < count; ++i) {
        const auto key = nvs_signal_key(i);
        size_t blob_size = 0;
        if (nvs_get_blob(handle, key.c_str(), nullptr, &blob_size) != ESP_OK || blob_size == 0 ||
            blob_size % sizeof(rmt_symbol_word_t) != 0) {
            continue;
        }

        IrSignal signal;
        signal.symbols.resize(blob_size / sizeof(rmt_symbol_word_t));
        if (nvs_get_blob(handle, key.c_str(), signal.symbols.data(), &blob_size) == ESP_OK) {
            _signals.push_back(std::move(signal));
        }
    }

    nvs_close(handle);
}

bool AppIrRemote::signals_match(const std::vector<rmt_symbol_word_t>& lhs,
                                const std::vector<rmt_symbol_word_t>& rhs) const
{
    if (lhs.size() != rhs.size()) {
        return false;
    }

    for (size_t i = 0; i < lhs.size(); ++i) {
        if (lhs[i].level0 != rhs[i].level0 || lhs[i].level1 != rhs[i].level1) {
            return false;
        }

        const int d0_l = static_cast<int>(lhs[i].duration0);
        const int d0_r = static_cast<int>(rhs[i].duration0);
        const int d1_l = static_cast<int>(lhs[i].duration1);
        const int d1_r = static_cast<int>(rhs[i].duration1);
        const int tol0 = std::max(180, d0_l / 4);
        const int tol1 = std::max(180, d1_l / 4);
        if (std::abs(d0_l - d0_r) > tol0 || std::abs(d1_l - d1_r) > tol1) {
            return false;
        }
    }

    return true;
}

void AppIrRemote::create_ui()
{
    _panel = std::make_unique<Container>(lv_screen_active());
    _panel->setSize(320, 240);
    _panel->setBgColor(lv_color_hex(0x120B08));
    _panel->setBorderWidth(0);
    _panel->setRadius(0);
    _panel->removeFlag(LV_OBJ_FLAG_SCROLLABLE);

    _title_label = std::make_unique<Label>(*_panel);
    _title_label->setText("IR REMOTE");
    _title_label->setTextFont(&lv_font_montserrat_20);
    _title_label->setTextColor(lv_color_hex(0xFFE3C2));
    _title_label->align(LV_ALIGN_TOP_MID, 0, 25);

    _status_label = std::make_unique<Label>(*_panel);
    _status_label->setText(_status);
    _status_label->setTextFont(&lv_font_montserrat_14);
    _status_label->setTextColor(lv_color_hex(0xFFC47D));
    _status_label->setWidth(288);
    _status_label->setTextAlign(LV_TEXT_ALIGN_CENTER);
    _status_label->align(LV_ALIGN_TOP_MID, 0, 52);

    _learn_button = std::make_unique<Button>(*_panel);
    _learn_button->setSize(282, 42);
    _learn_button->align(LV_ALIGN_TOP_MID, 0, 76);
    _learn_button->setBgColor(lv_color_hex(0xFF8A3D));
    _learn_button->setBorderWidth(0);
    _learn_button->setShadowWidth(0);
    _learn_button->setRadius(17);
    _learn_button->label().setText("Learn New Button");
    _learn_button->label().setTextFont(&lv_font_montserrat_16);
    _learn_button->label().setTextColor(lv_color_hex(0x211007));
    _learn_button->label().align(LV_ALIGN_CENTER, 0, 0);
    _learn_button->onClick().connect([this]() { _pending_learn = true; });

    _list_panel = std::make_unique<Container>(*_panel);
    _list_panel->setSize(320, 104);
    _list_panel->align(LV_ALIGN_BOTTOM_MID, 0, -22);
    _list_panel->setBgOpa(0);
    _list_panel->setBorderWidth(0);
    _list_panel->setRadius(0);
    _list_panel->setPadding(0, 12, 0, 0);
    _list_panel->setScrollDir(LV_DIR_VER);
    _list_panel->setScrollbarMode(LV_SCROLLBAR_MODE_ACTIVE);

    rebuild_signal_buttons();
}

void AppIrRemote::rebuild_signal_buttons()
{
    _signal_buttons.clear();
    _empty_label.reset();

    if (_signals.empty()) {
        _empty_label = std::make_unique<Label>(*_list_panel);
        _empty_label->setText("No learned buttons yet");
        _empty_label->setTextFont(&lv_font_montserrat_16);
        _empty_label->setTextColor(lv_color_hex(0x8D7568));
        _empty_label->align(LV_ALIGN_TOP_MID, 0, 18);
        return;
    }

    int cursor_y = 0;
    for (size_t i = 0; i < _signals.size(); ++i) {
        auto button = std::make_unique<Button>(*_list_panel);
        button->setSize(282, 42);
        button->align(LV_ALIGN_TOP_MID, 0, cursor_y);
        button->setBgColor(lv_color_hex(0x2D211C));
        button->setBorderColor(lv_color_hex(0x8C4F2D));
        button->setBorderWidth(1);
        button->setShadowWidth(0);
        button->setRadius(16);
        button->label().setText(signal_label(i));
        button->label().setTextFont(&lv_font_montserrat_16);
        button->label().setTextColor(lv_color_hex(0xFFE3C2));
        button->label().align(LV_ALIGN_CENTER, 0, 0);

        const int send_index = static_cast<int>(i);
        button->onClick().connect([this, send_index]() { _pending_send_index = send_index; });
        _signal_buttons.push_back(std::move(button));
        cursor_y += 50;
    }
}

void AppIrRemote::set_status(std::string status)
{
    _status = std::move(status);
    _ui_dirty = true;
}

std::string AppIrRemote::signal_label(size_t index) const
{
    char label[24] = {};
    snprintf(label, sizeof(label), "IR %02u  Send", static_cast<unsigned>(index + 1));
    return label;
}
