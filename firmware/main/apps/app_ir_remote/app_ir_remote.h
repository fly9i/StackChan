/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <driver/rmt_encoder.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <mooncake.h>
#include <smooth_lvgl.hpp>
#include <uitk/short_namespace.hpp>
#include <memory>
#include <string>
#include <vector>

class AppIrRemote : public mooncake::AppAbility {
public:
    AppIrRemote();

    void onCreate() override;
    void onOpen() override;
    void onRunning() override;
    void onClose() override;

private:
    struct IrSignal {
        std::vector<rmt_symbol_word_t> symbols;
    };

    struct RxEvent {
        size_t symbol_count = 0;
    };

    enum class LearnState {
        Idle,
        WaitingFirst,
        WaitingSecond,
        Learned,
    };

    static constexpr gpio_num_t kIrTxPin = GPIO_NUM_5;
    static constexpr gpio_num_t kIrRxPin = GPIO_NUM_10;
    static constexpr uint32_t kRmtResolutionHz = 1000000;
    static constexpr size_t kMaxSymbols = 512;
    static constexpr size_t kMaxSignals = 32;

    std::vector<IrSignal> _signals;
    std::vector<rmt_symbol_word_t> _rx_symbols;
    std::vector<rmt_symbol_word_t> _candidate_symbols;

    rmt_channel_handle_t _rx_channel = nullptr;
    rmt_channel_handle_t _tx_channel = nullptr;
    rmt_encoder_handle_t _copy_encoder = nullptr;
    QueueHandle_t _rx_queue = nullptr;
    bool _rx_pending = false;
    bool _rmt_ready = false;

    LearnState _learn_state = LearnState::Idle;
    bool _pending_learn = false;
    int _pending_send_index = -1;
    bool _ui_dirty = false;
    bool _list_dirty = false;
    std::string _status;

    std::unique_ptr<uitk::lvgl_cpp::Container> _panel;
    std::unique_ptr<uitk::lvgl_cpp::Label> _title_label;
    std::unique_ptr<uitk::lvgl_cpp::Label> _status_label;
    std::unique_ptr<uitk::lvgl_cpp::Button> _learn_button;
    std::unique_ptr<uitk::lvgl_cpp::Container> _list_panel;
    std::unique_ptr<uitk::lvgl_cpp::Label> _empty_label;
    std::vector<std::unique_ptr<uitk::lvgl_cpp::Button>> _signal_buttons;

    static bool on_rx_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t* event_data, void* user_data);

    void init_rmt();
    void release_rmt();
    void start_receive();
    void process_rx_events();
    void begin_learning();
    void send_signal(size_t index);
    bool save_signal(const std::vector<rmt_symbol_word_t>& symbols);
    void load_signals();
    bool signals_match(const std::vector<rmt_symbol_word_t>& lhs, const std::vector<rmt_symbol_word_t>& rhs) const;
    void create_ui();
    void rebuild_signal_buttons();
    void set_status(std::string status);
    std::string signal_label(size_t index) const;
};
