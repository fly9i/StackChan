/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <mooncake.h>

/**
 * @brief Derived App
 *
 */
class AppAiAgent : public mooncake::AppAbility {
public:
    enum class Profile {
        Official,
        ZfAgent,
    };

    explicit AppAiAgent(Profile profile = Profile::Official);

    // Override lifecycle callbacks
    void onCreate() override;
    void onOpen() override;
    void onRunning() override;
    void onClose() override;

private:
    Profile _profile = Profile::Official;
};
