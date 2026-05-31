/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "default.h"

using namespace uitk::lvgl_cpp;
using namespace stackchan::avatar;

void DefaultAvatar::init(lv_obj_t* parent, const lv_font_t* font)
{
    _pannel = std::make_unique<Container>(parent);
    _pannel->align(LV_ALIGN_CENTER, 0, 0);
    _pannel->setSize(320, 240);
    _pannel->setRadius(0);
    _pannel->setBorderWidth(0);
    _pannel->setBgColor(secondaryColor);
    _pannel->removeFlag(LV_OBJ_FLAG_SCROLLABLE);

    _background_image = lv_image_create(_pannel->get());
    lv_obj_set_size(_background_image, 320, 240);
    lv_obj_align(_background_image, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(_background_image, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_background(_background_image);

    _key_elements.leftEye  = std::make_unique<DefaultEyes>(_pannel->get(), primaryColor, secondaryColor, true);
    _key_elements.rightEye = std::make_unique<DefaultEyes>(_pannel->get(), primaryColor, secondaryColor, false);
    _key_elements.mouth    = std::make_unique<DefaultMouth>(_pannel->get(), primaryColor, secondaryColor);
    _key_elements.speechBubble =
        std::make_unique<DefaultSpeechBubble>(_pannel->get(), primaryColor, secondaryColor, font);
}

Container* DefaultAvatar::getPanel() const
{
    if (_pannel) {
        return _pannel.get();
    }
    return NULL;
}

void DefaultAvatar::setBackgroundColor(lv_color_t color)
{
    secondaryColor = color;
    if (_pannel) {
        _pannel->setBgColor(color);
    }
    if (_key_elements.leftEye) {
        static_cast<DefaultEyes*>(_key_elements.leftEye.get())->setBackgroundColor(color);
    }
    if (_key_elements.rightEye) {
        static_cast<DefaultEyes*>(_key_elements.rightEye.get())->setBackgroundColor(color);
    }
    if (_key_elements.speechBubble) {
        static_cast<DefaultSpeechBubble*>(_key_elements.speechBubble.get())->setTextColor(color);
    }
}

void DefaultAvatar::setBackgroundImage(std::unique_ptr<LvglImage> image)
{
    if (!_background_image) {
        return;
    }

    if (!image) {
        lv_obj_add_flag(_background_image, LV_OBJ_FLAG_HIDDEN);
        _background_image_cached.reset();
        return;
    }

    _background_image_cached = std::move(image);
    auto img_dsc             = _background_image_cached->image_dsc();
    lv_image_set_src(_background_image, img_dsc);
    if (img_dsc->header.w > 0 && img_dsc->header.h > 0) {
        auto scale_w = 256 * 320 / img_dsc->header.w;
        auto scale_h = 256 * 240 / img_dsc->header.h;
        lv_image_set_scale(_background_image, scale_w > scale_h ? scale_w : scale_h);
    }
    lv_obj_remove_flag(_background_image, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_background(_background_image);
}
