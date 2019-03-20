/* Copyright 2015-2017 Jack Humbert
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "muse.h"

extern keymap_config_t keymap_config;

enum planck_layers {
  _QWERTY,
  _LOWER,
  _RAISE,
  _MOVEL,
  _MOVER,
  _PLOVER,
  _ADJUST
};

enum planck_keycodes {
  QWERTY = SAFE_RANGE,
  PLOVER,
  BACKLIT,
  EXT_PLV
};

#define GRID(...) LAYOUT_planck_grid(__VA_ARGS__)

#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)
#define MOVEL MO(_MOVEL)
#define MOVER MO(_MOVER)

#define MY_PIPE S(KC_NUBS)
#define MY_STLD S(KC_NUHS)
#define MY_AT   KC_DQT
#define MY_DQT  S(KC_2)
#define MY_QUID KC_HASH

#define MU_REC KC_LCTL
#define MU_STOP KC_LALT
#define MU_PLAY KC_LGUI

// LP1 = Left pinky first row
// LF1 = Left fingers first row
// LTH = Left thumb
#define _QWERTY_LP1_ KC_TAB
#define _QWERTY_LP2_ LT(MOVEL, KC_ESC)
#define _QWERTY_LP3_ KC_NUBS
#define _QWERTY_LP4_ KC_TAB

#define _QWERTY_LF1_ KC_Q, KC_W, KC_E, KC_R, KC_T
#define _QWERTY_LF2_ LSFT_T(KC_A), LCTL_T(KC_S), LGUI_T(KC_D), LALT_T(KC_F), KC_G
#define _QWERTY_LF3_ KC_Z, KC_X, KC_C, KC_V, KC_B
#define _QWERTY_LTH_ KC_MUTE, XXXXXXX, KC_LBRC, LOWER, KC_ENT

#define _QWERTY_RP1_ KC_BSPC
#define _QWERTY_RP2_ LT(MOVER, KC_QUOT)
#define _QWERTY_RP3_ KC_ENT
#define _QWERTY_RP4_ KC_BSPC

#define _QWERTY_RF1_ KC_Y, KC_U, KC_I, KC_O, KC_P
#define _QWERTY_RF2_ KC_H, LALT_T(KC_J), RGUI_T(KC_K), RCTL_T(KC_L), RSFT_T(KC_SCLN)
#define _QWERTY_RF3_ KC_N, KC_M, KC_COMM, KC_DOT, KC_SLASH
#define _QWERTY_RTH_ KC_SPC, RAISE, KC_RBRC, KC_VOLD, KC_VOLU

#define _LOWER__LP1_ KC_LOCK
#define _LOWER__LP2_ KC_TAB
#define _LOWER__LP3_ _______
#define _LOWER__LP4_ _______

#define _LOWER__LF1_ KC_GRAVE, MY_DQT, MY_QUID, KC_DLR, KC_LCBR
#define _LOWER__LF2_ KC_EXLM, KC_NUHS, KC_EQL, KC_MINS, KC_LPRN
#define _LOWER__LF3_ KC_CIRC, KC_AMPR, KC_PERC, KC_ASTR, KC_LT
#define _LOWER__LTH_ XXXXXXX, XXXXXXX, _______, _______, KC_INS

#define _LOWER__RP1_ _______
#define _LOWER__RP2_ KC_DEL
#define _LOWER__RP3_ _______
#define _LOWER__RP4_ _______

#define _LOWER__RF1_ KC_RCBR, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX
#define _LOWER__RF2_ KC_RPRN, KC_LALT, KC_RGUI, KC_RCTL, KC_RSFT
#define _LOWER__RF3_ KC_GT, XXXXXXX, XXXXXXX, XXXXXXX, KC_SLASH
#define _LOWER__RTH_ KC_SPC, _______, _______, XXXXXXX, XXXXXXX

#define _RAISE__LP1_ _______
#define _RAISE__LP2_ _______
#define _RAISE__LP3_ _______
#define _RAISE__LP4_ _______

#define _RAISE__LF1_ XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX
#define _RAISE__LF2_ KC_LSFT, KC_LCTL, KC_LGUI, KC_LALT, KC_LPRN
#define _RAISE__LF3_ XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX
#define _RAISE__LTH_ XXXXXXX, XXXXXXX, _______, _______, KC_INS

#define _RAISE__RP1_ KC_LOCK
#define _RAISE__RP2_ KC_INS
#define _RAISE__RP3_ _______
#define _RAISE__RP4_ _______

#define _RAISE__RF1_ KC_MINS, KC_7, KC_8, KC_9, KC_PLUS
#define _RAISE__RF2_ KC_RPRN, KC_4, KC_5, KC_6, KC_ASTR
#define _RAISE__RF3_    KC_0, KC_1, KC_2, KC_3, KC_SLASH
#define _RAISE__RTH_ KC_SPC, _______, _______, XXXXXXX, XXXXXXX

#define _MOVEL__LP1_ _______
#define _MOVEL__LP2_ _______
#define _MOVEL__LP3_ _______
#define _MOVEL__LP4_ _______

#define _MOVEL__LF1_ KC_HOME, KC_UP  , KC_END , KC_PGUP, XXXXXXX
#define _MOVEL__LF2_ KC_LEFT, KC_DOWN, KC_RGHT, KC_PGDN, XXXXXXX
#define _MOVEL__LF3_ XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX
#define _MOVEL__LTH_ XXXXXXX, XXXXXXX, _______, _______, KC_INS

#define _MOVEL__RP1_ _______
#define _MOVEL__RP2_ _______
#define _MOVEL__RP3_ _______
#define _MOVEL__RP4_ _______

#define _MOVEL__RF1_ XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX
#define _MOVEL__RF2_ XXXXXXX, KC_LALT, KC_RGUI, KC_RCTL, KC_RSFT
#define _MOVEL__RF3_ XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX
#define _MOVEL__RTH_ KC_SPC , _______, _______, XXXXXXX, XXXXXXX

#define _MOVER__LP1_ _______
#define _MOVER__LP2_ _______
#define _MOVER__LP3_ _______
#define _MOVER__LP4_ _______

#define _MOVER__LF1_ XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_BTN2
#define _MOVER__LF2_ KC_LSFT, KC_LCTL, KC_LGUI, KC_LALT, KC_BTN1
#define _MOVER__LF3_ XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_BTN3
#define _MOVER__LTH_ XXXXXXX, XXXXXXX, _______, _______, KC_INS

#define _MOVER__RP1_ _______
#define _MOVER__RP2_ _______
#define _MOVER__RP3_ _______
#define _MOVER__RP4_ _______

#define _MOVER__RF1_ XXXXXXX, KC_WH_U, KC_BTN1, KC_MS_U, KC_BTN3
#define _MOVER__RF2_ XXXXXXX, KC_WH_D, KC_MS_L, KC_MS_D, KC_MS_R
#define _MOVER__RF3_ XXXXXXX, XXXXXXX, KC_ACL0, KC_ACL1, KC_ACL2
#define _MOVER__RTH_ KC_SPC , _______, _______, XXXXXXX, XXXXXXX

#define _ADJUST_LP1_ MUV_DE
#define _ADJUST_LP2_ MUV_IN
#define _ADJUST_LP3_ _______
#define _ADJUST_LP4_ _______

#define _ADJUST_LF1_ MU_MOD , MU_ON  , MU_OFF , AU_ON  , AU_OFF
#define _ADJUST_LF2_ KC_LSFT, KC_LCTL, KC_LGUI, KC_LALT, KC_UP
#define _ADJUST_LF3_ QWERTY , PLOVER , XXXXXXX, XXXXXXX, KC_DOWN
#define _ADJUST_LTH_ XXXXXXX, XXXXXXX, _______, _______, KC_INS

#define _ADJUST_RP1_ RESET
#define _ADJUST_RP2_ _______
#define _ADJUST_RP3_ _______
#define _ADJUST_RP4_ _______

#define _ADJUST_RF1_ KC_PSCR, KC_F7  , KC_F8  , KC_F9  , KC_F10
#define _ADJUST_RF2_ KC_PAUS, KC_F4  , KC_F5  , KC_F6  , KC_F11
#define _ADJUST_RF3_ KC_SLCK, KC_F1  , KC_F2  , KC_F3  , KC_F12
#define _ADJUST_RTH_ KC_SPC , _______, _______, XXXXXXX, XXXXXXX

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

// [_QWERTY] = GRID(
//     _QWERTY_LP1_, _QWERTY_LF1_, _QWERTY_RF1, _QWERTY_RP1_, 1 5 5 1 = 12
//     _QWERTY_LP2_, _QWERTY_LF2_, _QWERTY_RF2, _QWERTY_RP2_, 1 5 5 1 = 12
//     _QWERTY_LP3_, _QWERTY_LF3_, _QWERTY_RF3, _QWERTY_RP3_, 1 5 5 1 = 12
//     _QWERTY_LP4_, _QWERTY_LTH_, _QWERTY_RTH, _QWERTY_RP4_  1 5 5 1 = 12
// ),

[_QWERTY] = GRID(
    _QWERTY_LP1_, _QWERTY_LF1_, _QWERTY_RF1_, _QWERTY_RP1_,
    _QWERTY_LP2_, _QWERTY_LF2_, _QWERTY_RF2_, _QWERTY_RP2_,
    _QWERTY_LP3_, _QWERTY_LF3_, _QWERTY_RF3_, _QWERTY_RP3_,
    _QWERTY_LP4_, _QWERTY_LTH_, _QWERTY_RTH_, _QWERTY_RP4_
),

[_LOWER] = GRID(
    _LOWER__LP1_, _LOWER__LF1_, _LOWER__RF1_, _LOWER__RP1_,
    _LOWER__LP2_, _LOWER__LF2_, _LOWER__RF2_, _LOWER__RP2_,
    _LOWER__LP3_, _LOWER__LF3_, _LOWER__RF3_, _LOWER__RP3_,
    _LOWER__LP4_, _LOWER__LTH_, _LOWER__RTH_, _LOWER__RP4_
),

[_RAISE] = GRID(
    _RAISE__LP1_, _RAISE__LF1_, _RAISE__RF1_, _RAISE__RP1_,
    _RAISE__LP2_, _RAISE__LF2_, _RAISE__RF2_, _RAISE__RP2_,
    _RAISE__LP3_, _RAISE__LF3_, _RAISE__RF3_, _RAISE__RP3_,
    _RAISE__LP4_, _RAISE__LTH_, _RAISE__RTH_, _RAISE__RP4_
),

[_MOVEL] = GRID(
    _MOVEL__LP1_, _MOVEL__LF1_, _MOVEL__RF1_, _MOVEL__RP1_,
    _MOVEL__LP2_, _MOVEL__LF2_, _MOVEL__RF2_, _MOVEL__RP2_,
    _MOVEL__LP3_, _MOVEL__LF3_, _MOVEL__RF3_, _MOVEL__RP3_,
    _MOVEL__LP4_, _MOVEL__LTH_, _MOVEL__RTH_, _MOVEL__RP4_
),

[_MOVER] = GRID(
    _MOVER__LP1_, _MOVER__LF1_, _MOVER__RF1_, _MOVER__RP1_,
    _MOVER__LP2_, _MOVER__LF2_, _MOVER__RF2_, _MOVER__RP2_,
    _MOVER__LP3_, _MOVER__LF3_, _MOVER__RF3_, _MOVER__RP3_,
    _MOVER__LP4_, _MOVER__LTH_, _MOVER__RTH_, _MOVER__RP4_
),

/* Plover layer (http://opensteno.org) */
[_PLOVER] = GRID(
    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1   ,
    XXXXXXX, KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC,
    XXXXXXX, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,
    EXT_PLV, XXXXXXX, XXXXXXX, KC_C,    KC_V,    XXXXXXX, XXXXXXX, KC_N,    KC_M,    XXXXXXX, XXXXXXX, XXXXXXX
),

/* Adjust (LOWER + RAISE) */
[_ADJUST] = GRID(
    _ADJUST_LP1_, _ADJUST_LF1_, _ADJUST_RF1_, _ADJUST_RP1_,
    _ADJUST_LP2_, _ADJUST_LF2_, _ADJUST_RF2_, _ADJUST_RP2_,
    _ADJUST_LP3_, _ADJUST_LF3_, _ADJUST_RF3_, _ADJUST_RP3_,
    _ADJUST_LP4_, _ADJUST_LTH_, _ADJUST_RTH_, _ADJUST_RP4_
),

};

#ifdef AUDIO_ENABLE
  float plover_song[][2]     = SONG(PLOVER_SOUND);
  float plover_gb_song[][2]  = SONG(PLOVER_GOODBYE_SOUND);
#endif

uint32_t layer_state_set_user(uint32_t state) {
  return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case QWERTY:
      if (record->event.pressed) {
        print("mode just switched to qwerty and this is a huge string\n");
        set_single_persistent_default_layer(_QWERTY);
      }
      return false;
      break;
    case BACKLIT:
      if (record->event.pressed) {
        register_code(KC_RSFT);
        #ifdef BACKLIGHT_ENABLE
          backlight_step();
        #endif
        #ifdef KEYBOARD_planck_rev5
          PORTE &= ~(1<<6);
        #endif
      } else {
        unregister_code(KC_RSFT);
        #ifdef KEYBOARD_planck_rev5
          PORTE |= (1<<6);
        #endif
      }
      return false;
      break;
    case PLOVER:
      if (record->event.pressed) {
        #ifdef AUDIO_ENABLE
          stop_all_notes();
          PLAY_SONG(plover_song);
        #endif
        layer_off(_RAISE);
        layer_off(_LOWER);
        layer_off(_ADJUST);
        layer_on(_PLOVER);
        if (!eeconfig_is_enabled()) {
            eeconfig_init();
        }
        keymap_config.raw = eeconfig_read_keymap();
        keymap_config.nkro = 1;
        eeconfig_update_keymap(keymap_config.raw);
      }
      return false;
      break;
    case EXT_PLV:
      if (record->event.pressed) {
        #ifdef AUDIO_ENABLE
          PLAY_SONG(plover_gb_song);
        #endif
        layer_off(_PLOVER);
      }
      return false;
      break;
  }
  return true;
}

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
      #ifdef MOUSEKEY_ENABLE
        register_code(KC_MS_WH_DOWN);
        unregister_code(KC_MS_WH_DOWN);
      #else
        register_code(KC_PGDN);
        unregister_code(KC_PGDN);
      #endif
    } else {
      #ifdef MOUSEKEY_ENABLE
        register_code(KC_MS_WH_UP);
        unregister_code(KC_MS_WH_UP);
      #else
        register_code(KC_PGUP);
        unregister_code(KC_PGUP);
      #endif
    }
  }
}

void dip_update(uint8_t index, bool active) {
  switch (index) {
    case 0:
      if (active) {
        #ifdef AUDIO_ENABLE
          PLAY_SONG(plover_song);
        #endif
        layer_on(_ADJUST);
      } else {
        #ifdef AUDIO_ENABLE
          PLAY_SONG(plover_gb_song);
        #endif
        layer_off(_ADJUST);
      }
      break;
    case 1:
      if (active) {
        muse_mode = true;
      } else {
        muse_mode = false;
        #ifdef AUDIO_ENABLE
          stop_all_notes();
        #endif
      }
   }
}

void matrix_scan_user(void) {
  #ifdef AUDIO_ENABLE
    if (muse_mode) {
      if (muse_counter == 0) {
        uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
        if (muse_note != last_muse_note) {
          stop_note(compute_freq_for_midi_note(last_muse_note));
          play_note(compute_freq_for_midi_note(muse_note), 0xF);
          last_muse_note = muse_note;
        }
      }
      muse_counter = (muse_counter + 1) % muse_tempo;
    }
  #endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}
