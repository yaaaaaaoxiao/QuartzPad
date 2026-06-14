// QuartzPad -- Tab5 6-Minute Countdown Timer
//
// A tilt-driven countdown. The user arms the timer ("Press when Ready"),
// then the device's orientation drives a small state machine:
//
//   IDLE ("Not Active")  boot / after reset; tilt is ignored
//   READY ("Ready")      armed, waiting for the first tilt to start
//   RUNNING ("Walking")  device tilted up, countdown ticking
//   RESTING ("Resting")  device laid flat; countdown keeps ticking but the
//                        rest interval's duration is logged
//   FINISHED ("Finished") countdown hit 0:00; the finish screen takes over
//
// Tilt detection: |accel.z| > threshold => flat. Threshold is runtime-tunable
// (Calibrate screen slider), persisted in NVS, and debounced (TILT_DEBOUNCE_MS).
//
// UI (LVGL): main screen shows a phase-colored state pill, a left rest-count
// column, a center timer whose rounded-rect frame doubles as a perimeter
// progress ring (custom DRAW_POST handler), and Calibrate / History buttons.
// All colors are centralized in the `theme` table, indexed by phase. Finished
// sessions can be saved to NVS and reviewed on the History screen (max 15,
// oldest auto-evicts; permanent session numbering).
//
// All dev-only paths (debug overlay, SD logging) are tagged "DEV ONLY" so they
// can be removed cleanly later.
//
// Requirements:
//   - Board manager  >= 3.2.2 (board: M5Tab5)
//   - M5Unified      >= 0.2.13
//   - M5GFX          >= 0.2.19
//   - LVGL           9.5.0 or newer 9.x

#include <lvgl.h>
#include <M5Unified.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <esp_timer.h>

LV_FONT_DECLARE(inter_medium_digit);
LV_FONT_DECLARE(inter_24);

// ---------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------
static constexpr uint32_t COUNTDOWN_SECONDS         = 6 * 60;
static constexpr float    DEFAULT_THRESHOLD         = 0.85f;
static constexpr float    THRESHOLD_MIN             = 0.50f;
static constexpr float    THRESHOLD_MAX             = 0.99f;
static constexpr uint8_t  DEFAULT_VOLUME_PCT        = 70;     // chime volume 0-100%
// Pre-start ("Get Ready") lead-in: 0 = off, else 3-10 seconds.
static constexpr uint8_t  DEFAULT_PRESTART_SEC      = 3;
static constexpr uint8_t  PRESTART_MIN              = 3;
static constexpr uint8_t  PRESTART_MAX              = 10;
// Auto-calibration (hold-steady capture)
static constexpr float    CAL_STEADY_TOL            = 0.02f;  // |az| jitter tolerated as "steady"
static constexpr uint32_t CAL_HOLD_MS               = 1500;   // steady duration to lock a capture
// Min |az| gap required between the upright and tilted captures. On a measuring
// wheel the two positions can be close (a shallow push), so keep this small --
// but not below the EMA capture noise (~0.01), or the two can't be told apart.
static constexpr float    CAL_MIN_SEP               = 0.03f;
// Calibration UI works in degrees (tilt-limit angle); threshold = cos(angle).
static constexpr int      ANGLE_MIN_DEG             = 8;      // ~ acos(THRESHOLD_MAX 0.99)
static constexpr int      ANGLE_MAX_DEG             = 60;     // ~ acos(THRESHOLD_MIN 0.50)
static constexpr int      DEFAULT_ANGLE_DEG         = 32;     // ~ acos(DEFAULT_THRESHOLD 0.85)
static constexpr uint32_t TILT_DEBOUNCE_MS          = 200;
// IMU is polled at ~50 Hz (tilt is debounced over 200 ms, so faster just
// burns I2C/CPU). The loop sleeps longer when idle to save power.
static constexpr uint32_t IMU_POLL_INTERVAL_MS      = 20;     // ~50 Hz
static constexpr uint32_t LOOP_DELAY_ACTIVE_MS      = 5;      // session/capture running
static constexpr uint32_t LOOP_DELAY_IDLE_MS        = 30;     // waiting for a tap
// Auto power-off after this long idle on the Not Active screen, no touch.
static constexpr uint32_t AUTOSLEEP_MS              = 5 * 60 * 1000;  // 5 min
// Finish screen: auto-save + return to Not Active after this long.
static constexpr uint32_t FINISH_AUTOEXIT_MS        = 15 * 60 * 1000; // 15 min
// Charging mode: exit if the battery drops this many % below its peak (the
// charge-detect flag can get stuck "true" after unplug, so we don't trust it
// alone -- a charging battery never drops).
static constexpr int      CHARGE_DROP_EXIT_PCT      = 2;
// Charging mode: keep showing 100% this long before auto power-off.
static constexpr uint32_t CHARGE_FULL_OFF_MS        = 10 * 60 * 1000; // 10 min
// READY armed but never tilted for this long -> quietly disarm to Not Active
// (otherwise an armed-and-forgotten device never auto-sleeps).
static constexpr uint32_t READY_TIMEOUT_MS          = 10 * 60 * 1000; // 10 min
// Battery % at/below which arming a session is refused (unless charging).
static constexpr int      MIN_START_BATT_PCT        = 5;
// Show the "Sleeping soon" warning this long before the idle power-off.
static constexpr uint32_t SLEEP_WARN_MS             = 20 * 1000;
// Backlight while in charging mode (0-255); restored on exit.
static constexpr uint8_t  CHARGING_BRIGHTNESS       = 90;
static constexpr uint32_t TICK_INTERVAL_MS          = 1000;
static constexpr uint32_t SD_FLUSH_INTERVAL_MS      = 1000;     // DEV ONLY
static constexpr uint32_t OVERLAY_LABEL_INTERVAL_MS = 33;       // DEV ONLY (~30 Hz)
static constexpr uint32_t SETTINGS_READOUT_INTERVAL_MS = 50;    // ~20 Hz
static constexpr int      CHART_POINT_COUNT         = 100;      // DEV ONLY
static constexpr size_t   REST_LOG_CAPACITY         = 64;
static constexpr float    GRAVITY_LPF_ALPHA         = 0.04f;    // DEV ONLY
static constexpr uint8_t  DISPLAY_ROTATION          = 3;
// LVGL draw-buffer height in rows; 0 => full screen height. Two buffers of
// this size are allocated in PSRAM (see lvgl_init_for_tab5). Full-screen +
// double = ~3.7 MB; shrink this (e.g. 180) to trade smoothness for RAM.
static constexpr uint32_t LVGL_BUFFER_ROWS          = 180;
// DMA flush: when true AND two buffers were allocated, the pixel transfer
// runs in the background (DMA engine) while LVGL renders the next region
// into the other buffer -- the CPU no longer stalls during the push.
// Falls back to a blocking CPU copy automatically if only one buffer exists.
static constexpr bool     USE_DMA_FLUSH             = true;
static constexpr size_t   MAX_STORED_SESSIONS       = 15;

// Tab5 SD card pins (per official docs)  -- DEV ONLY
#define SD_SPI_CS_PIN    42
#define SD_SPI_SCK_PIN   43
#define SD_SPI_MOSI_PIN  44
#define SD_SPI_MISO_PIN  39

// NVS namespace + key names (Preferences keys must be <= 15 chars)
static constexpr const char *NVS_NS              = "timer";
static constexpr const char *NVS_K_THRESHOLD     = "threshold";
static constexpr const char *NVS_K_NEXT_SEQ      = "next_seq";
static constexpr const char *NVS_K_SESS_LIST     = "sess_list";
static constexpr const char *NVS_K_VOLUME        = "volume";
static constexpr const char *NVS_K_PRESTART      = "prestart";

// ---------------------------------------------------------------
// Types
// ---------------------------------------------------------------
//
// Phase order matters: index into theme.* arrays is (int)TimerPhase.
//   IDLE     = "Not Active" (boot / after reset; tilt does nothing here)
//   READY    = armed, waiting for first tilt to start the countdown
//   RUNNING  = "Walking"  (device tilted, countdown ticking)
//   RESTING  = device flat, countdown still ticks but rest is logged
//   FINISHED = countdown hit 0; finish screen takes over
//   PRESTART = "Get Ready" lead-in between READY and RUNNING; counts down
//              prestart_sec, then RUNNING begins at a clean 6:00. (Appended
//              at the end so the existing phase indices don't shift.)
enum class TimerPhase { IDLE, READY, RUNNING, RESTING, FINISHED, PRESTART };
static constexpr int PHASE_COUNT = 6;

enum class Screen {
  MAIN, FINISH,
  SETTINGS,            // settings menu (3 rows)
  SET_CALIBRATION,     // settings -> Calibration sub-screen
  SET_VOLUME,          // settings -> Volume sub-screen
  SET_PRESTART,        // settings -> Pre-start countdown sub-screen
  HISTORY_LIST, HISTORY_DETAIL,
  CHARGING             // minimal charge screen (auto-entered on plug-in)
};

struct RestEvent {
  uint32_t duration_sec = 0;
};

// On-disk record for a single saved session. Fixed size so NVS read/write is
// trivial. ~140 bytes per session; 15 sessions => ~2.1 KB in NVS.
struct StoredSession {
  uint32_t seq;
  uint32_t rest_count;
  uint32_t total_rest_sec;
  uint16_t rest_durations[REST_LOG_CAPACITY];   // first rest_count are valid
};

// In-RAM cached metadata for the history list. We never read 15 full session
// blobs just to draw the list rows; we read the full blob only when the user
// drills into one.
struct SessionMeta {
  uint32_t seq;
  uint32_t rest_count;
  uint32_t total_rest_sec;
};

struct AppState {
  // Countdown / tilt
  TimerPhase  phase                   = TimerPhase::IDLE;
  uint32_t    remaining_sec           = COUNTDOWN_SECONDS;
  uint32_t    rest_count              = 0;
  uint32_t    total_rest_sec          = 0;
  uint32_t    last_tick_ms            = 0;
  uint32_t    rest_started_ms         = 0;
  RestEvent   rest_log[REST_LOG_CAPACITY] = {};
  size_t      rest_log_count          = 0;
  bool        is_horizontal           = true;
  uint32_t    tilt_changed_ms         = 0;
  bool        ui_dirty                = true;

  // Tunables (loaded from NVS at boot)
  float       threshold               = DEFAULT_THRESHOLD;
  uint8_t     volume_pct              = DEFAULT_VOLUME_PCT;   // chime volume 0-100%
  uint8_t     prestart_sec            = DEFAULT_PRESTART_SEC; // 0 = off, else 3-10

  // Pre-start ("Get Ready") lead-in runtime state
  uint32_t    prestart_remaining_sec  = 0;
  uint32_t    prestart_tick_ms        = 0;

  // Active screen
  Screen      screen                  = Screen::MAIN;

  // IMU
  m5::imu_data_t last_imu             = {};
  bool        imu_fresh               = false;
  uint32_t    last_imu_poll_ms        = 0;
  bool        gravity_ready           = false;
  float       gravity_x               = 0.0f;
  float       gravity_y               = 0.0f;
  float       gravity_z               = 0.0f;
  float       accel_mag_g             = 0.0f;
  float       motion_g                = 0.0f;

  // Battery
  bool        battery_present         = false;
  bool        is_charging             = false;
  int         battery_level           = -1;
  uint32_t    last_batt_poll_ms       = 0;

  // Persistence
  uint32_t    next_seq                = 1;
  uint32_t    stored_seqs[MAX_STORED_SESSIONS]    = {};
  size_t      stored_count            = 0;
  SessionMeta meta_cache[MAX_STORED_SESSIONS]     = {};
  bool        finish_decided          = false;
  uint32_t    finish_entered_ms       = 0;   // for the 15-min finish auto-exit
  uint32_t    ready_since_ms          = 0;   // for the READY armed-timeout

  // Settings readout throttle
  uint32_t    last_settings_readout_ms = 0;

  // History drill-down target
  uint32_t    detail_seq              = 0;

  // DEV ONLY: debug overlay + SD logging
  uint64_t    log_start_us            = 0;
  bool        overlay_visible         = false;
  bool        sd_ok                   = false;
  bool        logging                 = false;
  File        log_file;
  uint32_t    last_flush_ms           = 0;
  uint32_t    log_lines               = 0;
  uint32_t    last_overlay_label_ms   = 0;
} state;

static Preferences prefs;

// ---------------------------------------------------------------
// Theme
//
// All colors live here so you can tune the look in one place. Arrays are
// indexed by TimerPhase (IDLE, READY, RUNNING, RESTING, FINISHED).
// ---------------------------------------------------------------
struct Theme {
  // Screen background per phase
  lv_color_t bg[PHASE_COUNT];
  // State pill (border + text); transparent fill
  lv_color_t pill[PHASE_COUNT];
  // Timer digits
  lv_color_t timer[PHASE_COUNT];
  // Progress ring drawn on the timer frame perimeter
  lv_color_t progress[PHASE_COUNT];
  lv_color_t progress_track[PHASE_COUNT];   // the empty/track part
  // Big rest-count digit (the left column)
  lv_color_t rest_count[PHASE_COUNT];
  // Captions ("Rested"/"Resting", "Times", "Min:Sec")
  lv_color_t caption[PHASE_COUNT];

  // Singletons
  lv_color_t side_btn_bg;
  lv_color_t side_btn_text;
  lv_color_t side_btn_border;
  lv_color_t ready_btn_bg;
  lv_color_t ready_btn_text;
  lv_color_t ready_btn_border;
  lv_color_t reset_btn_bg;
  lv_color_t reset_btn_text;
  lv_color_t reset_btn_border;
  lv_color_t battery_normal;
  lv_color_t battery_low;
  lv_color_t battery_charging;
  lv_color_t battery_usb_only;
};

// ---------------------------------------------------------------
// Style tunables -- sizes/widths the per-phase `theme` colors don't cover.
// (Phase colors live in `theme`, below.)
// ---------------------------------------------------------------
static constexpr int BTN_BORDER_PX      = 4;    // main-screen button border width
static constexpr int DIGIT_LETTER_SPACE = -10;  // tracking for the big digit font (neg = tighter)
static constexpr int FRAME_STROKE_PX    = 6;    // timer ring: stroke thickness (even everywhere)

static Theme theme = {
  // Per-phase arrays. Column order = (int)TimerPhase:
  //   IDLE, READY, RUNNING(Walking), RESTING, FINISHED, PRESTART(Get Ready)
  /* bg       */ { lv_color_hex(0x000000), lv_color_hex(0x60BE54),
                   lv_color_hex(0x000000), lv_color_hex(0xF09343),
                   lv_color_hex(0x3A0000), lv_color_hex(0x0E2436) },
  /* pill     */ { lv_color_hex(0xFF3B30), lv_color_hex(0xFFFFFF),
                   lv_color_hex(0x4CAF50), lv_color_hex(0xFFFFFF),
                   lv_color_hex(0xFF3B30), lv_color_hex(0x40C4FF) },
  /* timer    */ { lv_color_hex(0x3A3A3A), lv_color_hex(0xFFFFFF),
                   lv_color_hex(0xFFFFFF), lv_color_hex(0xFFFFFF),
                   lv_color_hex(0xFF3B30), lv_color_hex(0xFFFFFF) },
  /* progress */ { lv_color_hex(0x3A3A3A), lv_color_hex(0xFFFFFF),
                   lv_color_hex(0xFFA500), lv_color_hex(0xFFFFFF),
                   lv_color_hex(0xFF3B30), lv_color_hex(0x40C4FF) },
  /* prog_trk */ { lv_color_hex(0x1A1A1A), lv_color_hex(0x4E9447),
                   lv_color_hex(0x1A1A1A), lv_color_hex(0xCC6F1F),
                   lv_color_hex(0x551111), lv_color_hex(0x16384F) },
  /* rest_cnt */ { lv_color_hex(0x8B4513), lv_color_hex(0xAFD9A4),
                   lv_color_hex(0xB36422), lv_color_hex(0xFFFFFF),
                   lv_color_hex(0xB36422), lv_color_hex(0x40C4FF) },
  /* caption  */ { lv_color_hex(0x808080), lv_color_hex(0x4A8540),
                   lv_color_hex(0x808080), lv_color_hex(0xCC6F1F),
                   lv_color_hex(0x808080), lv_color_hex(0x7FB4D8) },

  /* side_btn_bg       */ lv_color_hex(0x2C2C2E),
  /* side_btn_text     */ lv_color_hex(0xFFFFFF),
  /* side_btn_border   */ lv_color_hex(0x4A4A4C),
  /* ready_btn_bg      */ lv_color_hex(0x60BE54),
  /* ready_btn_text    */ lv_color_hex(0xFFFFFF),
  /* ready_btn_border  */ lv_color_hex(0x3E7A33),
  /* reset_btn_bg      */ lv_color_hex(0xEB4C46),
  /* reset_btn_text    */ lv_color_hex(0xFFFFFF),
  /* reset_btn_border  */ lv_color_hex(0x98302E),
  /* battery_normal    */ lv_color_hex(0xFFFFFF),
  /* battery_low     */ lv_color_hex(0xFF3B30),
  /* battery_charging*/ lv_color_hex(0x4CAF50),
  /* battery_usb_only*/ lv_color_hex(0xAAAAAA),
};

// Perimeter progress fraction [0..1] -- written by refresh_ui(), read by the
// LV_EVENT_DRAW_POST handler on timer_frame. Stroke width is FRAME_STROKE_PX
// in the style-tunables block above.
static float g_timer_progress_frac = 1.0f;

// Angle (degrees) the calibration measuring-wheel illustration currently
// shows -- written by the calibration code, read by draw_cal_wheel_event().
// Idle: the set tilt-limit angle. During auto-cal capture: the live device angle.
static float g_cal_view_angle = (float)DEFAULT_ANGLE_DEG;

// Auto-sleep: timestamp of the last "activity" (touch, or an active session).
// While idle (Not Active, untouched) for AUTOSLEEP_MS -> power off.
static uint32_t g_idle_since_ms = 0;

// Charging mode: highest battery % seen since entering. If the level drops
// CHARGE_DROP_EXIT_PCT below this, the charger is really gone (regardless of
// the unreliable charge-detect flag) -> leave charging mode.
static int g_charge_peak = 0;
// Charging mode: when the battery first read 100% (0 = not full yet). Global
// (not function-static) so entering charging mode can reset it -- a stale
// value from a previous visit would otherwise power off instantly.
static uint32_t g_charge_full_since = 0;

static const char *phase_name(TimerPhase p) {
  switch (p) {
    case TimerPhase::IDLE:     return "Not Active";
    case TimerPhase::READY:    return "Ready";
    case TimerPhase::RUNNING:  return "Walking";
    case TimerPhase::RESTING:  return "Resting";
    case TimerPhase::FINISHED: return "Finished";
    case TimerPhase::PRESTART: return "Get Ready";
  }
  return "?";
}

// Tilt threshold expressed as the angle (degrees from flat) at which the
// device flips between flat/tilted. |az| = cos(angle), so angle = acos(thr).
// e.g. threshold 0.85 -> ~32 degrees.
static int threshold_to_deg(float thr) {
  if (thr > 1.0f) thr = 1.0f;
  if (thr < 0.0f) thr = 0.0f;
  return (int)lroundf(acosf(thr) * 57.2957795f);   // 180/pi
}

// The Calibration UI works in degrees (the tilt-limit angle from upright);
// threshold (used everywhere else) = cos(angle). ANGLE_*_DEG live in the
// config block up top (needed earlier by g_cal_view_angle).
static float deg_to_threshold(int deg) {
  float t = cosf((float)deg * 0.0174532925f);
  if (t < THRESHOLD_MIN) t = THRESHOLD_MIN;
  if (t > THRESHOLD_MAX) t = THRESHOLD_MAX;
  return t;
}

// ---------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------
static inline void mark_dirty() { state.ui_dirty = true; }
static uint32_t current_total_rest_sec(uint32_t now);
static void play_cancel_chime();
static lv_obj_t *make_settings_title(lv_obj_t *scr, const char *text);
static void enter_main_screen();
static void enter_finish_screen();
static void enter_charging_screen();
static void enter_settings_screen();        // settings menu
static void enter_calibration_screen();
static void enter_volume_screen();
static void enter_prestart_screen();
static void enter_history_list_screen();
static void enter_history_detail_screen(uint32_t seq);
static void refresh_history_list_rows();
static void update_debug_threshold_lines();   // DEV ONLY

// ---------------------------------------------------------------
// Widget handles (main screen)
// ---------------------------------------------------------------
static lv_obj_t *screen_main;
static lv_obj_t *screen_finish;
static lv_obj_t *screen_settings;          // settings menu
static lv_obj_t *screen_set_calibration;
static lv_obj_t *screen_set_volume;
static lv_obj_t *screen_set_prestart;
static lv_obj_t *screen_history_list;
static lv_obj_t *screen_history_detail;
static lv_obj_t *screen_charging;
static lv_obj_t *charge_pct_label;   // big battery % in charging mode
static lv_obj_t *sleep_warn_label;   // top-layer "sleeping soon" notice
static uint8_t   g_normal_brightness = 255;   // captured at boot; restored after charging dim

// Main screen widgets
static lv_obj_t *info_label;        // top-left: tilt threshold (deg) + volume
static lv_obj_t *state_pill;
static lv_obj_t *state_pill_label;
static lv_obj_t *battery_label;
static lv_obj_t *rested_caption_label;
static lv_obj_t *rested_count_label;
static lv_obj_t *rested_times_label;
static lv_obj_t *timer_frame;
static lv_obj_t *timer_label;
static lv_obj_t *timer_caption_label;
static lv_obj_t *calibrate_btn;
static lv_obj_t *calibrate_btn_label;
static lv_obj_t *history_btn;
static lv_obj_t *history_btn_label;
static lv_obj_t *main_button_btn;
static lv_obj_t *main_button_label;

// Finishing screen
static lv_obj_t *finish_title_label;
static lv_obj_t *finish_summary_label;
static lv_obj_t *finish_bar_container;
static lv_obj_t *finish_save_btn;
static lv_obj_t *finish_save_btn_label;
static lv_obj_t *finish_discard_btn;
static lv_obj_t *finish_discard_btn_label;
static lv_obj_t *finish_status_label;
static lv_obj_t *finish_reset_btn;

// Settings sub-screens
static lv_obj_t *settings_slider;        // calibration: tilt-limit angle (vertical)
static lv_obj_t *cal_wheel;              // calibration: rotating measuring-wheel viz
static lv_obj_t *cal_big_label;          // calibration: big degree number
static lv_obj_t *settings_state_label;   // doubles as wizard status/instructions
static lv_obj_t *cal_btn;                // calibration: Auto-Calibrate / Cancel
static lv_obj_t *cal_btn_label;
static lv_obj_t *cal_progress_bar;       // hold-steady progress
static lv_obj_t *volume_slider;          // volume
static lv_obj_t *volume_value_label;
static lv_obj_t *prestart_slider;        // pre-start countdown
static lv_obj_t *prestart_value_label;

// History screens
static lv_obj_t *history_list_container;
static lv_obj_t *history_empty_label;
static lv_obj_t *history_detail_title_label;
static lv_obj_t *history_detail_summary_label;
static lv_obj_t *history_detail_bar_container;

// Debug overlay  -- DEV ONLY
static lv_obj_t        *debug_panel;
static lv_obj_t        *debug_accel_label;
static lv_obj_t        *debug_gyro_label;
static lv_obj_t        *debug_phase_label;
static lv_obj_t        *debug_chart;
static lv_chart_series_t *debug_ax_series;
static lv_chart_series_t *debug_ay_series;
static lv_chart_series_t *debug_az_series;
static lv_chart_series_t *debug_motion_series;
static lv_chart_series_t *debug_thr_pos_series;
static lv_chart_series_t *debug_thr_neg_series;
static int32_t            debug_thr_pos_data[CHART_POINT_COUNT];
static int32_t            debug_thr_neg_data[CHART_POINT_COUNT];
static lv_obj_t        *record_btn;
static lv_obj_t        *record_btn_label;
static lv_obj_t        *record_status_label;

// ---------------------------------------------------------------
// Color helpers
// ---------------------------------------------------------------
static lv_color_t phase_bg_color(TimerPhase phase) {
  return theme.bg[(int)phase];
}

// ---------------------------------------------------------------
// Battery / charging
// ---------------------------------------------------------------
//
// See previous notes for Tab5 PMIC details. We just rely on M5Unified to
// report level + charging state, and poll once per second.

static void enable_battery_charging() {
  M5.Power.setBatteryCharge(true);
}

static void poll_battery(uint32_t now) {
  if (now - state.last_batt_poll_ms < 1000) return;
  state.last_batt_poll_ms = now;

  int  level    = M5.Power.getBatteryLevel();
  bool charging = M5.Power.isCharging();

  if (level != state.battery_level || charging != state.is_charging) {
    bool charging_rising = charging && !state.is_charging;
    state.battery_level   = level;
    state.is_charging     = charging;
    state.battery_present = (level >= 0);
    mark_dirty();

    // Auto-enter charging mode when a charger is connected while sitting idle
    // on the main screen. (Deliberately NOT from the Finish screen, so an
    // undecided/unsaved session isn't discarded by jumping away.)
    if (charging_rising &&
        state.screen == Screen::MAIN && state.phase == TimerPhase::IDLE) {
      enter_charging_screen();
    }
  }
}

// ---------------------------------------------------------------
// SD card / logging  -- DEV ONLY (entire section)
// ---------------------------------------------------------------
static bool sd_init() {
  SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);
  return SD.begin(SD_SPI_CS_PIN, SPI, 25000000);
}

static bool start_logging() {
  if (!state.sd_ok) return false;
  if (state.logging) return true;

  char path[48];
  snprintf(path, sizeof(path), "/imu_%lu.csv", millis());

  state.log_file = SD.open(path, FILE_WRITE, true);
  if (!state.log_file) return false;

  state.log_file.println(
      "ms,phase,ax,ay,az,gx,gy,gz,accel_mag_g,motion_g,is_horizontal");
  state.logging      = true;
  state.log_lines    = 0;
  state.last_flush_ms = millis();
  state.log_start_us  = esp_timer_get_time();
  return true;
}

static void stop_logging() {
  if (!state.logging) return;
  if (state.log_file) {
    state.log_file.flush();
    state.log_file.close();
  }
  state.logging = false;
}

static void log_imu_sample(uint32_t now, uint64_t now_us) {
  if (!state.logging || !state.log_file) return;
  uint32_t t_ms = (uint32_t)((now_us - state.log_start_us) / 1000ULL);

  // %.4g is plenty for a 16-bit accelerometer; halves file size and
  // roughly halves formatting cost vs %.9g.
  state.log_file.printf(
      "%lu,%d,%.4g,%.4g,%.4g,%.4g,%.4g,%.4g,%.4g,%.4g,%d\n",
      (unsigned long)t_ms,
      (int)state.phase,
      state.last_imu.accel.x,
      state.last_imu.accel.y,
      state.last_imu.accel.z,
      state.last_imu.gyro.x,
      state.last_imu.gyro.y,
      state.last_imu.gyro.z,
      state.accel_mag_g,
      state.motion_g,
      state.is_horizontal ? 1 : 0);
  state.log_lines++;

  if (now - state.last_flush_ms >= SD_FLUSH_INTERVAL_MS) {
    state.log_file.flush();
    state.last_flush_ms = now;
  }
}

// ---------------------------------------------------------------
// NVS / persistence layer
// ---------------------------------------------------------------
static void session_key(uint32_t seq, char *buf, size_t buf_len) {
  // "s_4294967295" is 12 chars + NUL = fits in 15 (NVS key limit).
  snprintf(buf, buf_len, "s_%lu", (unsigned long)seq);
}

// Read one session blob from NVS into *out.
static bool nvs_load_session(uint32_t seq, StoredSession *out) {
  char key[16];
  session_key(seq, key, sizeof(key));
  size_t len = prefs.getBytesLength(key);
  if (len != sizeof(StoredSession)) return false;
  return prefs.getBytes(key, out, sizeof(StoredSession)) == sizeof(StoredSession);
}

// Persist the current stored_seqs[] list to NVS.
static void nvs_save_stored_list() {
  if (state.stored_count > 0) {
    prefs.putBytes(NVS_K_SESS_LIST, state.stored_seqs,
                   state.stored_count * sizeof(uint32_t));
  } else {
    prefs.remove(NVS_K_SESS_LIST);
  }
}

static void nvs_save_threshold() {
  prefs.putFloat(NVS_K_THRESHOLD, state.threshold);
}

static void nvs_save_volume() {
  prefs.putUChar(NVS_K_VOLUME, state.volume_pct);
}

static void nvs_save_prestart() {
  prefs.putUChar(NVS_K_PRESTART, state.prestart_sec);
}

// Load everything at boot. Idempotent: safe to call once.
static void prefs_init() {
  prefs.begin(NVS_NS, false);

  state.threshold = prefs.getFloat(NVS_K_THRESHOLD, DEFAULT_THRESHOLD);
  if (state.threshold < THRESHOLD_MIN) state.threshold = THRESHOLD_MIN;
  if (state.threshold > THRESHOLD_MAX) state.threshold = THRESHOLD_MAX;

  state.volume_pct = prefs.getUChar(NVS_K_VOLUME, DEFAULT_VOLUME_PCT);
  if (state.volume_pct > 100) state.volume_pct = 100;

  // Pre-start: 0 = off; otherwise clamp into [PRESTART_MIN, PRESTART_MAX].
  state.prestart_sec = prefs.getUChar(NVS_K_PRESTART, DEFAULT_PRESTART_SEC);
  if (state.prestart_sec != 0) {
    if (state.prestart_sec < PRESTART_MIN) state.prestart_sec = PRESTART_MIN;
    if (state.prestart_sec > PRESTART_MAX) state.prestart_sec = PRESTART_MAX;
  }

  state.next_seq = prefs.getUInt(NVS_K_NEXT_SEQ, 1);

  size_t list_len = prefs.getBytesLength(NVS_K_SESS_LIST);
  state.stored_count = list_len / sizeof(uint32_t);
  if (state.stored_count > MAX_STORED_SESSIONS) {
    state.stored_count = MAX_STORED_SESSIONS;
  }
  if (state.stored_count > 0) {
    prefs.getBytes(NVS_K_SESS_LIST, state.stored_seqs,
                   state.stored_count * sizeof(uint32_t));
  }

  // Build the in-RAM metadata cache so the history list renders without
  // touching NVS each time.
  for (size_t i = 0; i < state.stored_count; i++) {
    StoredSession s;
    if (nvs_load_session(state.stored_seqs[i], &s)) {
      state.meta_cache[i].seq            = s.seq;
      state.meta_cache[i].rest_count     = s.rest_count;
      state.meta_cache[i].total_rest_sec = s.total_rest_sec;
    } else {
      // Corrupt or missing blob: skip but keep slot to avoid renumbering.
      state.meta_cache[i].seq            = state.stored_seqs[i];
      state.meta_cache[i].rest_count     = 0;
      state.meta_cache[i].total_rest_sec = 0;
    }
  }
}

// Take the current finished session and write it to NVS. Evicts the oldest
// session if we're already at MAX_STORED_SESSIONS. Returns the assigned seq#.
static uint32_t save_current_session() {
  uint32_t seq = state.next_seq;

  StoredSession rec = {};
  rec.seq            = seq;
  rec.rest_count     = (uint32_t)state.rest_log_count;
  rec.total_rest_sec = state.total_rest_sec;
  for (size_t i = 0; i < state.rest_log_count && i < REST_LOG_CAPACITY; i++) {
    uint32_t d = state.rest_log[i].duration_sec;
    rec.rest_durations[i] = (d > 65535) ? 65535 : (uint16_t)d;
  }

  // Evict oldest. stored_seqs[] is maintained sorted ascending (oldest first).
  if (state.stored_count >= MAX_STORED_SESSIONS) {
    uint32_t oldest = state.stored_seqs[0];
    char key[16];
    session_key(oldest, key, sizeof(key));
    prefs.remove(key);
    for (size_t i = 0; i < state.stored_count - 1; i++) {
      state.stored_seqs[i] = state.stored_seqs[i + 1];
      state.meta_cache[i]  = state.meta_cache[i + 1];
    }
    state.stored_count--;
  }

  // Append the new session
  state.stored_seqs[state.stored_count] = seq;
  state.meta_cache[state.stored_count].seq            = seq;
  state.meta_cache[state.stored_count].rest_count     = rec.rest_count;
  state.meta_cache[state.stored_count].total_rest_sec = rec.total_rest_sec;
  state.stored_count++;

  char key[16];
  session_key(seq, key, sizeof(key));
  prefs.putBytes(key, &rec, sizeof(rec));
  nvs_save_stored_list();

  state.next_seq++;
  prefs.putUInt(NVS_K_NEXT_SEQ, state.next_seq);

  return seq;
}

// ---------------------------------------------------------------
// IMU polling
// ---------------------------------------------------------------
static void poll_imu() {
  state.imu_fresh = M5.Imu.update();
  if (!state.imu_fresh) return;

  state.last_imu = M5.Imu.getImuData();

  const float ax = state.last_imu.accel.x;
  const float ay = state.last_imu.accel.y;
  const float az = state.last_imu.accel.z;

  state.accel_mag_g = sqrtf(ax * ax + ay * ay + az * az);

  // Gravity LPF + motion magnitude are only needed when something is
  // consuming them: the dev overlay chart or the CSV log.
  const bool need_motion = state.overlay_visible || state.logging;
  if (need_motion) {
    if (!state.gravity_ready) {
      state.gravity_x = ax;
      state.gravity_y = ay;
      state.gravity_z = az;
      state.gravity_ready = true;
    } else {
      state.gravity_x += GRAVITY_LPF_ALPHA * (ax - state.gravity_x);
      state.gravity_y += GRAVITY_LPF_ALPHA * (ay - state.gravity_y);
      state.gravity_z += GRAVITY_LPF_ALPHA * (az - state.gravity_z);
    }
    const float lx = ax - state.gravity_x;
    const float ly = ay - state.gravity_y;
    const float lz = az - state.gravity_z;
    state.motion_g = sqrtf(lx * lx + ly * ly + lz * lz);
  } else {
    state.motion_g = 0.0f;
    // Don't reset gravity_ready; the LPF can resume cleanly when re-enabled.
  }
}

// ---------------------------------------------------------------
// Tilt + countdown
// ---------------------------------------------------------------
static void begin_rest_segment(uint32_t now) {
  state.rest_count++;
  state.rest_started_ms = now;
}

static void finish_rest_segment(uint32_t now) {
  if (state.rest_started_ms == 0) return;

  uint32_t duration_sec = (now - state.rest_started_ms) / 1000;
  state.total_rest_sec += duration_sec;

  if (state.rest_log_count < REST_LOG_CAPACITY) {
    state.rest_log[state.rest_log_count++].duration_sec = duration_sec;
  }

  state.rest_started_ms = 0;
}

static uint32_t current_total_rest_sec(uint32_t now) {
  if (state.phase == TimerPhase::RESTING && state.rest_started_ms != 0) {
    return state.total_rest_sec + ((now - state.rest_started_ms) / 1000);
  }
  return state.total_rest_sec;
}

// Begin a session from the armed (READY) state. With a pre-start lead-in
// configured we enter PRESTART (a "Get Ready" countdown); otherwise we go
// straight to RUNNING at 6:00. The main countdown is untouched until RUNNING.
static void start_session_from_ready(uint32_t now) {
  if (state.prestart_sec > 0) {
    state.phase                  = TimerPhase::PRESTART;
    state.prestart_remaining_sec = state.prestart_sec;
    state.prestart_tick_ms       = now;
  } else {
    state.phase        = TimerPhase::RUNNING;
    state.last_tick_ms = now;
  }
  mark_dirty();
}

static void update_tilt_state(uint32_t now) {
  if (!state.imu_fresh) return;

  bool now_horizontal = fabsf(state.last_imu.accel.z) > state.threshold;

  if (now_horizontal != state.is_horizontal) {
    if (state.tilt_changed_ms == 0) {
      state.tilt_changed_ms = now;
      return;
    }
    if (now - state.tilt_changed_ms < TILT_DEBOUNCE_MS) return;

    state.is_horizontal   = now_horizontal;
    state.tilt_changed_ms = 0;

    // IDLE: tilt does nothing -- user must press "Press when Ready" first.
    // FINISHED: tilt does nothing -- session is over.
    // (PRESTART IS handled below: standing the wheel upright aborts the
    //  get-ready lead-in back to READY.)
    if (state.phase == TimerPhase::IDLE ||
        state.phase == TimerPhase::FINISHED) return;

    if (!state.is_horizontal) {
      // Picked up
      if (state.phase == TimerPhase::READY) {
        // First tilt after arming -> lead-in (PRESTART) or straight to RUNNING.
        start_session_from_ready(now);
      } else if (state.phase == TimerPhase::RESTING) {
        finish_rest_segment(now);
        state.phase = TimerPhase::RUNNING;
        mark_dirty();
      }
    } else {
      // Set down (wheel upright)
      if (state.phase == TimerPhase::RUNNING) {
        state.phase = TimerPhase::RESTING;
        begin_rest_segment(now);
        mark_dirty();
      } else if (state.phase == TimerPhase::PRESTART) {
        // Interrupt the lead-in: stand it upright -> back to READY (re-armed
        // at 6:00; the next tilt restarts the lead-in). The main countdown
        // hasn't started yet, so nothing to discard.
        state.phase = TimerPhase::READY;
        state.ready_since_ms = now;          // restart the armed-timeout
        state.prestart_remaining_sec = 0;
        play_cancel_chime();                 // soft "cancelled" blip
        mark_dirty();
      }
      // READY + set down: nothing to do, already waiting.
    }
  } else {
    state.tilt_changed_ms = 0;
  }
}

// Advance the "Get Ready" lead-in. When it reaches zero we hand off to
// RUNNING with a fresh tick base, so the 6:00 begins cleanly at GO.
static void update_prestart(uint32_t now) {
  if (state.phase != TimerPhase::PRESTART) return;

  uint32_t ticks = (now - state.prestart_tick_ms) / TICK_INTERVAL_MS;
  if (ticks == 0) return;
  state.prestart_tick_ms += ticks * TICK_INTERVAL_MS;

  if (ticks >= state.prestart_remaining_sec) {
    state.prestart_remaining_sec = 0;
    state.phase        = TimerPhase::RUNNING;   // GO
    state.last_tick_ms = now;                   // start the 6:00 fresh
    mark_dirty();
  } else {
    state.prestart_remaining_sec -= ticks;
    mark_dirty();
  }
}

static void update_countdown(uint32_t now) {
  if (state.phase != TimerPhase::RUNNING && state.phase != TimerPhase::RESTING) return;

  uint32_t ticks = (now - state.last_tick_ms) / TICK_INTERVAL_MS;
  if (ticks == 0) return;

  state.last_tick_ms += ticks * TICK_INTERVAL_MS;

  if (ticks >= state.remaining_sec) {
    state.remaining_sec = 0;
    if (state.phase == TimerPhase::RESTING) {
      finish_rest_segment(now);
    }
    state.phase = TimerPhase::FINISHED;
    state.finish_decided = false;
    mark_dirty();
    // Jump to finishing screen regardless of where the user currently is.
    enter_finish_screen();
  } else {
    state.remaining_sec -= ticks;
    mark_dirty();
  }
}

// ---------------------------------------------------------------
// Audio chimes
//
// Placeholder tones for now. M5.Speaker output is asynchronous (I2S + a
// background mixer), so these never block the loop, the 1 Hz redraw, or the
// DMA flush. Master volume is set from state.volume_pct (see apply_volume()).
// When the WAV assets are ready, swap the two play_* bodies for
// M5.Speaker.playWav(chime_wav, chime_wav_len, 1, -1, true) etc. -- the
// update_chimes() detector below does not change.
// ---------------------------------------------------------------
static void apply_volume() {
  // Master volume is floored at ~10% so functional feedback (calibration
  // capture beeps) stays audible even at "0%". Setting 0% mutes the regular
  // chimes in software instead (see the volume_pct checks in play_*_chime).
  uint8_t pct = state.volume_pct < 10 ? 10 : state.volume_pct;
  M5.Speaker.setVolume((uint8_t)((uint16_t)pct * 255 / 100));
}

static void play_state_chime()  { if (state.volume_pct) M5.Speaker.tone(1568.0f, 90);  }  // short high blip
static void play_finish_chime() { if (state.volume_pct) M5.Speaker.tone(784.0f, 350); }   // lower, longer
static void play_cancel_chime() { if (state.volume_pct) M5.Speaker.tone(523.0f, 120); }   // soft low blip

// Chime on activity transitions only: entering RUNNING / RESTING / FINISHED.
// Arming (READY) and reset (IDLE) stay silent. Called every loop.
static void update_chimes() {
  static TimerPhase last = TimerPhase::IDLE;
  static bool primed = false;
  if (!primed) { last = state.phase; primed = true; return; }  // no chime on boot
  if (state.phase == last) return;
  TimerPhase np = state.phase;
  last = np;
  switch (np) {
    case TimerPhase::FINISHED: play_finish_chime(); break;
    case TimerPhase::RUNNING:
    case TimerPhase::RESTING:  play_state_chime();  break;
    default: break;   // IDLE (reset), READY (arm): silent
  }
}

// ---------------------------------------------------------------
// UI: main screen refresh (only runs when main screen is active)
// ---------------------------------------------------------------
static void refresh_ui() {
  if (state.screen != Screen::MAIN) return;
  if (!state.ui_dirty) return;
  state.ui_dirty = false;

  const int  p_idx      = (int)state.phase;
  const bool is_idle    = (state.phase == TimerPhase::IDLE);
  const bool is_resting = (state.phase == TimerPhase::RESTING);
  const bool is_prestart = (state.phase == TimerPhase::PRESTART);

  // ----- Timer digits + perimeter progress ---------------------------
  // During the "Get Ready" lead-in the big digits show the lead-in count
  // (3..2..1) and the caption reads "Get Ready"; the 6:00 is still pending.
  if (is_prestart) {
    lv_label_set_text_fmt(timer_label, "%lu",
                          (unsigned long)state.prestart_remaining_sec);
  } else {
    uint32_t m = state.remaining_sec / 60;
    uint32_t s = state.remaining_sec % 60;
    lv_label_set_text_fmt(timer_label, "%lu:%02lu", m, s);
  }
  lv_obj_set_style_text_color(timer_label, theme.timer[p_idx], 0);

  // Drive the custom perimeter draw: update fraction + force a redraw.
  // Colors are read inside timer_frame_draw_event() from `theme` directly.
  // (During PRESTART remaining_sec is still 6:00, so the ring shows full.)
  g_timer_progress_frac =
      (float)state.remaining_sec / (float)COUNTDOWN_SECONDS;
  lv_obj_invalidate(timer_frame);

  // Caption doubles as guidance: tell a first-time user what READY expects.
  if (is_prestart) {
    lv_label_set_text(timer_caption_label, "Get Ready");
  } else if (state.phase == TimerPhase::READY) {
    lv_label_set_text(timer_caption_label, "Tilt the wheel to start");
  } else {
    lv_label_set_text(timer_caption_label, "Min:Sec");
  }
  lv_obj_set_style_text_color(timer_caption_label, theme.caption[p_idx], 0);

  // ----- Rest counter (left column) ----------------------------------
  // Label flips to "Resting" while we're currently in a rest segment.
  lv_label_set_text(rested_caption_label, is_resting ? "Resting" : "Rested");
  lv_obj_set_style_text_color(rested_caption_label, theme.caption[p_idx], 0);

  lv_label_set_text_fmt(rested_count_label, "%lu",
                        (unsigned long)state.rest_count);
  lv_obj_set_style_text_color(rested_count_label, theme.rest_count[p_idx], 0);

  lv_obj_set_style_text_color(rested_times_label, theme.caption[p_idx], 0);

  // ----- State pill (top-center) -------------------------------------
  lv_label_set_text(state_pill_label, phase_name(state.phase));
  lv_obj_set_style_border_color(state_pill, theme.pill[p_idx], 0);
  lv_obj_set_style_text_color(state_pill_label, theme.pill[p_idx], 0);

  // ----- Top-left info readout: tilt threshold (deg) + volume --------
  lv_label_set_text_fmt(info_label, "Tilt %d deg\nVol %d%%",
                        threshold_to_deg(state.threshold),
                        (int)state.volume_pct);
  lv_obj_set_style_text_color(info_label, theme.caption[p_idx], 0);

  // ----- Bottom button: "Press when Ready" (IDLE) vs "RESET" ---------
  if (is_idle) {
    lv_label_set_text(main_button_label, "Press when Ready");
    lv_obj_set_style_bg_color(main_button_btn, theme.ready_btn_bg, 0);
    lv_obj_set_style_border_color(main_button_btn, theme.ready_btn_border, 0);
    lv_obj_set_style_text_color(main_button_label, theme.ready_btn_text, 0);
  } else {
    lv_label_set_text(main_button_label, "RESET");
    lv_obj_set_style_bg_color(main_button_btn, theme.reset_btn_bg, 0);
    lv_obj_set_style_border_color(main_button_btn, theme.reset_btn_border, 0);
    lv_obj_set_style_text_color(main_button_label, theme.reset_btn_text, 0);
  }

  // ----- Background --------------------------------------------------
  // Only restyle on actual phase transitions; a full-screen bg restyle is
  // expensive and would tank the timer redraw if done every tick.
  static TimerPhase last_bg_phase = TimerPhase::IDLE;
  static bool       bg_initialized = false;
  if (!bg_initialized || state.phase != last_bg_phase) {
    lv_obj_set_style_bg_color(screen_main, phase_bg_color(state.phase), 0);
    last_bg_phase  = state.phase;
    bg_initialized = true;
  }

  // ----- Battery -----------------------------------------------------
  if (battery_label) {
    if (!state.battery_present) {
      lv_label_set_text(battery_label, LV_SYMBOL_USB);
      lv_obj_set_style_text_color(battery_label, theme.battery_usb_only, 0);
    } else {
      const char *icon;
      if (state.battery_level >= 80)      icon = LV_SYMBOL_BATTERY_FULL;
      else if (state.battery_level >= 60) icon = LV_SYMBOL_BATTERY_3;
      else if (state.battery_level >= 40) icon = LV_SYMBOL_BATTERY_2;
      else if (state.battery_level >= 20) icon = LV_SYMBOL_BATTERY_1;
      else                                icon = LV_SYMBOL_BATTERY_EMPTY;

      if (state.is_charging) {
        lv_label_set_text_fmt(battery_label, "%d%% %s %s",
                              state.battery_level, LV_SYMBOL_CHARGE, icon);
        lv_obj_set_style_text_color(battery_label, theme.battery_charging, 0);
      } else {
        lv_label_set_text_fmt(battery_label, "%d%% %s",
                              state.battery_level, icon);
        lv_color_t c = (state.battery_level <= 15) ? theme.battery_low
                                                   : theme.battery_normal;
        lv_obj_set_style_text_color(battery_label, c, 0);
      }
    }
  }
}

// ---------------------------------------------------------------
// UI: debug overlay refresh  -- DEV ONLY
// Chart values pushed every IMU sample; label formatting + chart redraw are
// throttled to ~30 Hz which is what the eye can track anyway.
// ---------------------------------------------------------------
static void refresh_debug_overlay(uint32_t now) {
  if (!state.overlay_visible) return;

  // Cheap: keep the trace dense.
  lv_chart_set_next_value(debug_chart, debug_ax_series,
                          (int32_t)(state.last_imu.accel.x * 100.0f));
  lv_chart_set_next_value(debug_chart, debug_ay_series,
                          (int32_t)(state.last_imu.accel.y * 100.0f));
  lv_chart_set_next_value(debug_chart, debug_az_series,
                          (int32_t)(state.last_imu.accel.z * 100.0f));
  lv_chart_set_next_value(debug_chart, debug_motion_series,
                          (int32_t)(state.motion_g * 100.0f));

  if (now - state.last_overlay_label_ms < OVERLAY_LABEL_INTERVAL_MS) return;
  state.last_overlay_label_ms = now;

  lv_label_set_text_fmt(debug_accel_label,
      "raw g  ax %+.3f\n       ay %+.3f\n       az %+.3f\n|a| %.3f",
      state.last_imu.accel.x, state.last_imu.accel.y,
      state.last_imu.accel.z, state.accel_mag_g);
  lv_label_set_text_fmt(debug_gyro_label,
      "gyro   gx %+.1f\n       gy %+.1f\n       gz %+.1f\nmotion %.3fg",
      state.last_imu.gyro.x, state.last_imu.gyro.y,
      state.last_imu.gyro.z, state.motion_g);

  static const char *kPhaseNames[] = {"IDLE", "READY", "RUNNING", "RESTING", "FINISHED", "PRESTART"};
  lv_label_set_text_fmt(debug_phase_label,
      "phase: %s\nrest: %lus/%lu seg\nlogged: %lu",
      kPhaseNames[(int)state.phase],
      (unsigned long)current_total_rest_sec(now),
      (unsigned long)state.rest_log_count,
      (unsigned long)state.log_lines);
}

// Refresh the debug chart's threshold lines when the user changes threshold.
static void update_debug_threshold_lines() {
  int32_t v = (int32_t)(state.threshold * 100.0f);
  for (int i = 0; i < CHART_POINT_COUNT; i++) {
    debug_thr_pos_data[i] =  v;
    debug_thr_neg_data[i] = -v;
  }
  if (debug_chart) lv_chart_refresh(debug_chart);
}

// ---------------------------------------------------------------
// Auto-calibration wizard (hold-steady capture)
//
// Two captures -- wheel upright (pad flat -> high |az|, REST) and the user's
// walking push angle (pad tilted -> lower |az|, WALK). No screen touch during
// the holds: each capture locks automatically once |az| has been steady for
// CAL_HOLD_MS. threshold = midpoint of the two captured |az| values. The
// manual slider remains available as a fine-tune / fallback.
// ---------------------------------------------------------------
enum class CalStep : uint8_t { IDLE, CAPTURE_REST, CAPTURE_WALK };

static struct {
  CalStep  step         = CalStep::IDLE;
  bool     inited       = false;   // ema/steady seeded for the current step
  float    ema          = 0.0f;    // smoothed |az|
  uint32_t steady_since = 0;       // ms since |az| last moved beyond tolerance
  float    az_rest      = 0.0f;
  uint32_t msg_until    = 0;       // keep a result/hint message visible until this ms
} cal;

static void cal_beep_ok()     { M5.Speaker.tone(2000.0f, 80);  }
static void cal_beep_reject() { M5.Speaker.tone(300.0f, 250);  }

// Invalidate only the wheel illustration's drawing region instead of the
// full-screen cal_wheel object -- slider drags / live capture redraw ~3x
// fewer pixels. Bounds cover the whole geometry across 0-90 degrees (pivot
// ~(794,540), handle+dashes up-left, arc labels right); padded for safety.
static void cal_wheel_invalidate() {
  if (!cal_wheel) return;
  lv_area_t a;
  a.x1 = 300; a.y1 = 30; a.x2 = 900; a.y2 = 660;
  lv_obj_invalidate_area(cal_wheel, &a);
}

static void cal_reset_wizard() {
  cal.step   = CalStep::IDLE;
  cal.inited = false;
  if (cal_btn_label)    lv_label_set_text(cal_btn_label, "Auto-Calibrate");
  if (cal_progress_bar) lv_obj_add_flag(cal_progress_bar, LV_OBJ_FLAG_HIDDEN);
  // Restore the wheel + big number to the SET tilt-limit angle.
  g_cal_view_angle = (float)threshold_to_deg(state.threshold);
  if (cal_big_label) lv_label_set_text_fmt(cal_big_label, "%d",
                                           (int)lroundf(g_cal_view_angle));
  cal_wheel_invalidate();
}

static void cal_btn_clicked(lv_event_t *) {
  if (cal.step == CalStep::IDLE) {
    cal.step         = CalStep::CAPTURE_REST;
    cal.inited       = false;
    cal.steady_since = millis();
    cal.msg_until    = 0;
    lv_label_set_text(cal_btn_label, "Cancel");
    lv_obj_clear_flag(cal_progress_bar, LV_OBJ_FLAG_HIDDEN);
    lv_bar_set_value(cal_progress_bar, 0, LV_ANIM_OFF);
  } else {
    cal_reset_wizard();
  }
}

// Called every loop while the Calibration sub-screen is active.
static void update_calibration(uint32_t now) {
  if (state.screen != Screen::SET_CALIBRATION) return;
  if (now - state.last_settings_readout_ms < SETTINGS_READOUT_INTERVAL_MS) return;
  state.last_settings_readout_ms = now;

  float az = fabsf(state.last_imu.accel.z);

  // ----- Idle: wheel shows the SET limit angle (set elsewhere); hint text ---
  if (cal.step == CalStep::IDLE) {
    if (now < cal.msg_until) return;
    lv_label_set_text(settings_state_label,
                      "Drag the slider, or tap Auto-Calibrate");
    lv_obj_set_style_text_color(settings_state_label, lv_color_hex(0x888888), 0);
    return;
  }

  // ----- Capturing: the wheel mirrors the LIVE device angle -----
  float live = (az > 1.0f ? 0.0f : acosf(az) * 57.2957795f);  // tilt from flat (deg)
  g_cal_view_angle = live;
  lv_label_set_text_fmt(cal_big_label, "%d", (int)lroundf(live));
  cal_wheel_invalidate();

  // steadiness detection (EMA + motion reset)
  if (!cal.inited) {
    cal.ema = az; cal.steady_since = now; cal.inited = true;
  } else {
    float dev = fabsf(az - cal.ema);
    cal.ema = cal.ema * 0.85f + az * 0.15f;
    if (dev > CAL_STEADY_TOL) cal.steady_since = now;   // motion -> restart hold
  }

  // In the WALK step, don't start the hold until the angle is clearly
  // different from the captured REST angle (avoids capturing the same spot).
  bool waiting_to_diverge =
      (cal.step == CalStep::CAPTURE_WALK &&
       fabsf(cal.ema - cal.az_rest) < CAL_MIN_SEP);
  if (waiting_to_diverge) cal.steady_since = now;

  uint32_t held = now - cal.steady_since;
  int pct = (int)((uint64_t)held * 100 / CAL_HOLD_MS);
  if (pct > 100) pct = 100;
  lv_bar_set_value(cal_progress_bar, pct, LV_ANIM_OFF);

  lv_obj_set_style_text_color(settings_state_label, lv_color_hex(0x40C4FF), 0);
  if (cal.step == CalStep::CAPTURE_REST) {
    lv_label_set_text(settings_state_label,
                      "Step 1/2: stand the wheel UPRIGHT and hold still");
  } else if (waiting_to_diverge) {
    lv_label_set_text(settings_state_label,
                      "Step 2/2: tilt to your walking angle");
  } else {
    lv_label_set_text(settings_state_label,
                      "Step 2/2: hold your walking angle still");
  }

  if (held < CAL_HOLD_MS) return;

  // ----- A capture has locked -----
  float captured = cal.ema;
  cal_beep_ok();

  if (cal.step == CalStep::CAPTURE_REST) {
    cal.az_rest      = captured;
    cal.step         = CalStep::CAPTURE_WALK;
    cal.inited       = false;
    cal.steady_since = now;
    lv_bar_set_value(cal_progress_bar, 0, LV_ANIM_OFF);
    return;
  }

  // WALK captured -> compute the midpoint threshold.
  float az_walk = captured;
  float hi = (cal.az_rest > az_walk) ? cal.az_rest : az_walk;
  float lo = (cal.az_rest > az_walk) ? az_walk : cal.az_rest;
  if (hi - lo < CAL_MIN_SEP) {              // safety net (gate usually prevents this)
    cal_beep_reject();
    lv_label_set_text(settings_state_label, "Positions too similar - try again");
    lv_obj_set_style_text_color(settings_state_label, lv_color_hex(0xFF3B30), 0);
    cal.msg_until = now + 2500;
    cal_reset_wizard();
    return;
  }
  float thr = (hi + lo) * 0.5f;
  if (thr < THRESHOLD_MIN) thr = THRESHOLD_MIN;
  if (thr > THRESHOLD_MAX) thr = THRESHOLD_MAX;
  state.threshold = thr;
  nvs_save_threshold();
  int deg = threshold_to_deg(thr);
  lv_slider_set_value(settings_slider, deg, LV_ANIM_OFF);  // slider is in degrees
  update_debug_threshold_lines();
  lv_label_set_text_fmt(settings_state_label, "Calibrated: %d deg limit", deg);
  lv_obj_set_style_text_color(settings_state_label, lv_color_hex(0x4CAF50), 0);
  cal.msg_until = now + 2500;
  cal_reset_wizard();          // sets wheel + big number to the new set angle
}

// ---------------------------------------------------------------
// Charging mode: keep the % live, auto power-off when full, auto-exit on unplug.
// (Charging is a hardware function via M5.Power.setBatteryCharge(true) at boot;
//  this screen just displays it and manages the full/unplug transitions.)
// ---------------------------------------------------------------
static void update_charging(uint32_t now) {
  if (state.screen != Screen::CHARGING) return;

  int lvl = state.battery_level;

  // Fully charged -> keep showing 100% for CHARGE_FULL_OFF_MS, then shut down.
  if (lvl >= 100) {
    lv_label_set_text(charge_pct_label, "100");
    if (g_charge_full_since == 0) g_charge_full_since = now;
    if (now - g_charge_full_since >= CHARGE_FULL_OFF_MS) {
      M5.Power.powerOff();   // does not return
    }
    return;
  }
  g_charge_full_since = 0;

  // Track the peak level seen since entering charging mode.
  if (lvl > g_charge_peak) g_charge_peak = lvl;

  // Leave charging mode if the charger is really gone. We DON'T trust the
  // charge-detect flag alone (it can stick "true" after unplug), so we also
  // treat an actual battery drop below the peak as "unplugged + draining".
  bool flag_unplugged  = (!state.is_charging && lvl >= 0);
  bool draining        = (lvl >= 0 && lvl <= g_charge_peak - CHARGE_DROP_EXIT_PCT);
  if (flag_unplugged || draining) {
    enter_main_screen();   // normal UI resumes (and its auto-sleep)
    return;
  }

  lv_label_set_text_fmt(charge_pct_label, "%d", lvl < 0 ? 0 : lvl);
}

// ---------------------------------------------------------------
// Rest bar chart helper
//
// Renders one row per rest into `parent`. Bars are scaled so the longest
// rest in this session fills the available width. Container is scrollable
// vertically if rows overflow.
// ---------------------------------------------------------------
static void render_rest_bar_chart(lv_obj_t *parent,
                                  const uint16_t *durations_sec,
                                  size_t count) {
  lv_obj_clean(parent);

  if (count == 0) {
    lv_obj_t *empty = lv_label_create(parent);
    lv_label_set_text(empty, "No rests this session");
    lv_obj_set_style_text_color(empty, lv_color_hex(0xAAAAAA), 0);
    lv_obj_center(empty);
    return;
  }

  // Force layout so we can read the parent's content width.
  lv_obj_update_layout(parent);
  int parent_w = lv_obj_get_content_width(parent);
  if (parent_w <= 0) parent_w = 1000;   // fallback

  // Column widths sized for the 32px inter_24 font: "#64" and "10m 59s"
  // must fit on ONE line -- a too-narrow label wraps, and a wrapped label
  // inside a fixed-height row renders as blank.
  const int idx_w   = 76;
  const int dur_w   = 170;
  const int gap     = 12;
  const int row_h   = 44;   // > 32px font height, with breathing room
  int bar_max_w = parent_w - idx_w - dur_w - gap * 2 - 8;
  if (bar_max_w < 80) bar_max_w = 80;

  uint16_t max_dur = 1;
  for (size_t i = 0; i < count; i++) {
    if (durations_sec[i] > max_dur) max_dur = durations_sec[i];
  }

  lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_START,
                        LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_set_style_pad_row(parent, 8, 0);

  for (size_t i = 0; i < count; i++) {
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, row_h);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row, gap, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *idx_lbl = lv_label_create(row);
    lv_label_set_text_fmt(idx_lbl, "#%u", (unsigned)(i + 1));
    lv_obj_set_width(idx_lbl, idx_w);
    lv_label_set_long_mode(idx_lbl, LV_LABEL_LONG_CLIP);   // never wrap
    lv_obj_set_style_text_color(idx_lbl, lv_color_hex(0xAAAAAA), 0);

    uint16_t dur = durations_sec[i];
    int bar_w = (int)((int64_t)bar_max_w * dur / max_dur);
    if (bar_w < 4) bar_w = 4;

    lv_obj_t *bar = lv_obj_create(row);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, bar_w, 26);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xFFA500), 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(bar, 6, 0);

    lv_obj_t *dur_lbl = lv_label_create(row);
    lv_obj_set_width(dur_lbl, dur_w);
    lv_label_set_long_mode(dur_lbl, LV_LABEL_LONG_CLIP);   // never wrap
    lv_obj_set_style_text_color(dur_lbl, lv_color_white(), 0);
    uint32_t mm = dur / 60;
    uint32_t ss = dur % 60;
    if (mm > 0) {
      lv_label_set_text_fmt(dur_lbl, "%lum %02lus",
                            (unsigned long)mm, (unsigned long)ss);
    } else {
      lv_label_set_text_fmt(dur_lbl, "%lus", (unsigned long)ss);
    }
  }
}

// Convenience: build a uint16_t[] from the current session's rest_log and render.
static void render_current_session_bar_chart(lv_obj_t *parent) {
  uint16_t durations[REST_LOG_CAPACITY];
  size_t n = state.rest_log_count;
  if (n > REST_LOG_CAPACITY) n = REST_LOG_CAPACITY;
  for (size_t i = 0; i < n; i++) {
    uint32_t d = state.rest_log[i].duration_sec;
    durations[i] = (d > 65535) ? 65535 : (uint16_t)d;
  }
  render_rest_bar_chart(parent, durations, n);
}

// ---------------------------------------------------------------
// Event handlers
// ---------------------------------------------------------------
static void reset_session_state() {
  state.phase           = TimerPhase::IDLE;
  state.remaining_sec   = COUNTDOWN_SECONDS;
  state.rest_count      = 0;
  state.total_rest_sec  = 0;
  state.rest_started_ms = 0;
  state.rest_log_count  = 0;
  state.is_horizontal   = true;
  state.gravity_ready   = false;
  state.motion_g        = 0.0f;
  state.accel_mag_g     = 0.0f;
  state.tilt_changed_ms = 0;
  state.finish_decided  = false;
  state.prestart_remaining_sec = 0;
  mark_dirty();
}

static void reset_clicked(lv_event_t *) {
  reset_session_state();
}

// Big bottom button on the main screen. Acts as "Press when Ready" while
// IDLE; "RESET" in any other phase.
static void main_button_clicked(lv_event_t *) {
  if (state.phase == TimerPhase::IDLE) {
    // Low-battery guard: refuse to arm if there isn't enough charge to be
    // confident a 6-minute session survives. (Label restores on next refresh.)
    if (state.battery_present && !state.is_charging &&
        state.battery_level >= 0 && state.battery_level <= MIN_START_BATT_PCT) {
      lv_label_set_text_fmt(main_button_label, "Battery too low (%d%%) - charge first",
                            state.battery_level);
      play_cancel_chime();
      return;
    }
    // Arm the session.
    reset_session_state();
    state.phase = TimerPhase::READY;
    state.ready_since_ms = millis();   // start the armed-timeout clock
    // Edge case: if the device is already being held tilted when the user
    // taps the button, begin immediately (lead-in or RUNNING) instead of
    // sitting in READY until the next horizontal -> tilted transition.
    if (!state.is_horizontal) {
      start_session_from_ready(millis());
    }
    mark_dirty();
  } else {
    reset_session_state();
  }
}

// DEV ONLY: long-press settings gear toggles debug overlay
static void settings_long_pressed(lv_event_t *) {
  state.overlay_visible = !state.overlay_visible;
  if (state.overlay_visible) {
    lv_obj_clear_flag(debug_panel, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(debug_panel, LV_OBJ_FLAG_HIDDEN);
    if (state.logging) stop_logging();
  }
}

// Short tap on settings gear opens the settings screen
static void settings_short_clicked(lv_event_t *) {
  enter_settings_screen();
}

static void history_clicked(lv_event_t *) {
  enter_history_list_screen();
}

static void back_to_main_clicked(lv_event_t *) {
  enter_main_screen();
}

static void back_to_history_list_clicked(lv_event_t *) {
  enter_history_list_screen();
}

// DEV ONLY: record button on debug overlay
static void record_clicked(lv_event_t *) {
  if (!state.sd_ok) {
    lv_label_set_text(record_status_label, "no SD");
    return;
  }
  if (state.logging) {
    stop_logging();
    lv_label_set_text(record_btn_label, "REC");
    lv_obj_set_style_bg_color(record_btn, lv_color_hex(0xCC0000), 0);
    lv_label_set_text(record_status_label, "stopped");
  } else {
    if (start_logging()) {
      lv_label_set_text(record_btn_label, "STOP");
      lv_obj_set_style_bg_color(record_btn, lv_color_hex(0x666666), 0);
      lv_label_set_text(record_status_label, "recording");
    } else {
      lv_label_set_text(record_status_label, "open failed");
    }
  }
}

// Angle slider drag: slider value is the tilt-limit in DEGREES. Update the
// threshold (= cos angle), the big number, and rotate the wheel live.
static void slider_value_changed_cb(lv_event_t *) {
  int deg = (int)lv_slider_get_value(settings_slider);
  state.threshold  = deg_to_threshold(deg);
  g_cal_view_angle = (float)deg;
  if (cal_big_label) lv_label_set_text_fmt(cal_big_label, "%d", deg);
  cal_wheel_invalidate();
  update_debug_threshold_lines();   // no-op if overlay isn't built/visible
}

// Slider released: persist to NVS so we don't write on every micro-movement.
static void slider_released_cb(lv_event_t *) {
  nvs_save_threshold();
}

static void reset_threshold_cb(lv_event_t *) {
  state.threshold  = DEFAULT_THRESHOLD;
  g_cal_view_angle = (float)DEFAULT_ANGLE_DEG;
  lv_slider_set_value(settings_slider, DEFAULT_ANGLE_DEG, LV_ANIM_OFF);
  if (cal_big_label) lv_label_set_text_fmt(cal_big_label, "%d", DEFAULT_ANGLE_DEG);
  cal_wheel_invalidate();
  nvs_save_threshold();
  update_debug_threshold_lines();
}

// Volume slider drag: update master volume live (0-100%).
static void volume_changed_cb(lv_event_t *) {
  int32_t v = lv_slider_get_value(volume_slider);
  if (v < 0) v = 0; else if (v > 100) v = 100;
  state.volume_pct = (uint8_t)v;
  apply_volume();
  lv_label_set_text_fmt(volume_value_label, "Volume: %d%%", (int)state.volume_pct);
}

// Volume slider released: persist + play a test chime at the new level.
static void volume_released_cb(lv_event_t *) {
  nvs_save_volume();
  play_state_chime();
}

// Pre-start slider: leftmost position (PRESTART_MIN-1) means Off; values
// PRESTART_MIN..PRESTART_MAX are the lead-in length in seconds.
static void prestart_changed_cb(lv_event_t *) {
  int32_t v = lv_slider_get_value(prestart_slider);
  state.prestart_sec = (v < (int32_t)PRESTART_MIN) ? 0 : (uint8_t)v;
  if (state.prestart_sec == 0) {
    lv_label_set_text(prestart_value_label, "Pre-start: Off");
  } else {
    lv_label_set_text_fmt(prestart_value_label, "Pre-start: %ds",
                          (int)state.prestart_sec);
  }
}

static void prestart_released_cb(lv_event_t *) {
  nvs_save_prestart();
}

// Settings sub-screen back buttons return to the settings menu.
static void back_to_settings_clicked(lv_event_t *) {
  enter_settings_screen();
}

// Finishing screen: Save
static void save_session_clicked(lv_event_t *) {
  if (state.finish_decided) return;
  uint32_t seq = save_current_session();
  state.finish_decided = true;

  // Disable both decision buttons.
  lv_obj_add_state(finish_save_btn,    LV_STATE_DISABLED);
  lv_obj_add_state(finish_discard_btn, LV_STATE_DISABLED);
  lv_obj_set_style_bg_color(finish_save_btn,        lv_color_hex(0x404040), 0);
  lv_obj_set_style_bg_color(finish_discard_btn,     lv_color_hex(0x404040), 0);
  lv_obj_set_style_border_color(finish_save_btn,    lv_color_hex(0x2A2A2A), 0);
  lv_obj_set_style_border_color(finish_discard_btn, lv_color_hex(0x2A2A2A), 0);

  lv_label_set_text_fmt(finish_status_label, "Saved as Session #%lu",
                        (unsigned long)seq);
  lv_obj_set_style_text_color(finish_status_label, lv_color_hex(0x4CAF50), 0);
}

// Finishing screen: Discard
static void discard_session_clicked(lv_event_t *) {
  if (state.finish_decided) return;
  state.finish_decided = true;

  lv_obj_add_state(finish_save_btn,    LV_STATE_DISABLED);
  lv_obj_add_state(finish_discard_btn, LV_STATE_DISABLED);
  lv_obj_set_style_bg_color(finish_save_btn,        lv_color_hex(0x404040), 0);
  lv_obj_set_style_bg_color(finish_discard_btn,     lv_color_hex(0x404040), 0);
  lv_obj_set_style_border_color(finish_save_btn,    lv_color_hex(0x2A2A2A), 0);
  lv_obj_set_style_border_color(finish_discard_btn, lv_color_hex(0x2A2A2A), 0);

  lv_label_set_text(finish_status_label, "Discarded");
  lv_obj_set_style_text_color(finish_status_label, lv_color_hex(0xAAAAAA), 0);
}

// Finishing screen: Reset returns to main + clears session state.
static void finish_reset_clicked(lv_event_t *e) {
  reset_clicked(e);
  enter_main_screen();
}

// Tapping a row in the history list = drill into detail view.
// The seq# is stashed in the row's user_data (cast to void*).
static void history_row_clicked(lv_event_t *e) {
  uint32_t seq = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
  enter_history_detail_screen(seq);
}

// ---------------------------------------------------------------
// Shared widget builders
// ---------------------------------------------------------------
static lv_obj_t *make_back_button(lv_obj_t *parent, lv_event_cb_t cb) {
  // Same look as the main-screen side buttons: bigger rounded rectangle,
  // bordered, no shadow.
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_size(btn, 110, 72);
  lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 24, 24);
  lv_obj_set_style_bg_color(btn, theme.side_btn_bg, 0);
  lv_obj_set_style_radius(btn, 18, 0);
  lv_obj_set_style_border_width(btn, BTN_BORDER_PX, 0);
  lv_obj_set_style_border_color(btn, theme.side_btn_border, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, LV_SYMBOL_LEFT);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(lbl, theme.side_btn_text, 0);
  lv_obj_center(lbl);
  return btn;
}

// ---------------------------------------------------------------
// Custom perimeter progress drawing for timer_frame
//
// The frame's own border is 0 width; the visible outline is drawn here in a
// LV_EVENT_DRAW_POST handler. Per draw call we paint the full perimeter once
// in the dim track color, then overlay the bright "progress" color clockwise
// from top-center for `g_timer_progress_frac` of the perimeter length.
//
// The whole outline -- straight edges AND corners -- is stroked with a SINGLE
// lv_draw_line primitive (corners sampled into short segments). One primitive
// at one width + round caps gives perfectly even thickness everywhere; the
// old approach mixed lv_draw_line (edges) with lv_draw_arc (corners), whose
// rasterizers don't match, which made the corners look uneven.
// ---------------------------------------------------------------
static void draw_rrect_progress(lv_layer_t *layer,
                                int x, int y, int w, int h, int r,
                                int stroke,
                                lv_color_t color, float frac) {
  if (frac <= 0.0f) return;
  if (frac > 1.0f) frac = 1.0f;

  // --- sample the perimeter into points, clockwise from top-center ---
  static const int CSEG = 10;                 // line segments per 90-deg corner
  static lv_point_precise_t pts[96];
  int n = 0;
  #define RRPUSH(px,py) do { if (n < (int)(sizeof(pts)/sizeof(pts[0]))) {      \
      pts[n].x = (lv_value_precise_t)lroundf(px);                             \
      pts[n].y = (lv_value_precise_t)lroundf(py); ++n; } } while (0)
  #define RRARC(cx,cy,a0) do { for (int _i = 0; _i <= CSEG; ++_i) {           \
      float _a = ((a0) + 90.0f * _i / CSEG) * 0.0174532925f;                 \
      RRPUSH((cx) + r * cosf(_a), (cy) + r * sinf(_a)); } } while (0)

  RRPUSH(x + w / 2.0f, y);                     // top center
  RRPUSH(x + w - r,    y);                     // top edge -> TR
  RRARC (x + w - r, y + r,       270.0f);      // TR corner
  RRPUSH(x + w,        y + h - r);             // right edge -> BR
  RRARC (x + w - r, y + h - r,     0.0f);      // BR corner
  RRPUSH(x + r,        y + h);                 // bottom edge -> BL
  RRARC (x + r,     y + h - r,    90.0f);      // BL corner
  RRPUSH(x,            y + r);                 // left edge -> TL
  RRARC (x + r,     y + r,       180.0f);      // TL corner
  RRPUSH(x + w / 2.0f, y);                     // close at top center
  #undef RRPUSH
  #undef RRARC

  // The path above is built clockwise from top-center in logical coords;
  // reverse it so the sweep reads clockwise on the rotated/mounted display.
  // (Both endpoints are top-center, so the anchor is unchanged.)
  for (int i = 0, j = n - 1; i < j; ++i, --j) {
    lv_point_precise_t tmp = pts[i]; pts[i] = pts[j]; pts[j] = tmp;
  }

  // --- total perimeter length ---
  float total = 0.0f;
  for (int i = 1; i < n; ++i) {
    float dx = (float)(pts[i].x - pts[i - 1].x);
    float dy = (float)(pts[i].y - pts[i - 1].y);
    total += sqrtf(dx * dx + dy * dy);
  }
  if (total <= 0.0f) return;
  float target = frac * total;

  lv_draw_line_dsc_t dsc;
  lv_draw_line_dsc_init(&dsc);
  dsc.color       = color;
  dsc.width       = stroke;
  dsc.round_start = 1;          // round caps make the segment joints seamless
  dsc.round_end   = 1;

  // --- stroke segments up to `target` arc-length ---
  float drawn = 0.0f;
  for (int i = 1; i < n && drawn < target; ++i) {
    float x1 = (float)pts[i - 1].x, y1 = (float)pts[i - 1].y;
    float x2 = (float)pts[i].x,     y2 = (float)pts[i].y;
    float seg = sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    if (seg <= 0.0f) continue;
    float t = (target - drawn < seg) ? (target - drawn) / seg : 1.0f;
    dsc.p1.x = (lv_value_precise_t)lroundf(x1);
    dsc.p1.y = (lv_value_precise_t)lroundf(y1);
    dsc.p2.x = (lv_value_precise_t)lroundf(x1 + (x2 - x1) * t);
    dsc.p2.y = (lv_value_precise_t)lroundf(y1 + (y2 - y1) * t);
    lv_draw_line(layer, &dsc);
    drawn += seg;
  }
}

static void timer_frame_draw_event(lv_event_t *e) {
  lv_layer_t *layer = lv_event_get_layer(e);
  lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);

  lv_area_t coords;
  lv_obj_get_coords(obj, &coords);
  const int x = coords.x1;
  const int y = coords.y1;
  const int w = lv_area_get_width(&coords);
  const int h = lv_area_get_height(&coords);
  const int r = (int)lv_obj_get_style_radius(obj, LV_PART_MAIN);

  const int p_idx = (int)state.phase;

  // Track first (full perimeter, dim), then highlighted progress on top.
  draw_rrect_progress(layer, x, y, w, h, r, FRAME_STROKE_PX,
                      theme.progress_track[p_idx], 1.0f);
  if (g_timer_progress_frac > 0.0f) {
    draw_rrect_progress(layer, x, y, w, h, r, FRAME_STROKE_PX,
                        theme.progress[p_idx], g_timer_progress_frac);
  }
}

// ---------------------------------------------------------------
// Calibration measuring-wheel illustration (procedural).
//
// Draws a measuring wheel with its handle tilted `g_cal_view_angle` degrees
// from vertical: a dashed reference line along the handle axis, a short
// vertical reference, the angle arc + number between them, the wheel (a few
// concentric circles), and the thick handle pole. Pivot = wheel center, near
// the lower-center of the container; handle points up-and-to-the-left.
// ---------------------------------------------------------------
static void draw_cal_wheel_event(lv_event_t *e) {
  lv_layer_t *layer = lv_event_get_layer(e);
  lv_obj_t  *obj    = (lv_obj_t *)lv_event_get_target(e);
  lv_area_t c; lv_obj_get_coords(obj, &c);
  const int W = lv_area_get_width(&c);
  const int H = lv_area_get_height(&c);

  float ang = g_cal_view_angle;
  if (ang < 0.0f) ang = 0.0f; else if (ang > 90.0f) ang = 90.0f;
  const float a = ang * 0.0174532925f;

  // Pivot (wheel center) lower center-right; handle points up-and-left.
  const float px = c.x1 + W * 0.62f;
  const float py = c.y2 - 180.0f;
  const float dx = -sinf(a), dy = -cosf(a);
  const float L  = H * 0.55f;
  const float tipx = px + dx * L, tipy = py + dy * L;

  #define CWX(v) ((lv_value_precise_t)lroundf(v))

  // Dashed reference line along the handle axis (extends past both ends).
  lv_draw_line_dsc_t dl; lv_draw_line_dsc_init(&dl);
  dl.color = lv_color_hex(0x666666); dl.width = 2;
  dl.dash_width = 10; dl.dash_gap = 8;
  dl.p1.x = CWX(px - dx * 80.0f);  dl.p1.y = CWX(py - dy * 80.0f);
  dl.p2.x = CWX(px + dx * (L + 70.0f)); dl.p2.y = CWX(py + dy * (L + 70.0f));
  lv_draw_line(layer, &dl);

  // Short vertical reference up from the pivot.
  lv_draw_line_dsc_t vl; lv_draw_line_dsc_init(&vl);
  vl.color = lv_color_hex(0xFFFFFF); vl.width = 2;
  vl.p1.x = CWX(px); vl.p1.y = CWX(py);
  vl.p2.x = CWX(px); vl.p2.y = CWX(py - 210.0f);
  lv_draw_line(layer, &vl);

  // Angle arc between the handle direction and vertical-up.
  float hdeg = atan2f(dy, dx) * 57.2957795f;   // handle angle (screen, y-down)
  if (hdeg < 0.0f) hdeg += 360.0f;
  lv_draw_arc_dsc_t ad; lv_draw_arc_dsc_init(&ad);
  ad.color = lv_color_hex(0xFFFFFF); ad.width = 3; ad.rounded = 0;
  ad.center.x = CWX(px); ad.center.y = CWX(py);
  ad.radius = 95;
  ad.start_angle = (uint16_t)lroundf(hdeg);
  ad.end_angle   = 270;                         // vertical up
  lv_draw_arc(layer, &ad);

  // Angle number near the arc (static buffer so the deferred draw sees it).
  static char wheel_num[8];
  snprintf(wheel_num, sizeof(wheel_num), "%d", (int)lroundf(ang));
  lv_draw_label_dsc_t nd; lv_draw_label_dsc_init(&nd);
  nd.color = lv_color_hex(0xFFFFFF);
  nd.font  = &lv_font_montserrat_24;
  nd.text  = wheel_num;
  nd.align = LV_TEXT_ALIGN_CENTER;
  float midr = ((hdeg + 270.0f) * 0.5f) * 0.0174532925f;
  int lx = (int)lroundf(px + 135.0f * cosf(midr));
  int ly = (int)lroundf(py + 135.0f * sinf(midr));
  lv_area_t na; na.x1 = lx - 24; na.y1 = ly - 16; na.x2 = lx + 24; na.y2 = ly + 16;
  lv_draw_label(layer, &nd, &na);

  // Handle pole.
  lv_draw_line_dsc_t hl; lv_draw_line_dsc_init(&hl);
  hl.color = lv_color_hex(0xC8C8C8); hl.width = 12;
  hl.round_start = 1; hl.round_end = 1;
  hl.p1.x = CWX(px); hl.p1.y = CWX(py);
  hl.p2.x = CWX(tipx); hl.p2.y = CWX(tipy);
  lv_draw_line(layer, &hl);

  // Wheel: concentric circles via rounded-rect fills.
  lv_draw_rect_dsc_t rd; lv_draw_rect_dsc_init(&rd);
  rd.radius = LV_RADIUS_CIRCLE; rd.bg_opa = LV_OPA_COVER;
  struct { int r; uint32_t col; } rings[] = {
    { 40, 0xBFBFBF }, { 33, 0xE0B43C }, { 11, 0x333333 }
  };
  for (auto &g : rings) {
    rd.bg_color = lv_color_hex(g.col);
    lv_area_t a2;
    a2.x1 = CWX(px - g.r); a2.y1 = CWX(py - g.r);
    a2.x2 = CWX(px + g.r); a2.y2 = CWX(py + g.r);
    lv_draw_rect(layer, &rd, &a2);
  }
  #undef CWX
}

// ---------------------------------------------------------------
// Main screen builder
//
// Layout (Tab5 = 1280x720 landscape):
//   Top strip:  state pill (center), battery (right)
//   Middle:     rest-count column (left), timer frame w/ perimeter progress
//               (center), Calibrate + History buttons (right stack)
//   Bottom:     full-width Ready/Reset button
//
// Colors come from the global `theme` and are repainted per phase in
// refresh_ui(). The initial values here are placeholders.
// ---------------------------------------------------------------
static void build_main_screen(lv_obj_t *scr) {
  lv_obj_set_style_text_font(scr, &inter_24, 0);
  lv_obj_set_style_bg_color(scr, theme.bg[(int)TimerPhase::IDLE], 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // ----- Top strip -----------------------------------------------------

  // Top-left: current tilt threshold (in degrees) + chime volume.
  info_label = lv_label_create(scr);
  lv_obj_set_style_text_font(info_label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(info_label, theme.caption[(int)TimerPhase::IDLE], 0);
  lv_label_set_text(info_label, "");
  lv_obj_align(info_label, LV_ALIGN_TOP_LEFT, 32, 28);

  // State pill (top-center)
  state_pill = lv_obj_create(scr);
  lv_obj_remove_style_all(state_pill);
  lv_obj_set_size(state_pill, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_pad_hor(state_pill, 22, 0);
  lv_obj_set_style_pad_ver(state_pill, 8, 0);
  lv_obj_set_style_border_width(state_pill, 2, 0);
  lv_obj_set_style_border_color(state_pill, theme.pill[(int)TimerPhase::IDLE], 0);
  lv_obj_set_style_radius(state_pill, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(state_pill, LV_OPA_TRANSP, 0);
  lv_obj_align(state_pill, LV_ALIGN_TOP_MID, 0, 70);
  lv_obj_clear_flag(state_pill, LV_OBJ_FLAG_SCROLLABLE);

  state_pill_label = lv_label_create(state_pill);
  lv_label_set_text(state_pill_label, phase_name(TimerPhase::IDLE));
  lv_obj_set_style_text_color(state_pill_label,
                              theme.pill[(int)TimerPhase::IDLE], 0);

  // Battery (top-right)
  battery_label = lv_label_create(scr);
  lv_obj_set_style_text_color(battery_label, theme.battery_normal, 0);
  lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_24, 0);
  lv_obj_align(battery_label, LV_ALIGN_TOP_RIGHT, -32, 36);

  // ----- Left column: rest counter ------------------------------------
  //
  // Wrapped in a flex column so "Rested" and "Times" stay horizontally
  // centered on the big digit no matter how wide the digit grows
  // (single -> double -> triple digit).
  lv_obj_t *rest_column = lv_obj_create(scr);
  lv_obj_remove_style_all(rest_column);
  lv_obj_set_size(rest_column, 300, 420);
  lv_obj_align(rest_column, LV_ALIGN_LEFT_MID, 20, 0);
  lv_obj_set_flex_flow(rest_column, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(rest_column,
                        LV_FLEX_ALIGN_CENTER,   // main axis: stack packed vertically centered
                        LV_FLEX_ALIGN_CENTER,   // cross axis: each child horizontally centered
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(rest_column, 20, 0);
  lv_obj_clear_flag(rest_column, LV_OBJ_FLAG_SCROLLABLE);

  rested_caption_label = lv_label_create(rest_column);
  lv_label_set_text(rested_caption_label, "Rested");
  lv_obj_set_style_text_color(rested_caption_label,
                              theme.caption[(int)TimerPhase::IDLE], 0);

  rested_count_label = lv_label_create(rest_column);
  lv_obj_set_style_text_font(rested_count_label, &inter_medium_digit, 0);
  lv_obj_set_style_text_letter_space(rested_count_label, DIGIT_LETTER_SPACE, 0);
  lv_obj_set_style_text_color(rested_count_label,
                              theme.rest_count[(int)TimerPhase::IDLE], 0);
  lv_label_set_text(rested_count_label, "0");

  rested_times_label = lv_label_create(rest_column);
  lv_label_set_text(rested_times_label, "Times");
  lv_obj_set_style_text_color(rested_times_label,
                              theme.caption[(int)TimerPhase::IDLE], 0);

  // ----- Center: timer frame + digits + progress + caption -------------

  timer_frame = lv_obj_create(scr);
  lv_obj_remove_style_all(timer_frame);
  lv_obj_set_size(timer_frame, 600, 280);   // <-- progress-indicator size
  // Let the big digits show even if they're wider/taller than the frame, so
  // shrinking the frame never clips the timer text (it just overflows).
  lv_obj_add_flag(timer_frame, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_set_style_radius(timer_frame, 70, 0);   // <-- corner radius
  // Native border off -- the visible outline is drawn in the DRAW_POST
  // handler so we can highlight only the progress portion of it.
  lv_obj_set_style_border_width(timer_frame, 0, 0);
  lv_obj_set_style_bg_opa(timer_frame, LV_OPA_TRANSP, 0);
  lv_obj_align(timer_frame, LV_ALIGN_CENTER, 0, -20);
  lv_obj_clear_flag(timer_frame, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(timer_frame, timer_frame_draw_event,
                      LV_EVENT_DRAW_POST, nullptr);

  // Big timer digits, child of the frame so the frame sizing wraps them.
  timer_label = lv_label_create(timer_frame);
  lv_obj_set_style_text_font(timer_label, &inter_medium_digit, 0);
  lv_obj_set_style_text_letter_space(timer_label, DIGIT_LETTER_SPACE, 0);
  lv_obj_set_style_text_color(timer_label,
                              theme.timer[(int)TimerPhase::IDLE], 0);
  lv_label_set_text(timer_label, "0:00");
  lv_obj_center(timer_label);

  // "Min:Sec" caption sits BELOW the frame (sibling of frame, child of scr).
  timer_caption_label = lv_label_create(scr);
  lv_label_set_text(timer_caption_label, "Min Sec");
  lv_obj_set_style_text_color(timer_caption_label,
                              theme.caption[(int)TimerPhase::IDLE], 0);
  lv_obj_align_to(timer_caption_label, timer_frame,
                  LV_ALIGN_OUT_BOTTOM_MID, 0, 12);

  // ----- Right column: Settings + History stacked ---------------------

  // Buttons are icon-only now; the text labels are siblings of `scr` placed
  // just below each button via LV_ALIGN_OUT_BOTTOM_MID.
  // (var named calibrate_btn for historical reasons; it opens the Settings
  //  screen, which now hosts both tilt calibration and volume.)
  calibrate_btn = lv_btn_create(scr);
  lv_obj_set_size(calibrate_btn, 195, 110);
  lv_obj_set_style_radius(calibrate_btn, 30, 0);
  lv_obj_set_style_bg_color(calibrate_btn, theme.side_btn_bg, 0);
  lv_obj_set_style_border_width(calibrate_btn, BTN_BORDER_PX, 0);
  lv_obj_set_style_border_color(calibrate_btn, theme.side_btn_border, 0);
  lv_obj_set_style_shadow_width(calibrate_btn, 0, 0);
  lv_obj_align(calibrate_btn, LV_ALIGN_RIGHT_MID, -48, -120);
  lv_obj_add_event_cb(calibrate_btn, settings_short_clicked,
                      LV_EVENT_SHORT_CLICKED, nullptr);
  lv_obj_add_event_cb(calibrate_btn, settings_long_pressed,
                      LV_EVENT_LONG_PRESSED, nullptr);

  // Centered icon inside the button.
  lv_obj_t *calibrate_icon = lv_label_create(calibrate_btn);
  lv_label_set_text(calibrate_icon, LV_SYMBOL_SETTINGS);
  lv_obj_set_style_text_font(calibrate_icon, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(calibrate_icon, theme.side_btn_text, 0);
  lv_obj_center(calibrate_icon);

  // Label sits OUTSIDE the button, just below it.
  calibrate_btn_label = lv_label_create(scr);
  lv_label_set_text(calibrate_btn_label, "SETTINGS");
  lv_obj_set_style_text_color(calibrate_btn_label, theme.side_btn_text, 0);
  lv_obj_align_to(calibrate_btn_label, calibrate_btn,
                  LV_ALIGN_OUT_BOTTOM_MID, 0, 12);

  history_btn = lv_btn_create(scr);
  lv_obj_set_size(history_btn, 195, 110);
  lv_obj_set_style_radius(history_btn, 30, 0);
  lv_obj_set_style_bg_color(history_btn, theme.side_btn_bg, 0);
  lv_obj_set_style_border_width(history_btn, BTN_BORDER_PX, 0);
  lv_obj_set_style_border_color(history_btn, theme.side_btn_border, 0);
  lv_obj_set_style_shadow_width(history_btn, 0, 0);
  lv_obj_align(history_btn, LV_ALIGN_RIGHT_MID, -48, 80);
  lv_obj_add_event_cb(history_btn, history_clicked, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *history_icon = lv_label_create(history_btn);
  lv_label_set_text(history_icon, LV_SYMBOL_LOOP);
  lv_obj_set_style_text_font(history_icon, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(history_icon, theme.side_btn_text, 0);
  lv_obj_center(history_icon);

  history_btn_label = lv_label_create(scr);
  lv_label_set_text(history_btn_label, "HISTORY");
  lv_obj_set_style_text_color(history_btn_label, theme.side_btn_text, 0);
  lv_obj_align_to(history_btn_label, history_btn,
                  LV_ALIGN_OUT_BOTTOM_MID, 0, 12);

  // ----- Bottom: dual-purpose Ready/Reset button ----------------------

  main_button_btn = lv_btn_create(scr);
  lv_obj_set_size(main_button_btn, 720, 80);
  lv_obj_set_style_radius(main_button_btn, 30, 0);
  lv_obj_set_style_bg_color(main_button_btn, theme.ready_btn_bg, 0);
  lv_obj_set_style_border_width(main_button_btn, BTN_BORDER_PX, 0);
  lv_obj_set_style_border_color(main_button_btn, theme.ready_btn_border, 0);
  lv_obj_set_style_shadow_width(main_button_btn, 0, 0);
  lv_obj_align(main_button_btn, LV_ALIGN_BOTTOM_MID, 0, -40);
  lv_obj_add_event_cb(main_button_btn, main_button_clicked,
                      LV_EVENT_CLICKED, nullptr);

  main_button_label = lv_label_create(main_button_btn);
  lv_label_set_text(main_button_label, "Press when Ready");
  lv_obj_set_style_text_color(main_button_label, theme.ready_btn_text, 0);
  lv_obj_center(main_button_label);
}

// ---------------------------------------------------------------
// Finishing screen builder
// ---------------------------------------------------------------
// Small helper: style a finish-screen action button in the app's shared
// bordered-rounded-rect language.
static lv_obj_t *make_finish_button(lv_obj_t *scr, lv_color_t bg, lv_color_t border,
                                    lv_event_cb_t cb) {
  lv_obj_t *btn = lv_btn_create(scr);
  lv_obj_set_size(btn, 240, 72);
  lv_obj_set_style_bg_color(btn, bg, 0);
  lv_obj_set_style_border_width(btn, BTN_BORDER_PX, 0);
  lv_obj_set_style_border_color(btn, border, 0);
  lv_obj_set_style_radius(btn, 18, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
  return btn;
}

static void build_finish_screen(lv_obj_t *scr) {
  finish_title_label = make_settings_title(scr, "Session Complete");  // black bg
  lv_obj_set_style_text_color(finish_title_label, lv_color_hex(0xFF3B30), 0);

  // Summary line ("3 rests · 1m 45s total rest")
  finish_summary_label = lv_label_create(scr);
  lv_obj_set_style_text_color(finish_summary_label, lv_color_white(), 0);
  lv_label_set_text(finish_summary_label, "");
  lv_obj_align(finish_summary_label, LV_ALIGN_TOP_MID, 0, 76);

  // Bar chart container (scrollable) -- same boxed style as history detail.
  finish_bar_container = lv_obj_create(scr);
  lv_obj_set_size(finish_bar_container, 1120, 430);
  lv_obj_align(finish_bar_container, LV_ALIGN_TOP_MID, 0, 124);
  lv_obj_set_style_bg_color(finish_bar_container, theme.side_btn_bg, 0);
  lv_obj_set_style_border_color(finish_bar_container, theme.side_btn_border, 0);
  lv_obj_set_style_border_width(finish_bar_container, BTN_BORDER_PX, 0);
  lv_obj_set_style_radius(finish_bar_container, 18, 0);
  lv_obj_set_style_pad_all(finish_bar_container, 20, 0);
  lv_obj_set_scroll_dir(finish_bar_container, LV_DIR_VER);

  // Save / Discard (left), Reset (right) -- shared button language.
  finish_save_btn = make_finish_button(scr, theme.ready_btn_bg,
                                       theme.ready_btn_border,
                                       save_session_clicked);
  lv_obj_align(finish_save_btn, LV_ALIGN_BOTTOM_LEFT, 60, -36);
  finish_save_btn_label = lv_label_create(finish_save_btn);
  lv_label_set_text(finish_save_btn_label, "Save");
  lv_obj_set_style_text_color(finish_save_btn_label, lv_color_white(), 0);
  lv_obj_center(finish_save_btn_label);

  finish_discard_btn = make_finish_button(scr, theme.side_btn_bg,
                                          theme.side_btn_border,
                                          discard_session_clicked);
  lv_obj_align(finish_discard_btn, LV_ALIGN_BOTTOM_LEFT, 330, -36);
  finish_discard_btn_label = lv_label_create(finish_discard_btn);
  lv_label_set_text(finish_discard_btn_label, "Discard");
  lv_obj_set_style_text_color(finish_discard_btn_label, lv_color_white(), 0);
  lv_obj_center(finish_discard_btn_label);

  // Status line ("Saved as Session #14" / "Discarded" / auto-save countdown)
  finish_status_label = lv_label_create(scr);
  lv_obj_set_style_text_color(finish_status_label, lv_color_hex(0xAAAAAA), 0);
  lv_label_set_text(finish_status_label, "");
  lv_obj_align(finish_status_label, LV_ALIGN_BOTTOM_MID, 60, -58);

  finish_reset_btn = make_finish_button(scr, theme.reset_btn_bg,
                                        theme.reset_btn_border,
                                        finish_reset_clicked);
  lv_obj_align(finish_reset_btn, LV_ALIGN_BOTTOM_RIGHT, -60, -36);
  lv_obj_t *fr_lbl = lv_label_create(finish_reset_btn);
  lv_label_set_text(fr_lbl, "Reset");
  lv_obj_set_style_text_color(fr_lbl, lv_color_white(), 0);
  lv_obj_center(fr_lbl);
}

// ---------------------------------------------------------------
// Charging-mode screen: big % (digit font) + "%" sign, info text, Exit.
// ---------------------------------------------------------------
static void build_charging_screen(lv_obj_t *scr) {
  lv_obj_set_style_text_font(scr, &inter_24, 0);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // Big battery percentage (digits only -- the digit font has no '%').
  charge_pct_label = lv_label_create(scr);
  lv_obj_set_style_text_font(charge_pct_label, &inter_medium_digit, 0);
  lv_obj_set_style_text_color(charge_pct_label, lv_color_hex(0x4CAF50), 0);
  lv_label_set_text(charge_pct_label, "0");
  lv_obj_align(charge_pct_label, LV_ALIGN_CENTER, -40, -130);

  // "%" sign in the UI font, beside the big number.
  lv_obj_t *pct_sign = lv_label_create(scr);
  lv_obj_set_style_text_font(pct_sign, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(pct_sign, lv_color_hex(0x4CAF50), 0);
  lv_label_set_text(pct_sign, "%");
  lv_obj_align_to(pct_sign, charge_pct_label, LV_ALIGN_OUT_RIGHT_TOP, 16, 40);

  // Info text.
  lv_obj_t *info = lv_label_create(scr);
  lv_obj_set_width(info, 1000);
  lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(info, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(info, lv_color_hex(0xAAAAAA), 0);
  lv_label_set_text(info,
      "QuartzPad only charges while powered on -- plugging into power while "
      "off will not charge the battery.\n\n"
      "Exiting charging mode does not stop charging. The device shuts down "
      "automatically once the battery is full.");
  lv_obj_align(info, LV_ALIGN_CENTER, 0, 140);

  // Exit button. Wide enough for "Exit Charging Mode" in the 32px font.
  lv_obj_t *exit_btn = lv_btn_create(scr);
  lv_obj_set_size(exit_btn, 460, 84);
  lv_obj_align(exit_btn, LV_ALIGN_BOTTOM_MID, 0, -36);
  lv_obj_set_style_bg_color(exit_btn, theme.side_btn_bg, 0);
  lv_obj_set_style_radius(exit_btn, 18, 0);
  lv_obj_set_style_border_width(exit_btn, BTN_BORDER_PX, 0);
  lv_obj_set_style_border_color(exit_btn, theme.side_btn_border, 0);
  lv_obj_set_style_shadow_width(exit_btn, 0, 0);
  lv_obj_add_event_cb(exit_btn, back_to_main_clicked, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *el = lv_label_create(exit_btn);
  lv_label_set_text(el, "Exit Charging Mode");
  lv_obj_set_style_text_color(el, theme.side_btn_text, 0);
  lv_obj_center(el);
}

// ---------------------------------------------------------------
// Settings: a menu of sub-screens (Calibration / Volume / Pre-start).
// Each row opens its own sub-screen; their back buttons return here.
// ---------------------------------------------------------------
static lv_obj_t *make_menu_row(lv_obj_t *parent, const char *text,
                               lv_event_cb_t cb) {
  // Same look as the main-screen side buttons: bordered rounded rect, no shadow.
  lv_obj_t *row = lv_btn_create(parent);
  lv_obj_set_width(row, lv_pct(100));
  lv_obj_set_height(row, 84);
  lv_obj_set_style_bg_color(row, theme.side_btn_bg, 0);
  lv_obj_set_style_border_width(row, BTN_BORDER_PX, 0);
  lv_obj_set_style_border_color(row, theme.side_btn_border, 0);
  lv_obj_set_style_shadow_width(row, 0, 0);
  lv_obj_set_style_radius(row, 18, 0);
  lv_obj_set_style_pad_hor(row, 28, 0);
  lv_obj_add_event_cb(row, cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *lbl = lv_label_create(row);
  lv_label_set_text(lbl, text);
  lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
  lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 0, 0);

  lv_obj_t *arrow = lv_label_create(row);
  lv_label_set_text(arrow, LV_SYMBOL_RIGHT);
  lv_obj_set_style_text_font(arrow, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(arrow, lv_color_hex(0x888888), 0);
  lv_obj_align(arrow, LV_ALIGN_RIGHT_MID, 0, 0);
  return row;
}

static void cal_row_clicked(lv_event_t *)      { enter_calibration_screen(); }
static void vol_row_clicked(lv_event_t *)      { enter_volume_screen(); }
static void prestart_row_clicked(lv_event_t *) { enter_prestart_screen(); }

// Shared style for a settings slider -- same red-knob language as the
// calibration page's angle slider.
static void style_settings_slider(lv_obj_t *sl) {
  lv_obj_set_style_bg_color(sl, lv_color_hex(0x3A3A3A), LV_PART_MAIN);
  lv_obj_set_style_bg_color(sl, theme.reset_btn_bg, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(sl, theme.reset_btn_bg, LV_PART_KNOB);
  lv_obj_set_style_radius(sl, LV_RADIUS_CIRCLE, LV_PART_KNOB);
  lv_obj_set_style_pad_all(sl, 8, LV_PART_KNOB);
}

static lv_obj_t *make_settings_title(lv_obj_t *scr, const char *text) {
  lv_obj_set_style_text_font(scr, &inter_24, 0);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);   // match main screen
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t *title = lv_label_create(scr);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_label_set_text(title, text);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 28);
  return title;
}

// ----- Settings menu -----
static void build_settings_screen(lv_obj_t *scr) {
  make_settings_title(scr, "Settings");
  make_back_button(scr, back_to_main_clicked);

  lv_obj_t *list = lv_obj_create(scr);
  lv_obj_remove_style_all(list);
  lv_obj_set_size(list, 900, 380);
  lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 120);
  lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(list, 18, 0);
  lv_obj_clear_flag(list, LV_OBJ_FLAG_SCROLLABLE);

  make_menu_row(list, "Calibration",         cal_row_clicked);
  make_menu_row(list, "Volume",              vol_row_clicked);
  make_menu_row(list, "Pre-start Countdown", prestart_row_clicked);
}

// ----- Calibration sub-screen: vertical angle slider + rotating wheel ------
// Layout matches the mockup: Exit (top-left), vertical slider (left), the
// rotating measuring-wheel illustration (center), "Tilting Limit NN" (right),
// Auto-Calibrate, and a big "RESET TO NN DEGREE" at the bottom.
static void build_calibration_screen(lv_obj_t *scr) {
  lv_obj_set_style_text_font(scr, &inter_24, 0);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // Wheel illustration -- full-screen, transparent, created FIRST so it sits
  // behind the controls and its handle/dashed lines never clip. Not clickable.
  cal_wheel = lv_obj_create(scr);
  lv_obj_remove_style_all(cal_wheel);
  lv_obj_set_size(cal_wheel, lv_pct(100), lv_pct(100));
  lv_obj_align(cal_wheel, LV_ALIGN_CENTER, 0, 0);
  lv_obj_clear_flag(cal_wheel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(cal_wheel, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(cal_wheel, draw_cal_wheel_event, LV_EVENT_DRAW_POST, nullptr);

  // Exit button (top-left, text -- not the arrow icon used elsewhere).
  lv_obj_t *exit_btn = lv_btn_create(scr);
  lv_obj_set_size(exit_btn, 160, 64);
  lv_obj_align(exit_btn, LV_ALIGN_TOP_LEFT, 32, 28);
  lv_obj_set_style_bg_color(exit_btn, theme.side_btn_bg, 0);
  lv_obj_set_style_radius(exit_btn, 18, 0);
  lv_obj_set_style_border_width(exit_btn, BTN_BORDER_PX, 0);
  lv_obj_set_style_border_color(exit_btn, theme.side_btn_border, 0);
  lv_obj_set_style_shadow_width(exit_btn, 0, 0);
  lv_obj_add_event_cb(exit_btn, back_to_settings_clicked, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *exit_lbl = lv_label_create(exit_btn);
  lv_label_set_text(exit_lbl, "Exit");
  lv_obj_set_style_text_color(exit_lbl, theme.side_btn_text, 0);
  lv_obj_center(exit_lbl);

  // Vertical slider (left) -- value is the tilt-limit in DEGREES.
  settings_slider = lv_slider_create(scr);
  lv_obj_set_size(settings_slider, 28, 440);
  lv_obj_align(settings_slider, LV_ALIGN_LEFT_MID, 300, -10);
  lv_slider_set_range(settings_slider, ANGLE_MIN_DEG, ANGLE_MAX_DEG);
  lv_slider_set_value(settings_slider, threshold_to_deg(state.threshold), LV_ANIM_OFF);
  lv_obj_set_style_bg_color(settings_slider, lv_color_hex(0x3A3A3A), LV_PART_MAIN);
  lv_obj_set_style_bg_color(settings_slider, lv_color_hex(0xEB4C46), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(settings_slider, lv_color_hex(0xEB4C46), LV_PART_KNOB);
  lv_obj_set_style_radius(settings_slider, LV_RADIUS_CIRCLE, LV_PART_KNOB);
  lv_obj_set_style_pad_all(settings_slider, 10, LV_PART_KNOB);   // bigger round knob
  lv_obj_add_event_cb(settings_slider, slider_value_changed_cb,
                      LV_EVENT_VALUE_CHANGED, nullptr);
  lv_obj_add_event_cb(settings_slider, slider_released_cb,
                      LV_EVENT_RELEASED, nullptr);

  // "Tilting Limit" + big degree number (right).
  lv_obj_t *tl = lv_label_create(scr);
  lv_obj_set_style_text_font(tl, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(tl, lv_color_white(), 0);
  lv_label_set_text(tl, "Tilting Limit");
  lv_obj_align(tl, LV_ALIGN_RIGHT_MID, -90, -160);

  cal_big_label = lv_label_create(scr);
  lv_obj_set_style_text_font(cal_big_label, &inter_medium_digit, 0);
  lv_obj_set_style_text_color(cal_big_label, lv_color_white(), 0);
  lv_label_set_text_fmt(cal_big_label, "%d", threshold_to_deg(state.threshold));
  lv_obj_align(cal_big_label, LV_ALIGN_RIGHT_MID, -90, -10);

  // Auto-Calibrate / Cancel button (right, under the number).
  cal_btn = lv_btn_create(scr);
  lv_obj_set_size(cal_btn, 280, 60);
  lv_obj_align(cal_btn, LV_ALIGN_RIGHT_MID, -80, 180);
  lv_obj_set_style_bg_color(cal_btn, lv_color_hex(0x2E6FB0), 0);
  lv_obj_set_style_radius(cal_btn, 30, 0);
  lv_obj_set_style_shadow_width(cal_btn, 0, 0);
  lv_obj_add_event_cb(cal_btn, cal_btn_clicked, LV_EVENT_CLICKED, nullptr);
  cal_btn_label = lv_label_create(cal_btn);
  lv_label_set_text(cal_btn_label, "Auto-Calibrate");
  lv_obj_set_style_text_color(cal_btn_label, lv_color_white(), 0);
  lv_obj_center(cal_btn_label);

  // Hold-steady progress bar (hidden until a capture step is active).
  cal_progress_bar = lv_bar_create(scr);
  lv_obj_set_size(cal_progress_bar, 700, 16);
  lv_obj_align(cal_progress_bar, LV_ALIGN_BOTTOM_MID, 0, -170);
  lv_bar_set_range(cal_progress_bar, 0, 100);
  lv_bar_set_value(cal_progress_bar, 0, LV_ANIM_OFF);
  lv_obj_set_style_radius(cal_progress_bar, 8, LV_PART_MAIN);
  lv_obj_set_style_radius(cal_progress_bar, 8, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(cal_progress_bar, lv_color_hex(0x16384F), LV_PART_MAIN);
  lv_obj_set_style_bg_color(cal_progress_bar, lv_color_hex(0x40C4FF),
                            LV_PART_INDICATOR);
  lv_obj_add_flag(cal_progress_bar, LV_OBJ_FLAG_HIDDEN);

  // Status / wizard instructions (above the reset button, centered).
  settings_state_label = lv_label_create(scr);
  lv_obj_set_width(settings_state_label, 1180);
  lv_obj_set_style_text_align(settings_state_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(settings_state_label, lv_color_hex(0x888888), 0);
  lv_label_set_text(settings_state_label, "");
  lv_obj_align(settings_state_label, LV_ALIGN_BOTTOM_MID, 0, -120);

  // Big red RESET button (bottom).
  lv_obj_t *rst_btn = lv_btn_create(scr);
  lv_obj_set_size(rst_btn, 740, 66);
  lv_obj_align(rst_btn, LV_ALIGN_BOTTOM_MID, 0, -28);
  lv_obj_set_style_bg_color(rst_btn, theme.reset_btn_bg, 0);
  lv_obj_set_style_radius(rst_btn, 14, 0);
  lv_obj_set_style_shadow_width(rst_btn, 0, 0);
  lv_obj_add_event_cb(rst_btn, reset_threshold_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *rst_lbl = lv_label_create(rst_btn);
  lv_label_set_text_fmt(rst_lbl, "RESET TO %d DEGREE", DEFAULT_ANGLE_DEG);
  lv_obj_set_style_text_color(rst_lbl, theme.reset_btn_text, 0);
  lv_obj_center(rst_lbl);
}

// ----- Volume sub-screen -----
static void build_volume_screen(lv_obj_t *scr) {
  make_settings_title(scr, "Volume");
  make_back_button(scr, back_to_settings_clicked);

  volume_value_label = lv_label_create(scr);
  lv_obj_set_style_text_font(volume_value_label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(volume_value_label, lv_color_hex(0xFFA500), 0);
  lv_label_set_text_fmt(volume_value_label, "Volume: %d%%", (int)state.volume_pct);
  lv_obj_align(volume_value_label, LV_ALIGN_TOP_MID, 0, 130);

  volume_slider = lv_slider_create(scr);
  lv_obj_set_size(volume_slider, 1100, 28);
  lv_obj_align(volume_slider, LV_ALIGN_TOP_MID, 0, 200);
  lv_slider_set_range(volume_slider, 0, 100);
  lv_slider_set_value(volume_slider, state.volume_pct, LV_ANIM_OFF);
  style_settings_slider(volume_slider);
  lv_obj_add_event_cb(volume_slider, volume_changed_cb,
                      LV_EVENT_VALUE_CHANGED, nullptr);
  lv_obj_add_event_cb(volume_slider, volume_released_cb,
                      LV_EVENT_RELEASED, nullptr);

  lv_obj_t *vol_hint = lv_label_create(scr);
  lv_obj_set_style_text_color(vol_hint, lv_color_hex(0x777777), 0);
  lv_label_set_text(vol_hint, "Release the slider to hear a test chime");
  lv_obj_align(vol_hint, LV_ALIGN_TOP_MID, 0, 270);
}

// ----- Pre-start countdown sub-screen -----
static void build_prestart_screen(lv_obj_t *scr) {
  make_settings_title(scr, "Pre-start Countdown");
  make_back_button(scr, back_to_settings_clicked);

  lv_obj_t *desc = lv_label_create(scr);
  lv_obj_set_style_text_color(desc, lv_color_hex(0xAAAAAA), 0);
  lv_obj_set_style_text_align(desc, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(desc,
      "A get-ready countdown runs when you start, then the 6:00 begins.\n"
      "Slide fully left for Off.");
  lv_obj_align(desc, LV_ALIGN_TOP_MID, 0, 100);

  prestart_value_label = lv_label_create(scr);
  lv_obj_set_style_text_font(prestart_value_label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(prestart_value_label, lv_color_hex(0xFFA500), 0);
  if (state.prestart_sec == 0)
    lv_label_set_text(prestart_value_label, "Pre-start: Off");
  else
    lv_label_set_text_fmt(prestart_value_label, "Pre-start: %ds",
                          (int)state.prestart_sec);
  lv_obj_align(prestart_value_label, LV_ALIGN_TOP_MID, 0, 220);

  prestart_slider = lv_slider_create(scr);
  lv_obj_set_size(prestart_slider, 1100, 28);
  lv_obj_align(prestart_slider, LV_ALIGN_TOP_MID, 0, 290);
  // Leftmost step (PRESTART_MIN-1) = Off; PRESTART_MIN..PRESTART_MAX = seconds.
  lv_slider_set_range(prestart_slider, PRESTART_MIN - 1, PRESTART_MAX);
  lv_slider_set_value(prestart_slider,
                      state.prestart_sec >= PRESTART_MIN ? state.prestart_sec
                                                         : (PRESTART_MIN - 1),
                      LV_ANIM_OFF);
  style_settings_slider(prestart_slider);
  lv_obj_add_event_cb(prestart_slider, prestart_changed_cb,
                      LV_EVENT_VALUE_CHANGED, nullptr);
  lv_obj_add_event_cb(prestart_slider, prestart_released_cb,
                      LV_EVENT_RELEASED, nullptr);

  lv_obj_t *off_lbl = lv_label_create(scr);
  lv_label_set_text(off_lbl, "Off");
  lv_obj_set_style_text_color(off_lbl, lv_color_hex(0x777777), 0);
  lv_obj_align_to(off_lbl, prestart_slider, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 8);

  lv_obj_t *max_lbl = lv_label_create(scr);
  lv_label_set_text_fmt(max_lbl, "%ds", (int)PRESTART_MAX);
  lv_obj_set_style_text_color(max_lbl, lv_color_hex(0x777777), 0);
  lv_obj_align_to(max_lbl, prestart_slider, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 8);
}

// ---------------------------------------------------------------
// History list screen builder
// ---------------------------------------------------------------
static void build_history_list_screen(lv_obj_t *scr) {
  make_settings_title(scr, "History");   // black bg + centered title, like Settings

  make_back_button(scr, back_to_main_clicked);

  history_list_container = lv_obj_create(scr);
  lv_obj_set_size(history_list_container, 1120, 580);
  lv_obj_align(history_list_container, LV_ALIGN_TOP_MID, 0, 110);
  lv_obj_set_style_bg_opa(history_list_container, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(history_list_container, 0, 0);
  lv_obj_set_style_pad_all(history_list_container, 8, 0);
  lv_obj_set_scroll_dir(history_list_container, LV_DIR_VER);
  lv_obj_set_flex_flow(history_list_container, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(history_list_container, 14, 0);

  history_empty_label = lv_label_create(scr);
  lv_label_set_text(history_empty_label, "No sessions yet");
  lv_obj_set_style_text_color(history_empty_label, lv_color_hex(0xAAAAAA), 0);
  lv_obj_align(history_empty_label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_flag(history_empty_label, LV_OBJ_FLAG_HIDDEN);
}

// Rebuild list rows from the in-RAM meta_cache. Cheap text rows only.
static void refresh_history_list_rows() {
  lv_obj_clean(history_list_container);

  if (state.stored_count == 0) {
    lv_obj_clear_flag(history_empty_label, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  lv_obj_add_flag(history_empty_label, LV_OBJ_FLAG_HIDDEN);

  // Newest first (highest seq) -> iterate stored_seqs descending.
  for (size_t i = state.stored_count; i-- > 0; ) {
    SessionMeta &m = state.meta_cache[i];

    // Rows styled like the main-screen buttons: bordered rounded rect.
    lv_obj_t *row = lv_btn_create(history_list_container);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, 72);
    lv_obj_set_style_bg_color(row, theme.side_btn_bg, 0);
    lv_obj_set_style_border_width(row, BTN_BORDER_PX, 0);
    lv_obj_set_style_border_color(row, theme.side_btn_border, 0);
    lv_obj_set_style_shadow_width(row, 0, 0);
    lv_obj_set_style_radius(row, 18, 0);
    lv_obj_set_style_pad_hor(row, 28, 0);
    lv_obj_add_event_cb(row, history_row_clicked, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)m.seq);

    uint32_t mm = m.total_rest_sec / 60;
    uint32_t ss = m.total_rest_sec % 60;
    char total_buf[24];
    if (mm > 0) {
      snprintf(total_buf, sizeof(total_buf), "%lum %02lus",
               (unsigned long)mm, (unsigned long)ss);
    } else {
      snprintf(total_buf, sizeof(total_buf), "%lus", (unsigned long)ss);
    }

    lv_obj_t *lbl = lv_label_create(row);
    lv_label_set_text_fmt(lbl, "Session #%lu     %lu rest%s     %s",
                          (unsigned long)m.seq,
                          (unsigned long)m.rest_count,
                          m.rest_count == 1 ? "" : "s",
                          total_buf);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *arrow = lv_label_create(row);
    lv_label_set_text(arrow, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(arrow, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_font(arrow, &lv_font_montserrat_24, 0);
    lv_obj_align(arrow, LV_ALIGN_RIGHT_MID, 0, 0);
  }
}

// ---------------------------------------------------------------
// History detail screen builder
// ---------------------------------------------------------------
static void build_history_detail_screen(lv_obj_t *scr) {
  history_detail_title_label = make_settings_title(scr, "Session");  // black bg

  make_back_button(scr, back_to_history_list_clicked);

  history_detail_summary_label = lv_label_create(scr);
  lv_obj_set_style_text_color(history_detail_summary_label,
                              lv_color_hex(0xAAAAAA), 0);
  lv_label_set_text(history_detail_summary_label, "");
  lv_obj_align(history_detail_summary_label, LV_ALIGN_TOP_MID, 0, 76);

  history_detail_bar_container = lv_obj_create(scr);
  lv_obj_set_size(history_detail_bar_container, 1120, 540);
  lv_obj_align(history_detail_bar_container, LV_ALIGN_TOP_MID, 0, 124);
  lv_obj_set_style_bg_color(history_detail_bar_container,
                            theme.side_btn_bg, 0);
  lv_obj_set_style_border_color(history_detail_bar_container,
                                theme.side_btn_border, 0);
  lv_obj_set_style_border_width(history_detail_bar_container, BTN_BORDER_PX, 0);
  lv_obj_set_style_radius(history_detail_bar_container, 18, 0);
  lv_obj_set_style_pad_all(history_detail_bar_container, 20, 0);
  lv_obj_set_scroll_dir(history_detail_bar_container, LV_DIR_VER);
}

// ---------------------------------------------------------------
// Debug overlay builder  -- DEV ONLY
// ---------------------------------------------------------------
static void build_debug_overlay(lv_obj_t *parent) {
  debug_panel = lv_obj_create(parent);
  lv_obj_remove_style_all(debug_panel);
  lv_obj_set_size(debug_panel, 560, 480);
  lv_obj_align(debug_panel, LV_ALIGN_RIGHT_MID, -40, 0);
  lv_obj_set_style_bg_color(debug_panel, lv_color_hex(0x111111), 0);
  lv_obj_set_style_bg_opa(debug_panel, LV_OPA_90, 0);
  lv_obj_set_style_radius(debug_panel, 16, 0);
  lv_obj_set_style_border_color(debug_panel, lv_color_hex(0x444444), 0);
  lv_obj_set_style_border_width(debug_panel, 2, 0);
  lv_obj_set_style_pad_all(debug_panel, 16, 0);
  lv_obj_clear_flag(debug_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(debug_panel, LV_OBJ_FLAG_HIDDEN);

  debug_accel_label = lv_label_create(debug_panel);
  lv_obj_align(debug_accel_label, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_text_color(debug_accel_label, lv_color_white(), 0);
  lv_label_set_text(debug_accel_label,
                    "raw g  ax --\n       ay --\n       az --\n|a| --");

  debug_gyro_label = lv_label_create(debug_panel);
  lv_obj_align(debug_gyro_label, LV_ALIGN_TOP_LEFT, 180, 0);
  lv_obj_set_style_text_color(debug_gyro_label, lv_color_white(), 0);
  lv_label_set_text(debug_gyro_label,
                    "gyro   gx --\n       gy --\n       gz --\nmotion --");

  debug_phase_label = lv_label_create(debug_panel);
  lv_obj_align(debug_phase_label, LV_ALIGN_TOP_LEFT, 360, 0);
  lv_obj_set_style_text_color(debug_phase_label, lv_color_hex(0xAAAAAA), 0);
  lv_label_set_text(debug_phase_label,
                    "phase: ---\nrest: 0s/0 seg\nlogged: 0");

  debug_chart = lv_chart_create(debug_panel);
  lv_obj_set_size(debug_chart, 528, 240);
  lv_obj_align(debug_chart, LV_ALIGN_TOP_LEFT, 0, 110);
  lv_chart_set_type(debug_chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(debug_chart, CHART_POINT_COUNT);
  lv_chart_set_range(debug_chart, LV_CHART_AXIS_PRIMARY_Y, -150, 150);
  // Circular mode: chart_set_next_value updates ring buffer without scheduling
  // a redraw every push. We invalidate explicitly in the throttled section.
  lv_chart_set_update_mode(debug_chart, LV_CHART_UPDATE_MODE_CIRCULAR);
  lv_obj_set_style_bg_color(debug_chart, lv_color_hex(0x000000), 0);
  lv_obj_set_style_size(debug_chart, 0, 0, LV_PART_INDICATOR);

  // Threshold lines (initial values; updated when user changes threshold)
  int32_t v = (int32_t)(state.threshold * 100.0f);
  for (int i = 0; i < CHART_POINT_COUNT; i++) {
    debug_thr_pos_data[i] =  v;
    debug_thr_neg_data[i] = -v;
  }
  debug_thr_pos_series = lv_chart_add_series(
      debug_chart, lv_color_hex(0x666666), LV_CHART_AXIS_PRIMARY_Y);
  debug_thr_neg_series = lv_chart_add_series(
      debug_chart, lv_color_hex(0x666666), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_series_ext_y_array(debug_chart, debug_thr_pos_series,
                                  debug_thr_pos_data);
  lv_chart_set_series_ext_y_array(debug_chart, debug_thr_neg_series,
                                  debug_thr_neg_data);

  debug_ax_series = lv_chart_add_series(
      debug_chart, lv_color_hex(0xFF3B30), LV_CHART_AXIS_PRIMARY_Y);
  debug_ay_series = lv_chart_add_series(
      debug_chart, lv_color_hex(0x4CAF50), LV_CHART_AXIS_PRIMARY_Y);
  debug_az_series = lv_chart_add_series(
      debug_chart, lv_color_hex(0x00A0FF), LV_CHART_AXIS_PRIMARY_Y);
  debug_motion_series = lv_chart_add_series(
      debug_chart, lv_color_hex(0xFFFFFF), LV_CHART_AXIS_PRIMARY_Y);

  record_btn = lv_btn_create(debug_panel);
  lv_obj_set_size(record_btn, 140, 60);
  lv_obj_align(record_btn, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_bg_color(record_btn, lv_color_hex(0xCC0000), 0);
  lv_obj_set_style_radius(record_btn, 30, 0);
  lv_obj_add_event_cb(record_btn, record_clicked, LV_EVENT_CLICKED, nullptr);
  record_btn_label = lv_label_create(record_btn);
  lv_label_set_text(record_btn_label, "REC");
  lv_obj_set_style_text_color(record_btn_label, lv_color_white(), 0);
  lv_obj_center(record_btn_label);

  record_status_label = lv_label_create(debug_panel);
  lv_obj_align(record_status_label, LV_ALIGN_BOTTOM_LEFT, 160, -20);
  lv_obj_set_style_text_color(record_status_label, lv_color_hex(0xAAAAAA), 0);
  lv_label_set_text(record_status_label, state.sd_ok ? "ready" : "no SD");
}

// ---------------------------------------------------------------
// Screen navigation
// ---------------------------------------------------------------
static void enter_main_screen() {
  state.screen = Screen::MAIN;
  M5.Display.setBrightness(g_normal_brightness);  // undo charging-mode dim
  mark_dirty();             // force a refresh once we're back
  lv_screen_load(screen_main);
}

static void enter_finish_screen() {
  state.screen = Screen::FINISH;
  state.finish_decided = false;
  state.finish_entered_ms = millis();   // start the 15-min auto-save+exit timer

  // Reset buttons to their initial enabled appearance
  lv_obj_clear_state(finish_save_btn,    LV_STATE_DISABLED);
  lv_obj_clear_state(finish_discard_btn, LV_STATE_DISABLED);
  lv_obj_set_style_bg_color(finish_save_btn,        theme.ready_btn_bg, 0);
  lv_obj_set_style_border_color(finish_save_btn,    theme.ready_btn_border, 0);
  lv_obj_set_style_bg_color(finish_discard_btn,     theme.side_btn_bg, 0);
  lv_obj_set_style_border_color(finish_discard_btn, theme.side_btn_border, 0);
  lv_label_set_text(finish_status_label, "");

  // Fill in summary and bar chart
  uint32_t mm = state.total_rest_sec / 60;
  uint32_t ss = state.total_rest_sec % 60;
  if (mm > 0) {
    lv_label_set_text_fmt(finish_summary_label,
                          "%lu rest%s  ·  %lum %02lus total rest",
                          (unsigned long)state.rest_count,
                          state.rest_count == 1 ? "" : "s",
                          (unsigned long)mm, (unsigned long)ss);
  } else {
    lv_label_set_text_fmt(finish_summary_label,
                          "%lu rest%s  ·  %lus total rest",
                          (unsigned long)state.rest_count,
                          state.rest_count == 1 ? "" : "s",
                          (unsigned long)ss);
  }

  render_current_session_bar_chart(finish_bar_container);

  lv_screen_load(screen_finish);
}

static void enter_charging_screen() {
  state.screen = Screen::CHARGING;
  int lvl = state.battery_level < 0 ? 0 : state.battery_level;
  g_charge_peak       = lvl;   // seed peak for the drop-detect exit
  g_charge_full_since = 0;     // fresh full-charge timer for this visit
  lv_label_set_text_fmt(charge_pct_label, "%d", lvl);
  M5.Display.setBrightness(CHARGING_BRIGHTNESS);  // dim while charging
  lv_screen_load(screen_charging);
}

// Settings menu (the hub).
static void enter_settings_screen() {
  state.screen = Screen::SETTINGS;
  lv_screen_load(screen_settings);
}

// Calibration sub-screen: sync the angle slider + wheel + number, then show it.
static void enter_calibration_screen() {
  state.screen = Screen::SET_CALIBRATION;
  cal.msg_until = 0;
  int deg = threshold_to_deg(state.threshold);
  lv_slider_set_value(settings_slider, deg, LV_ANIM_OFF);
  cal_reset_wizard();                   // idle; sets wheel + big number to `deg`
  lv_label_set_text(settings_state_label, "");
  state.last_settings_readout_ms = 0;   // force an immediate readout update
  lv_screen_load(screen_set_calibration);
}

// Volume sub-screen.
static void enter_volume_screen() {
  state.screen = Screen::SET_VOLUME;
  lv_slider_set_value(volume_slider, state.volume_pct, LV_ANIM_OFF);
  lv_label_set_text_fmt(volume_value_label, "Volume: %d%%", (int)state.volume_pct);
  lv_screen_load(screen_set_volume);
}

// Pre-start countdown sub-screen.
static void enter_prestart_screen() {
  state.screen = Screen::SET_PRESTART;
  lv_slider_set_value(prestart_slider,
                      state.prestart_sec >= PRESTART_MIN ? state.prestart_sec
                                                         : (PRESTART_MIN - 1),
                      LV_ANIM_OFF);
  if (state.prestart_sec == 0)
    lv_label_set_text(prestart_value_label, "Pre-start: Off");
  else
    lv_label_set_text_fmt(prestart_value_label, "Pre-start: %ds",
                          (int)state.prestart_sec);
  lv_screen_load(screen_set_prestart);
}

static void enter_history_list_screen() {
  state.screen = Screen::HISTORY_LIST;
  refresh_history_list_rows();
  lv_screen_load(screen_history_list);
}

static void enter_history_detail_screen(uint32_t seq) {
  StoredSession s;
  if (!nvs_load_session(seq, &s)) {
    // Corrupted or missing -> just return to list
    enter_history_list_screen();
    return;
  }
  state.detail_seq = seq;
  state.screen     = Screen::HISTORY_DETAIL;

  lv_label_set_text_fmt(history_detail_title_label,
                        "Session #%lu", (unsigned long)s.seq);

  uint32_t mm = s.total_rest_sec / 60;
  uint32_t ss = s.total_rest_sec % 60;
  if (mm > 0) {
    lv_label_set_text_fmt(history_detail_summary_label,
                          "%lu rest%s  ·  %lum %02lus total",
                          (unsigned long)s.rest_count,
                          s.rest_count == 1 ? "" : "s",
                          (unsigned long)mm, (unsigned long)ss);
  } else {
    lv_label_set_text_fmt(history_detail_summary_label,
                          "%lu rest%s  ·  %lus total",
                          (unsigned long)s.rest_count,
                          s.rest_count == 1 ? "" : "s",
                          (unsigned long)ss);
  }

  size_t n = s.rest_count;
  if (n > REST_LOG_CAPACITY) n = REST_LOG_CAPACITY;
  render_rest_bar_chart(history_detail_bar_container, s.rest_durations, n);

  lv_screen_load(screen_history_detail);
}

// ---------------------------------------------------------------
// LVGL bridge to M5GFX (display + touch)
// ---------------------------------------------------------------
static uint32_t lvgl_tick_cb() {
  return (uint32_t)(esp_timer_get_time() / 1000LL);
}

// Set true in lvgl_init_for_tab5() only if BOTH draw buffers were allocated.
// Background (non-blocking) DMA is only safe when double-buffered: LVGL must
// have a second buffer to render into while the first one is still being
// transferred. With one buffer we fall back to a blocking copy.
static bool g_double_buffered = false;

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area,
                          uint8_t *px_map) {
  uint16_t *buf16 = (uint16_t *)px_map;
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  M5.Display.startWrite();
  if (USE_DMA_FLUSH && g_double_buffered) {
    // Non-blocking: the DMA engine transfers this buffer while LVGL renders
    // the next region into the other buffer. We do NOT waitDMA() here --
    // LovyanGFX waits for any in-flight DMA at the start of the next
    // pushImageDMA, so a buffer is never reused mid-transfer.
    M5.Display.pushImageDMA(area->x1, area->y1, w, h, buf16);
  } else {
    // Single buffer (or DMA disabled): blocking CPU copy. Safe because the
    // push completes before we tell LVGL the buffer is free again.
    M5.Display.pushImage(area->x1, area->y1, w, h, buf16);
  }
  M5.Display.endWrite();

  lv_display_flush_ready(disp);
}

static void lvgl_indev_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  (void)indev;
  lgfx::touch_point_t tp;
  uint8_t n = M5.Display.getTouch(&tp, 1);
  if (n > 0) {
    data->point.x = tp.x;
    data->point.y = tp.y;
    data->state   = LV_INDEV_STATE_PRESSED;
    g_idle_since_ms = millis();   // touch counts as activity (defers auto-sleep)
  } else {
    data->state   = LV_INDEV_STATE_RELEASED;
  }
}

static void lvgl_init_for_tab5() {
  lv_init();
  lv_tick_set_cb(lvgl_tick_cb);

  lv_display_t *disp = lv_display_create(M5.Display.width(),
                                         M5.Display.height());
  lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565_SWAPPED);

  uint32_t buf_rows = LVGL_BUFFER_ROWS;
  if (buf_rows == 0 || buf_rows > (uint32_t)M5.Display.height()) {
    buf_rows = M5.Display.height();
  }
  size_t buf_bytes = (size_t)M5.Display.width() * buf_rows * 2;

  // Buffer 1 is mandatory.
  uint8_t *buf1 = (uint8_t *)ps_malloc(buf_bytes);
  if (!buf1) {
    M5.Display.fillScreen(TFT_RED);
    M5.Display.setTextColor(TFT_WHITE, TFT_RED);
    M5.Display.setCursor(20, 20);
    M5.Display.printf("LVGL buffer failed\n%u bytes", (unsigned)buf_bytes);
    while (true) delay(1000);
  }

  // Buffer 2 is optional: it's what makes the background DMA flush safe (LVGL
  // renders into one buffer while the other is mid-transfer). If PSRAM can't
  // satisfy it we degrade gracefully to single-buffered + blocking copy
  // instead of failing to boot.
  uint8_t *buf2 = (uint8_t *)ps_malloc(buf_bytes);
  g_double_buffered = (buf2 != NULL);

  lv_display_set_buffers(disp, buf1, buf2, buf_bytes,
                         LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(disp, lvgl_flush_cb);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, lvgl_indev_read_cb);
}

// ---------------------------------------------------------------
// Build all screens
// ---------------------------------------------------------------
static void build_all_screens() {
  screen_main            = lv_obj_create(NULL);
  screen_finish          = lv_obj_create(NULL);
  screen_settings        = lv_obj_create(NULL);
  screen_set_calibration = lv_obj_create(NULL);
  screen_set_volume      = lv_obj_create(NULL);
  screen_set_prestart    = lv_obj_create(NULL);
  screen_history_list    = lv_obj_create(NULL);
  screen_history_detail  = lv_obj_create(NULL);
  screen_charging        = lv_obj_create(NULL);

  build_main_screen(screen_main);
  build_finish_screen(screen_finish);
  build_settings_screen(screen_settings);
  build_calibration_screen(screen_set_calibration);
  build_volume_screen(screen_set_volume);
  build_prestart_screen(screen_set_prestart);
  build_history_list_screen(screen_history_list);
  build_history_detail_screen(screen_history_detail);
  build_charging_screen(screen_charging);

  // Debug overlay lives on top of the main screen.  -- DEV ONLY
  build_debug_overlay(screen_main);

  // "Sleeping soon" notice lives on the top layer so it shows over whichever
  // screen is active when the idle power-off approaches. Hidden by default.
  sleep_warn_label = lv_label_create(lv_layer_top());
  lv_obj_set_style_text_font(sleep_warn_label, &inter_24, 0);
  lv_obj_set_style_text_color(sleep_warn_label, lv_color_white(), 0);
  lv_obj_set_style_bg_color(sleep_warn_label, lv_color_hex(0x202020), 0);
  lv_obj_set_style_bg_opa(sleep_warn_label, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(sleep_warn_label, theme.side_btn_border, 0);
  lv_obj_set_style_border_width(sleep_warn_label, BTN_BORDER_PX, 0);
  lv_obj_set_style_radius(sleep_warn_label, 18, 0);
  lv_obj_set_style_pad_hor(sleep_warn_label, 28, 0);
  lv_obj_set_style_pad_ver(sleep_warn_label, 14, 0);
  lv_label_set_text(sleep_warn_label, "");
  lv_obj_align(sleep_warn_label, LV_ALIGN_TOP_MID, 0, 150);
  lv_obj_add_flag(sleep_warn_label, LV_OBJ_FLAG_HIDDEN);
}

// ---------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  //Serial.begin(115200);

  M5.Display.setRotation(DISPLAY_ROTATION);

  enable_battery_charging();

  // DEV ONLY: SD init (non-fatal)
  state.sd_ok = sd_init();

  // Load threshold + volume + session history from NVS BEFORE building
  // screens, so the settings sliders and history list pick up the right values.
  prefs_init();

  // M5.begin() already brought up the speaker (cfg.internal_spk defaults true);
  // apply the persisted chime volume.
  apply_volume();

  lvgl_init_for_tab5();
  build_all_screens();

  state.last_tick_ms = millis();
  g_idle_since_ms    = millis();   // start the auto-sleep idle clock
  g_normal_brightness = M5.Display.getBrightness();   // for charging-mode dim/restore
  state.screen = Screen::MAIN;
  lv_screen_load(screen_main);
  refresh_ui();
}

void loop() {
  M5.update();
  uint32_t now = millis();

  poll_battery(now);

  // Poll the IMU at ~50 Hz (gates the downstream tilt/log/overlay work too).
  if (now - state.last_imu_poll_ms >= IMU_POLL_INTERVAL_MS) {
    state.last_imu_poll_ms = now;
    poll_imu();
    if (state.imu_fresh) {
      uint64_t now_us = esp_timer_get_time();
      log_imu_sample(now, now_us);          // DEV ONLY (no-op if not logging)
      refresh_debug_overlay(now);           // DEV ONLY (no-op if hidden)
      update_tilt_state(now);
    }
  }

  update_prestart(now);     // advance the "Get Ready" lead-in, hand off to RUNNING
  update_countdown(now);
  update_chimes();          // chime on activity phase transitions
  refresh_ui();
  update_calibration(now);  // calibration sub-screen: live readout + auto-cal wizard
  update_charging(now);     // charging mode: live %, full -> off, unplug -> exit

  // Finish screen: after FINISH_AUTOEXIT_MS, auto-save (unless the user already
  // chose Save/Discard) and return to Not Active -- instead of powering off.
  if (state.phase == TimerPhase::FINISHED &&
      now - state.finish_entered_ms >= FINISH_AUTOEXIT_MS) {
    if (!state.finish_decided) save_current_session();
    reset_session_state();
    enter_main_screen();
  }

  // While undecided on the finish screen, surface the auto-save deadline in
  // the status line ("Auto-saves in 14:32"). Save/Discard replace this text.
  if (state.screen == Screen::FINISH && state.phase == TimerPhase::FINISHED &&
      !state.finish_decided) {
    static uint32_t last_shown_s = UINT32_MAX;
    uint32_t elapsed = now - state.finish_entered_ms;
    uint32_t left_s  = (elapsed < FINISH_AUTOEXIT_MS)
                         ? (FINISH_AUTOEXIT_MS - elapsed + 999) / 1000 : 0;
    if (left_s != last_shown_s) {
      last_shown_s = left_s;
      lv_label_set_text_fmt(finish_status_label, "Auto-saves in %lu:%02lu",
                            (unsigned long)(left_s / 60),
                            (unsigned long)(left_s % 60));
    }
  }

  // Armed-and-forgotten guard: READY with no tilt for READY_TIMEOUT_MS ->
  // quietly disarm to Not Active (whose idle auto-off then applies). Without
  // this, an armed device would stay awake until the battery died.
  if (state.phase == TimerPhase::READY &&
      now - state.ready_since_ms >= READY_TIMEOUT_MS) {
    reset_session_state();   // -> IDLE, marks dirty
  }

  // Adaptive idle: sleep longer when nothing time-sensitive is happening
  // (waiting for a tap), shorter while a session is armed/running or a
  // calibration capture is in progress (need snappy tilt/IMU response).
  // Finished is handled above (15-min auto-save+exit) and charging stays on to
  // charge, so both are excluded from the idle auto-off.
  bool active = (state.phase != TimerPhase::IDLE) ||
                (cal.step != CalStep::IDLE) ||
                (state.screen == Screen::CHARGING);

  // Auto power-off: anything active keeps the idle clock reset; once truly
  // idle (Not Active screen, untouched) for AUTOSLEEP_MS, shut down -- with a
  // visible "sleeping soon" warning for the last SLEEP_WARN_MS (any touch
  // resets the idle clock and hides it). powerOff() does not return.
  if (active) {
    g_idle_since_ms = now;
    lv_obj_add_flag(sleep_warn_label, LV_OBJ_FLAG_HIDDEN);
  } else {
    uint32_t idle = now - g_idle_since_ms;
    if (idle >= AUTOSLEEP_MS) {
      M5.Power.powerOff();
    } else if (AUTOSLEEP_MS - idle <= SLEEP_WARN_MS) {
      static uint32_t last_warn_s = UINT32_MAX;
      uint32_t left_s = (AUTOSLEEP_MS - idle + 999) / 1000;
      if (left_s != last_warn_s) {
        last_warn_s = left_s;
        lv_label_set_text_fmt(sleep_warn_label,
                              "Sleeping in %lus - touch to stay awake",
                              (unsigned long)left_s);
      }
      lv_obj_clear_flag(sleep_warn_label, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(sleep_warn_label, LV_OBJ_FLAG_HIDDEN);
    }
  }

  uint32_t cap = active ? LOOP_DELAY_ACTIVE_MS : LOOP_DELAY_IDLE_MS;
  uint32_t next_ms = lv_timer_handler();
  if (next_ms > cap) next_ms = cap;
  delay(next_ms);
}

// ---------------------------------------------------------------
// IF SD.begin() FAILS ON YOUR TAB5:  -- DEV ONLY
// Replace the SPI version of sd_init() with this SD_MMC version,
// also replace `#include <SD.h>` with `#include <SD_MMC.h>`,
// and replace `SD.open(...)` calls with `SD_MMC.open(...)`.
//
//   #define sd_D0    GPIO_NUM_39
//   #define sd_D1    GPIO_NUM_40
//   #define sd_D2    GPIO_NUM_41
//   #define sd_D3    GPIO_NUM_42
//   #define sd_CLK   GPIO_NUM_43
//   #define sd_CMD   GPIO_NUM_44
//
//   static bool sd_init() {
//     SD_MMC.setPins(sd_CLK, sd_CMD, sd_D0, sd_D1, sd_D2, sd_D3);
//     return SD_MMC.begin("/sdcard", false, false, 20000);
//   }
// ---------------------------------------------------------------
