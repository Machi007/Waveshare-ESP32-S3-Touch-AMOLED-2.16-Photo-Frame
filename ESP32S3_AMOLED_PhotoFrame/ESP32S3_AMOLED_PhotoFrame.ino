#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <esp_heap_caps.h>
#include <esp_system.h>

#include <Arduino_GFX_Library.h>
#include <TouchDrv.hpp>
#include <XPowersLib.h>
#include <JPEGDEC.h>

// JPEG decoding and rendering can exceed loopTask's default 8 KB stack.
// Arduino-ESP32 3.x supports increasing the stack with this macro.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// ============================================================================
// Waveshare ESP32-S3-Touch-AMOLED-2.16
// 480x480 CO5300 AMOLED, CST9220 touch, AXP2101 PMU, microSD via SPI
// ============================================================================

// Display: CO5300 over QSPI
static constexpr int LCD_SDIO0 = 4;
static constexpr int LCD_SDIO1 = 5;
static constexpr int LCD_SDIO2 = 6;
static constexpr int LCD_SDIO3 = 7;
static constexpr int LCD_SCLK  = 38;
static constexpr int LCD_CS    = 12;
static constexpr int LCD_RESET = 39;
static constexpr int LCD_WIDTH = 480;
static constexpr int LCD_HEIGHT = 480;

// Shared I2C bus, touch controller pins
static constexpr int IIC_SDA = 15;
static constexpr int IIC_SCL = 14;
static constexpr int TP_INT = 11;
static constexpr int TP_RESET = 40;

// microSD (SPI)
static constexpr int SD_MOSI = 1;
static constexpr int SD_SCK  = 2;
static constexpr int SD_MISO = 3;
static constexpr int SD_CS   = 41;

// Built-in brightness controls. GPIO0/BOOT remains reserved for bootloader use.
static constexpr int BUTTON_BRIGHT_UP = 18;  // + / KEY
// Brightness down is handled by the AXP2101 PWR-key short-press event.

// Available slideshow intervals. Default is 10 seconds.
static constexpr uint32_t SLIDE_INTERVALS_MS[] = {
  5000, 10000, 15000, 20000, 30000, 60000, 120000, 300000, 600000
};
static constexpr const char *SLIDE_INTERVAL_LABELS[] = {
  "5 S", "10 S", "15 S", "20 S", "30 S", "1 MIN", "2 MIN", "5 MIN", "10 MIN"
};
static constexpr size_t SLIDE_INTERVAL_COUNT =
    sizeof(SLIDE_INTERVALS_MS) / sizeof(SLIDE_INTERVALS_MS[0]);
static constexpr uint32_t BATTERY_OVERLAY_MS = 8000;
static constexpr uint32_t BRIGHTNESS_OVERLAY_MS = 1300;
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 35;
static constexpr uint32_t TOUCH_DEBOUNCE_MS = 350;
static constexpr uint8_t BRIGHTNESS_STEP = 20;
static constexpr uint8_t MIN_BRIGHTNESS = 8;
static constexpr uint8_t START_BRIGHTNESS = 160;
static constexpr size_t MAX_PHOTOS = 500;

// Two touch targets inside the battery panel.
static constexpr int RANDOM_BUTTON_X = 36;
static constexpr int RANDOM_BUTTON_Y = 292;
static constexpr int RANDOM_BUTTON_W = 196;
static constexpr int RANDOM_BUTTON_H = 46;
static constexpr int FIT_BUTTON_X = 248;
static constexpr int FIT_BUTTON_Y = 292;
static constexpr int FIT_BUTTON_W = 196;
static constexpr int FIT_BUTTON_H = 46;

// Slideshow interval controls inside the battery overlay.
static constexpr int INTERVAL_MINUS_X = 36;
static constexpr int INTERVAL_PLUS_X = 390;
static constexpr int INTERVAL_BUTTON_Y = 344;
static constexpr int INTERVAL_EDGE_BUTTON_W = 54;
static constexpr int INTERVAL_CENTER_X = 100;
static constexpr int INTERVAL_CENTER_W = 280;
static constexpr int INTERVAL_BUTTON_H = 44;

static constexpr uint16_t C_BLACK  = 0x0000;
static constexpr uint16_t C_WHITE  = 0xFFFF;
static constexpr uint16_t C_CYAN   = 0x07FF;
static constexpr uint16_t C_BLUE   = 0x001F;
static constexpr uint16_t C_GREEN  = 0x07E0;
static constexpr uint16_t C_YELLOW = 0xFFE0;
static constexpr uint16_t C_GRAY   = 0x4208;

Arduino_DataBus *displayBus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *displayDriver = new Arduino_CO5300(
    displayBus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 0, 0, 0, 0);

// The CO5300 can flip axes but does not provide a true 90-degree hardware
// rotation. Render through an Arduino_Canvas rotated clockwise instead. Put
// the 480x480 framebuffer explicitly in PSRAM so it doesn't consume scarce
// internal RAM. Canvas::flush() then sends the already-rotated frame to CO5300.
class PsramCanvas : public Arduino_Canvas {
 public:
  using Arduino_Canvas::Arduino_Canvas;

  bool begin(int32_t speed = GFX_NOT_DEFINED) override {
    if ((speed != GFX_SKIP_OUTPUT_BEGIN) && _output) {
      if (!_output->begin(speed)) return false;
    }
    if (!_framebuffer) {
      const size_t bytes = static_cast<size_t>(LCD_WIDTH) * LCD_HEIGHT * sizeof(uint16_t);
      _framebuffer = static_cast<uint16_t *>(
          heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
      if (!_framebuffer) {
        // Fall back to general heap for variants where malloc can use PSRAM.
        _framebuffer = static_cast<uint16_t *>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
      }
      if (!_framebuffer) return false;
      memset(_framebuffer, 0, bytes);
    }
    return true;
  }
};

// Rotation 1 means 90 degrees clockwise in Arduino_GFX's canvas coordinates.
PsramCanvas *gfx = new PsramCanvas(LCD_WIDTH, LCD_HEIGHT, displayDriver, 0, 0, 1);

// JPEGDEC owns large internal work buffers (~17.9 KB in the observed build).
// Keep it in static/global RAM, not on loopTask's limited stack.
JPEGDEC jpegDecoder;

// To prevent thin black scan-line artefacts, all reduced-resolution JPEG blocks
// are assembled into one complete image in PSRAM. EXIF orientation and the chosen
// crop/fit mode are applied after decoding, then one finished bitmap is displayed.
static bool jpegCaptureActive = false;
static uint16_t *jpegCaptureBuffer = nullptr;
static int jpegCaptureRegionX = 0;
static int jpegCaptureRegionY = 0;
static int jpegCaptureRegionWidth = 0;
static int jpegCaptureRegionHeight = 0;
static int jpegCaptureImageWidth = 0;
static int jpegCaptureImageHeight = 0;

TouchDrvCST92xx touch;
int16_t touchX[2] = {0, 0};
int16_t touchY[2] = {0, 0};
volatile bool touchPending = false;

// Keep SD on HSPI (SPI3). The AMOLED QSPI driver uses FSPI/SPI2;
// sharing that host makes SD.open() fail after drawing to the display.
SPIClass sdSPI(HSPI);
std::vector<String> photoPaths;
size_t currentPhoto = 0;          // index of the currently displayed file in photoPaths
bool randomPlayback = false;      // false = sorted order, true = shuffle without repeats
bool fitWholePhoto = false;       // false = fill/crop square; true = show entire photo
std::vector<size_t> shuffleOrder; // sequence of photoPaths indices used in random mode
size_t shufflePosition = 0;
size_t slideIntervalIndex = 1;  // Default: 10 seconds.
bool sdReady = false;

XPowersAXP2101 *pmu = nullptr;
bool pmuReady = false;

uint8_t displayBrightness = START_BRIGHTNESS;
uint32_t lastSlideAt = 0;
uint32_t lastTouchActionAt = 0;
uint32_t overlayUntil = 0;
enum OverlayKind { OVERLAY_NONE, OVERLAY_BATTERY, OVERLAY_BRIGHTNESS };
OverlayKind currentOverlay = OVERLAY_NONE;

// Keep the settings persistence promised by the previous project version.
Preferences settingsStore;
bool settingsStoreReady = false;
void saveSettings();

// Keep button state in primitive variables. This avoids an Arduino IDE
// auto-generated prototype referring to a custom ButtonState type too early.
bool brightnessUpLastRead = HIGH;
bool brightnessUpStableRead = HIGH;
uint32_t brightnessUpLastChangeAt = 0;
uint32_t lastPmuPollAt = 0;

// JPEGDEC uses the native FS::File integration provided by JPEGDEC.h.
// This keeps the file handle and seek/read callbacks consistent with the library driver.

int jpegDrawCallback(JPEGDRAW *draw) {
  if (draw == nullptr || draw->pPixels == nullptr) return 0;

  // Capture decoded blocks into a full reduced-resolution frame first.
  // This avoids line seams and lets us apply EXIF orientation consistently.
  if (!jpegCaptureActive || jpegCaptureBuffer == nullptr ||
      jpegCaptureRegionWidth <= 0 || jpegCaptureRegionHeight <= 0) {
    gfx->draw16bitRGBBitmap(draw->x, draw->y, draw->pPixels,
                            draw->iWidth, draw->iHeight);
    return 1;
  }

  const int blockWidth = (draw->iWidthUsed > 0)
      ? min(draw->iWidthUsed, draw->iWidth) : draw->iWidth;
  const int blockX0 = max(draw->x, jpegCaptureRegionX);
  const int blockX1 = min(draw->x + blockWidth,
                          jpegCaptureRegionX + jpegCaptureRegionWidth);
  const int usableY1 = min(draw->y + draw->iHeight, jpegCaptureImageHeight);
  const int blockY0 = max(draw->y, jpegCaptureRegionY);
  const int blockY1 = min(usableY1,
                          jpegCaptureRegionY + jpegCaptureRegionHeight);
  if (blockX1 <= blockX0 || blockY1 <= blockY0) return 1;

  const int copyWidth = blockX1 - blockX0;
  const size_t copyBytes = static_cast<size_t>(copyWidth) * sizeof(uint16_t);
  const int sourceX = blockX0 - draw->x;
  const int destX = blockX0 - jpegCaptureRegionX;

  for (int sourceY = blockY0; sourceY < blockY1; ++sourceY) {
    const int sourceRow = sourceY - draw->y;
    const int destRow = sourceY - jpegCaptureRegionY;
    const uint16_t *src = draw->pPixels + sourceRow * draw->iWidth + sourceX;
    uint16_t *dst = jpegCaptureBuffer + destRow * jpegCaptureRegionWidth + destX;
    memcpy(dst, src, copyBytes);
  }
  return 1;
}

// ---------------------------------------------------------------------------
// User interface drawing
// ---------------------------------------------------------------------------
void drawCenteredMessage(const char *line1, const char *line2 = nullptr) {
  gfx->fillScreen(C_BLACK);
  gfx->setTextColor(C_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(24, 205);
  gfx->println(line1);
  if (line2 != nullptr) {
    gfx->setCursor(24, 240);
    gfx->println(line2);
  }
  gfx->flush();
}

// Display startup/loading status while the SD card and photo list are prepared.
// Three centered lines keep the message readable on the rotated 480x480 canvas.
void drawLoadingStatus(const char *title, const char *detail1 = nullptr,
                       const char *detail2 = nullptr) {
  gfx->fillScreen(C_BLACK);
  gfx->setTextColor(C_WHITE);
  gfx->setTextSize(2);

  auto drawCenteredLine = [](const char *text, int y) {
    if (text == nullptr) return;
    const int textWidth = static_cast<int>(strlen(text)) * 12; // built-in 6x8 font at size 2
    const int x = max(8, (LCD_WIDTH - textWidth) / 2);
    gfx->setCursor(x, y);
    gfx->print(text);
  };

  drawCenteredLine(title, 165);
  drawCenteredLine(detail1, 215);
  drawCenteredLine(detail2, 255);
  gfx->flush();
}

void drawOverlayPanel() {
  const int x = 36;
  const int y = 155;
  const int w = 408;
  const int h = 170;
  gfx->fillRoundRect(x, y, w, h, 16, 0x1082);
  gfx->drawRoundRect(x, y, w, h, 16, C_CYAN);
}

void drawBatteryOverlayPanel() {
  const int x = 24;
  const int y = 100;
  const int w = 432;
  const int h = 300;
  gfx->fillRoundRect(x, y, w, h, 16, 0x1082);
  gfx->drawRoundRect(x, y, w, h, 16, C_CYAN);
}

void drawRandomPlaybackButton() {
  const uint16_t fillColor = randomPlayback ? 0x0320 : 0x1082;
  const uint16_t borderColor = randomPlayback ? C_GREEN : C_CYAN;
  gfx->fillRoundRect(RANDOM_BUTTON_X, RANDOM_BUTTON_Y,
                     RANDOM_BUTTON_W, RANDOM_BUTTON_H, 9, fillColor);
  gfx->drawRoundRect(RANDOM_BUTTON_X, RANDOM_BUTTON_Y,
                     RANDOM_BUTTON_W, RANDOM_BUTTON_H, 9, borderColor);

  // Use a larger, short label: the 6x8 built-in font at text size 1 was too
  // small to read comfortably on the 480x480 high-density AMOLED panel.
  const char *label = randomPlayback ? "SHUFFLE: ON" : "SHUFFLE: OFF";
  constexpr int labelScale = 2;
  const int textWidth = static_cast<int>(strlen(label)) * 6 * labelScale;
  gfx->setTextColor(C_WHITE);
  gfx->setTextSize(labelScale);
  gfx->setCursor(RANDOM_BUTTON_X + (RANDOM_BUTTON_W - textWidth) / 2,
                 RANDOM_BUTTON_Y + (RANDOM_BUTTON_H - 8 * labelScale) / 2);
  gfx->print(label);
}

void drawPhotoFitButton() {
  const uint16_t fillColor = fitWholePhoto ? 0x0320 : 0x1082;
  const uint16_t borderColor = fitWholePhoto ? C_GREEN : C_CYAN;
  gfx->fillRoundRect(FIT_BUTTON_X, FIT_BUTTON_Y,
                     FIT_BUTTON_W, FIT_BUTTON_H, 9, fillColor);
  gfx->drawRoundRect(FIT_BUTTON_X, FIT_BUTTON_Y,
                     FIT_BUTTON_W, FIT_BUTTON_H, 9, borderColor);

  // Keep the text short enough to render at double size with generous margins.
  const char *label = fitWholePhoto ? "VIEW: FULL" : "VIEW: CROP";
  constexpr int labelScale = 2;
  const int textWidth = static_cast<int>(strlen(label)) * 6 * labelScale;
  gfx->setTextColor(C_WHITE);
  gfx->setTextSize(labelScale);
  gfx->setCursor(FIT_BUTTON_X + (FIT_BUTTON_W - textWidth) / 2,
                 FIT_BUTTON_Y + (FIT_BUTTON_H - 8 * labelScale) / 2);
  gfx->print(label);
}

void drawIntervalButtons() {
  // Previous interval button.
  gfx->fillRoundRect(INTERVAL_MINUS_X, INTERVAL_BUTTON_Y,
                     INTERVAL_EDGE_BUTTON_W, INTERVAL_BUTTON_H, 9, 0x1082);
  gfx->drawRoundRect(INTERVAL_MINUS_X, INTERVAL_BUTTON_Y,
                     INTERVAL_EDGE_BUTTON_W, INTERVAL_BUTTON_H, 9, C_CYAN);
  gfx->setTextColor(C_WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(INTERVAL_MINUS_X + (INTERVAL_EDGE_BUTTON_W - 18) / 2,
                 INTERVAL_BUTTON_Y + (INTERVAL_BUTTON_H - 24) / 2);
  gfx->print("-");

  // Current interval, shown between the minus and plus buttons.
  gfx->fillRoundRect(INTERVAL_CENTER_X, INTERVAL_BUTTON_Y,
                     INTERVAL_CENTER_W, INTERVAL_BUTTON_H, 9, 0x1082);
  gfx->drawRoundRect(INTERVAL_CENTER_X, INTERVAL_BUTTON_Y,
                     INTERVAL_CENTER_W, INTERVAL_BUTTON_H, 9, C_CYAN);
  String label = String("INTERVAL: ") + SLIDE_INTERVAL_LABELS[slideIntervalIndex];
  const int labelScale = 2;
  const int textWidth = static_cast<int>(label.length()) * 6 * labelScale;
  gfx->setTextSize(labelScale);
  gfx->setCursor(INTERVAL_CENTER_X + (INTERVAL_CENTER_W - textWidth) / 2,
                 INTERVAL_BUTTON_Y + (INTERVAL_BUTTON_H - 8 * labelScale) / 2);
  gfx->print(label);

  // Next interval button.
  gfx->fillRoundRect(INTERVAL_PLUS_X, INTERVAL_BUTTON_Y,
                     INTERVAL_EDGE_BUTTON_W, INTERVAL_BUTTON_H, 9, 0x1082);
  gfx->drawRoundRect(INTERVAL_PLUS_X, INTERVAL_BUTTON_Y,
                     INTERVAL_EDGE_BUTTON_W, INTERVAL_BUTTON_H, 9, C_CYAN);
  gfx->setTextSize(3);
  gfx->setCursor(INTERVAL_PLUS_X + (INTERVAL_EDGE_BUTTON_W - 18) / 2,
                 INTERVAL_BUTTON_Y + (INTERVAL_BUTTON_H - 24) / 2);
  gfx->print("+");
}

void showBatteryOverlay() {
  currentOverlay = OVERLAY_BATTERY;
  overlayUntil = millis() + BATTERY_OVERLAY_MS;

  drawBatteryOverlayPanel();
  gfx->setTextColor(C_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(57, 139);
  gfx->println("BATTERY STATUS");

  if (!pmuReady || pmu == nullptr) {
    gfx->setCursor(57, 196);
    gfx->println("PMU UNAVAILABLE");
  } else if (!pmu->isBatteryConnect()) {
    gfx->setCursor(57, 189);
    gfx->println("NO BATTERY CONNECTED");
    gfx->setCursor(57, 219);
    gfx->println("Check the connector");
  } else {
    int percent = constrain(static_cast<int>(pmu->getBatteryPercent()), 0, 100);
    float volts = static_cast<float>(pmu->getBattVoltage()) / 1000.0f;

    gfx->setTextSize(3);
    gfx->setCursor(57, 174);
    gfx->printf("%d %%", percent);

    gfx->setTextSize(2);
    gfx->setCursor(57, 222);
    gfx->printf("Voltage: %.2f V", volts);
    gfx->setCursor(260, 222);
    gfx->println(pmu->isCharging() ? "CHARGING" : "NOT CHARGING");

    // Battery level bar.
    gfx->drawRoundRect(70, 254, 336, 14, 5, C_WHITE);
    int fillWidth = (330 * percent) / 100;
    if (fillWidth > 0) {
      uint16_t fillColor = percent <= 15 ? C_YELLOW : C_GREEN;
      gfx->fillRoundRect(73, 257, fillWidth, 8, 3, fillColor);
    }
  }

  // The shuffle control is always available, even if the PMU cannot report
  // battery state. Tap it to switch between sorted and random playback.
  drawRandomPlaybackButton();
  drawPhotoFitButton();
  drawIntervalButtons();
  gfx->flush();
}

void showBrightnessOverlay() {
  currentOverlay = OVERLAY_BRIGHTNESS;
  overlayUntil = millis() + BRIGHTNESS_OVERLAY_MS;

  drawOverlayPanel();
  gfx->setTextColor(C_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(70, 188);
  gfx->println("DISPLAY BRIGHTNESS");
  gfx->setTextSize(3);
  gfx->setCursor(70, 225);
  gfx->printf("%u %%", (static_cast<unsigned int>(displayBrightness) * 100U) / 255U);
  gfx->drawRoundRect(70, 278, 336, 18, 5, C_WHITE);
  int fillWidth = (330 * displayBrightness) / 255;
  if (fillWidth > 0) gfx->fillRoundRect(73, 281, fillWidth, 12, 3, C_CYAN);
  gfx->flush();
}

// ---------------------------------------------------------------------------
// SD card and slideshow
// ---------------------------------------------------------------------------
bool isJpegName(String name) {
  name.toLowerCase();
  return name.endsWith(".jpg") || name.endsWith(".jpeg");
}

// Join a directory and a base name without producing double slashes.
String makeChildPath(const String &directoryPath, const String &entryName) {
  if (entryName.startsWith("/")) return entryName;
  String path = directoryPath;
  if (path.isEmpty()) path = "/";
  if (!path.endsWith("/")) path += "/";
  path += entryName;
  return path;
}

void scanPhotos() {
  photoPaths.clear();
  if (!sdReady) return;

  drawLoadingStatus("LOADING PHOTOS", "Scanning SD card...", "Found: 0 photos");
  size_t scannedEntries = 0;
  size_t lastProgressPhotos = 0;
  uint32_t lastProgressAt = millis();

  // Use an explicit directory queue instead of recursive calls. This avoids
  // holding an open File handle for every parent directory and supports
  // deeply nested photo folders without using more task stack.
  std::vector<String> directories;
  directories.push_back("/");
  size_t directoryIndex = 0;

  while (directoryIndex < directories.size() && photoPaths.size() < MAX_PHOTOS) {
    const String directoryPath = directories[directoryIndex++];
    File directory = SD.open(directoryPath.c_str());
    if (!directory) {
      Serial.printf("WARNING: cannot open directory '%s'\n", directoryPath.c_str());
      continue;
    }
    if (!directory.isDirectory()) {
      Serial.printf("WARNING: path is not a directory '%s'\n", directoryPath.c_str());
      directory.close();
      continue;
    }

    while (photoPaths.size() < MAX_PHOTOS) {
      File entry = directory.openNextFile();
      if (!entry) break;

      const bool isDirectory = entry.isDirectory();
      const String entryName = String(entry.name() ? entry.name() : "");
      const String reportedPath = String(entry.path() ? entry.path() : "");
      const uint32_t sizeBytes = entry.size();
      entry.close();
      ++scannedEntries;

      // Refresh the loading screen periodically, not for every file, to avoid
      // slowing down the SD scan with unnecessary full-screen redraws.
      const uint32_t progressNow = millis();
      if ((progressNow - lastProgressAt >= 500) ||
          (photoPaths.size() >= lastProgressPhotos + 50)) {
        char foundLine[48];
        char scannedLine[48];
        snprintf(foundLine, sizeof(foundLine), "Found: %u photos",
                 static_cast<unsigned int>(photoPaths.size()));
        snprintf(scannedLine, sizeof(scannedLine), "Scanned: %u entries",
                 static_cast<unsigned int>(scannedEntries));
        drawLoadingStatus("LOADING PHOTOS", foundLine, scannedLine);
        lastProgressAt = progressNow;
        lastProgressPhotos = photoPaths.size();
      }

      if (entryName.isEmpty() || entryName == "." || entryName == "..") continue;

      // File.path() is absolute on current Arduino-ESP32; if a core version
      // returns a relative/empty path, join the current directory and name.
      String entryPath = reportedPath.startsWith("/")
          ? reportedPath : makeChildPath(directoryPath, entryName);

      if (isDirectory) {
        directories.push_back(entryPath);
        Serial.printf("SD scan DIR: %s\n", entryPath.c_str());
      } else if (isJpegName(entryName)) {
        photoPaths.push_back(entryPath);
        Serial.printf("SD scan JPEG: name='%s', path='%s', listedSize=%lu\n",
                      entryName.c_str(), entryPath.c_str(),
                      static_cast<unsigned long>(sizeBytes));
      }
    }

    directory.close();
  }

  std::sort(photoPaths.begin(), photoPaths.end(), [](const String &a, const String &b) {
    return a.compareTo(b) < 0;
  });

  // Reopen photos after all directory handles are closed. This checks paths
  // without creating extra simultaneous open handles while walking folders.
  drawLoadingStatus("VERIFYING PHOTOS", "Checking files on SD card...", "Please wait");
  size_t verifiedPhotos = 0;
  uint32_t lastVerifyProgressAt = millis();
  for (const String &path : photoPaths) {
    File probe = SD.open(path.c_str(), FILE_READ);
    const bool opened = static_cast<bool>(probe);
    const uint32_t sizeBytes = opened ? probe.size() : 0;
    probe.close();
    Serial.printf("SD verify JPEG: path='%s', exists=%d, open=%d, size=%lu\n",
                  path.c_str(), SD.exists(path.c_str()) ? 1 : 0,
                  opened ? 1 : 0, static_cast<unsigned long>(sizeBytes));
    ++verifiedPhotos;
    const uint32_t verifyNow = millis();
    if ((verifyNow - lastVerifyProgressAt >= 500) || verifiedPhotos == photoPaths.size()) {
      char verifyLine[48];
      snprintf(verifyLine, sizeof(verifyLine), "Verified: %u / %u",
               static_cast<unsigned int>(verifiedPhotos),
               static_cast<unsigned int>(photoPaths.size()));
      drawLoadingStatus("VERIFYING PHOTOS", verifyLine, "Please wait...");
      lastVerifyProgressAt = verifyNow;
    }
  }

  if (photoPaths.size() >= MAX_PHOTOS) {
    Serial.printf("WARNING: photo limit (%u) reached; additional files will be ignored\n",
                  static_cast<unsigned int>(MAX_PHOTOS));
  }
}

bool drawPhoto(const String &path) {
  gfx->fillScreen(C_BLACK);

  // Open the photo from the SD card.
  JPEGDEC &jpeg = jpegDecoder;
  File imageFile = SD.open(path.c_str(), FILE_READ);
  if (!imageFile && path.startsWith("/")) {
    String pathWithoutSlash = path.substring(1);
    imageFile = SD.open(pathWithoutSlash.c_str(), FILE_READ);
  }
  if (!imageFile) {
    Serial.printf("SD.open failed: '%s' (SD.exists=%d)\n",
                  path.c_str(), SD.exists(path.c_str()) ? 1 : 0);
    gfx->setTextColor(C_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(18, 220);
    gfx->println("CANNOT OPEN FILE");
    gfx->flush();
    return false;
  }

  const uint32_t jpegFileSize = imageFile.size();
  if (!jpeg.open(imageFile, jpegDrawCallback)) {
    const int openError = jpeg.getLastError();
    Serial.printf("JPEGDEC could not open '%s', size=%lu, error=%d\n",
                  path.c_str(), static_cast<unsigned long>(jpegFileSize), openError);
    jpeg.close();
    gfx->setTextColor(C_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(18, 220);
    gfx->println("INVALID OR UNSUPPORTED JPG");
    gfx->flush();
    return false;
  }

  const int srcW = jpeg.getWidth();
  const int srcH = jpeg.getHeight();
  // JPEGDEC parses EXIF Orientation from APP1. Zero/invalid values mean normal.
  int exifOrientation = jpeg.getOrientation();
  if (exifOrientation < 1 || exifOrientation > 8) exifOrientation = 1;
  const int jpegType = jpeg.getJPEGType();
  const bool isProgressive = (jpegType == JPEG_MODE_PROGRESSIVE);
  Serial.printf("JPEG: %s, file=%lu B, dimensions=%dx%d, EXIF orientation=%d, thumbnail=%s\n",
                isProgressive ? "PROGRESSIVE" : "BASELINE",
                static_cast<unsigned long>(jpegFileSize), srcW, srcH,
                exifOrientation, jpeg.hasThumb() ? "yes" : "no");
  if (isProgressive) {
    Serial.println("WARNING: JPEGDEC does not fully support progressive JPEG decoding; convert the file to baseline JPEG for best compatibility.");
  }

  // JPEGDEC supports 1/2, 1/4 and 1/8 decode scaling. Use the largest
  // reduction that keeps the short side around the panel resolution when possible.
  int divisor = 1;
  int decodeOptions = 0;
  while (divisor < 8) {
    const int nextDivisor = divisor * 2;
    const int nextW = (srcW + nextDivisor - 1) / nextDivisor;
    const int nextH = (srcH + nextDivisor - 1) / nextDivisor;
    if (nextW >= LCD_WIDTH && nextH >= LCD_HEIGHT) {
      divisor = nextDivisor;
      if (divisor == 2) decodeOptions = JPEG_SCALE_HALF;
      else if (divisor == 4) decodeOptions = JPEG_SCALE_QUARTER;
      else if (divisor == 8) decodeOptions = JPEG_SCALE_EIGHTH;
    } else {
      break;
    }
  }

  const int decodedW = (srcW + divisor - 1) / divisor;
  const int decodedH = (srcH + divisor - 1) / divisor;
  if (decodedW <= 0 || decodedH <= 0) {
    jpeg.close();
    Serial.printf("Invalid dimensions after scaling: %dx%d\n", decodedW, decodedH);
    gfx->flush();
    return false;
  }

  // JPEG pixels are decoded into raw sensor order first. EXIF orientation is
  // applied later while mapping pixels to the oriented image/crop coordinates.
  // This correctly handles portrait photos whose JPEG pixels are landscape.
  const int orientedW = (exifOrientation >= 5 && exifOrientation <= 8) ? decodedH : decodedW;
  const int orientedH = (exifOrientation >= 5 && exifOrientation <= 8) ? decodedW : decodedH;
  const int regionX = fitWholePhoto ? 0 : (orientedW - min(orientedW, orientedH)) / 2;
  const int regionY = fitWholePhoto ? 0 : (orientedH - min(orientedW, orientedH)) / 2;
  const int regionW = fitWholePhoto ? orientedW : min(orientedW, orientedH);
  const int regionH = fitWholePhoto ? orientedH : min(orientedW, orientedH);

  const size_t captureBytes = static_cast<size_t>(decodedW) * decodedH * sizeof(uint16_t);
  const size_t outputBytes = static_cast<size_t>(LCD_WIDTH) * LCD_HEIGHT * sizeof(uint16_t);
  uint16_t *regionBuffer = static_cast<uint16_t *>(
      heap_caps_malloc(captureBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (regionBuffer == nullptr) {
    // Try internal RAM as a fallback on boards/configurations without PSRAM.
    regionBuffer = static_cast<uint16_t *>(heap_caps_malloc(captureBytes, MALLOC_CAP_8BIT));
  }
  uint16_t *outputBuffer = static_cast<uint16_t *>(
      heap_caps_malloc(outputBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (outputBuffer == nullptr) {
    outputBuffer = static_cast<uint16_t *>(heap_caps_malloc(outputBytes, MALLOC_CAP_8BIT));
  }
  if (regionBuffer == nullptr || outputBuffer == nullptr) {
    if (regionBuffer) heap_caps_free(regionBuffer);
    if (outputBuffer) heap_caps_free(outputBuffer);
    jpeg.close();
    Serial.printf("Not enough RAM/PSRAM for image %dx%d (decoded buffer %u B, output %u B)\n",
                  decodedW, decodedH,
                  static_cast<unsigned int>(captureBytes),
                  static_cast<unsigned int>(outputBytes));
    gfx->setTextColor(C_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(18, 220);
    gfx->println("NOT ENOUGH MEMORY FOR JPG");
    gfx->flush();
    return false;
  }
  memset(regionBuffer, 0, captureBytes);

  jpegCaptureBuffer = regionBuffer;
  jpegCaptureRegionX = 0;
  jpegCaptureRegionY = 0;
  jpegCaptureRegionWidth = decodedW;
  jpegCaptureRegionHeight = decodedH;
  jpegCaptureImageWidth = decodedW;
  jpegCaptureImageHeight = decodedH;
  jpegCaptureActive = true;
  jpeg.setPixelType(RGB565_LITTLE_ENDIAN);
  const bool ok = (jpeg.decode(0, 0, decodeOptions) != 0);
  jpegCaptureActive = false;
  const int decodeError = jpeg.getLastError();
  jpeg.close();
  jpegCaptureBuffer = nullptr;
  jpegCaptureRegionWidth = 0;
  jpegCaptureRegionHeight = 0;

  if (!ok) {
    heap_caps_free(regionBuffer);
    heap_caps_free(outputBuffer);
    Serial.printf("JPEG decode error (%d): %s\n", decodeError, path.c_str());
    gfx->fillScreen(C_BLACK);
    gfx->setTextColor(C_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(20, 210);
    gfx->println("UNSUPPORTED JPG");
    gfx->setCursor(20, 245);
    gfx->println(isProgressive ? "Convert to baseline JPEG" : "File may be corrupted");
    gfx->flush();
    return false;
  }

  // Fit mode keeps the complete photo and adds bars if the aspect ratio differs
  // from the square panel. Cover mode fills the panel by cropping a centered square.
  memset(outputBuffer, 0, outputBytes);
  int destW = LCD_WIDTH;
  int destH = LCD_HEIGHT;
  if (fitWholePhoto) {
    if (static_cast<int64_t>(regionW) * LCD_HEIGHT >=
        static_cast<int64_t>(regionH) * LCD_WIDTH) {
      destW = LCD_WIDTH;
      destH = max(1, static_cast<int>((static_cast<int64_t>(regionH) * LCD_WIDTH + regionW / 2) / regionW));
    } else {
      destH = LCD_HEIGHT;
      destW = max(1, static_cast<int>((static_cast<int64_t>(regionW) * LCD_HEIGHT + regionH / 2) / regionH));
    }
  }
  const int destX = (LCD_WIDTH - destW) / 2;
  const int destY = (LCD_HEIGHT - destH) / 2;
  for (int y = 0; y < destH; ++y) {
    int orientedY = regionY + static_cast<int>((static_cast<uint64_t>(2 * y + 1) * regionH) /
                                               (2ULL * destH));
    orientedY = constrain(orientedY, 0, orientedH - 1);
    for (int x = 0; x < destW; ++x) {
      int orientedX = regionX + static_cast<int>((static_cast<uint64_t>(2 * x + 1) * regionW) /
                                                 (2ULL * destW));
      orientedX = constrain(orientedX, 0, orientedW - 1);

      // Convert oriented/display coordinates to raw decoded JPEG coordinates.
      int rawX = orientedX;
      int rawY = orientedY;
      switch (exifOrientation) {
        case 2: rawX = decodedW - 1 - orientedX; break; // mirror horizontal
        case 3: rawX = decodedW - 1 - orientedX; rawY = decodedH - 1 - orientedY; break;
        case 4: rawY = decodedH - 1 - orientedY; break; // mirror vertical
        case 5: rawX = orientedY; rawY = orientedX; break; // transpose
        case 6: rawX = orientedY; rawY = decodedH - 1 - orientedX; break; // 90 CW
        case 7: rawX = decodedW - 1 - orientedY; rawY = decodedH - 1 - orientedX; break; // transverse
        case 8: rawX = decodedW - 1 - orientedY; rawY = orientedX; break; // 270 CW
        default: break; // orientation 1 or invalid EXIF value
      }
      rawX = constrain(rawX, 0, decodedW - 1);
      rawY = constrain(rawY, 0, decodedH - 1);
      outputBuffer[(destY + y) * LCD_WIDTH + destX + x] =
          regionBuffer[static_cast<size_t>(rawY) * decodedW + rawX];
    }
  }
  gfx->draw16bitRGBBitmap(0, 0, outputBuffer, LCD_WIDTH, LCD_HEIGHT);
  gfx->flush();

  Serial.printf("Photo %u/%u: %s (%dx%d -> decoded %dx%d -> EXIF=%d, oriented %dx%d -> region %dx%d -> display %dx%d; mode=%s; %s)\n",
                static_cast<unsigned int>(currentPhoto + 1),
                static_cast<unsigned int>(photoPaths.size()), path.c_str(),
                srcW, srcH, decodedW, decodedH, exifOrientation, orientedW, orientedH,
                regionW, regionH, destW, destH,
                fitWholePhoto ? "full photo, no crop" : "fill with crop",
                isProgressive ? "progressive - limited support" : "baseline");
  heap_caps_free(regionBuffer);
  heap_caps_free(outputBuffer);
  return true;
}

void showCurrentPhoto() {
  currentOverlay = OVERLAY_NONE;
  if (!sdReady) {
    drawCenteredMessage("SD CARD UNAVAILABLE", "Check card and format");
  } else if (photoPaths.empty()) {
    drawCenteredMessage("NO PHOTOS FOUND", "Place JPG/JPEG files on SD card");
  } else {
    if (currentPhoto >= photoPaths.size()) currentPhoto = 0;
    drawPhoto(photoPaths[currentPhoto]);
  }
  lastSlideAt = millis();
}

void prepareShuffleOrder(bool keepCurrentPhotoFirst) {
  shuffleOrder.clear();
  shufflePosition = 0;
  const size_t count = photoPaths.size();
  if (count == 0) return;

  if (keepCurrentPhotoFirst) {
    // Keep the already visible picture at position 0; random playback begins
    // with a different picture at the next 10-second slide transition.
    shuffleOrder.push_back(currentPhoto);
    for (size_t i = 0; i < count; ++i) {
      if (i != currentPhoto) shuffleOrder.push_back(i);
    }
    if (shuffleOrder.size() > 2) {
      for (size_t i = shuffleOrder.size() - 1; i > 1; --i) {
        const size_t j = 1 + static_cast<size_t>(random(static_cast<long>(i)));
        std::swap(shuffleOrder[i], shuffleOrder[j]);
      }
    }
    return;
  }

  for (size_t i = 0; i < count; ++i) shuffleOrder.push_back(i);
  for (size_t i = shuffleOrder.size(); i > 1; --i) {
    const size_t j = static_cast<size_t>(random(static_cast<long>(i)));
    std::swap(shuffleOrder[i - 1], shuffleOrder[j]);
  }

  // At the end of a shuffled cycle, avoid showing the same photo twice in a row.
  if (shuffleOrder.size() > 1 && shuffleOrder[0] == currentPhoto) {
    std::swap(shuffleOrder[0], shuffleOrder[1]);
  }
}

void toggleRandomPlayback() {
  randomPlayback = !randomPlayback;
  if (randomPlayback) {
    prepareShuffleOrder(true);
    Serial.println("Shuffle playback: ON (no repeats within a cycle)");
  } else {
    shuffleOrder.clear();
    shufflePosition = 0;
    Serial.println("Shuffle playback: OFF (sorted order)");
  }
  saveSettings();
  showBatteryOverlay();
}

void togglePhotoFitMode() {
  fitWholePhoto = !fitWholePhoto;
  Serial.println(fitWholePhoto
      ? "Display mode: FULL photo, no crop (may show bars)"
      : "Display mode: FILL screen (center crop)");
  saveSettings();
  // Keep the battery controls open so both settings can be changed together.
  showBatteryOverlay();
}

void changeSlideInterval(int delta) {
  int nextIndex = static_cast<int>(slideIntervalIndex) + delta;
  nextIndex = constrain(nextIndex, 0, static_cast<int>(SLIDE_INTERVAL_COUNT) - 1);
  if (static_cast<size_t>(nextIndex) != slideIntervalIndex) {
    slideIntervalIndex = static_cast<size_t>(nextIndex);
    // Start a fresh interval after the setting is changed. Closing the overlay
    // later redraws the visible photo and resets this timestamp as before.
    lastSlideAt = millis();
    Serial.printf("Slideshow interval: %s (%lu ms)\n",
                  SLIDE_INTERVAL_LABELS[slideIntervalIndex],
                  static_cast<unsigned long>(SLIDE_INTERVALS_MS[slideIntervalIndex]));
    saveSettings();
  }
  showBatteryOverlay();
}

void advancePhoto() {
  if (photoPaths.empty()) return;

  if (!randomPlayback) {
    currentPhoto = (currentPhoto + 1) % photoPaths.size();
  } else {
    if (shuffleOrder.size() != photoPaths.size()) {
      prepareShuffleOrder(true);
    }
    if (shuffleOrder.empty()) return;

    if (shufflePosition + 1 < shuffleOrder.size()) {
      ++shufflePosition;
    } else {
      // All photos were shown once. Start another shuffled cycle, avoiding an
      // immediate repeat of the picture currently on screen.
      prepareShuffleOrder(false);
    }
    currentPhoto = shuffleOrder[shufflePosition];
  }

  showCurrentPhoto();
}

// ---------------------------------------------------------------------------
// Persistent settings (ESP32-S3 NVS)
// ---------------------------------------------------------------------------
void loadSettings() {
  settingsStoreReady = settingsStore.begin("photoframe", false);
  if (!settingsStoreReady) {
    Serial.println("WARNING: cannot open NVS; using default settings");
    return;
  }

  const int loadedBrightness = settingsStore.getUChar("brightness", START_BRIGHTNESS);
  displayBrightness = static_cast<uint8_t>(constrain(loadedBrightness,
                                                       static_cast<int>(MIN_BRIGHTNESS), 255));
  const uint8_t loadedInterval = settingsStore.getUChar("interval", 1);
  slideIntervalIndex = (loadedInterval < SLIDE_INTERVAL_COUNT) ? loadedInterval : 1;
  randomPlayback = settingsStore.getBool("random", false);
  fitWholePhoto = settingsStore.getBool("fit", false);

  Serial.printf("Settings loaded: brightness=%u/255, interval=%s, shuffle=%s, display=%s\n",
                displayBrightness, SLIDE_INTERVAL_LABELS[slideIntervalIndex],
                randomPlayback ? "on" : "off", fitWholePhoto ? "full photo" : "fill/crop");
}

void saveSettings() {
  if (!settingsStoreReady) return;
  settingsStore.putUChar("brightness", displayBrightness);
  settingsStore.putUChar("interval", static_cast<uint8_t>(slideIntervalIndex));
  settingsStore.putBool("random", randomPlayback);
  settingsStore.putBool("fit", fitWholePhoto);
}

// ---------------------------------------------------------------------------
// Touch and brightness buttons
// ---------------------------------------------------------------------------
void IRAM_ATTR onTouchInterrupt() {
  touchPending = true;
}

void applyBrightnessChange(int delta) {
  int newBrightness = static_cast<int>(displayBrightness) + delta;
  newBrightness = constrain(newBrightness, MIN_BRIGHTNESS, 255);
  if (newBrightness == displayBrightness) return;
  displayBrightness = static_cast<uint8_t>(newBrightness);
  displayDriver->setBrightness(displayBrightness);
  saveSettings();
  showBrightnessOverlay();
  Serial.printf("Display brightness: %u/255\n", displayBrightness);
}

void handleButton(int pin, bool &lastRead, bool &stableRead,
                  uint32_t &lastChangeAt, bool brightnessUp) {
  const uint32_t now = millis();
  const bool reading = digitalRead(pin);

  if (reading != lastRead) {
    lastRead = reading;
    lastChangeAt = now;
  }

  if ((now - lastChangeAt) >= BUTTON_DEBOUNCE_MS && reading != stableRead) {
    stableRead = reading;
    if (stableRead == LOW) {
      applyBrightnessChange(brightnessUp ? BRIGHTNESS_STEP : -BRIGHTNESS_STEP);
    }
  }
}

void pollPmuPowerKey() {
  if (!pmuReady || pmu == nullptr) return;
  const uint32_t now = millis();
  if (now - lastPmuPollAt < 25) return;
  lastPmuPollAt = now;

  // PWR is wired to the AXP2101 power-key input, not to an ESP32 GPIO.
  // Poll the PMU latched short-press flag instead of using the BOOT/ GPIO0 key.
  pmu->getIrqStatus();
  if (pmu->isPekeyShortPressIrq()) {
    applyBrightnessChange(-static_cast<int>(BRIGHTNESS_STEP));
  }
  pmu->clearIrqStatus();
}

void handleTouch() {
  bool pending;
  noInterrupts();
  pending = touchPending;
  touchPending = false;
  interrupts();
  if (!pending) return;

  uint8_t count = touch.getPoint(touchX, touchY, touch.getSupportTouchPoint());
  const uint32_t now = millis();
  if (count > 0 && (now - lastTouchActionAt) >= TOUCH_DEBOUNCE_MS) {
    lastTouchActionAt = now;
    // TouchDrv coordinates are still in the panel's original orientation.
    // Convert physical coordinates back into the 90-degree-clockwise canvas
    // coordinate system: inverse of (x,y)->(W-1-y,x).
    const int x = touchY[0];
    const int y = LCD_HEIGHT - 1 - touchX[0];
    const bool hitRandomButton =
        currentOverlay == OVERLAY_BATTERY &&
        x >= RANDOM_BUTTON_X && x < RANDOM_BUTTON_X + RANDOM_BUTTON_W &&
        y >= RANDOM_BUTTON_Y && y < RANDOM_BUTTON_Y + RANDOM_BUTTON_H;
    const bool hitFitButton =
        currentOverlay == OVERLAY_BATTERY &&
        x >= FIT_BUTTON_X && x < FIT_BUTTON_X + FIT_BUTTON_W &&
        y >= FIT_BUTTON_Y && y < FIT_BUTTON_Y + FIT_BUTTON_H;
    const bool hitIntervalMinus =
        currentOverlay == OVERLAY_BATTERY &&
        x >= INTERVAL_MINUS_X && x < INTERVAL_MINUS_X + INTERVAL_EDGE_BUTTON_W &&
        y >= INTERVAL_BUTTON_Y && y < INTERVAL_BUTTON_Y + INTERVAL_BUTTON_H;
    const bool hitIntervalPlus =
        currentOverlay == OVERLAY_BATTERY &&
        x >= INTERVAL_PLUS_X && x < INTERVAL_PLUS_X + INTERVAL_EDGE_BUTTON_W &&
        y >= INTERVAL_BUTTON_Y && y < INTERVAL_BUTTON_Y + INTERVAL_BUTTON_H;

    if (hitRandomButton) {
      toggleRandomPlayback();
    } else if (hitFitButton) {
      togglePhotoFitMode();
    } else if (hitIntervalMinus) {
      changeSlideInterval(-1);
    } else if (hitIntervalPlus) {
      changeSlideInterval(1);
    } else {
      // Any ordinary tap reveals the battery panel. When it is already open,
      // tapping outside the random button simply keeps it visible a little longer.
      showBatteryOverlay();
    }
  }
}

// ---------------------------------------------------------------------------
// Setup and main loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.println("ESP32-S3-Touch-AMOLED-2.16 PhotoFrame start");
  randomSeed(static_cast<unsigned long>(esp_random()));
  Serial.printf("Loop task stack size: %u bytes\n",
                static_cast<unsigned int>(getArduinoLoopTaskStackSize()));
  loadSettings();

  pinMode(BUTTON_BRIGHT_UP, INPUT_PULLUP);
  brightnessUpLastRead = brightnessUpStableRead = digitalRead(BUTTON_BRIGHT_UP);

  Wire.begin(IIC_SDA, IIC_SCL);

  // Initialize the AMOLED driver and rotated PSRAM canvas.
  if (!gfx->begin()) {
    Serial.println("ERROR: display/PSRAM canvas initialization failed");
    while (true) delay(1000);
  }
  displayBus->writeC8D8(0x36, 0xA0); // keep the board's proven CO5300 native orientation
  gfx->fillScreen(C_BLACK);
  gfx->flush();
  displayDriver->setBrightness(displayBrightness);
  drawLoadingStatus("STARTING APPLICATION", "Preparing photo frame...");
  Serial.println("Display orientation: 90 degrees clockwise (software rotation in PSRAM)");

  // Initialize CST9220 touch panel
  touch.setPins(TP_RESET, TP_INT);
  if (touch.begin(Wire, CST92XX_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    touch.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
    touch.setSwapXY(true);
    touch.setMirrorXY(true, false);
    pinMode(TP_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TP_INT), onTouchInterrupt, FALLING);
    Serial.println("CST9220 touch initialized");
  } else {
    Serial.println("WARNING: touch initialization failed");
  }

  // Initialize the AXP2101 power-management IC (battery status)
  pmu = new XPowersAXP2101(Wire, IIC_SDA, IIC_SCL);
  if (pmu != nullptr && pmu->init()) {
    pmuReady = true;
    // This board does not use battery-temperature sensing on the TS pin.
    pmu->disableTSPinMeasure();
    pmu->enableBattDetection();
    pmu->enableBattVoltageMeasure();
    pmu->enableVbusVoltageMeasure();
    pmu->enableSystemVoltageMeasure();
    pmu->enableIRQ(XPOWERS_AXP2101_PKEY_SHORT_IRQ);
    pmu->clearIrqStatus();
    Serial.println("AXP2101 power management initialized");
  } else {
    pmuReady = false;
    Serial.println("WARNING: AXP2101 unavailable; battery status will not be available");
  }

  // Show explicit feedback before mounting and scanning the card.
  drawLoadingStatus("MOUNTING SD CARD", "Please wait...");

  // Mount microSD on HSPI/SPI3, separate from the display QSPI host (FSPI/SPI2).
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdReady = SD.begin(SD_CS, sdSPI, 20000000);
  if (sdReady) {
    Serial.println("SD card initialized");
    scanPhotos();
    Serial.printf("Photos found: %u\n", static_cast<unsigned int>(photoPaths.size()));
    if (!photoPaths.empty()) {
      char countLine[48];
      snprintf(countLine, sizeof(countLine), "Found: %u photos",
               static_cast<unsigned int>(photoPaths.size()));
      drawLoadingStatus("LOADING COMPLETE", countLine, "Starting slideshow...");
      delay(300); // briefly let the user see that indexing finished before photo 1
    }
  } else {
    Serial.println("WARNING: failed to initialize SD card");
  }

  showCurrentPhoto();
}

void loop() {
  handleTouch();
  handleButton(BUTTON_BRIGHT_UP, brightnessUpLastRead, brightnessUpStableRead,
               brightnessUpLastChangeAt, true);
  pollPmuPowerKey();

  const uint32_t now = millis();
  if (currentOverlay != OVERLAY_NONE && static_cast<int32_t>(now - overlayUntil) >= 0) {
    currentOverlay = OVERLAY_NONE;
    showCurrentPhoto();
  } else if (currentOverlay == OVERLAY_NONE && !photoPaths.empty() &&
             (now - lastSlideAt) >= SLIDE_INTERVALS_MS[slideIntervalIndex]) {
    advancePhoto();
  }

  delay(2);
}
