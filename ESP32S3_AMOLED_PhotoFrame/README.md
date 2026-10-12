# PhotoFrame for Waveshare ESP32-S3-Touch-AMOLED-2.16

An Arduino project that plays JPG/JPEG photos from a microSD card, advances images at a configurable interval, shows battery status when the screen is tapped, and uses the board controls to adjust display brightness.

## Features

- A startup screen appears as soon as the display is initialized. While the SD card is mounted, folders are scanned, and discovered JPEG files are checked, the screen shows loading status and progress. When indexing is complete, the number of photos found is briefly shown before the first image.
- `.jpg` and `.jpeg` images are searched recursively in the SD card root directory and all subfolders. The current limit is 500 photos in total.
- The entire display content, including photos and controls, is rotated 90 degrees clockwise. Rotation is implemented in software using a PSRAM canvas because the CO5300 controller does not support a true 90-degree rotation.
- Photo orientation is corrected automatically using EXIF metadata, including 90°, 180°, and 270° rotations and mirroring. After EXIF correction, the whole display output is rotated 90° clockwise.
- Configurable slideshow intervals: 5, 10, 15, 20, or 30 seconds, or 1, 2, 5, or 10 minutes. The default is 10 seconds.
- Tapping the screen displays battery percentage, voltage, charging status, and controls. The panel closes after 8 seconds if you do not interact with it.
- In the battery panel, the **SHUFFLE: OFF/ON** button switches between sorted and random playback. Shuffle mode plays every discovered photo once per cycle, then reshuffles the order for the next cycle.
- The `+/KEY` button (GPIO18) increases brightness. A short press of the `PWR` button decreases brightness through an AXP2101 event.
- Brightness changes in 20-step increments across an approximate 3–100% range.
- When the information overlay closes, the current photo is redrawn and the selected slideshow interval starts again.
- The `−` and `+` buttons in the battery panel decrease or increase the slideshow interval by one step. The value stops at the ends of the list; the currently selected value is shown between the buttons.
- Settings are saved to non-volatile memory so that brightness, interval, shuffle mode, and display mode persist after power is removed.

## Required libraries

Install these libraries using Arduino IDE → Library Manager:

1. **GFX Library for Arduino** (Waveshare recommends the 1.6.x series; for example, 1.6.4).
2. **SensorLib** (version 0.4.1 or newer; provides `TouchDrv.hpp` and the CST9220 driver).
3. **XPowersLib** (0.2.x series; provides `XPowersAXP2101`).
4. **JPEGDEC** by Larry Bank / bitbank2.

Use the **esp32 by Espressif Systems** board package in the 3.3.x series. The project includes `Arduino_GFX_Library.h`, `TouchDrv.hpp`, `XPowersLib.h`, and `JPEGDEC.h`.

## Uploading to the board

1. Format the microSD card as **FAT32**.
2. Copy `.jpg` / `.jpeg` photos to the card's root directory or any subfolder. The program searches folders automatically; the total limit is 500 photos.
3. Open `ESP32S3_AMOLED_PhotoFrame.ino` in Arduino IDE.
4. Select **ESP32S3 Dev Module** and the correct port.
5. Enable **USB CDC On Boot** in the board settings. If available, select the PSRAM mode appropriate for the board (usually OPI PSRAM).
6. Compile and upload the project.

## Recommended photo format

For the broadest compatibility, use regular **baseline JPEG** files. JPEGDEC can decode at 1/2, 1/4, or 1/8 scale. The application first assembles the complete reduced-resolution decoded image in PSRAM, applies EXIF orientation, and then converts it to the 480×480 display. In **CROP** mode, a centered square is used and the edges are intentionally cropped. In **FULL** mode, the entire image is kept and its aspect ratio is preserved. The complete image is drawn in one operation, which helps prevent horizontal seams between JPEG blocks. For example, a 5184×3888 image is decoded at approximately 648×486 before orientation and fitting/cropping are applied. A compressed file size of 7.5 MB is not by itself a problem; it mainly affects read and decode time. **Progressive JPEG support in JPEGDEC is limited**, so export as baseline JPEG for best compatibility.

## Photo orientation correction (EXIF)

Some phones store JPEG pixels in one orientation and add an EXIF instruction describing how the image should be rotated. Computers usually respect this instruction, but a simple decoder may not apply it automatically. The application reads the Orientation value using `JPEGDEC::getOrientation()` and handles all eight standard values, including rotation and mirroring. In Serial Monitor, the value is shown as `EXIF orientation=1..8`; value 1 means no rotation or mirroring is required.

## Buttons

- **+/KEY** — increase brightness (GPIO18).
- **PWR** — a short press decreases brightness; the application reads the short-press event from AXP2101.
- **BOOT/-** — not used by the application; it remains reserved for bootloader/programming use. Holding BOOT while powering on or resetting can put the ESP32-S3 into download mode.

## Pin mapping used by the code

| Function | GPIO |
|---|---:|
| AMOLED QSPI | 4, 5, 6, 7, 38, 12, reset 39 |
| CST9220 touch | SDA 15, SCL 14, INT 11, reset 40 |
| I2C / AXP2101 | SDA 15, SCL 14 |
| microSD SPI | MOSI 1, SCK 2, MISO 3, CS 41 |
| Brightness +/KEY | 18 |
| Brightness down / short PWR press | AXP2101 event |
| BOOT (reserved) | 0 |

## Fix for `Stack canary watchpoint triggered (loopTask)`

`SET_LOOP_TASK_STACK_SIZE(16 * 1024);` is included near the beginning of the sketch. Arduino-ESP32 normally allocates an 8 KB stack to `loopTask`; JPEG decoding and image rendering can require more. The macro is supported by Arduino-ESP32 3.x. The crash occurred after SD initialization but before the first photo was confirmed as rendered, so a stack overflow during decoding/rendering was considered a likely cause. This was not conclusively proven from the crash dump alone; that would require decoding the backtrace against the exact `.elf` file.

## Display driver and software rotation

The physical controller is accessed through `Arduino_CO5300 *displayDriver`; brightness is set with `displayDriver->setBrightness()`. Rendering is performed through `PsramCanvas *gfx`, which stores the whole frame in PSRAM and sends the already rotated image to the panel when `flush()` is called. This is necessary because the CO5300 driver in the selected library does not support a true 90-degree hardware rotation. The canvas uses approximately 450 KiB of PSRAM. Touch coordinates are transformed so that the controls remain usable after rotation.

## Notes

- The project reads battery status from the actual AXP2101 power-management IC. The reported percentage depends on the PMU's battery estimate; according to the manufacturer, it may be inaccurate initially until the PMU learns the cell's discharge curve.
- The code is intended for the Waveshare ESP32-S3-Touch-AMOLED-2.16 board. On the physical board, verify touch orientation, the exact library versions, and behavior with your SD card.

## Links

- Official board documentation: https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-2.16
- Official touch guide: https://docs.waveshare.com/ESP32-Peripheral-Tutorials/Display/Touch
- Manufacturer's project: https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-2.16
- JPEGDEC: https://github.com/bitbank2/JPEGDEC

## Fix for the compile error `ButtonState has not been declared`

Some Arduino IDE versions generate function prototypes before declarations of custom structures. To avoid this issue, button state was changed from a custom `ButtonState` structure to ordinary variables, and `handleButton()` now accepts references to primitive types. Button behavior is unchanged.

## Fix for repeated crashes when displaying the first photo

The Serial Monitor continued to show `Stack canary watchpoint triggered (loopTask)` even after the stack was set to 16 KB. The crash registers included a length of `0x45DC` (17,884 bytes) while a stack region was being initialized, matching the size of a large local `JPEGDEC` object in `drawPhoto()`. The decoder is now a global object named `jpegDecoder` and is reused rather than being created on the stack. At startup, the configured `loopTask` stack size is also printed. This change should be verified on the device; a final compilation and physical-device test have not been performed here.

### JPEG open diagnostics

The current sketch opens images using the native `JPEGDEC::open(File&, drawCallback)` overload. If `SD.open()` fails, Serial Monitor reports `SD.open failed` and the result of `SD.exists`. If the file opens but JPEGDEC cannot recognize it, the file size and error code are printed. This distinguishes a path/SD access problem from an unsupported or corrupted JPEG.

## Diagnosing `SD.exists=0` after a photo was found

The scanner stores file paths and recursively searches the root directory and all subfolders. It logs `SD scan DIR: ...` and `SD scan JPEG: ...` while scanning. After closing the directory handles, it checks each stored path again and logs `SD verify JPEG: path='...', exists=..., open=..., size=...`. If `exists=0` or `open=0`, the issue is with path access or the file system before JPEG decoding begins.

## Separate SPI hosts

The CO5300 display uses QSPI on the FSPI/SPI2 host. The SD card is therefore connected to the separate HSPI/SPI3 host in this version (`SPIClass sdSPI(HSPI)`), using GPIO 1/2/3 and CS on GPIO 41. If the SD card worked during startup scanning but `SD.open()` failed after rendering to the display, a shared SPI host was considered the likely cause.

## Fix for horizontal black lines and large photos

An earlier version resized and drew each JPEG block separately, which could produce thin horizontal black lines at block boundaries. The current version assembles all decoded blocks into one complete reduced-resolution image in PSRAM. After decoding, it applies EXIF orientation and the selected crop/full-image mode, then draws the finished bitmap in one operation. For example, a 5184×3888 image is decoded to approximately 648×486. Correct rendering requires a somewhat larger working buffer than storing only a centered square.

When an image is opened, the sketch prints the JPEG type (`BASELINE` or `PROGRESSIVE`) and the file size. A 7.5 MB compressed file is not itself a critical limit; JPEG format and available PSRAM matter more. JPEGDEC has more complete support for baseline JPEG, while progressive JPEG support is limited. If Serial Monitor reports `PROGRESSIVE`, convert the image to **baseline JPEG** using the standard/progressive option in your image export software.

## Shuffle playback and photo display mode

The panel opened by tapping the screen includes larger, more readable buttons:

- **SHUFFLE: OFF / ON** switches between sorted and random playback. In shuffle mode, photos are not repeated until the whole cycle has been played; the order is then shuffled again.
- **VIEW: CROP / FULL** switches the display mode. `CROP` is the original behavior: a centered square fills the screen, but image edges may be cropped. `FULL` shows the whole image without cropping and preserves its aspect ratio. Because the display is square, black areas may appear at the top/bottom or on the sides. This is expected and is not a decoding error.

After you tap one of the buttons, the panel remains open so you can change shuffle mode, display mode, and interval. When the panel closes automatically, the current photo is rendered using the selected mode.

## Slideshow interval settings

The bottom row of the battery panel contains `−`, the current interval, and `+`. Each press of `−` selects a shorter interval; `+` selects a longer one. Available values are 5 s, 10 s, 15 s, 20 s, 30 s, 1 min, 2 min, 5 min, and 10 min. The default on first startup is 10 seconds. At 5 seconds, `−` cannot decrease the interval further; at 10 minutes, `+` cannot increase it further. The selected interval is also printed in Serial Monitor after a change.

## Preserving settings after power-off

The application stores the display brightness, shuffle/sorted playback mode, display mode (`FILL`/`FULL`), and slideshow interval in the ESP32-S3's non-volatile storage (NVS via the `Preferences` library). Each change is saved immediately, so settings persist after power is disconnected. On the first startup, or if no settings are stored in NVS, the defaults are used: brightness 160/255, sorted playback, `FILL` display mode, and a 10-second interval. The loaded settings are printed in Serial Monitor during startup. The current photo position is not restored after restart; the slideshow starts again from the beginning of the list according to the saved playback mode.

## Rotating the entire display 90° clockwise

The current version renders all content (photos, text, battery panel, and buttons) rotated 90° clockwise. Rotation is performed in software rather than by calling `setRotation()` on the CO5300 controller: the CO5300 driver in Arduino_GFX supports axis flipping but not a full 90-degree rotation. The project therefore uses `PsramCanvas`, whose 480×480 RGB565 framebuffer is explicitly allocated in PSRAM (approximately 450 KiB). Touch coordinates are transformed using the inverse rotation so that the battery panel and its buttons remain usable. At startup, Serial Monitor prints `Display orientation: 90 degrees clockwise`.
