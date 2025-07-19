#include <Arduino.h>
#include <M5Unified.h>
#include <esp_camera.h>

namespace Camera {
  static camera_config_t camera_config = {
      .pin_pwdn     = -1,
      .pin_reset    = -1,
      .pin_xclk     = 2,
      .pin_sscb_sda = 12,
      .pin_sscb_scl = 11,

      .pin_d7 = 47,
      .pin_d6 = 48,
      .pin_d5 = 16,
      .pin_d4 = 15,
      .pin_d3 = 42,
      .pin_d2 = 41,
      .pin_d1 = 40,
      .pin_d0 = 39,

      .pin_vsync = 46,
      .pin_href  = 38,
      .pin_pclk  = 45,

      .xclk_freq_hz = 20000000,
      .ledc_timer   = LEDC_TIMER_0,
      .ledc_channel = LEDC_CHANNEL_0,

      .pixel_format = PIXFORMAT_RGB565,
      // .pixel_format = PIXFORMAT_JPEG,
      .frame_size = FRAMESIZE_QVGA,
      // .jpeg_quality = 4,
      .jpeg_quality = 0,
      .fb_count     = 2,
      .fb_location  = CAMERA_FB_IN_PSRAM,
      .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
  };

  class GC0308 {
  private:
    size_t width_   = 320;
    size_t height_  = 240;
    bool visualize_ = true;
    camera_fb_t* fb_;
    camera_fb_t jpg_fb_;
    bool get_framebuffer_ = false;
    // inline static camera_config_t camera_config = {
    //     .pin_pwdn = 32, // GPIO32
    //     .pin_reset = -1,
    //     .pin_xclk = 0,      // GPIO0
    //     .pin_sscb_sda = 26, // GPIO26
    //     .pin_sscb_scl = 27, // GPIO27
    //     .pin_d7 = 35,       // GPIO35
    //     .pin_d6 = 34,       // GPIO34
    //     .pin_d5 = 39,       // GPIO39
    //     .pin_d4 = 36,       // GPIO36
    //     .pin_d3 = 21,       // GPIO21
    //     .pin_d2 = 19,       // GPIO19
    //     .pin_d1 = 18,       // GPIO18
    //     .pin_d0 = 5,        // GPIO5
    //     .pin_vsync = 25,    // GPIO25
    //     .pin_href = 23,     // GPIO23
    //     .pin_pclk = 22,     // GPIO22

    //     .xclk_freq_hz = 20000000,
    //     .ledc_timer = LEDC_TIMER_0,
    //     .ledc_channel = LEDC_CHANNEL_0,
    //     .pixel_format = PIXFORMAT_JPEG,
    //     .frame_size = FRAMESIZE_QVGA,
    //     .jpeg_quality = 12,
    //     .fb_count = 2,
    // };

  public:
    GC0308() {}

    esp_err_t begin() {
      // initialize the camera
      M5.In_I2C.release();
      esp_err_t err = esp_camera_init(&camera_config);
      // M5.In_I2C.begin();
      if (err != ESP_OK) {
        Serial.println("Camera Init Failed");
        M5.Display.println("Camera Init Failed");
        return err;
      }
      return ESP_OK;
    }
    esp_err_t capture() {
      // acquire a frame
      M5.In_I2C.release();
      fb_ = esp_camera_fb_get();
      // M5.In_I2C.begin();
      if (!fb_) {
        Serial.println("Camera Capture Failed");
        M5.Display.println("Camera Capture Failed");
        return ESP_FAIL;
      }
      get_framebuffer_ = true;
      return ESP_OK;
    }
    size_t width() const { return width_; }
    size_t height() const { return height_; }
    void draw() {
      if (visualize_) {
        M5.Display.startWrite();
        M5.Display.setAddrWindow(0, 0, width_, height_);
        M5.Display.writePixels((uint16_t*)fb_->buf, int(fb_->len / 2));
        M5.Display.endWrite();
      }
    }
    void draw_jpg() {
      if (visualize_) {
        M5.Display.drawJpg(jpg_fb_.buf, jpg_fb_.len, 0, 0, width_, height_);
      }
    }
    camera_fb_t* getFrameBuffer() { return fb_; }
    bool calcJpgFrameBuffer(uint8_t quality) {
      if (get_framebuffer_) {
        jpg_fb_.width  = width_;
        jpg_fb_.height = height_;
        jpg_fb_.format = PIXFORMAT_JPEG;
        return fmt2jpg(fb_->buf, fb_->len, width_, height_, fb_->format, quality, &jpg_fb_.buf, &jpg_fb_.len);
      }
      return false;
    }
    camera_fb_t* getJpgFrameBuffer() { return &jpg_fb_; }
    void returnFrameBuffer() {
      if (get_framebuffer_) {
        esp_camera_fb_return(fb_);
        get_framebuffer_ = false;
      }
    }
  };
} // namespace Camera
