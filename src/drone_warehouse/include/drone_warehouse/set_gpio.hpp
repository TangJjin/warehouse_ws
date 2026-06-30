#include <gpiod.h>
#include <stdexcept>
#include <string>

class GpioOutput {
public:
    GpioOutput(const std::string& chipName, unsigned int lineOffset, const std::string& consumer)
        : chip_(nullptr), line_(nullptr) {
        chip_ = gpiod_chip_open_by_name(chipName.c_str());
        if (!chip_) {
            throw std::runtime_error("Failed to open gpio chip: " + chipName);
        }

        line_ = gpiod_chip_get_line(chip_, lineOffset);
        if (!line_) {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
            throw std::runtime_error("Failed to get gpio line: " + std::to_string(lineOffset));
        }

        int ret = gpiod_line_request_output(line_, consumer.c_str(), 0);
        if (ret < 0) {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
            line_ = nullptr;
            throw std::runtime_error("Failed to request line as output");
        }
    }

    ~GpioOutput() {
        if (line_) {
            gpiod_line_release(line_);
        }
        if (chip_) {
            gpiod_chip_close(chip_);
        }
    }

    void setHigh() {
        write(1);
    }

    void setLow() {
        write(0);
    }

    void write(int value) {
        if (!line_) {
            throw std::runtime_error("GPIO line is not initialized");
        }
        int ret = gpiod_line_set_value(line_, value);
        if (ret < 0) {
            throw std::runtime_error("Failed to set gpio value");
        }
    }

private:
    struct gpiod_chip* chip_;
    struct gpiod_line* line_;
};