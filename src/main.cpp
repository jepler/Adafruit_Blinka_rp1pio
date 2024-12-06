#include <iostream>
#include <pybind11/pybind11.h>

#include "piolib.h"
#include "utils/piolib/examples/ws2812.pio.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

static PIO pio{};
static int sm{-1};
static int offset{-1};
static int last_gpio{-1};

static void neopixel_write(py::object gpio_obj, py::buffer buf) {
    int gpio = py::getattr(gpio_obj, "_pin", gpio_obj).attr("id").cast<int>();
    py::buffer_info info = buf.request();

    if (!pio || sm < 0) {
        // (is safe to call twice)
        if (pio_init()) {
            throw std::runtime_error("pio_init() failed");
        }

        // can't use `pio0` macro as it will call exit() on failure!
        pio = pio_open(0);
        if(PIO_IS_ERR(pio)) {
            throw std::runtime_error(
                py::str("Failed to open PIO device (error {})").attr("format")(PIO_ERR_VAL(pio)).cast<std::string>());
        }

        sm = pio_claim_unused_sm(pio, true);

        offset = pio_add_program(pio, &ws2812_program);

        pio_sm_clear_fifos(pio, sm);
        pio_sm_set_clkdiv(pio, sm, 1.0);

        printf("Loaded program at %d, using sm %d\n", offset, sm, gpio);
        last_gpio = -1;
    }

    if (gpio != last_gpio) {
        ws2812_program_init(pio, sm, offset, gpio, 800000.0, false);
        printf("Initialized program at %d, using sm %d, gpio %d\n", offset, sm, gpio);
        last_gpio = gpio;
    }

    size_t size = info.size * info.itemsize;

    if(size > UINT16_MAX) {
        throw py::value_error("Too much data");
    }

    if (pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, size, 1)) {
        throw std::runtime_error("pio_sm_config_xfer() failed");
    }
    if (pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, size, info.ptr)) {
        throw std::runtime_error("pio_sm_xfer_data() failed");
    }

}

static void free_pio(void) {
    if (!pio) { return; }
    if (offset <= 0) { pio_remove_program(pio, &ws2812_program, offset); };
    offset = -1;
    if (sm >= 0) { pio_sm_unclaim(pio, sm); }
    sm = -1;
    pio_close(pio);
    pio = nullptr;
}

PYBIND11_MODULE(neopixel_write_pi5, m) {
    m.doc() = R"pbdoc(
        neopixel_write for pi5
        -----------------------

        .. currentmodule:: neopixel_write_pi5

        .. autosummary::
           :toctree: _generate

           neopixel_write
    )pbdoc";

    m.def("neopixel_write", &neopixel_write,
            py::arg("gpio"),
            py::arg("buf"),
            R"pbdoc(NeoPixel writing function)pbdoc");

    m.def("_free_pio", &free_pio,
            R"pbdoc(Release any held PIO resource)pbdoc");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
