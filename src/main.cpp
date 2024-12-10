#include <iostream>
#include <pybind11/pybind11.h>

#include "piolib.h"
#include "utils/piolib/examples/ws2812.pio.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

namespace {

PIO pio_open_check() {
    if (pio_init()) {
        throw std::runtime_error("PIO not available");
    }

    PIO pio = pio_open(0);
    if(PIO_IS_ERR(pio)) {
        throw std::runtime_error(
            py::str("Failed to open PIO device (error {})").attr("format")(PIO_ERR_VAL(pio)).cast<std::string>());
    }
    return pio;
}

int pio_sm_claim_unused_sm_check(PIO pio) {
    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        throw std::runtime_error("No state machine available");
    }
    return sm;
}

int pio_add_program_check(PIO pio, const struct pio_program *program) {
    int offset = pio_add_program(pio, program);
    if (offset < 0) {
        throw std::runtime_error("Could not load program");
    }
    return offset;
}

int get_pin_number(py::object gpio_obj) {
    return py::getattr(gpio_obj, "_pin", gpio_obj).attr("id").cast<int>();
}

template<class T>
int get_default(py::object o, T default_value) {
    if (o.is_none()) { return default_value; }
    return o.cast<T>();
}

class StateMachine {
    PIO pio{};
    int sm{-1};
    int offset{-1};

public:
    StateMachine(py::buffer assembled,
            double frequency,
            py::object first_sideset_pin,
            int sideset_pin_count,
            bool auto_pull,
            bool out_shift_right,
            int pull_threshold) {
        pio = pio_open_check();
        sm = pio_sm_claim_unused_sm_check(pio);
        py::buffer_info info = assembled.request();
        if (info.itemsize != 2) {
            throw py::value_error("assembled: Expected array of type `h`");
        }
        if (info.size >= 32) {
            throw py::value_error("assembled: Exceeds maximum program length (32)");
        }

        struct pio_program program = {
            .instructions = reinterpret_cast<uint16_t*>(info.ptr),
            .length = static_cast<uint8_t>(info.size),
            .origin = -1,
        };
        offset = pio_add_program_check(pio, &program);

        pio_sm_config c = {0, 0, 0};
        sm_config_set_wrap(&c, offset, offset + info.size - 1);

        if (!first_sideset_pin.is_none()) {
            if (sideset_pin_count < 1 || sideset_pin_count > 5) {
                throw py::value_error("sideset_pin_count out of range (must be 1 to 5)");
            }
            auto first_sideset_pin_number = get_pin_number(first_sideset_pin);
            for(int i=0; i<sideset_pin_count; i++) {
                pio_gpio_init(pio, first_sideset_pin_number + i);
            }
            sm_config_set_sideset(&c, sideset_pin_count, /* optional */ false, /* pindirs */ false);
            sm_config_set_sideset_pins(&c, first_sideset_pin_number);
        }

printf("pull threshold=%d\n", pull_threshold);
        sm_config_set_out_shift(&c, out_shift_right, auto_pull, pull_threshold);

        double div = clock_get_hz(clk_sys) / frequency;
printf("frequency=%f div=%f\n", frequency, div);
        sm_config_set_clkdiv(&c, div);

        pio_sm_init(pio, sm, offset, &c);
        pio_sm_set_enabled(pio, sm, true);

    }

    ~StateMachine() {
        if(!PIO_IS_ERR(pio)) pio_close(pio);
    }

    void write(py::buffer b) {
        py::buffer_info info = b.request();
        uint32_t *ptr = reinterpret_cast<uint32_t*>(info.ptr);
        std::vector<uint32_t> vec;
        printf("itemsize=%zd size=%zd\n", info.itemsize, info.size);
        // the DMA controller doesn't replicate 8- and 16-bit values like on rp2, so we have to do it ourselves
        if (info.itemsize != 4) {
            vec.reserve(info.size);
            switch(info.itemsize) {
                case 1:
                {
        printf("replicating 8-bit values to 32 bits\n");
                    auto *buf = reinterpret_cast<uint8_t*>(info.ptr);
                    for(pybind11::ssize_t i=0; i<info.size; i++) {
                        vec.push_back(buf[i] * 0x01010101);
                    }
                    break;
                }
                case 2:
                {
        printf("replicating 16-bit values to 32 bits\n");
                    auto *buf = reinterpret_cast<uint16_t*>(info.ptr);
                    for(pybind11::ssize_t i=0; i<info.size; i++) {
                        vec.push_back(buf[i] * 0x00010001);
                    }
                }
                    break;
                default:
                    throw py::value_error("buffer must contain items of 1, 2, or 4 bytes");
            }
        printf("setting ptr to vec[0]\n");
            ptr = &vec[0];
        } else {
        printf("using items as-is\n");
        }
        size_t size = info.size * sizeof(uint32_t);
        printf("transfer size %zd\n", size);
        printf("ptr @ %p info.ptr @ %p\n", ptr, info.ptr);
printf("%08x %08x %08x %08x\n", ptr[0], ptr[1], ptr[2], ptr[3]);
        if (pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, size, 1)) {
            throw std::runtime_error("pio_sm_config_xfer() failed");
        }
        if (pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, size, ptr)) {
            throw std::runtime_error("pio_sm_xfer_data() failed");
        }
    }
};

PYBIND11_MODULE(adafruit_rp1pio, m) {
    m.doc() = R"pbdoc(
        Control the RP1 I/O coprocessor
        -------------------------------

        .. currentmodule:: adafruit_rp1pio

        .. autosummary::
           :toctree: _generate

           StateMachine
    )pbdoc";


    py::class_<StateMachine>(m, "StateMachine")
        .def(py::init<py::buffer /* assembled */,
                double /* frequency */,
                py::object /* first_sideset_pin */,
                int /* sideset_pin_count */,
                bool /* auto_pull */,
                bool /* out_shift_right */,
                int /* pull_threshold */>(),
            py::arg("assembled"),
            py::arg("frequency"),
            py::arg("first_sideset_pin") = py::none(),
            py::arg("sideset_pin_count") = 1,
            py::arg("auto_pull") = false,
            py::arg("out_shift_right") = true,
            py::arg("pull_threshold") = 32
            )
        .def("write", &StateMachine::write);

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
}
