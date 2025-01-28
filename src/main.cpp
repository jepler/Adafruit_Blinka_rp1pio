#include <iostream>
#include <pybind11/pybind11.h>

#include "piolib.h"
#include "utils/piolib/examples/ws2812.pio.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

namespace {

typedef struct { uint32_t value;
} pio_pinmask_t;
typedef uint32_t pio_pinmask_value_t;
#define PIO_PINMASK_C(c) UINT32_C(c)
#define PIO_PINMASK_BIT (32)
#define PIO_PINMASK(i) (UINT32_C(1) << (i))
#define PIO_PINMASK_PRINT(p) mp_printf(&mp_plat_print, "%s:%d: %s = %08x\n", \
    __FILE__, __LINE__, #p, \
    (uint32_t)(PIO_PINMASK_VALUE(p)));
#define PIO_PINMASK_ALL PIO_PINMASK_FROM_VALUE(~UINT32_C(0))

#define PIO_PINMASK_VALUE(p) ((p).value)
#define PIO_PINMASK_FROM_VALUE(v) ((pio_pinmask_t) {(v)})
#define PIO_PINMASK_FROM_PIN(i) ((pio_pinmask_t) {(PIO_PINMASK(i))})
#define PIO_PINMASK_NONE PIO_PINMASK_FROM_VALUE(0)
#define PIO_PINMASK_SET(p, i) ((p).value |= PIO_PINMASK(i))
#define PIO_PINMASK_CLEAR(p, i) ((p).value &= ~PIO_PINMASK(i))
#define PIO_PINMASK_IS_SET(p, i) (((p).value & PIO_PINMASK(i)) != 0)
#define PIO_PINMASK_BINOP(op, p, q) PIO_PINMASK_FROM_VALUE((p).value op(q).value)
#define PIO_PINMASK_BINOP_ASSIGN(op, p, q) ((p).value op(q).value)
#define PIO_PINMASK_EQUAL(p, q) ((p).value == (q).value)
#define PIO_PINMASK_AND(p, q) PIO_PINMASK_BINOP(&, (p), (q))
#define PIO_PINMASK_AND_NOT(p, q) PIO_PINMASK_BINOP(&~, (p), (q))
#define PIO_PINMASK_OR(p, q) PIO_PINMASK_BINOP(|, (p), (q))
#define PIO_PINMASK_OR3(p, q, r) PIO_PINMASK_OR((p), PIO_PINMASK_OR((q), (r)))
#define PIO_PINMASK_INTERSECT(p, q) PIO_PINMASK_BINOP_ASSIGN( &=, (p), (q))
#define PIO_PINMASK_DIFFERENCE(p, q) PIO_PINMASK_BINOP_ASSIGN( &= ~, (p), (q))
#define PIO_PINMASK_MERGE(p, q) PIO_PINMASK_BINOP_ASSIGN( |=, (p), (q))

#define PIO_PINMASK_CONSECUTIVE_PINS(start, count) PIO_PINMASK_FROM_VALUE(((PIO_PINMASK_C(1) << count) - 1) << start)
#define PIO_PINMASK_SHIFTED(p, count) PIO_PINMASK_FROM_VALUE(PIO_PINMASK_FROM_VALUE(p) << count)

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

#ifndef NUM_BANK0_GPIOS
#define NUM_BANK0_GPIOS (28)
#endif

static void rp2pio_statemachine_set_pull(pio_pinmask_t pull_pin_up, pio_pinmask_t pull_pin_down, pio_pinmask_t pins_we_use) {
    for (size_t i = 0; i < NUM_BANK0_GPIOS; i++) {
        bool used = PIO_PINMASK_IS_SET(pins_we_use, i);
        if (used) {
            bool pull_up = PIO_PINMASK_IS_SET(pull_pin_up, i);
            bool pull_down = PIO_PINMASK_IS_SET(pull_pin_down, i);
            gpio_set_pulls(i, pull_up, pull_down);
        }
    }
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
            int8_t offset,
            py::buffer init,
            py::object first_sideset_pin,
            int sideset_pin_count,
            bool sideset_enable,
            py::object first_in_pin,
            int in_pin_count,
            uint32_t pull_in_pin_up,
            uint32_t pull_in_pin_down,
            bool auto_pull,
            bool out_shift_right,
            int pull_threshold,
            bool auto_push,
            bool in_shift_right,
            int push_threshold,
            int wrap,
            int wrap_target) {
        pio = pio_open_check();
        sm = pio_sm_claim_unused_sm_check(pio);
        py::buffer_info info = assembled.request();
        if (info.itemsize != 2) {
            throw py::value_error("assembled: Expected array of type `H`");
        }
        if (info.size >= 32) {
            throw py::value_error("assembled: Exceeds maximum program length (32)");
        }
        if (offset < -1 || offset > 31) {
            throw py::value_error("offset exceeds valid range of -1 to 31 inclusive");
        }
        if (!init.is_none()) {
            py::buffer_info init_info = init.request();
            if (info.itemsize != 2) {
                throw py::value_error("init: Expected array of type `H`");
            }
        }

        ssize_t program_len = info.size;
        if(wrap == -1)  {
            wrap = program_len - 1;
        }

        if(wrap < 0 || wrap >= program_len) {
            throw py::value_error("wrap out of range");
        }

        if(wrap_target < 0 || wrap >= program_len) {
            throw py::value_error("wrap_target out of range");
        }

        pio_pinmask_t pindirs = PIO_PINMASK_NONE;
        pio_pinmask_t pins_we_use = PIO_PINMASK_NONE;
        pio_pinmask_t pin_pull_up = PIO_PINMASK_NONE;
        pio_pinmask_t pin_pull_down = PIO_PINMASK_NONE;

        struct pio_program program = {
            .instructions = reinterpret_cast<uint16_t*>(info.ptr),
            .length = static_cast<uint8_t>(info.size),
            .origin = offset,
        };

        offset = pio_add_program_check(pio, &program);
        wrap += offset;
        wrap_target += offset;

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, wrap_target, wrap);

        if (!first_sideset_pin.is_none()) {
            auto first_sideset_pin_number = get_pin_number(first_sideset_pin);
            if (sideset_pin_count < 1 || (sideset_enable + sideset_pin_count) > 5 || first_sideset_pin_number + sideset_pin_count > NUM_BANK0_GPIOS) {
                throw py::value_error("sideset_pin_count out of range");
            }

            PIO_PINMASK_MERGE(pindirs, PIO_PINMASK_CONSECUTIVE_PINS(sideset_pin_count, first_sideset_pin_number));
            PIO_PINMASK_MERGE(pins_we_use, PIO_PINMASK_CONSECUTIVE_PINS(sideset_pin_count, first_sideset_pin_number));

            for(int i=0; i<sideset_pin_count; i++) {
                pio_gpio_init(pio, first_sideset_pin_number + i);
            }
            sm_config_set_sideset(&c, sideset_pin_count + sideset_enable, /* optional */ sideset_enable, /* pindirs */ false);
            sm_config_set_sideset_pins(&c, first_sideset_pin_number);
        }

        if (!first_in_pin.is_none()) {
            auto first_in_pin_number = get_pin_number(first_in_pin);
            if (in_pin_count < 1 || first_in_pin_number + in_pin_count > NUM_BANK0_GPIOS) {
                throw py::value_error("sideset_pin_count out of range");
            }
            for(int i=0; i<in_pin_count; i++) {
                pio_gpio_init(pio, first_in_pin_number + i);
            }
            sm_config_set_in_pins(&c, first_in_pin_number);
            PIO_PINMASK_MERGE(pin_pull_up, PIO_PINMASK_FROM_VALUE(pull_in_pin_up << first_in_pin_number));
            PIO_PINMASK_MERGE(pin_pull_down, PIO_PINMASK_FROM_VALUE(pull_in_pin_down << first_in_pin_number));
            PIO_PINMASK_MERGE(pins_we_use, PIO_PINMASK_CONSECUTIVE_PINS(in_pin_count, first_in_pin_number));
        }

        pio_sm_set_pindirs_with_mask(pio, sm, PIO_PINMASK_VALUE(pindirs), PIO_PINMASK_VALUE(pins_we_use));
        rp2pio_statemachine_set_pull(pin_pull_up, pin_pull_down, pins_we_use);

        if (!init.is_none()) {
            run(init);
        }

        sm_config_set_out_shift(&c, out_shift_right, auto_pull, pull_threshold);
        sm_config_set_in_shift(&c, in_shift_right, auto_push, push_threshold);

        double div = frequency ? clock_get_hz(clk_sys) / frequency : 1.0;
        sm_config_set_clkdiv(&c, div);

        pio_sm_init(pio, sm, offset, &c);
        pio_sm_set_enabled(pio, sm, true);

        if (pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 65532, 2)) {
            throw std::runtime_error("pio_sm_config_xfer(PIO_DIR_TO_SM) failed");
        }
        if (pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 65532, 2)) {
            throw std::runtime_error("pio_sm_config_xfer(PIO_DIR_FROM_SM) failed");
        }
    }

    void run(py::buffer instructions) {
            py::buffer_info info = instructions.request();
            if (info.itemsize != 2) {
                throw py::value_error("instructions: Expected array of type `H`");
            }
            auto raw_instructions = reinterpret_cast<const uint16_t *>(info.ptr);
            for (ssize_t i = 0; i < info.size; i++) {
                pio_sm_exec(pio, sm, raw_instructions[i]);
            }
    }

    ~StateMachine() {
        if(!PIO_IS_ERR(pio)) pio_close(pio);
    }

    void readinto(py::buffer b) {
        py::buffer_info info = b.request();
        uint32_t *info_ptr32 = reinterpret_cast<uint32_t*>(info.ptr);
        uint32_t *ptr = info_ptr32;

        if (info.readonly) {
            throw py::type_error("read-only buffer");
        }

        std::vector<uint32_t> vec;
        // the DMA controller doesn't replicate 8- and 16-bit values like on rp2, so we have to do it ourselves
        if (info.itemsize != 4) {
            vec.reserve(info.size);
            switch(info.itemsize) {
                case 1:
                    break;
                case 2:
                    break;
                default:
                    throw py::value_error("buffer must contain items of 1, 2, or 4 bytes");
            }
            ptr = &vec[0];
        }

        size_t size = info.size * sizeof(uint32_t);
        if (pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, size, ptr)) {
            throw std::runtime_error("pio_sm_xfer_data() failed");
        }

        switch(info.itemsize) {
            case 1:
                {
                    uint8_t *info_ptr8 = reinterpret_cast<uint8_t*>(info.ptr);
                    for(ssize_t i=0; i<info.size; i++) {
                        info_ptr8[i] = vec[i];
                    }
                }
                break;
            case 2:
                {
                    uint16_t *info_ptr16 = reinterpret_cast<uint16_t*>(info.ptr);
                    for(ssize_t i=0; i<info.size; i++) {
                        info_ptr16[i] = vec[i];
                    }
                }
                break;
        }
    }

    void write(py::buffer b) {
        py::buffer_info info = b.request();
        uint32_t *ptr = reinterpret_cast<uint32_t*>(info.ptr);
        std::vector<uint32_t> vec;
        // the DMA controller doesn't replicate 8- and 16-bit values like on rp2, so we have to do it ourselves
        if (info.itemsize != 4) {
            vec.reserve(info.size);
            switch(info.itemsize) {
                case 1:
                {
                    auto *buf = reinterpret_cast<uint8_t*>(info.ptr);
                    for(pybind11::ssize_t i=0; i<info.size; i++) {
                        vec.push_back(buf[i] * 0x01010101);
                    }
                    break;
                }
                case 2:
                {
                    auto *buf = reinterpret_cast<uint16_t*>(info.ptr);
                    for(pybind11::ssize_t i=0; i<info.size; i++) {
                        vec.push_back(buf[i] * 0x00010001);
                    }
                }
                    break;
                default:
                    throw py::value_error("buffer must contain items of 1, 2, or 4 bytes");
            }
            ptr = &vec[0];
        }
        size_t size = info.size * sizeof(uint32_t);
        if (pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, size, ptr)) {
            throw std::runtime_error("pio_sm_xfer_data() failed");
        }
    }
};

PYBIND11_MODULE(adafruit_rp1pio, m) {
    m.doc() = R"pbdoc(
        Hardware interface to RP1 series’ programmable IO (PIO) peripheral.
        -------------------------------------------------------------------

        Except as noted, this is intended to be a subset of the functionality
        in CircuitPython's ``rp2pio`` module.

        .. currentmodule:: adafruit_rp1pio

        .. autosummary::
           :toctree: _generate

           StateMachine
    )pbdoc";


    py::class_<StateMachine>(m, "StateMachine", "A single PIO StateMachine")
        .def(py::init<py::buffer /* assembled */,
                double /* frequency */,
                int8_t /* offset */,
                py::buffer /* init */,
                py::object /* first_sideset_pin */,
                int /* sideset_pin_count */,
                bool /* sideset_enable */,
                py::object /* first_in_pin */,
                int /* in_pin_count */,
                uint32_t /* pin_in_pull_up */,
                uint32_t /* pin_in_pull_down */,
                bool /* auto_pull */,
                bool /* out_shift_right */,
                int /* pull_threshold */,
                bool /* auto_push */,
                bool /* in_shift_right */,
                int /* push_threshold */,
                int /* wrap */,
                int /* wrap_target */ >(),
            "Construct a StateMachine",
            py::arg("assembled"),
            py::arg("frequency"),
            py::kw_only(),
            py::arg("offset") = -1,
            py::arg("init") = py::none(),
            py::arg("first_sideset_pin") = py::none(),
            py::arg("sideset_pin_count") = 1,
            py::arg("sideset_enable") = false,
            py::arg("first_in_pin") = py::none(),
            py::arg("in_pin_count") = 1,
            py::arg("pull_in_pin_up") = 0,
            py::arg("pull_in_pin_down") = 0,
            py::arg("auto_pull") = false,
            py::arg("out_shift_right") = true,
            py::arg("pull_threshold") = 32,
            py::arg("auto_push") = false,
            py::arg("in_shift_right") = true,
            py::arg("push_threshold") = 32,
            py::arg("wrap") = -1,
            py::arg("wrap_target") = 0
            )
        .def("write", &StateMachine::write, "Write the data contained in buffer to the state machine", py::arg("buffer"))
        .def("readinto", &StateMachine::readinto, "Read data from the state machine into a buffer", py::arg("buffer"))
        .def("run", &StateMachine::run, "Execute instructions on the state machine", py::arg("instructions"));

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
}
