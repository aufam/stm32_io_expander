#ifndef IO_EXPANDER_H
#define IO_EXPANDER_H

#include "periph/i2c.h"
#include "periph/uart.h"
#include "periph/gpio.h"
#include "etl/array.h"

namespace Project {

    template <size_t NBytes>
    class IOExpander {
        static_assert(NBytes == 1 || NBytes == 2 || NBytes == 4);
        using T = etl::conditional_t<NBytes == 1, uint8_t, etl::conditional_t<NBytes == 2, uint16_t, uint32_t>>;

        inline static const uint32_t timeoutDefault = 100;
        mutable T writeMode = 0, readMode = 0;
        mutable T writeBuffer = 0, readBuffer = 0;

    public:
        periph::I2C* i2c;
        periph::UART* uart;
        uint8_t address; ///< device 7 bit address

        struct Constructor1Args { periph::I2C& i2c; uint8_t address;};
        struct Constructor2Args { periph::I2C& i2c; bool a2, a1, a0;};
        struct Constructor3Args { periph::UART& uart; };
        
        /// construct with specified address
        /// @param args
        ///     - .i2c reference to periph::I2C object 
        ///     - .address device address
        constexpr IOExpander(Constructor1Args args) : i2c(&args.i2c), uart(nullptr), address(args.address) {}

        /// construct with specified input pins state
        /// @param args
        ///     - .i2c reference to periph::I2C object 
        ///     - .a2 input pin a2 state
        ///     - .a1 input pin a1 state
        ///     - .a0 input pin a0 state
        constexpr IOExpander(Constructor2Args args) : i2c(&args.i2c), uart(nullptr), address(addressReference(args.a2, args.a1, args.a0)) {}

        /// construct with specified address
        /// @param args
        ///     - .uart reference to periph::UART object 
        ///     - .address device address
        constexpr IOExpander(Constructor3Args args) : i2c(nullptr), uart(&args.uart), address(0) {}

        IOExpander(const IOExpander&) = delete; ///< disable copy constructor
        IOExpander& operator=(const IOExpander&) = delete; ///< disable copy assignment

        /// write value to the io expander
        /// @param value desired value
        /// @param timeout in tick. default = timeoutDefault
        void write(T value, uint32_t timeout = timeoutDefault) const { 
            auto buffer = etl::byte_array_cast_le<T>(value);
            if (i2c)
                HAL_I2C_Master_Transmit(&i2c->hi2c, address, buffer.data(), buffer.len(), timeout); 
            if (uart)
                uart->transmitBlocking(buffer.data(), buffer.len(), {.timeout=etl::time::milliseconds(timeout)});
            writeBuffer = value; // save to write buffer
        }

        /// read value from the io expander
        /// @param timeout in tick. default = timeoutDefault
        /// @return value from the io expander
        T read(uint32_t timeout = timeoutDefault) const { 
            auto buffer = etl::array<uint8_t, NBytes>();
            if (i2c) {
                HAL_I2C_Master_Receive(&i2c->hi2c, address, buffer.data(), buffer.len(), timeout); 
                T res = etl::byte_array_cast_back_le<T>(buffer);
                if (res & readMode) readBuffer |= res; // only save to buffer if the pin is in read mode
                return res;
            }
            // uart not supported yet
            return 0;
        }

        struct GPIO;

        /// create GPIO object
        constexpr GPIO io(T pin, bool activeMode) { return GPIO{this, pin, activeMode}; }

    private:
        constexpr uint8_t addressReference(bool a2, bool a1, bool a0) { return (a2 << 2 | a1 << 1 | a0) + 0x40; }
    };

    template <size_t NBytes>
    struct IOExpander<NBytes>::GPIO {
        const IOExpander<NBytes>* port = nullptr; ///< reference to IOExpander object
        IOExpander<NBytes>::T pin = 0;      ///< GPIO_PIN_x
        bool activeMode = false;            ///< periph::GPIO::activeLow or periph::GPIO::activeHigh

        /// init GPIO
        /// @param args
        ///     - .mode GPIO_MODE_INPUT or GPIO_MODE_OUTPUT_PP
        ///     - .pull unused
        ///     - .speed unused
        void init(periph::GPIO::InitArgs args) const {
            if (args.mode == GPIO_MODE_INPUT) {
                port->readMode |= pin;
                port->writeMode &= ~pin;
            } else {
                port->writeMode |= pin;
                port->readMode &= ~pin;
                off();
            }
        }

        /// return true if this object is valid
        explicit operator bool() { return bool(port); }

        /// write pin high (true) or low (false)
        void write(bool value) const {
            port->writeBuffer = value ? port->writeBuffer | pin : port->writeBuffer & (~pin);
            port->write(port->writeBuffer & port->writeMode); // only write pin that is in write mode
        }

        /// toggle pin
        void toggle() const { port->writeBuffer & pin ? write(false) : write(true); }

        /// read pin
        /// @retval high (true) or low (false)
        [[nodiscard]] bool read() const {
            if (pin & port->writeMode) // if pin is in write mode, return pin status in write buffer
                return pin & port->writeBuffer;
            port->read();
            return pin & port->readBuffer;
        }

        /// turn on
        /// @param args
        ///     - .sleepFor sleep for a while. default = time::immediate
        void on(periph::GPIO::OnOffArgs args = {}) const {
            write(activeMode);
            etl::time::sleep(args.sleepFor);
        }

        /// turn off
        /// @param args
        ///     - .sleepFor sleep for a while. default = time::immediate
        void off(periph::GPIO::OnOffArgs args = {}) const {
            write(!activeMode);
            etl::time::sleep(args.sleepFor);
        }

        [[nodiscard]] bool isOn() const { return !(read() ^ activeMode); }
        [[nodiscard]] bool isOff() const { return (read() ^ activeMode); }
    };

    /// IO expander and GPIO peripheral
    struct IO {
        const IOExpander<2>* portExpander;    ///< pointer to IOExpander object
        GPIO_TypeDef* portGPIO;         ///< pointer to GPIOx
        uint16_t pin;                   ///< GPIO_PIN_x
        bool activeMode;                ///< periph::GPIO::activeLow or periph::GPIO::activeHigh

        /// empty constructor
        constexpr IO() 
            : portExpander(nullptr)
            , portGPIO(nullptr)
            , pin(0)
            , activeMode(0) {}
        
        /// construct from IO expander
        constexpr IO(const IOExpander<2>::GPIO& other) 
            : portExpander(other.port)
            , portGPIO(nullptr)
            , pin(other.pin)
            , activeMode(other.activeMode) {}

        /// construct from GPIO peripheral
        constexpr IO(const periph::GPIO& other)
            : portExpander(nullptr)
            , portGPIO(other.port)
            , pin(other.pin)
            , activeMode(other.activeMode) {}

        /// return true if this object is valid
        explicit operator bool() const { return portExpander || portGPIO; }

        /// init IO and turn off
        /// @param args
        ///     - .mode GPIO_MODE_xxx
        ///     - .pull @ref GPIO_NOPULL (default), @ref GPIO_PULLUP, @ref GPIO_PULLDOWN
        ///     - .speed @ref GPIO_SPEED_FREQ_LOW (default), @ref GPIO_SPEED_FREQ_MEDIUM, @ref GPIO_SPEED_FREQ_HIGH
        void init(periph::GPIO::InitArgs args) const {
            if (portExpander)
                IOExpander<2>::GPIO{portExpander, pin, activeMode}.init(args);
            if (portGPIO)
                periph::GPIO{portGPIO, pin, activeMode}.init(args);
        }

        /// write pin high (true) or low (false)
        void write(bool value) const {
            if (portExpander)
                IOExpander<2>::GPIO{portExpander, pin, activeMode}.write(value);
            if (portGPIO)
                periph::GPIO{portGPIO, pin, activeMode}.write(value);
        }

        /// toggle pin
        void toggle() const { 
            if (portExpander)
                IOExpander<2>::GPIO{portExpander, pin, activeMode}.toggle();
            if (portGPIO)
                periph::GPIO{portGPIO, pin, activeMode}.toggle(); 
        }

        /// read pin
        /// @retval high (true) or low (false)
        [[nodiscard]] bool read() const {
            if (portExpander)
                return IOExpander<2>::GPIO{portExpander, pin, activeMode}.read();
            if (portGPIO)
                return periph::GPIO{portGPIO, pin, activeMode}.read(); 
            return false;
        }

        /// turn on
        /// @param args
        ///     - .sleepFor sleep for a while. default = time::immediate
        void on(periph::GPIO::OnOffArgs args = periph::GPIO::OnOffDefault) const {
            if (portExpander)
                IOExpander<2>::GPIO{portExpander, pin, activeMode}.on(args);
            if (portGPIO)
                periph::GPIO{portGPIO, pin, activeMode}.on(args); 
        }

        /// turn off
        /// @param args
        ///     - .sleepFor sleep for a while. default = time::immediate
        void off(periph::GPIO::OnOffArgs args = periph::GPIO::OnOffDefault) const {
            if (portExpander)
                IOExpander<2>::GPIO{portExpander, pin, activeMode}.off(args);
            if (portGPIO)
                periph::GPIO{portGPIO, pin, activeMode}.off(args); 
        }

        [[nodiscard]] bool isOn() const { return !(read() ^ activeMode); }
        [[nodiscard]] bool isOff() const { return (read() ^ activeMode); }
    };
}



#endif // IO_EXPANDER_H