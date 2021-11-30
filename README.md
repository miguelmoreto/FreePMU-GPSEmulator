# FreePMU GPS Emulator
A STM32F4 based device to emulate a GPS PPS signal and the timestamp to test the [FreePMU](https://github.com/gustavowd/FreePMU) prototypes without a proper GPS module or when the GPS signal is weak.

## Hardware

This project is based on a [STM32 Nucleo F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) board from ST. Although, any STM32 microcontroller with a RTC clock can be used with little modification.

## How it works

A Timer is configured to generate a 1PPS or 30PPS pulse in Pin XXX. In addition, at every second, a NMEA string is formatted with the date and time and sent to USART, effectively emulating a GPS module.

Thu microcontroler is configured to use the RTC clock to count and store the time and date. Thus, the use of a 3V battery connected to VBAT pin is recommended to keep track of the time when the device is turned off.

## Limitations

This device is for testing purposes only. It does not substitute a proper GPS module. The time accuracy of the generated PPS signal is subjected to the accuracy of the board oscillator crystal. Also, if using this emulator with two or more PMUs, there has to be a common ground connection. They also have to be placed closely together.