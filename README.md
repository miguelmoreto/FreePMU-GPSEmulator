# FreePMU GPS Emulator
A STM32F4 based device to emulate a GPS PPS signal and the timestamp to test the [FreePMU](https://github.com/gustavowd/FreePMU) prototypes without a proper GPS module or when the GPS signal is weak.

## Hardware

This project is based on a [STM32 Nucleo F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) board from ST. Although, any STM32 microcontroller with a RTC clock can be used with little modification.

## How it works

A Timer is configured to generate a 1PPS or 30PPS pulse in Pin XXX. In addition, at every second, a NMEA string is formatted with the date and time and sent to USART, effectively emulating a GPS module.

The microcontroler is configured to use the RTC clock to count and store the time and date. Thus, the use of a 3V battery connected to VBAT pin is recommended to keep track of the time when the device is turned off.

When pressing the user button during boot, the date and time are reset to 2022 january 1st, 12h:00,000s.

## Limitations

This device is for testing purposes only. It does not substitute a proper GPS module. The time accuracy of the generated PPS signal is subjected to the accuracy of the board oscillator crystal. Also, if using this emulator with two or more PMUs, there has to be a common ground connection. They also have to be placed closely together.

## Peripherals used

* USART1: used for transmission of the GPS NMEA string. Configured as 115200 bps, 8bits, no parity and 1 stop bit.
* USART6: used for setting/update the date and time and other control functions. Configured as 115200 bps, 8bits, no parity and 1 stop bit.
* TIM2: 32bits timer used for generating the PPS signal. Configured in PWM Output Compare mode, 10% duty cycle. The timer clock is 84MHz -> time resolution 12 ns.

## Pins used

* PA0: Emulated PPS.
* PA9: USART1 TX (GPS TX)
* PA10 USART1 RX (GPS RX)
* PA11: USART6 TX (serial terminal).
* PA12: USART6 RX (serial terminal).

## Serial commands

| Command | Description |
|--------|--------|
| h | Show help message with the available commands. |
| s | Set the date and time by entering a string with the folowing format: *hh,mm,ss,DD,MM,YY* where *hh* is the hour, *mm* the minutes, *ss* the seconds, *DD* the day number (1-31), *MM* the month number (1-12) and YY the year number. |
| p | Print the last GPS string sent. |
| P | Print formated date and time. |