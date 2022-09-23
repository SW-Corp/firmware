# Water Treatment Lab Controller v1.0

Kod źródłowy programu dla mikrokontrolera sterującego stanowiskiem laboratoryjnym.

Został on oparty o platformę [PlatformIO](https://platformio.org/) i jest przeznaczony dla mikrokontrolerów opartych o układ ATMega328P. W projekcie zostało użyte Arduino Nano, ze względu na swoją kompaktową formę i wbudowany moduł komunikacji USB.

W projekcie zostały wykorzystane dwie biblioteki - `BMP280_DEV` oraz `INA219_WE`. Obie są dostępne z poziomu managera bibliotek PlatformIO. Zostały one uwzględnione w pliku konfiguracyjnym platformio.ini, dzięki czemu nie powinno być problemu z ich instalacją (powinna nastąpić automatycznie podczas budowania projektu).

Projekt również czerpie z projektu [grbl](https://github.com/gnea/grbl) - zaczerpnięta została komunikacja z portem szeregowym (pliki `print.c`, `print.h`, `serial.c`, `serial.h`).

W zakładce "Release" dostępny jest skompilowany plik z najnowszą wersją oprogramowania. Plik **.hex** można wgrać do mikrokontrolera używając gotowych programów do flashowania, takich jak Flash Wizard.