Raspberry Pi4
🧰 Előkészületek
✅ Szükséges eszközök:
•	Raspberry Pi 4 (ajánlott min. 2GB RAM)
•	microSD kártya (legalább 16GB, Class 10)
•	Monitor, billentyűzet az első indításhoz (vagy SSH ha headless)
•	Internet kapcsolat (Ethernet vagy Wi-Fi)
________________________________________
📦 1. Raspbian (Raspberry Pi OS) telepítése
1.	Töltsd le a Raspberry Pi Imager-t:
o	https://www.raspberrypi.com/software
2.	Telepítsd a Raspberry Pi OS Lite verzióját (nincs GUI, kevesebb erőforrás kell):
o	Válaszd a „Raspberry Pi OS Lite (64-bit)” opciót.
o	Flash-eld az SD-kártyára.
3.	SSH engedélyezése (ha headless):
o	A boot partícióba hozz létre egy üres ssh nevű fájlt (touch ssh).
4.	Első indítás után:
•	sudo apt update && sudo apt upgrade
•	sudo apt install git build-essential cmake libi2c-dev i2c-tools

🧠 2. Sunray landrumower-linux projekt letöltése és fordítása
•	git clone -b landrumower-linux https://github.com/EinEinfach/Sunray.git
•	cd Sunray
•	make
Ez lefordítja a fő programot, amiből lesz egy ./sunray bináris.
📁 A src/ mappában található a fő logika, main.cpp a belépési pont.

🧪 3. Futás tesztelése:
./sunray
Ha minden jól megy, kiírja a konzolra a rendszer állapotát, és várja a szenzor/vezérlő csatlakozásokat (pl. UART-on a Pico-tól).

🔌 4. UART kommunikáció engedélyezése (később a Pico-hoz)
A Pi-n a GPIO14 (TX) és GPIO15 (RX) lábakon érkezik majd a soros adat a Pico-tól.
1.	Engedélyezd a hardveres UART-ot:
•	sudo raspi-config
Interfacing Options → Serial 
  		Login shell over serial: No 
  		Enable serial port hardware: Yes

2.	Indítsd újra:

•	sudo reboot

3.	A Pi oldalon a /dev/serial0 (vagy /dev/ttyAMA0) lesz az elérési út.

🧠 5. Raspberry Pi Pico firmware
A pico/ könyvtárban található külön firmware, amit a Pico-ra kell fordítani (külön lépés, CMake-t igényel). Erről tudok külön útmutatót írni.

Pico: 

✅ Pico firmware feltöltése – legegyszerűbb mód
🔌 1. Tedd DFU (bootloader) módba a Pico-t:
1.	Tartsd lenyomva a BOOTSEL gombot.
2.	Miközben nyomva tartod, csatlakoztasd a Pico-t USB-n a számítógéphez.
3.	Engedd el a BOOTSEL gombot.
4.	Meg fog jelenni egy új USB meghajtó pl. RPI-RP2 néven.
5.	https://micropython.org/resources/firmware/RPI_PICO-20241129-v1.24.1.uf2 másold a Pico-ra

📁 2. Töltsd le a firmware-t:
Menj ide:
📂 https://github.com/EinEinfach/Sunray/tree/landrumower-linux/landrumower/firmware
➡️ Közvetlen letöltés (jobb klikk + "Mentés másként"):
landrumower/firmware

💾 3. Másold rá a Pico-ra:
Egyszerűen másold rá a .uf2 fájlt az RPI-RP2 meghajtóra, és a Pico automatikusan újraindul az új firmware-rel.

🔁 4. Csatlakoztasd a Raspberry Pi-hez:
•	A Pico most már soros protokollon keresztül kommunikál a Pi-vel.
•	Csatlakoztasd az alábbi GPIO lábakat a Pi és a Pico között:
PI GPIO	FUNKCIÓ	PICO GPIO
GPIO14	TXD	GPIO1 (UART RX)
GPIO15	RXD	GPIO0 (UART TX)
GND	GND	GND
A Pico-n az alapértelmezett UART0 a GPIO 0 (TX) és GPIO 1 (RX) lábakon van.

🧪 Tesztelés:
Indítsd el újra a Sunray-t a Raspberry Pi-n:
•	./sunray
Ha minden jól ment, elindul a kommunikáció a Pico-val, és az állapotlogban látod, hogy érzékeli a mikrokontrollert és olvassa az adatokat (pl. IMU, motor állapot).

TESZT mód

📌 Mit kell módosítani a config.h fájlban a teszt módhoz?
1.	Nyisd meg a fájlt szerkesztésre a Raspberry Pi-n:
•	nano Sunray/landrumower/config.h
//define DRV_SERIAL_ROBOT  1     // Linux (Alfred)
//#define DRV_ARDUMOWER     1   // keep this for Ardumower
#define DRV_SIM_ROBOT     1   // simul
2.	Mentsd el a módosításokat (CTRL + X, Y, Enter).


3.	Fordítsd újra a kódot:

•	make clean
•	make

4.	Indítsd el a Sunray-t teszt módban:

•	./sunray



