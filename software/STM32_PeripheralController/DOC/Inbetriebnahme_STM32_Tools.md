# Inbetriebnahme der STM32 Toolchain

Im Git Verzeichnis wird nur die folgende Ordnerstruktur ausgecheckt:

<pre><code>
/STM32_PeripheralController/DOC // documentation folder
/STM32_PeripheralController/SRC // source files 
/STM32_PeripheralController/RPI // Raspberry Pi interface test project
</pre></code>

Zum Erstellen der Files für das ARM Embedded Board sind noch die folgenden Compiler und Tools notwendig:

- ARM GCC Compiler (zum Download unter https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)
- Flash Tool ST-LINK (zum Download unter https://github.com/texane/stlink , nur Sourcen, muss unter Linux mit make erstellt werden, siehe Anleitungen)
- STM32 CubeMX Software zur Konfiguration des ARM Prozessors (Pin Belegungen, Funktionen, usw., zum Download unter https://www.st.com/en/development-tools/stm32cubemx.html)

Die Tools sollten im Verzeichnis TOOLS wie folgt angelegt werden:
<pre><code>
├── gcc-arm-none-eabi-7-2018-q2-update-linux
│   ├── arm-none-eabi
│   ├── bin
│   ├── lib
│   └── share
├── stlink-master
│   ├── ChangeLog.md
│   ├── cmake
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   ├── cmake_install.cmake
│   ├── CMakeLists.txt
│   ├── CPackConfig.cmake
│   ├── CPackSourceConfig.cmake
│   ├── CTestTestfile.cmake
│   ├── DartConfiguration.tcl
│   ├── debian
│   ├── doc
│   ├── etc
│   ├── flashloaders
│   ├── include
│   ├── libstlink.a
│   ├── libstlink.so -> libstlink.so.1
│   ├── libstlink.so.1 -> libstlink.so.1.5.0
│   ├── libstlink.so.1.5.0
│   ├── LICENSE
│   ├── Makefile
│   ├── README.md
│   ├── scripts
│   ├── src
│   ├── st-flash
│   ├── st-info
│   ├── stlinkv1_macosx_driver
│   ├── Testing
│   ├── tests
│   └── usr
├── STM32CubeMX (optional)
</pre></code>

Der Code für den STM32 Controller wird mithilfe von CMake im Verzeichnis <code>/BUILD</code> erstellt. Dazu ins Verzeichnis <code>/BUILD</code> wechseln, und den Befehl <code> cmake ../SRC </code> ausführen.

Die daraus entstandene Binärdatei <code>/BUILD/STM32_HSPCarBin</code> kann dann dann mithilfe des Scripts </code>/SRC/upload.sh</code> auf den STM32 Controller übertragen werden.

# Programmierung des STM32 Boards

Der STM32 Controller kann wie folgt programmiert werden:

1. Das STM32 Controller Board mit Strom versorgen.
2. Das USB Kabel an den PC anstecken.
3. Die Binärdatei mit CMake und make erstellen (im Verzeichnis <code>/BUILD/cmake ../SRC</code> und <code>make</code> ausführen).
4. Die Binärdatei im Verzeichnis <code>/BUILD</code> mit dem Befehl <code>sh ../SRC/upload.sh</code> auf den Controller laden.

