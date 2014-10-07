#SAMR21 Xplained Pro test program

Blinks the LED on the SAMR21 Xplained Pro. Builds on OSX & Ubuntu.

Prereqs:

* OpenOCD
* arm-gcc

##Building

###Install arm-gcc
On OSX:

```
brew install gcc-arm
```

On Ubuntu:

```
sudo apt-get install gcc-arm-none-eabi
```

###Install latest version of OpenOCD

On OSX:

```
brew install openocd --HEAD
```

On Ubuntu:

```
git clone https://github.com/ntfreak/openocd.git;
cd openocd;
./bootstrap;
./configure;
make;
sudo make install;
```

##Running

```
cd apps/Peer2Peer/make;
make -f Makefile_XplainedPro_ATSAMR21;
arm-none-eabi-gdb Debug/firmware.elf -ex 'target remote | openocd -c "gdb_port pipe;" -f tools/samR21.cfg'
```

Then when gdb inits:

```
> (gdb) load
> (gdb) monitor reset halt
> (gdb) c
```

This will load up the elf file, reset the device, and continue running.

