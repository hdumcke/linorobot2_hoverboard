Hoverboard
=========

We will leverage the work done and documented in the following projects:

https://github.com/lucysrausch/hoverboard-firmware-hack.git

https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2.git

We are using a Gen2 hoverboard and assembled firmware from different forks of the original repo.

Building the firmware
---------------------

The original (hacked) firmware requires Keil to compile the code. As Keil is only available on Windows we have build a procedure that allows to build the SW on many OS: Windows, Linux and Mac OS. As the cross compiler runs on X86 we currently do not support ARM based PCs.

The build procedure consists of starting a Ubuntu VM, downloading the compiler and the source code and then building the firmware for the master and the slave board.

Install Multipass
^^^^^^^^^^^^^^^^^

Follow the guide at https://multipass.run to install Multipass for your operating system

Install Multipass Orchestrator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Follow the README in https://github.com/hdumcke/multipass-orchestrator

Deploy the Build VM
^^^^^^^^^^^^^^^^^^^

Clone https://github.com/hdumcke/multipass-orchestrator-configurations and use mpo-deploy to deploy the configuration in multipass-orchestrator-configurations/armtoolchain/config.yaml

This will start a VM, install the build system, clones the repository with the source code and compiles the firmware for the master and the slave board. You will find the firmware in the directories ~/master and ~/slave in the VM. Use SCP to transfer the firmware to your host PB

Flashing the boards
-------------------

You will find the schematics of the board here: https://github.com/hdumcke/linorobot2_hoverboard/blob/main/Hoverboard-Firmware-Hack-Gen2/Schematics/HoverBoard_CoolAndFun.pdf

It is convenient to solder pins into the holes for the STLink connector. Do only connect GDN, SWDIO and SWCLK, do not connect power otherwise you risk to break the board.

st-flash --reset write firmware.bin 0x8000000

Make sure you use the correct firmware for the board you are flashing (master or slave).

Future directions
-----------------

There are different ways to control the phases of a BLDC motor: trapezoidal, sinusoidal or FOC (field oriented control)

There are open source projects that implement BLDC control algorithm but we have not found any project that applies to Gen2 boards that we are using.

We should investigate and write our own control algorithm.
