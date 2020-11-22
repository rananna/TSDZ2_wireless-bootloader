# TSDZ2 Bluetooth Bootloader
A secure wireless bluetooth and USB based DFU bootloader for the TSDZ2 ebike remote control and motor controller 
## Bootloader Features
 Based on the 'Secure Bootloader' example provided in the Nordic nRF52 development kit. A video showing a DFU process can be seen here:
[![video](https://img.youtube.com/vi/va3LJoiosoc/hqdefault.jpg)](https://youtu.be/va3LJoiosoc) 
- DFU over BLE (bluetooth low energy) OTA 
- DFU using USB (Windows or Unix PC's)
- Bootloader is self upgradable via either OTA or USB Serial
- Bootloader will time out and load a valid application if no DFU is initiated 
- Bootloader will sense gpio button pins on start up.
  If button is pressed then an adjustable timer starts, when timed out enters DFU mode. 
  If button is released before timeout, the bootloader will start the installed application immediately.
- Optionally, the firmware upgrade can be downgradable (`NRF_DFU_BL_ALLOW_DOWNGRADE` Makefile option)

## DFU mode will be initiated by any one of the following actions:
- Bootloader cannot find a valid application to load
- Bluetooth access in DFU mode
-  pressing the DFU button on the board or the wireless remote (see "Bootloader Features" above)

## The Garmin S340 softdevice
This bootloader (as well as the TSDZ2 remote control and controller firmwares) require the Garmin S340 softdevice to gain access to the bluetooth stack. Because of this, the S340 softdevice hex file needs to be flashed along with the bootloader hex file. The MAKE file will automatically combine these two hex files and flash the resulting combined file to the board. 
## Release Binary Hex file
If you're just looking to flash the combined bootloader and softdevice, simply flash this hex file using OPENOCD from the launch json menu in vscode for the respective giyhub repository source code for the TSDZ2 remote and controller. 
> The firmware HEX file to flash is this one: [ebike_wireless_remote_with_sd-v.1.0.hex](firmware/release/TSDZ2_bootloader_with_sd_v1.0.hex)
## Security Encryption
Note that the TSDZ2 bootloader source code is distributed with a public and private keypair for application signing. (private.key & public.key). This will permit anyone who has access to the private key to be able to flash and overwrite any application you have while in OTA DFU mode. If you want to protect your eBike firmware fron unauthorized access, build your own binary with a new generated keypair.
### Generating new keys
You will need to generate a pair of Public and Private Keys to encrypt the signature and sign the DFU image using ECDSA_P256_SHA256.
Nordic provides the nRFutil tool to generate these keys. nRFutil.exe can be downloaded from github [here](https://github.com/NordicSemiconductor/pc-nrfutil), the user guide is [here](https://www.nordicsemi.com/-/media/DocLib/Other/User_Guides/nrfutilv16pdf.pdf).
Or acquired from python using pip3 install nrfutil. To check for update, call pip3 install nrfutil --upgrade (or use python -m pip install nrfutil)
 
A1. Generate your own private key. Type this command line:
nrfutil.exe keys generate private.key
private.key file will be generated. You will keep this key secret
A2. Generate your public key based on your private key.
nrfutil keys display --key pk --format code private.key --out_file public_key.c
After you have the public_key.c file we can build the bootloader using the make file.
## Bootloader build options
Hex for the combined bootloaded and S340 softdevice flashes are output to the `_build` directory after compilation and linking success.
`NRF_DFU_BL_ALLOW_DOWNGRADE`  will allow the firmware to accept a downgraded version if desired
 `NRF_DFU_BL_ACCEPT_SAME_VERSION`).will allow the firmware to acccept the same version
 `BOARD_USE_SF_CLOCK` is defined to utilize the onboard Soft Device LFCLK.
### TSDZ2 remote control and Controller builds
for both firmwares, the respective MAKE files will automatically put the combined hex file and the signed firmware upgrade zip files in the _upgrade folder after the build process is completed.
## Boot-up Sequence
Based on the settings stored in the bootloader settings page, the bootloader determines: whether the application exists, and the location of it. The secure bootloader performs a signature verification of the application before booting into it.

Here are the boot-up steps that occur from reset to starting the application:

First, the MBR is booted.
The MBR looks up the location of the bootloader.
If a bootloader is found, it is run by the MBR.
A secure bootloader:
 (1) uses cryptographic operations to verify the signature of the firmware (Authenticity) and 
 (2) that is it not corrupted (Data Integrity). This is performed in two scenarios: at bootup, and when a new firmware image is received.
If a bootloader is not found, the MBR boots the image that follows it (the MBR) at address 0x1000 (the SoftDevice).
The SoftDevice then boots the application.

There are four different boot validation modes that can be configured:

1. Signature validation (ECDSA) – most secure, and data integrity check.

2. Hash validation (SHA-256) – less security, and data integrity check.
  
3. CRC validation (CRC32) – no security, only data integrity check.
  
4.  No validation – no security, no integrity check.

These modes are configured as part of the firmware update package. If a signature mode is specified, then the signature will exist in the package. For hash and CRC validation, the cryptographic digest is created on-chip and written to flash when the update is applied.

Important Note: the boot validation is independent of the firmware update validation process. This means that the update package is signed regardless of the secure boot mode contained in it. This ensures that the system is protected from unauthorized firmware updates even with no boot validation.

## The Device Firmware Update (DFU) Process
The DFU process can be run by using one of the following Nordic tools. Each of these tools is used to send the DFU package to the target device to perform the update.

* The nrfutil command-line tool
* nRF Connect for desktop
* nRF Connect for mobile
Two devices are involved in the DFU process: the DFU controller which transfers the DFU package, and the DFU target which receives and applies the DFU package. (the TSDZ2 remote or controller)
### Step 1. Generate DFU .zip packet
A DFU .zip packet is required for the DFU master to send new image(s) to the TSDZ2 remote control or controller boards.(the target) The .zip file contains the image hex file(s) we want to update and the init packet, including the signature of the packet. 
Prepare the TSDZ2 application zip file. Build the TSDZ2 remote control and/or the TSDZ2 controller and find the .zip file inside the _build folder. It's named TSDZ2_combined_with_sd. 
The make file uses nrfutil to generate the packet zip file.
The syntax is as follows:
nrfutil pkg generate --hw-version 52 --application-version 1 --application nrf52832_xxaa.hex --sd-req 0x98 --key-file private.key app_dfu_package.zip
Explanation:
--hw-version: This should match the chip on the dongle. We use the nRF52840 SOC chip, hence "52". 
--application-version: By default the start number for the application version is 0.  
--sd-req: For the TSDZ2, we need Softdevice S340 v6.1.1. The number to use for this softdevice version is 0xB9. 
--application : Tells nrfutil that you going to update the application and an application image is provided.
### Step 2. Performing DFU (Bluetooth or USB)
#### BLE DFU
Now you have your DFU .zip file containing the application update and the bootloader and bootloader settings installed on the board, it's time to actually do wireless DFU.
1.  Start DFU mode Verify the bootloader starts advertising as "TSDZ2_DFU".You’ll need to make sure the package you created in step #7 is accessible on the mobile phone you’re using (if you’re using nRF Connect for Mobile). Now, connect to the DFU target:

2. Copy the DFU .zip file you just generated to the phone or to the folder on PC of your choice.
   
3. Use nRFConnect/nRFToolbox app either on the phone or on PC to connect and do an OTA DFU using the .zip file you copied (Press DFU button).After you’ve connected to the DFU target in nRF Connect on Mobile, swipe to the left twice to navigate to the DFU screen. Then click “Open Document Picker”:Choose the .zip file  and start the DFU process.
Click the “Start” button:
   
4. Once the DFU process is complete, you should see that your new application (that was part of the DFU package) is now running on the DFU Target 

Here is a video showing a wireless DFU for the TZDZ2 remote control:
[![video](https://img.youtube.com/vi/va3LJoiosoc/hqdefault.jpg)](https://youtu.be/va3LJoiosoc)
#### USB DFU
The Nrfutil utility can be used to perform DFU over the USB serial port.
Open a terminal and run nrfutil DFU --help as follows:

![](./Images/nrfutil.png)
compose the command to update the firmware ie:
nrfutil dfu usb-serial -pkg app_dfu_package.zip -p /tty/ttyACM0
A video explaining this can be found below:
[![video](https://img.youtube.com/vi/CRthZeFhfZY/hqdefault.jpg)](https://youtu.be/CRthZeFhfZY)
## Appendix - Advanced features
### 1. Secure Boot Validation
There is a boot validation performed at the booting phase of the bootloader involving for the application, softdevice and the bootloader.
In short, a normal secure booting sequence starts with the MBR booting first, it then starts the bootloader. The bootloader does two tasks, it protects it self using BPROT/ACL protection and then validate the softdevice and the application image. You can choose the boot option when generating the DFU package. Use --sd-boot-validation for softdevice validation and --app-boot-validation when generating the DFU .zip package with nrfutil, you have 4 options to choose from:
- NO_VALIDATION
- VALIDATE_GENERATED_CRC
- VALIDATE_GENERATED_SHA256
- VALIDATE_ECDSA_P256_SHA256. 
This also applies for bootloader setting. You can generate the bootloader setting with the same option for boot validating the application and/or the softdevice, instead of just CRC validation for application as before. 
### 2. Button based DFU
If, for any reason, you want to force the firmware into DFU mode, this can bee done using physical buttons. To enter DFU bootloader mode on the remote control, press any button on the keypad for 10 seconds or longer.
To enter DFU mode on the wireless TSDZ2 controller, press the button on the board. 
### 3. Firmware memory map
See this video for an explanation of the memory map for the nRF52840 board:
[![video](https://img.youtube.com/vi/MZ6Qz32tY0c/hqdefault.jpg)](https://youtu.be/MZ6Qz32tY0c)
### 4. Buttonless DFU DFU details
a BLE packet is used to switch to DFU mode without physical contact (buttonless DFU).
The way it works is pretty simple, to switch, we write to the retention register GPREGRET a flag (BOOTLOADER_DFU_START = 0xB1) and then we do a soft reset. This is done in bootloader_start() in ble_dfu.c file.
Since the retention register keeps its value after the reset, the bootloader will check this value when it booting up after the reset and then can enter DFU mode instead of starting the normal application. This is the same as when we hold the Bootloader button and trigger a reset.
