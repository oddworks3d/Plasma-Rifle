# Plasma Rifle

Code repository for the software running on the Plasma Rifle. Created using MicroPython with a new handy dandy GUI Installer that will flash micropython and also upload the code so you no longer have to use Thonny!



# Installation
## Download appropiate installer
Under the releases tab, you'll find a download for both the Mac OS and Windows
versions of the installer. Download the appropriate installer to suit your operating system.

(Currently only Windows 11 x86, Windows 11 arm64 and MacOS arm64 has been tested.)

## Flash MicroPython
After downloading and unzipping your selected version of the installer, and accepting all the pop ups that warn you it's dangerous (it's 100% not, the code for the installer is in this repository in the `installer` folder)

Run the installer and you'll be greeted with this:

![alt text](https://github.com/oddworks3d/Plasma-Rifle/blob/main/Images/step1-1.jpg?raw=true)
*Before continuing, ensure your Pi Pico is in USB Mass Storage mode by holding down the BOOTSEL button while plugging the pico in to your computer.*



Click `Select Pi Pico Directory` and search your filesystem for the mounted drive that is your Pi Pico.

>On mac it will look something like:
>```/Volumes/RPI-RP2```

>On windows it will look something like: 
>```F:/```

After selecting your Pi Pico, hit `Install MicroPython` to begin installing MicroPython on your Pico.

## Install Plasma Rifle Code

Once MicroPython is installed (Or if you have already installed MicroPython you can hit the `Skip` button) you'll move onto Step 2:

![alt text](https://github.com/oddworks3d/Plasma-Rifle/blob/main/Images/step2-1.jpg?raw=true)
*Before continuing ensure your Pi Pico is **NOT** in USB Mass Storage mode by and that it is connected to your computer*

From the dropdown, select your connected Pi Pico which will look different depending on what devices are connected and if you are on windows or MacOS

> For mac it will look similar to:
> `/dev/tty.usbmodem21301`

>For windows it will look similar to:
>`COM3`

After selecting the correct device from the list of devices, hit the `Install Plasma Rifle Code` button to begin installing.

If nothing fails and there are no errors you're done! Enjoy!


## Troubleshooting

If you don't want to run the installer for whatever reason, the code is available to download and install on the Pi Pico using Thonny (https://thonny.org)

Simply download the `main.py` file from the binaries release under the Releases tab and open it in thonny, save it to the pico, and that's it!.
