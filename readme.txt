If you want to build the driver from source codes, please copy or download the content of this folder into your home directory. Open a terminal and change your path into the source codes folder.
Use sudo su or su command to work as root user.
Type "make" to build the V4L2 driver from the source codes.
Then you can use shell command : 
sh load.sh    --- manually load the driver.
sh install.sh    --- manually load the driver.

Current driver is for real-time video capture with V4L2 video capture devices and audio capture with ALSA sound card PCM capture with 8 substreams.

You can type :   ls  /dev/video*    ---- to list the installed video devices.
You can try different video for Linux applications, like mplayer, VLC player, TVtime, etc.
If you installed mplayer, you can type:
mplayer tv:// -tv device=/dev/video0:outfmt=yuy2:normid=3:width=704:height=480 for NTSC, you can use height=576 to PAL50 signal.

open different terminals and use the command line with different videoX  number to test all 8 real-time capture video device and playback on windows.

You can also use tvtime, xawtv,vlc player to test each video device. Videp standard (PAL50Hz/NTSC60Hz) will be auto detected.
Default video frame size is 704*480 for NTSC, 704*576 for PAL50Hz.

After installed VLC player, you can use command line: 
vlc v4l2:///dev/video0  to play /dev/video0
vlc v4l2:///dev/video4  to play /dev/video4

ALSA support:

Using following command to list the registered TW68 audio devices

[simon@localhost ~]$ ls /proc/asound -l
total 0
dr-xr-xr-x. 7 root root 0 Jan 10 04:25 card0
dr-xr-xr-x. 3 root root 0 Jan 10 04:25 card1
dr-xr-xr-x. 3 root root 0 Jan 10 04:25 card2
-r--r--r--. 1 root root 0 Jan 10 04:25 cards
-r--r--r--. 1 root root 0 Jan 10 04:25 devices
lrwxrwxrwx. 1 root root 5 Jan 10 04:25 HDMI -> card1
-r--r--r--. 1 root root 0 Jan 10 04:25 hwdep
-r--r--r--. 1 root root 0 Jan 10 04:25 modules
dr-xr-xr-x. 2 root root 0 Jan 10 04:25 oss
-r--r--r--. 1 root root 0 Jan 10 04:25 pcm
lrwxrwxrwx. 1 root root 5 Jan 10 04:25 SB -> card0
dr-xr-xr-x. 2 root root 0 Jan 10 04:25 seq
-r--r--r--. 1 root root 0 Jan 10 04:25 timers
lrwxrwxrwx. 1 root root 5 Jan 10 04:25 TW68SoundCard -> card2
-r--r--r--. 1 root root 0 Jan 10 04:25 version



[simon@localhost ~]$ arecord -l
**** List of CAPTURE Hardware Devices ****
card 0: SB [HDA ATI SB], device 0: ALC889A Analog [ALC889A Analog]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 0: SB [HDA ATI SB], device 1: ALC889A Digital [ALC889A Digital]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 0: SB [HDA ATI SB], device 2: ALC889A Analog [ALC889A Analog]
  Subdevices: 2/2
  Subdevice #0: subdevice #0
  Subdevice #1: subdevice #1
card 2: TW68SoundCard [TW68 PCM], device 0: TW68 PCM [TW68 Analog Audio Capture]
  Subdevices: 8/8
  Subdevice #0: TW68 #0 Audio In 
  Subdevice #1: TW68 #1 Audio In 
  Subdevice #2: TW68 #2 Audio In 
  Subdevice #3: TW68 #3 Audio In 
  Subdevice #4: TW68 #4 Audio In 
  Subdevice #5: TW68 #5 Audio In 
  Subdevice #6: TW68 #6 Audio In 
  Subdevice #7: TW68 #7 Audio In 


ALSA utilty command line live capture and playback:
arecord -f S16_LE -r 48000 -D hw:TW68SoundCard,0,7 |aplay
arecord -f S16_LE -r 32000 -D hw:TW68SoundCard,0,0 |aplay

recording:
arecord -f S16_LE -r 48000 -D hw:TW68SoundCard,0,7  a7.wav
arecord -f S16_LE -r 8000 -D hw:TW68SoundCard,0,0 a0.wav

You can also install VLC player
open the GUI pulldown menu  Media - Open Capture Device

fill the Video device name with "/dev/videon"
fill the Audio device name with "hw:TW68SoundCard,0,n"
n range 0 ~ 7

Audio capture hardware only support one audio sample rates for all 8 audio decosers.

