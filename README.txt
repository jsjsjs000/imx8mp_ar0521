vi /boot/bootenv.txt
# --------------------------------------------------------------------------------
overlays=imx8mp-isi-csi1.dtbo imx8mp-vm017-csi1.dtbo
# --------------------------------------------------------------------------------

  # Visual Studio Code
# Ctrl+Shift+B  # see .vscode/tasks.js

	# usefull aliases
alias co='make -j $(nproc)'
# alias co='make -j $(nproc) LOCALVERSION="-bsp-yocto-nxp-i.mx8mp-pd23.1.0" EXTRAVERSION=""'
# alias upimg='scp arch/arm64/boot/Image root@192.168.3.11:/boot/'
alias up='ssh root@192.168.3.11 " \
  mkdir -p /lib/modules/5.15.71-bsp-yocto-nxp-i.mx8mp-pd23.1.0/kernel/drivers/media/i2c/" && \
  scp ar0521.ko root@192.168.3.11:/lib/modules/5.15.71-bsp-yocto-nxp-i.mx8mp-pd23.1.0/kernel/drivers/media/i2c/'
alias re='ssh root@192.168.3.11 reboot'
alias mod='ssh root@192.168.3.11 "modprobe -r ar0521; sleep 1; modprobe ar0521; ls /dev/video*; ls /dev/csi*; ls /dev/media*; ls /dev/cam-*; ls /dev/v4l*; media-ctl -p"'

clear; co && up && mod
