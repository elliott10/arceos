fada: build
	gzip -9 -cvf $(OUT_BIN) > arceos-fada.bin.gz
	mkimage -f tools/bsta1000b/bsta1000b-fada-arceos.its arceos-fada.itb
	@echo 'Built the FIT-uImage arceos-fada.itb'

rk3588: build
	mkimage -n ArceOS -A arm64 -O linux -T kernel -C none -a 0x00480000 -e 0x00480000 -d $(OUT_BIN) arceos-rk3588.itb
	cp arceos-rk3588.itb /srv/tftp/
	#scp arceos-rk3588.itb os@192.168.0.250:/srv/tftp/
	@echo 'Built the u-boot image arceos-rk3588.itb'
