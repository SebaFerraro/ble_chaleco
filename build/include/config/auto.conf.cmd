deps_config := \
	/home/sferraro/esp/esp-idf/components/app_trace/Kconfig \
	/home/sferraro/esp/esp-idf/components/aws_iot/Kconfig \
	/home/sferraro/esp/esp-idf/components/bt/Kconfig \
	/home/sferraro/esp/esp-idf/components/esp32/Kconfig \
	/home/sferraro/esp/esp-idf/components/ethernet/Kconfig \
	/home/sferraro/esp/esp-idf/components/fatfs/Kconfig \
	/home/sferraro/esp/esp-idf/components/freertos/Kconfig \
	/home/sferraro/esp/esp-idf/components/heap/Kconfig \
	/home/sferraro/esp/esp-idf/components/libsodium/Kconfig \
	/home/sferraro/esp/esp-idf/components/log/Kconfig \
	/home/sferraro/esp/esp-idf/components/lwip/Kconfig \
	/home/sferraro/esp/esp-idf/components/mbedtls/Kconfig \
	/home/sferraro/esp/esp-idf/components/openssl/Kconfig \
	/home/sferraro/esp/esp-idf/components/pthread/Kconfig \
	/home/sferraro/esp/esp-idf/components/spi_flash/Kconfig \
	/home/sferraro/esp/esp-idf/components/spiffs/Kconfig \
	/home/sferraro/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/home/sferraro/esp/esp-idf/components/wear_levelling/Kconfig \
	/home/sferraro/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/sferraro/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/sferraro/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/sferraro/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
