idf_component_register(SRCS "midi_ethernet_sender.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver spi_flash usb freertos)