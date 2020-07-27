set(COMPONENT_SRCS "pgpemu.c aes.c cert-test.c pgp-cert.c secrets.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(REQUIRES tft spiffs)

register_component()