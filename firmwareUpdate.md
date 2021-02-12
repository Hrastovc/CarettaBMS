---
page_name: firmwareUpdate
title: Firmware update
layout: main
---

To enter the bootloader send 0x62 (on 8N1 or 8E2) until you receive 0xEB
response or until now response is received. To test if the mouldes are in the
bootloader send 0x30 0x20 (on 8N1), if in bootloader 0x14 0x10 response should
be received.
