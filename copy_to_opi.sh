#!/bin/bash

scp -r *.ko orangepi@orangepi:/home/orangepi/modules/
scp rt9467_charger_i2c.dtbo orangepi@orangepi:/home/orangepi/modules/

